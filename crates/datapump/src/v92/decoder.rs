//! The digital modem's upstream decoder (6.4 read backwards): equalised levels
//! in, scrambled bits out, with the V.34 trellis the 4D symbols ride on and
//! the twelve-interval modulus decoder behind it.
//!
//! V92-20. What the digital modem has to undo, in the order the analogue modem
//! did it:
//!
//! ```text
//! levels -> cheapest point per residue class -> Viterbi over four-symbol frames
//!        -> etas -> Ki (6.4.2 backwards) -> modulus decoder (6.4.1) -> GPA
//! ```
//!
//! The trellis frame is four symbols (Figure 1) and the code is V.34's (6.4.4),
//! so this runs [`crate::v34::trellis::Code`] exactly as V.34 does and changes
//! only how often it is clocked.
//!
//! Two things make this decoder smaller than V.34's, and both are properties of
//! the upstream rather than shortcuts. There is no superframe and no modulo
//! encoding -- 6.4.4 cites 9.6.3.2 alone -- so the only structure over four
//! symbols is the code. And the constellation sets are the *chosen* ones,
//! uploaded in CPd, so a point's level is known exactly instead of being
//! derived from a shell mapping.
//!
//! The fourth index is not free: 6.4.2's class at k = 3 steps by 2 x Mi, and
//! [`Chain`](super::precoder::Chain) takes its parity bit from the three
//! indices before it and Y0. That is why the per-frame table below is built
//! over residue classes rather than over points.

use std::collections::VecDeque;

use crate::v32::{Mode, Scrambler};
use crate::v34::trellis::{self, Code};

use super::modulus::{self, FrameBits, Moduli12};
use super::precoder::{
    Constellation, code_for, index_to_ki, index_to_ki_class, label_of, opens_a_pair, pair_first_index_offset,
};
use super::{B1U_FRAMES, Parameters, TRELLIS_FRAME, UP_INTERVALS, UP_RATES, up_bits};

/// Trellis frames held before the oldest is decided on. Forty frames is a
/// hundred and sixty symbols, twenty milliseconds: long enough for every code
/// here to have settled, short enough that the tail a cleardown cuts off is
/// nothing.
pub const SURVIVOR_DEPTH: usize = 40;

/// `V92_K3`: the receive-side correction `docs/47` measures, and nothing else.
///
/// Two tables, because the measurement separates two things that look alike:
///
/// * `levels=d0,d1,d2,d3` is a sub-step displacement of the level at a trellis
///   position, in half-steps -- the units `a(eta) = 2|eta| + 1` is written in,
///   so one level step is 2. It is taken off the level before it is sliced, at
///   the decoder's own symbol count, from the first symbol of B1u on.
/// * `rot=r0,r1,r2,r3` is the far end's class rotation, in the same convention
///   as `v92_b1u_class --offset`: the model's class is the observed one minus
///   the table's entry, and the one this decoder already applies is the pair
///   offset at k = 0 and 2 and 0 at k = 1 and 3 (`PAIR_FIRST_INDEX_OFFSET`).
///   Only the difference from that one is applied, so the default path is
///   untouched.
///
/// `V92_K3=levels=0,0,0,0.2;rot=6,6,6,7` is the shape the measurement found.
/// Unset, off, or unparsable is no correction at all -- the level and the class
/// are the ones this decoder has always fed and taken, bit for bit.
fn correction() -> Option<&'static Correction> {
    use std::sync::OnceLock;
    static FIX: OnceLock<Option<Correction>> = OnceLock::new();
    FIX.get_or_init(|| {
        let text = std::env::var("V92_K3").ok()?;
        let mut fix = Correction { levels: [0.0; TRELLIS_FRAME], rotation: None };
        let mut any = false;
        // Fields are separated by `;` or a space, not by a comma -- the
        // comma is what the tables themselves are made of.
        for field in text.split([';', ' ']).filter(|f| !f.is_empty()) {
            if let Some(value) = field.strip_prefix("levels=") {
                fix.levels = table_of(value)?;
                any = true;
            } else if let Some(value) = field.strip_prefix("rot=") {
                fix.rotation = Some(table_of(value)?);
                any = true;
            }
        }
        any.then_some(fix)
    })
    .as_ref()
}

/// Four values, or nothing.
fn table_of<T: std::str::FromStr>(text: &str) -> Option<[T; TRELLIS_FRAME]> {
    let values: Vec<T> = text.split(',').map(|word| word.trim().parse().ok()).collect::<Option<Vec<T>>>()?;
    values.try_into().ok()
}

/// What `docs/47`'s two tables are, when `V92_K3` names them.
struct Correction {
    /// Half-steps taken off the level at each trellis position.
    levels: [f64; TRELLIS_FRAME],
    /// The far end's class rotation, in `v92_b1u_class --offset`'s convention.
    rotation: Option<[i64; TRELLIS_FRAME]>,
}

/// `V92_BY_CONTENT=1`: read B1u's data frames against 8.7.1's known content
/// instead of 6.4.1's chain, above 24 000 only. Unset, off, or anything but `1`
/// leaves the spec chain in place, which is what every 24 000 number is taken
/// with -- `by_content` cannot engage at that rung at all (`BITS_AT_THE_CONSTANT`).
fn by_content_wanted() -> bool {
    std::env::var("V92_BY_CONTENT").ok().as_deref() == Some("1")
}

/// `V92_ROT`: measure this call's class rotation off B1u, once, and use it in
/// place of `PAIR_FIRST_INDEX_OFFSET`.
///
/// The constant is a measurement of *one* far end, and so is `V92_K3`'s `rot=`
/// table: both name what one particular peer does, and a peer that does
/// anything else has its indices rotated by the difference -- which is how a
/// fixed table takes an Agere from 563 of 576 to 323 (`docs/47 §5`). 8.7.1
/// makes B1u's 576 symbols known and `docs/46` shows the model that predicts
/// their classes is exact (360/360 on an Agere), so the rotation can be *read
/// off the call* instead of assumed:
///
/// ```text
/// per position k: rotation[k] = mode over frames of (observed - predicted) mod 8
/// ```
///
/// "Predicted" is B1u's class sequence, which is known up to **one global
/// sign**: 6.4.1's differential step exists so that an inverted line decodes
/// the same, so the sequence is defined either as the model reads it or as
/// `Mi - 1 - Ki`, and which of the two a given capture is aligned with is one
/// bit, not five hundred and seventy-six (`docs/46 §2`; `docs/46 §3` lists both
/// readings for Agere, and only the second is 90/90). So the histogram is kept
/// for both signs and the sign that the line actually reads is taken, by
/// agreement -- no far end is named, and a wrong choice cannot hurt: the bar
/// below throws away a position that does not have a mode anyway.
///
/// Unset, empty or `0` is off, and off is the constant and nothing else: the
/// same classes, the same bits, the same files as before this existed.
pub fn rotation_wanted() -> bool {
    use std::sync::OnceLock;
    static WANTED: OnceLock<bool> = OnceLock::new();
    *WANTED.get_or_init(|| match std::env::var("V92_ROT") {
        Ok(text) => !text.trim().is_empty() && text.trim() != "0",
        Err(_) => false,
    })
}

/// How many symbols B1u is measured over: forty per trellis position, i.e.
/// a hundred and sixty symbols of B1u's five hundred and seventy-six.
///
/// Forty and not more because a decision cannot be re-taken: the Viterbi
/// decides its first frame a hundred and sixty-four symbols in (`SURVIVOR_DEPTH`
/// frames plus one, from the first symbol this decoder is fed), so a
/// measurement that ran past that would either decide the first frames on the
/// constant and the rest on the measurement -- two rotations in one B1u -- or
/// have to hold the window open, which changes the decisions of exactly the
/// frames it held. Forty samples a position is what fits before the first
/// decision with the window untouched, and it is far more than the bar in
/// [`Measuring::rotation`] needs: `docs/46`'s plateaus are per-position
/// properties of the far end, not something that only shows up after ninety
/// symbols.
pub const ROTATION_SYMBOLS: u64 = 40 * TRELLIS_FRAME as u64;

/// The classes a rotation can name: the period of 6.4.2's class map, which is
/// the CPU's Mi and therefore the rung (`docs/46 §2`). Eight until `docs/74`:
/// a rate above 24 000 has Mi = 16, and the measurement used to refuse to run
/// there at all rather than name a class outside its histogram.
const CLASSES: usize = 16;

/// The most bits one data frame carries at the rung the constant rotation was
/// measured on (drn 1, K = 36). Above it the class map is wider than the
/// constant was measured at, so the rotation has to be taken off the line.
const BITS_AT_THE_CONSTANT: u32 = 36;

/// B1u's expected class sequence, and what a capture's classes have left
/// against it.
///
/// At class level this is 8.7.1's known content against 6.4.2 read backwards
/// with no Viterbi, no equaliser and no descrambler in the way: one nearest
/// point per symbol, off the table the slicing has just built, and one
/// histogram bin.
#[derive(Debug, Clone)]
struct Measuring {
    /// The model's 576 classes, B1u's first symbol first.
    predicted: Vec<u8>,
    /// Which of those 576 the first symbol fed to this decoder is.
    first: usize,
    /// `counts[sign][k][c]`: how many symbols at trellis position `k` came out
    /// `c` classes away from the model's, mod 8, when the model is read with
    /// global sign `sign` (`0` as 6.4.1 prints it, `1` complemented).
    counts: [[[u16; CLASSES]; TRELLIS_FRAME]; 2],
    /// The class width this rung's moduli name, which is Mi: eight at 24 000
    /// and sixteen at 32 000. The histogram bins are mod this, not mod the
    /// array's width -- `CLASSES` is only how wide the array is.
    classes: usize,
    /// Symbols measured.
    seen: u64,
    /// What the model cost to build, and what the counting cost, for
    /// `V92_ROT_TRACE`. Neither is on the per-symbol path: this runs once.
    model_cost: std::time::Duration,
    count_cost: std::time::Duration,
}

impl Measuring {
    /// A measurement of B1u from its `first` symbol on, or nothing when a
    /// modulus is wider than the histogram: the rotation lives in the class
    /// space, and a rate whose moduli are not eight does not have eight classes
    /// to name.
    fn new(bits: u32, moduli: [u8; UP_INTERVALS], first: usize) -> Option<Self> {
        if moduli.iter().any(|&modulus| usize::from(modulus) > CLASSES) {
            return None;
        }
        let started = std::time::Instant::now();
        let predicted = b1u_classes(bits, &moduli);
        let model_cost = started.elapsed();
        let classes = usize::from(*moduli.iter().max().expect("twelve moduli"));
        (first < predicted.len()).then(|| Self {
            predicted,
            classes,
            first,
            counts: [[[0; CLASSES]; TRELLIS_FRAME]; 2],
            seen: 0,
            model_cost,
            count_cost: std::time::Duration::ZERO,
        })
    }

    /// One symbol's class against the model's, under both readings of it.
    fn observe(&mut self, modulus: u8, eta: i32) {
        let index = self.first + self.seen as usize;
        let Some(&predicted) = self.predicted.get(index) else { return };
        // The trellis position in B1u's own numbering, not in this decoder's:
        // they differ by the acquisition skip, and only coincidentally by a
        // multiple of four.
        let k = index % TRELLIS_FRAME;
        self.seen += 1;
        let started = std::time::Instant::now();
        let class = i64::from(index_to_ki_class(eta, modulus, k));
        let complement = self.classes as i64 - 1 - i64::from(predicted);
        for (sign, predicted) in [i64::from(predicted), complement].into_iter().enumerate() {
            let away = (class - predicted).rem_euclid(self.classes as i64);
            self.counts[sign][k][away as usize] += 1;
        }
        self.count_cost += started.elapsed();
    }

    /// Whether B1u has been measured for long enough.
    fn done(&self) -> bool {
        self.seen >= ROTATION_SYMBOLS || self.first + self.seen as usize >= self.predicted.len()
    }

    /// The rotation at each position where the mode is clear enough to be one,
    /// and nothing where it is not.
    ///
    /// The bar, per position: the mode holds at least half the samples *and*
    /// at least twice the next class. Both halves are chosen against
    /// `docs/46`'s tables rather than for a round number.
    ///
    /// * Half the samples is what rejects a distribution with no mode at all --
    ///   eight classes evenly is 12.5 % -- and, just as importantly, a
    ///   measurement taken against the wrong 576 symbols, which is what a
    ///   misaligned model looks like: noise.
    /// * Twice the next class is what rejects the case the class agreement
    ///   alone cannot settle, a split between two candidates. A wrong rotation
    ///   is not a smaller error than no rotation, it is about half the bits,
    ///   so a near-tie has to fall back rather than guess.
    ///
    /// Where the two verified far ends sit against it, in the counts this
    /// measurement actually sees (48 samples a position):
    ///
    /// * every position of an Agere and positions 0-2 of an HSF are 90/90 in
    ///   `docs/46`, i.e. 48/48 here: both halves clear by everything;
    /// * HSF's k = 3 is the one plateau -- 54/90 there, runner-up 14/90, so
    ///   60 % and 3.9:1 -- which here is about 29 of 48 against 8. It clears
    ///   both halves (29 >= 24, 29 >= 16), and that matters: `docs/47 §5` reads
    ///   438 of 576 with k = 3 at 7 and 356 without it, so a bar that threw the
    ///   plateau away would leave HSF short of the only configuration that
    ///   connects. A bar at 90 % would do exactly that, and is why the bar is
    ///   at half.
    fn rotation(&self) -> [Option<i64>; TRELLIS_FRAME] {
        // Which reading of the model this line is on: the one whose four
        // positions agree best. Ties go to the first, and a tie is not a
        // measurement anyway -- the per-position bar below is what decides
        // whether anything is taken from it.
        let agreement = |sign: usize| -> u32 {
            self.counts[sign]
                .iter()
                .map(|counts| u32::from(counts.iter().copied().max().unwrap_or(0)))
                .sum()
        };
        let _sign = usize::from(agreement(1) > agreement(0));
        let mut table = [None; TRELLIS_FRAME];
        // Both readings of the model are allowed to explain a symbol, and at
        // 32 000 both are needed: the far end there reflects whole frames on a
        // chain of its own, so a frame reads against one reading or against the
        // other and neither reading alone is a mode (`docs/74`). The rotation
        // is a property of the far end's index map, which does not alternate,
        // so it is the *union* that has to be sharp -- and it is: 90 of 90
        // classes a position at 32 000, against 36 and 54 taken separately.
        for (k, counts) in self.counts[0].iter().enumerate() {
            let union: Vec<u32> = (0..self.classes)
                .map(|class| u32::from(counts[class]) + u32::from(self.counts[1][k][class]))
                .collect();
            let samples: u32 = counts.iter().map(|&count| u32::from(count)).sum();
            let mut ranked: Vec<(u32, usize)> =
                union.iter().enumerate().map(|(class, &count)| (count, class)).collect();
            ranked.sort_unstable_by(|a, b| b.cmp(a));
            let best = ranked[0];
            let next = ranked.get(1).copied().unwrap_or((0, 0));
            if best.0 > 0 && best.0 * 2 >= samples && best.0 >= next.0 * 2 {
                table[k] = Some(best.1 as i64);
            }
        }
        table
    }
}

/// `V92_ROT_TRACE`: what one measurement saw and what it accepted. Diagnostic,
/// written once per decoder that finished one; nothing in the data path reads
/// it.
fn trace_rotation(measuring: &Measuring, table: &[Option<i64>; TRELLIS_FRAME]) {
    let Some(path) = std::env::var_os("V92_ROT_TRACE") else { return };
    let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) else { return };
    use std::io::Write;
    let _ = writeln!(
        file,
        "rot first {} seen {} model_ns {} count_ns {}",
        measuring.first,
        measuring.seen,
        measuring.model_cost.as_nanos(),
        measuring.count_cost.as_nanos()
    );
    for (sign, positions) in measuring.counts.iter().enumerate() {
        for (k, counts) in positions.iter().enumerate() {
            let row: Vec<String> = counts.iter().map(u16::to_string).collect();
            let _ =
                writeln!(file, "sign {sign} k {k} [{}] accepted {}", row.join(" "), u8::from(table[k].is_some()));
        }
    }
    let table: Vec<String> = table.iter().map(|entry| entry.map_or("-".to_string(), |v| v.to_string())).collect();
    let _ = writeln!(file, "table {}", table.join(","));
}

/// B1u's class sequence: 8.7.1's forty-eight data frames of ones through 6.3's
/// scrambler and 6.4.1's modulus encoder, from memories that start zero.
///
/// Nothing the precoder does is an input. The precoder, the prefilter, the
/// trellis and G choose *which member* of a class goes out and not which class
/// (`docs/46 §1`), so these 576 classes are the same for every far end and all
/// a capture can differ by is a rotation of them. The scrambler and the mixed
/// radix are the production objects the encoder uses, not a second reading of
/// them.
fn b1u_classes(bits: u32, moduli: &[u8; UP_INTERVALS]) -> Vec<u8> {
    let moduli = Moduli12::new(*moduli);
    let mut scrambler = Scrambler::new(Mode::Answer);
    let mut encoder = modulus::Encoder::new();
    let mut classes = Vec::with_capacity(B1U_FRAMES * UP_INTERVALS);
    for _ in 0..B1U_FRAMES {
        let scrambled: Vec<bool> = (0..bits).map(|_| scrambler.scramble(true)).collect();
        let Ok(frame) = encoder.encode(FrameBits::from_bits(&scrambled), &moduli) else { break };
        classes.extend_from_slice(&frame);
    }
    classes
}

/// Where to write one decided trellis frame's four indices, when
/// `V92_ETA_TRACE` names a file: a header line naming K and the twelve moduli,
/// then `stage eta0 eta1 eta2 eta3` per decided frame.
///
/// Everything the bit layer does -- the mixed radix, the differential sign
/// step and the scrambler -- happens downstream of these indices, so writing
/// them out once is what lets a suspected convention be settled by enumerating
/// it offline over the same capture instead of one bench run per guess. The
/// indices are what the equaliser and the Viterbi decided, and do not depend on
/// any of the conventions under test.
fn eta_trace_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_ETA_TRACE")?);
        let file = std::fs::File::create(&path).ok()?;
        Some(Arc::new(Mutex::new(file)))
    })
    .clone()
}

/// One decided frame's indices, to `V92_ETA_TRACE` if it is set.
fn trace_etas(decoder: &Decoder, stage: u64, etas: [i32; TRELLIS_FRAME]) {
    let Some(file) = eta_trace_file() else { return };
    let Ok(mut file) = file.lock() else { return };
    use std::io::Write;
    // The header is written by whoever gets the empty file first, so that the
    // reader does not have to be told K and the moduli separately.
    if file.metadata().map(|meta| meta.len()).unwrap_or(1) == 0 {
        let moduli: Vec<String> = decoder.moduli.values().iter().map(u8::to_string).collect();
        let _ = writeln!(file, "k {} moduli {}", decoder.bits_per_frame, moduli.join(" "));
    }
    let _ = writeln!(file, "{stage} {} {} {} {}", etas[0], etas[1], etas[2], etas[3]);
}

/// The cheapest index of one symbol in each residue class modulo 4, which is
/// all a 4D subset label depends on ([`label_of`] works on 2 x eta + 1).
#[derive(Debug, Clone, Copy)]
struct Bests {
    cost: [f64; 4],
    eta: [i32; 4],
}

impl Default for Bests {
    fn default() -> Self {
        Self { cost: [f64::MAX; 4], eta: [0; 4] }
    }
}

impl Bests {
    fn new(set: &Constellation, level: f64) -> Self {
        let mut bests = Self::default();
        for eta in set.lowest()..=set.highest() {
            let Some(point) = set.level(eta) else { continue };
            let residue = eta.rem_euclid(4) as usize;
            let cost = (level - point) * (level - point);
            if cost < bests.cost[residue] {
                bests.cost[residue] = cost;
                bests.eta[residue] = eta;
            }
        }
        bests
    }

    /// The nearest point overall, which is what a hard slice would give and
    /// what the class measurement needs: each residue class kept its own
    /// cheapest, so the nearest is the cheapest of the four. Two comparisons
    /// more than a four-way scan, and only while B1u is being measured.
    fn nearest(&self) -> Option<i32> {
        let mut best: Option<usize> = None;
        for residue in 0..4usize {
            if self.cost[residue] == f64::MAX {
                continue;
            }
            if best.is_none_or(|best| self.cost[residue] < self.cost[best]) {
                best = Some(residue);
            }
        }
        best.map(|residue| self.eta[residue])
    }
}

/// One trellis frame in the Viterbi's window: its branch metrics and the four
/// indices each of them means.
#[derive(Debug, Clone, Copy)]
struct Stage {
    /// `table[out * 16 + y]`, the cost of the best four indices for that (Y0,
    /// y), or infinite when no indices make it.
    table: [f64; 32],
    /// Which residue class each of the four symbols was taken from, per
    /// (out, y).
    residues: [[(i32, i32, i32, i32); 16]; 2],
    /// The per-symbol tables it was built from, so that a decision can name
    /// the indices rather than the classes.
    bests: [Bests; TRELLIS_FRAME],
    /// Which trellis frame this is, counted from the first symbol of B1u.
    /// Three of them make a data frame (Figure 1), so only every third may
    /// start one -- a decision that begins part way through a data frame has
    /// nothing to give.
    index: u64,
}

/// The upstream decoder: one equalised level a symbol in, bits out every
/// twelve.
#[derive(Debug, Clone)]
pub struct Decoder {
    moduli: Moduli12,
    code: Code,
    params: Parameters,
    descrambler: Scrambler,
    modulus: modulus::Decoder,
    bits_per_frame: u32,
    /// The symbol count since the first symbol of B1u (8.7.1).
    n: u64,
    frame: [Bests; TRELLIS_FRAME],
    filled: usize,
    metrics: Vec<f64>,
    stages: VecDeque<Stage>,
    /// `back[i][state]` is where the state at frame i + 1 came from, and
    /// whether Y0 went out inverted to get there.
    back: VecDeque<Vec<(u8, bool)>>,
    /// Ki decided but not yet a whole data frame's worth.
    labels: Vec<u8>,
    bits: Vec<bool>,
    decided: u64,
    /// B1u's known classes, while they are being measured against
    /// (`V92_ROT`); nothing once the measurement is done, or if it was off.
    measuring: Option<Measuring>,
    /// The rotation in force at each position: the measurement where it was
    /// clear enough, and nothing where the constant is to be kept.
    rotation: [Option<i64>; TRELLIS_FRAME],
    /// `V92_K3`'s `rot=` table, read once here so that the per-symbol path is
    /// an array index and not an environment lookup.
    named_rotation: [Option<i64>; TRELLIS_FRAME],
    /// Whether a data frame's polarity is to be settled from B1u's known
    /// content instead of from 6.4.1's chain. **Off unless `V92_BY_CONTENT=1`
    /// asks for it**, and then only above 24 000 and only while B1u is being
    /// read: it is a workaround for a far end whose 6.4.1 chain a payload
    /// cannot follow (`docs/74`), and it was on by default above 24 000 while
    /// `PAIR_FIRST_INDEX_OFFSET` was wrong -- the branch it picks is a symptom
    /// of that, not a fix for it. The caller says when:
    /// [`Self::read_by_content`].
    by_content: bool,
    /// Data frames read so far, so that `by_content` stops at the end of B1u:
    /// 8.7.1's 48 frames are the only ones whose content is known.
    pushed: usize,
}

impl Decoder {
    /// A decoder for a far end sending these parameters, starting at the first
    /// symbol of B1u: every memory zero and n = 0, exactly as the encoder
    /// starts (8.7.1).
    pub fn new(parameters: &Parameters) -> Self {
        let code = code_for(parameters.trellis);
        let mut metrics = vec![f64::MAX; code.states()];
        // Which convolutional state the encoder is assumed to be in at the
        // first symbol this decoder is handed. Zero is right only if that
        // symbol is the first symbol of B1u -- 8.7.1 zeroes the memories and
        // then starts encoding -- and it is not: `b1u_skip` drops eighteen
        // data frames first, so the far end's encoder has been running for
        // 216 symbols by the time this sees anything. `V92_B1U_STATE` sweeps
        // the assumption (diagnostic only; the default is the behaviour every
        // number here was taken with).
        let start = std::env::var("V92_B1U_STATE")
            .ok()
            .and_then(|text| text.parse::<usize>().ok())
            .filter(|state| *state < code.states())
            .unwrap_or(0);
        metrics[start] = 0.0;
        Self {
            moduli: Moduli12::new(parameters.moduli),
            code,
            params: parameters.clone(),
            descrambler: Scrambler::new(crate::v32::Mode::Answer),
            modulus: modulus::Decoder::new(),
            bits_per_frame: up_bits(parameters.drn),
            n: 0,
            frame: [Bests::default(); TRELLIS_FRAME],
            filled: 0,
            metrics,
            stages: VecDeque::new(),
            back: VecDeque::new(),
            labels: Vec::with_capacity(UP_INTERVALS),
            bits: Vec::new(),
            decided: 0,
            measuring: None,
            rotation: [None; TRELLIS_FRAME],
            by_content: false,
            pushed: 0,
            named_rotation: correction()
                .and_then(|fix| fix.rotation)
                .map_or([None; TRELLIS_FRAME], |table| table.map(Some)),
        }
    }

    /// One equalised symbol, on the scale the parameters' levels are in.
    pub fn feed(&mut self, level: f64) {
        // The wire carries G x v (8.8.3), while the constellation sets are the
        // pre-gain magnitudes a(eta), so the equalised level is divided back by
        // G before it is sliced against them.
        let mut level = level / self.params.gain;
        // `docs/47`'s per-position displacement, if it was asked for: one
        // subtract, keyed on the position this symbol is in.
        if let Some(fix) = correction() {
            level -= fix.levels[self.n as usize % TRELLIS_FRAME];
        }
        let interval = super::interval(self.n);
        let set: &[u16] = self.params.set_for(interval).unwrap_or(&[]);
        self.frame[self.filled] = Bests::new(&Constellation::new(set), level);
        // Which class this symbol fell into, against B1u's known one, while the
        // rotation is being measured: one nearest-point read off the table the
        // slicing has just built, for the first 192 symbols of B1u and never
        // again.
        if self.measuring.is_some() {
            let bests = self.frame[self.filled];
            let modulus = self.moduli.modulus(interval);
            let measuring = self.measuring.as_mut().expect("just checked");
            if let Some(eta) = bests.nearest() {
                measuring.observe(modulus, eta);
            }
            if measuring.done() {
                let table = measuring.rotation();
                trace_rotation(measuring, &table);
                self.rotation = table;
                self.measuring = None;
            }
        }
        self.filled += 1;
        self.n += 1;
        if self.filled == TRELLIS_FRAME {
            self.filled = 0;
            self.fill();
        }
    }

    /// Bits decoded so far, taken. Empty until the window has filled.
    pub fn take_bits(&mut self) -> Vec<bool> {
        std::mem::take(&mut self.bits)
    }

    /// Settle each B1u data frame's polarity from 8.7.1's known content, or
    /// stop doing that. On only where the far end's 6.4.1 chain cannot be read
    /// out of what it sends -- above 24 000 (`docs/74`) -- and only for as long
    /// as the content is known, which is B1u and nothing after it.
    ///
    /// **Opt-in, by `V92_BY_CONTENT=1`.** Off is the plain spec chain: 8.7.1
    /// zeroes the differential memory at B1u, so the decoder starts from
    /// `d(-1) = 0` and 6.4.1's reflection tells it which way up each frame is.
    /// Reading both sign candidates and keeping whichever comes out closer to
    /// B1u's ones is what a wrong pair-first offset looks like from here -- a
    /// correct offset leaves the chain nothing to choose -- so it is not the
    /// default while the offset is the measured one.
    pub fn read_by_content(&mut self, on: bool) {
        self.by_content = on && by_content_wanted() && self.bits_per_frame > BITS_AT_THE_CONSTANT;
    }

    /// Start 6.4.1's sign chain somewhere other than 8.7.1's zero.
    ///
    /// For a decoder built at B1u's boundary but fed from later in it: the
    /// first data frame it decodes is B1u's frame `frames_skipped`, and the far
    /// end's d(f - 1) there is
    /// [`b1u_differential_at`](crate::v92::modulus::b1u_differential_at) --
    /// known, because 8.7.1 makes B1u's payload known. A decoder nobody calls
    /// this on keeps the clause's `d(-1)`, which is what every other caller
    /// (E2u, RM, FB1u, a rate renegotiation) has to keep.
    pub fn seed_differential(&mut self, d: bool) {
        self.modulus.set_d(d);
    }

    /// Prime 6.3's descrambler with the bits that went out before the first one
    /// this decoder is fed.
    ///
    /// 8.7.1 makes them known for the same reason it makes the differential
    /// state known -- B1u's frames are scrambled ones, so the stream before
    /// this decoder starts is
    /// [`b1u_scrambled_bits`](crate::v92::modulus::b1u_scrambled_bits) -- and
    /// they matter because the descrambler is fed by what it decodes: from a
    /// zeroed register its first twenty-three outputs are wrong, however
    /// perfectly the levels are read.
    pub fn seed_descrambler(&mut self, preceding: &[bool]) {
        for &bit in preceding {
            self.descrambler.descramble(bit);
        }
    }

    /// Symbols fed since the first symbol of B1u.
    pub fn symbols(&self) -> u64 {
        self.n
    }

    /// Trellis frames decided on so far.
    pub fn decided(&self) -> u64 {
        self.decided
    }

    /// The rate this decoder was built for, in bit/s.
    pub fn rate(&self) -> u32 {
        self.bits_per_frame * 8000 / UP_INTERVALS as u32
    }

    /// The rung of 6.1's ladder this decoder was built for.
    pub fn drn(&self) -> u8 {
        (1..=UP_RATES as u8).find(|&drn| up_bits(drn) == self.bits_per_frame).unwrap_or(0)
    }

    /// Measure this call's class rotation off B1u, from the `first` symbol of
    /// B1u that this decoder is about to be fed.
    ///
    /// `first` is the caller's to know: the decoder is built at B1u's boundary
    /// and is fed from `b1u_skip + 1` symbols after it (`digital.rs`), while the
    /// model counts from B1u's own first symbol. A decoder nobody calls this on
    /// keeps the constant, which is what every B1u number before `docs/48` was
    /// taken with.
    pub fn measure_rotation_from(&mut self, first: usize) {
        // `V92_ROT` asks for it anywhere; above 24 000 it is not optional,
        // because `PAIR_FIRST_INDEX_OFFSET` is what an Agere's Mi = 8 line was
        // measured at and at Mi = 16 that constant is not the far end's
        // (`docs/74`). Nothing below 24 000 changes: there the measurement
        // returns the constant it would have been taken as.
        let wider = self.bits_per_frame > BITS_AT_THE_CONSTANT;
        if (!rotation_wanted() && !wider) || self.measuring.is_some() {
            return;
        }
        self.measuring = Measuring::new(self.bits_per_frame, self.params.moduli, first);
    }

    /// This frame's branch metrics, and one trellis step.
    fn fill(&mut self) {
        let mut stage = build(&self.frame, self.code);
        stage.index = self.n / TRELLIS_FRAME as u64 - 1;
        let states = self.code.states();
        let mut next = vec![f64::MAX; states];
        let mut from = vec![(0u8, false); states];
        for (state, &metric) in self.metrics.iter().enumerate() {
            if metric == f64::MAX {
                continue;
            }
            let out = usize::from(self.code.output(state as u8));
            for y in 0..16usize {
                if y & !self.code.inputs() as usize != 0 {
                    continue;
                }
                // Y0 is the state's own output and nothing else. V.34 lets a
                // half frame invert it; V.92's upstream has no superframe to be
                // half way through, so 6.4.3's parity is the only rule and a
                // flipped table entry is not a path this receiver may take.
                let cost = stage.table[out * 16 + y];
                if cost == f64::MAX {
                    continue;
                }
                let to = usize::from(self.code.next(state as u8, y as u8));
                if metric + cost < next[to] {
                    next[to] = metric + cost;
                    from[to] = (state as u8, false);
                }
            }
        }
        let floor = next.iter().copied().fold(f64::MAX, f64::min);
        if floor == f64::MAX {
            // Nothing in the constellation is anywhere near the line: hold
            // what is left of the window and start again rather than decide
            // on noise.
            self.stages.clear();
            self.back.clear();
            self.metrics.iter_mut().for_each(|metric| *metric = f64::MAX);
            self.metrics[0] = 0.0;
            return;
        }
        for metric in &mut next {
            if *metric != f64::MAX {
                *metric -= floor;
            }
        }
        self.metrics = next;
        self.stages.push_back(stage);
        self.back.push_back(from);
        // Belt and braces, and it does not fire with the acquisition skip this
        // receiver uses: a measurement that has not finished by the time the
        // window is full would leave the first frames decided on the constant
        // and the rest on the measurement, so the window is held open until it
        // has. With `ROTATION_SYMBOLS` of a hundred and sixty the measurement
        // is in a hundred and sixty symbols in and the first decision is at a
        // hundred and sixty-four, so nothing here is ever held and the
        // decisions of a measured call are the decisions of an unmeasured one.
        if self.measuring.is_some() {
            return;
        }
        while self.stages.len() > SURVIVOR_DEPTH {
            self.decide();
        }
    }

    /// Decide the oldest frame in the window, from the cheapest path through
    /// the whole window.
    fn decide(&mut self) {
        let Some(best) = self
            .metrics
            .iter()
            .enumerate()
            .filter(|(_, metric)| **metric != f64::MAX)
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(state, _)| state as u8)
        else {
            return;
        };
        // The state the oldest frame started in, by walking the window's own
        // back pointers.
        // `back[k][s]` is where the state after frame k came from, so the
        // oldest frame's decision needs the state it *started* in: every
        // pointer in the window, from the newest state back past the oldest.
        let mut state = best;
        for step in self.back.iter().rev() {
            state = step[usize::from(state)].0;
        }
        let stage = self.stages.pop_front().expect("a stage is held before one is decided");
        self.back.pop_front();
        let out = usize::from(self.code.output(state));
        let mut chosen = (f64::MAX, 0usize);
        for y in 0..16usize {
            if y & !self.code.inputs() as usize != 0 {
                continue;
            }
            let cost = stage.table[out * 16 + y];
            if cost < chosen.0 {
                chosen = (cost, y);
            }
        }
        let residues = stage.residues[out][chosen.1];
        let mut etas = [0i32; TRELLIS_FRAME];
        let classes = [residues.0, residues.1, residues.2, residues.3];
        for (k, &residue) in classes.iter().enumerate() {
            etas[k] = stage.bests[k].eta[residue.rem_euclid(4) as usize];
        }
        debug_assert_eq!(
            trellis::convert(label_of(etas[0], etas[1]), label_of(etas[2], etas[3])) & self.code.inputs(),
            chosen.1 as u8,
            "the indices decided on do not carry the inputs the path took"
        );
        self.decided += 1;
        trace_etas(self, stage.index, etas);
        // 8.7.1 and Figure 1: a data frame is three trellis frames, and the
        // first symbol of B1u begins one. A decision that lands in the middle
        // of one -- which is every decision until the window has passed -- has
        // no whole data frame's worth of indices and is dropped.
        if !self.labels.is_empty() || stage.index.is_multiple_of(3) {
            self.push_etas(etas);
        }
    }

    /// One frame's indices to Ki, and twelve symbols' worth to bits.
    fn push_etas(&mut self, etas: [i32; TRELLIS_FRAME]) {
        for (k, &eta) in etas.iter().enumerate() {
            let interval = self.labels.len();
            let modulus = self.moduli.modulus(interval);
            let mut ki = index_to_ki(eta, modulus, k);
            // The rotation in force at this position, as the difference from
            // the one `index_to_ki` has just applied: the measured one when
            // `V92_ROT` accepted it, `V92_K3`'s `rot=` table otherwise, and
            // nothing at all by default.
            //
            // Written as a difference so that the constant *cancels*: the class
            // that comes out is the observed one minus the rotation in force,
            // whatever `PAIR_FIRST_INDEX_OFFSET` is. That is why a measurement
            // that agrees with the constant changes no bit (Agere), why one
            // that disagrees is not fighting it (HSF), and why the constant can
            // be moved to a value of its own without moving a measured call.
            if let Some(want) = self.rotation[k].or(self.named_rotation[k]) {
                let applied = if opens_a_pair(k) { pair_first_index_offset() } else { 0 };
                if applied != want {
                    ki = (i64::from(ki) + applied - want).rem_euclid(i64::from(modulus.max(1))) as u8;
                }
            }
            self.labels.push(ki);
            if self.labels.len() == UP_INTERVALS {
                let labels: [u8; UP_INTERVALS] =
                    std::mem::take(&mut self.labels).try_into().expect("twelve intervals");
                let known = self.by_content && self.pushed < B1U_FRAMES;
                self.pushed += 1;
                if !known {
                    if let Ok(frame) = self.modulus.decode(&labels, &self.moduli, self.bits_per_frame) {
                        for bit in frame.bits() {
                            let out = self.descrambler.descramble(bit);
                            self.bits.push(out);
                        }
                    }
                } else {
                    self.push_frame_by_content(&labels);
                }
            }
        }
    }
}

impl Decoder {
    /// One data frame's bits, its polarity settled from 8.7.1's known content.
    ///
    /// 6.4.1's differential step is what tells a receiver which way up a frame
    /// is: d(f) = s(f) xor d(f - 1) at the far end, the reflection undone here.
    /// That works only while the far end's d is a function of what the far end
    /// sends. At 32 000 it is not -- the far end there reports the upper half
    /// on every frame, so its d is the frame counter and its reflection
    /// alternates -- and no chain read out of the payload can follow it. Which
    /// is a property of one far end and not of the clause: the clause's own
    /// answer is that both readings of a frame are legal and the data says
    /// which (`docs/74`).
    ///
    /// So both candidates are read, each descrambled on a copy of the state,
    /// and the one that comes out closer to B1u's 48 ones is the frame. No
    /// constant is fitted: the criterion is 8.7.1's content, it is applied to
    /// every frame at every rung above 24 000, and at 24 000 -- where the chain
    /// is the far end's -- it is not applied at all.
    fn push_frame_by_content(&mut self, labels: &[u8; UP_INTERVALS]) {
        let mut best: Option<(usize, Vec<bool>, Scrambler, FrameBits)> = None;
        for candidate in self.modulus.candidates(labels, &self.moduli, self.bits_per_frame) {
            let Ok(frame) = candidate else { continue };
            let mut probe = self.descrambler.clone();
            let bits: Vec<bool> = frame.bits().map(|bit| probe.descramble(bit)).collect();
            let ones = bits.iter().filter(|bit| **bit).count();
            if best.as_ref().is_none_or(|(most, ..)| ones > *most) {
                best = Some((ones, bits, probe, frame));
            }
        }
        if let Some((_, bits, state, frame)) = best {
            self.modulus.adopt(frame, &self.moduli);
            self.descrambler = state;
            self.bits.extend(bits);
        }
    }
}

/// The 32 costs of one frame: the best four indices for every (Y0, y).
///
/// The search is over residue classes -- four per symbol -- because a 4D subset
/// label depends only on 2 x eta mod 8, and because the fourth index's parity
/// is fixed by the first three and Y0 (6.4.2's class at k = 3).
fn build(bests: &[Bests; TRELLIS_FRAME], code: Code) -> Stage {
    let mut stage = Stage { table: [f64::MAX; 32], residues: [[(0, 0, 0, 0); 16]; 2], bests: *bests, index: 0 };
    for out in 0..2usize {
        for y in 0..16usize {
            if y & !code.inputs() as usize != 0 {
                continue;
            }
            let mut cost = f64::MAX;
            let mut chosen = (0i32, 0i32, 0i32, 0i32);
            for r0 in 0..4usize {
                if bests[0].cost[r0] == f64::MAX {
                    continue;
                }
                for r1 in 0..4usize {
                    if bests[1].cost[r1] == f64::MAX {
                        continue;
                    }
                    let first = label_of(r0 as i32, r1 as i32);
                    for r2 in 0..4usize {
                        if bests[2].cost[r2] == f64::MAX {
                            continue;
                        }
                        let parity = (r0 + r1 + r2 + out) % 2;
                        for r3 in (parity..4).step_by(2) {
                            if bests[3].cost[r3] == f64::MAX {
                                continue;
                            }
                            let second = label_of(r2 as i32, r3 as i32);
                            // The code reads only the bits it has: 16 states
                            // take Y1 and Y2, 32 add Y4 and 64 all four
                            // (9.6.3.2), so a frame whose converter spells a
                            // larger number is the same branch to a shorter
                            // code.
                            if trellis::convert(first, second) & code.inputs() != y as u8 {
                                continue;
                            }
                            let total =
                                bests[0].cost[r0] + bests[1].cost[r1] + bests[2].cost[r2] + bests[3].cost[r3];
                            if total < cost {
                                cost = total;
                                chosen = (r0 as i32, r1 as i32, r2 as i32, r3 as i32);
                            }
                        }
                    }
                }
            }
            stage.table[out * 16 + y] = cost;
            stage.residues[out][y] = chosen;
        }
    }
    stage
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::CONSTELLATION_FRAME;
    use crate::v92::precoder::Chain;
    use crate::v92::upstream::UpEncoder;
    use crate::v92::{Filters, Trellis};

    /// Parameters with one constellation set of `levels` positive points
    /// spaced by two, every interval on the modulus that holds its share of
    /// the frame, and a prefilter that passes x straight through.
    pub(super) fn parameters(drn: u8, trellis: Trellis) -> Parameters {
        Parameters {
            drn,
            trellis,
            extend_e2u: false,
            gain: 1.0,
            moduli: [1u8 << (up_bits(drn) / UP_INTERVALS as u32); UP_INTERVALS],
            filters: Filters { z2: vec![1.0], ..Filters::default() },
            sets: vec![(0..64).map(|index| (2 * index + 1) as u16).collect()],
            indices: [0; CONSTELLATION_FRAME],
        }
    }

    /// A deterministic bit source.
    pub(super) struct Random(pub u64);

    impl Random {
        pub(super) fn new(seed: u64) -> Self {
            Self(seed | 1)
        }

        pub(super) fn next(&mut self) -> bool {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 63 == 0
        }
    }

    /// 6.4.1 to 6.4.4 over a channel with nothing in it: the encoder's levels
    /// straight into the decoder, and every bit back, at all three codes and
    /// at both ends of the ladder. This is the whole claim of the package.
    #[test]
    fn every_bit_comes_back_over_a_perfect_channel() {
        for trellis in [Trellis::Sixteen, Trellis::ThirtyTwo, Trellis::SixtyFour] {
            for drn in [1u8, 19] {
                let parameters = parameters(drn, trellis);
                let mut encoder = UpEncoder::new(&parameters).expect("feasible");
                let mut decoder = Decoder::new(&parameters);
                let mut random = Random::new(0x5eed);
                let mut sent = Vec::new();
                let mut received = Vec::new();
                for _ in 0..(SURVIVOR_DEPTH * 6 * UP_INTERVALS) {
                    let symbol = encoder.next_symbol(|| {
                        let bit = random.next();
                        sent.push(bit);
                        bit
                    });
                    // "Over a perfect channel": the level the chain emitted,
                    // with nothing between it and the receiver.
                    decoder.feed(symbol.u);
                    received.extend(decoder.take_bits());
                }
                assert_eq!(decoder.drn(), drn, "{trellis:?}");
                assert_eq!(decoder.rate(), crate::v92::up_rate(drn), "{trellis:?}");
                assert!(!received.is_empty(), "{trellis:?} drn {drn}: nothing decoded");
                // The decoder's output starts SURVIVOR_DEPTH frames late, so
                // the two streams are compared from where they line up.
                let (skipped, errors) = align(&sent, &received);
                assert!(skipped.is_some(), "{trellis:?} drn {drn}: the streams do not line up");
                assert_eq!(errors, 0, "{trellis:?} drn {drn}: {errors} bits wrong");
            }
        }
    }

    /// Where the decoded stream starts relative to the sent one, and how many
    /// bits differ from there to the end of the shorter.
    fn align(sent: &[bool], received: &[bool]) -> (Option<usize>, usize) {
        let shortest = sent.len().min(received.len());
        let at = (0..shortest.min(300)).find(|&offset| {
            let length = shortest - offset;
            length > 64 && sent[offset..offset + length].iter().zip(&received[..length]).all(|(a, b)| a == b)
        });
        if at.is_none() {
            let mut best = (usize::MAX, 0usize);
            for offset in 0..shortest.min(400) {
                let length = shortest - offset;
                let errors = sent[offset..offset + length].iter().zip(&received[..length]).filter(|(a, b)| a != b).count();
                if errors < best.0 { best = (errors, offset); }
            }
            eprintln!("align: shortest {shortest} best {best:?} (errors, offset)");
        }
        match at {
            Some(offset) => {
                let length = shortest - offset;
                let errors = sent[offset..].iter().zip(&received[..length]).filter(|(a, b)| a != b).count();
                (Some(offset), errors)
            }
            None => (None, sent.iter().zip(received).filter(|(a, b)| a != b).count()),
        }
    }

    /// 6.4.1 and Figure 1: twelve symbols are one data frame of K bits, so the
    /// decoder hands bits over in whole frames and only once the window is
    /// past.
    #[test]
    fn bits_come_out_a_whole_data_frame_at_a_time_and_late() {
        let parameters = parameters(13, Trellis::Sixteen);
        let mut encoder = UpEncoder::new(&parameters).expect("feasible");
        let mut decoder = Decoder::new(&parameters);
        let bits = parameters.bits() as usize;
        let mut random = Random::new(9);
        let mut decoded = 0usize;
        let mut early = 0usize;
        for symbol in 0..(SURVIVOR_DEPTH * UP_INTERVALS) {
            let level = encoder.next_symbol(|| random.next()).u;
            decoder.feed(level);
            let got = decoder.take_bits().len();
            if got > 0 && early == 0 {
                early = symbol;
            }
            decoded += got;
        }
        assert_eq!(decoded % bits, 0, "{decoded} bits are not whole data frames");
        assert!(early >= SURVIVOR_DEPTH * TRELLIS_FRAME, "bits came out before the window was past");
        // 480 symbols are 120 trellis frames, forty of which the window holds,
        // so thirty data frames at most and a few short of that in practice.
        assert!(decoded >= bits * 20, "{decoded} bits from {} symbols", decoder.symbols());
        let _: fn(&Parameters) -> Result<Chain, &'static str> = Chain::new;
    }

    /// A line that is not carrying the constellation is the *receiver's*
    /// business (V92-22, which has the presence and error-rate watches a
    /// receiver has), and not this decoder's: every level it is given has a
    /// nearest point in every class, so a decoder on its own always answers.
    /// What it must not do is run off the end of its window or lose its place.
    #[test]
    fn a_dead_line_still_leaves_the_window_whole() {
        let parameters = parameters(13, Trellis::Sixteen);
        let mut decoder = Decoder::new(&parameters);
        for _ in 0..(SURVIVOR_DEPTH * 4 * UP_INTERVALS) {
            decoder.feed(0.0);
        }
        assert_eq!(decoder.symbols(), (SURVIVOR_DEPTH * 4 * UP_INTERVALS) as u64);
        assert!(decoder.stages.len() <= SURVIVOR_DEPTH);
        assert!(decoder.back.len() <= SURVIVOR_DEPTH);
        let _ = decoder.take_bits();
    }

    /// The rotation is *read off* B1u and not named.
    ///
    /// A synthetic far end sends B1u's model classes rotated by a table of its
    /// own -- including tables that are not `PAIR_FIRST_INDEX_OFFSET` -- and the
    /// measurement has to come back with that table, at every sample count the
    /// window has. Both readings of the model are sent, because the sequence is
    /// defined up to one global sign (`docs/46 §2`) and the measurement is what
    /// decides which of the two this line is on.
    ///
    /// This is the test that says the estimator is not a lookup for the one case
    /// that produced it: a table nothing has ever used has to come out of B1u's
    /// known content alone.
    #[test]
    fn a_rotation_that_is_not_the_constant_is_measured_off_b1u() {
        // drn 1 is K = 36, the rate an Agere SV92PP's B1u is read at, and the
        // one whose moduli are eight: the rotation is named in the class space.
        let parameters = parameters(1, Trellis::Sixteen);
        assert!(parameters.moduli.iter().all(|&modulus| modulus == 8), "eight classes");
        let bits = up_bits(parameters.drn);
        let classes = b1u_classes(bits, &parameters.moduli);
        assert_eq!(classes.len(), B1U_FRAMES * UP_INTERVALS, "B1u is 576 known classes");
        let set = Constellation::new(&parameters.sets[0]);
        for rotation in [[2, 0, 2, 0], [6, 6, 6, 7], [5, 3, 7, 1]] {
            for complemented in [false, true] {
                let mut measuring = Measuring::new(bits, parameters.moduli, 0).expect("eight classes");
                for n in 0..ROTATION_SYMBOLS as usize {
                    let k = n % TRELLIS_FRAME;
                    let model = usize::from(classes[n]);
                    let model = if complemented { CLASSES - 1 - model } else { model };
                    let class = (model + rotation[k] as usize) % CLASSES;
                    // Any member of the class will do: the class is what is
                    // measured and which member carries it is the free bit.
                    let eta = if k == TRELLIS_FRAME - 1 { 2 * class as i32 } else { class as i32 };
                    let level = set.level(eta).expect("every class has a member in the set");
                    let nearest = Bests::new(&set, level).nearest().expect("the set is not empty");
                    measuring.observe(parameters.moduli[n % UP_INTERVALS], nearest);
                }
                assert_eq!(
                    measuring.rotation(),
                    rotation.map(Some),
                    "rotation {rotation:?} with the model complemented = {complemented}"
                );
            }
        }
        // A line that is not carrying B1u: no position has a mode, so nothing
        // is taken and the decoder keeps the constant.
        let mut measuring = Measuring::new(bits, parameters.moduli, 0).expect("eight classes");
        let mut random = Random::new(11);
        for n in 0..ROTATION_SYMBOLS as usize {
            let class = 4 * usize::from(random.next()) + 2 * usize::from(random.next())
                + usize::from(random.next());
            let eta = if n % TRELLIS_FRAME == TRELLIS_FRAME - 1 { 2 * class as i32 } else { class as i32 };
            let level = set.level(eta).expect("every class has a member in the set");
            let nearest = Bests::new(&set, level).nearest().expect("the set is not empty");
            measuring.observe(parameters.moduli[n % UP_INTERVALS], nearest);
        }
        assert_eq!(measuring.rotation(), [None; TRELLIS_FRAME], "noise was taken for a rotation");
    }
}

