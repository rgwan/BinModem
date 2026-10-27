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
//!
//! **Two decoders, one switch.** [`TrellisMode`] and `V92_TRELLIS` choose how a
//! trellis frame's four indices are decided. The default, `slice`, decides each
//! of them from its own nearest point and is the one every B1u number in
//! `docs/96` is taken with. `viterbi` decides all four jointly over 9.6.3.2's
//! code, and is off until it has been validated: `docs/96` §25 deleted it
//! because the candidate set could not hold the answer, and that measurement
//! was made through the wrong 2D pair order (`docs/110`) -- with
//! [`inverse_map`](super::precoder::inverse_map) corrected the 4D constraint is
//! the one the far end obeys, so the decoder it deleted is worth another look.
//! Nothing on the bench changes until `V92_TRELLIS=viterbi` is set.

use std::collections::VecDeque;

use crate::v32::{Mode, Scrambler};
use crate::v34::trellis::{self, Code};

use super::codegrid;
use super::modulus::{self, FrameBits, Moduli12};
use super::precoder::{
    Constellation, code_for, index_to_ki, index_to_ki_class, inverse_map, label_of, opens_a_pair,
    pair_first_index_offset,
};
use super::{B1U_FRAMES, Parameters, TRELLIS_FRAME, UP_INTERVALS, UP_RATES, up_bits};

/// Trellis frames held before the oldest is decided on. Forty frames is a
/// hundred and sixty symbols, twenty milliseconds: the delay the rotation
/// measurement needs before a decision is taken (a decided frame cannot be
/// re-taken), and short enough that the tail a cleardown cuts off is nothing.
///
/// In `slice` it is not a survivor path memory: the decision is the slice's
/// own (`Decoder::fill`), so nothing is being waited for beyond the
/// measurement. In `viterbi` it is the survivor depth again -- forty frames is
/// twenty-four constraint lengths of the 16-state code, so a window's
/// traceback starts from a path that has converged.
pub const SURVIVOR_DEPTH: usize = 40;

/// Which decision a trellis frame's four indices get: `V92_TRELLIS`.
///
/// * [`Slice`](Self::Slice) -- **the default**. Each of the four indices is the
///   nearest point of its own symbol, in its own 6.4.2 class, and the 4D code
///   is not consulted (`Decoder::fill`).
/// * [`Viterbi`](Self::Viterbi) -- a maximum-likelihood path through
///   9.6.3.2's code over the whole survivor window (`Decoder::fill`,
///   [`build`]), with 6.4.3's parity at `k = 3` as the branch constraint and
///   [`inverse_map`] (with `docs/110`'s pair order) as the label chain.
///
/// Read once per [`Decoder`], so a caller that wants a different one has to ask
/// before the decoder is built; [`Decoder::with_trellis`] says so without the
/// environment. Unset, `slice`, or anything else is `Slice` -- a bench run
/// that does not set the variable is the run it was before this existed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrellisMode {
    Slice,
    Viterbi,
}

impl TrellisMode {
    /// `V92_TRELLIS=viterbi` asks for the joint decision; everything else,
    /// including nothing at all, leaves the slicing decoder in place.
    pub fn from_env() -> Self {
        match std::env::var("V92_TRELLIS").ok().as_deref() {
            Some("viterbi" | "VITERBI") => Self::Viterbi,
            _ => Self::Slice,
        }
    }
}

/// `V92_B1U_STATE`: which convolutional state the encoder is assumed to be in
/// at the first symbol this decoder is handed, for [`TrellisMode::Viterbi`].
///
/// `None` -- the default, and the maximum-likelihood answer to an unknown
/// start -- gives every state the same metric, so the survivor path is free to
/// begin anywhere. A number pins that one state instead, which is the sweep the
/// deleted decoder used (`docs/96` §25): zero state is right only if the symbol
/// handed over is B1u's first, and it is not -- `b1u_skip` drops eighteen data
/// frames first, so the far end's encoder has been running for 216 symbols by
/// then. Neither reading is a bench measurement; `docs/110` §5 measures the far
/// end's state at a *trace's* first frame and says what that does and does not
/// settle.
fn start_state(states: usize) -> Option<usize> {
    std::env::var("V92_B1U_STATE")
        .ok()
        .and_then(|text| text.parse::<usize>().ok())
        .filter(|state| *state < states)
}

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
/// **On by default since `/dev/shm/reports/e2u-boundary.md`; `V92_ROT=0` is the
/// off switch.** It used to be opt-in, on the argument that at 24 000 the
/// constant *is* this far end's and the measurement has nothing to add. The two
/// failing Agere captures of 2026-10-25 falsified that: at the same 24 000, the
/// same analogue modem, the same line and the same B1u, `/tmp/agr-v90-interop/
/// 152741-prime-b` measures `(3, 0, 3, 0)` -- the constant, 550 of 576 ones --
/// and `152653-prime-a` measures `(0, 0, 0, 0)`, where the constant reads 399
/// and the measurement reads **576 of 576**. So the pair-first offset is a
/// per-call property of the far end's index map at this rung too, and 8.7.1
/// makes it measurable on every call: it is not a number to assume.
pub fn rotation_wanted() -> bool {
    use std::sync::OnceLock;
    static WANTED: OnceLock<bool> = OnceLock::new();
    *WANTED.get_or_init(|| match std::env::var("V92_ROT") {
        Ok(text) => !(text.trim().is_empty() || text.trim() == "0"),
        Err(_) => true,
    })
}

/// How many symbols B1u is measured over: forty per trellis position, i.e.
/// a hundred and sixty symbols of B1u's five hundred and seventy-six.
///
/// Forty and not more because a decision cannot be re-taken: the first frame
/// is decided a hundred and sixty-four symbols in (`SURVIVOR_DEPTH` frames plus
/// one, from the first symbol this decoder is fed), so a measurement that ran
/// past that would decide the first frames on the constant and the rest on the
/// measurement -- two rotations in one B1u. Forty samples a position is what
/// fits before the first decision, and it is far more than the bar in
/// [`Measuring::rotation`] needs: `docs/46`'s plateaus are per-position
/// properties of the far end, not something that only shows up after ninety
/// symbols.
pub const ROTATION_SYMBOLS: u64 = 40 * TRELLIS_FRAME as u64;

/// The classes a rotation can name: the period of 6.4.2's class map, which is
/// the CPU's Mi and therefore the rung (`docs/46 §2`). Eight until `docs/74`:
/// a rate above 24 000 has Mi = 16, and the measurement used to refuse to run
/// there at all rather than name a class outside its histogram.
const CLASSES: usize = 16;

/// Which of the twelve intervals of B1u's group the symbol at trellis position
/// `k` of trellis frame `index` belongs to.
///
/// **The one source of truth for the interval assignment, and the invariant it
/// carries is that the interval is the symbol's own position in B1u** -- not a
/// count of anything the decoder has collected so far. `feed` slices and
/// measures with [`super::interval`] of `n`, and frame `index`'s four symbols
/// arrive at `n = 4 * index .. 4 * index + 3`, so this is the same number for
/// the frames the decision window hands back.
///
/// It is *not* `labels.len()`, and that was the defect (`docs/301` sec 1). A
/// frame the window declines pushes no labels, so a container's length falls
/// behind the symbol counter by four: from then on every symbol is named with
/// the wrong interval's `Mi` *and* the twelve labels of a group are stapled
/// together across the hole, which is a frame's worth of wrong bits plus an
/// offset that lasts the rest of B1u. The call's own evidence is exactly that
/// shape -- the winners' first 160 symbols carry the correct digits and the
/// call still reads 362/576 (`docs/299`, `docs/300`).
fn interval_at(index: u64, k: usize) -> usize {
    super::interval(index * TRELLIS_FRAME as u64 + k as u64)
}

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
    /// `half[sign][h]`: the k = 3 position measured on the **half-step grid**
    /// that the class map is read on, `h = (eta - 2 * predicted) mod 2 * Mi`.
    ///
    /// At k = 3 6.4.2's class is `((eta - (eta mod 2)) / 2) mod Mi`
    /// (`precoder.rs:430`), so one index step is *half* a class and the class
    /// histogram cannot name it: an odd index offset is split across the two
    /// classes that bracket it by the transmitted parity bit, which is what
    /// `docs/136` measured as `22/18` on `g4`. The index itself is the
    /// quantity the far end rotates, so this is the grid it is measured on;
    /// `measure` folds it back to classes with [`Self::folded`].
    half: [[u16; 2 * CLASSES]; 2],
    /// `radix[k]`: the widest `Mi` any sample at trellis position `k` was
    /// measured in, so the histogram for that position is cut to a width its
    /// samples can actually land in.
    ///
    /// The binning radix is the **interval's own** `Mi`, per sample
    /// (`index_to_ki_class`'s modulus, `precoder.rs:430`), which is what 6.4.2
    /// writes and what `feed` hands `observe`. A position can see two moduli at
    /// a mixed rung -- at Agere's `drn 2`, `Mi = 16` on intervals 0 and 1 and 8
    /// on the other ten -- and then the wider samples' bins are the narrower
    /// ones' free second member: a rotation the narrow intervals cannot name
    /// is two bins the bar refuses, not a `+-Mi/2` lobe.
    radix: [usize; TRELLIS_FRAME],
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
        (first < predicted.len()).then(|| Self {
            predicted,
            radix: [0; TRELLIS_FRAME],
            first,
            counts: [[[0; CLASSES]; TRELLIS_FRAME]; 2],
            half: [[0; 2 * CLASSES]; 2],
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
        // The radix this sample is measured in: the modulus of the interval it
        // came from, which is what 6.4.1's mixed radix and 6.4.2's class map
        // are written in (`precoder.rs:430`). Never `self.classes`: at a rung
        // whose moduli are not all equal that bins a `Mi = 8` interval's class
        // in radix 16 and splits its samples eight bins apart -- the `+-Mi/2`
        // secondary peak of `docs/270` sec 1.3, which `docs/272`/`docs/275`
        // found in the analysis tool (`b1d8223`) and which had this, the
        // production decoder, as its other instance (`docs/301` sec 2).
        let radix = usize::from(modulus.max(1));
        self.radix[k] = self.radix[k].max(radix);
        let complement = i64::from(modulus.max(1)) - 1 - i64::from(predicted);
        if k == TRELLIS_FRAME - 1 {
            // The half-step grid (see `Self::half`): one index step, not one
            // class, is what the far end rotates at this position, so this is
            // the count that can name an odd rotation at all. `2 * Mi` wide,
            // like the class it folds back to.
            let width = 2 * radix as i64;
            for (sign, predicted) in [i64::from(predicted), complement].into_iter().enumerate() {
                let away = (i64::from(eta) - 2 * predicted).rem_euclid(width);
                self.half[sign][away as usize] += 1;
            }
        } else {
            let class = i64::from(index_to_ki_class(eta, modulus, k));
            for (sign, predicted) in [i64::from(predicted), complement].into_iter().enumerate() {
                let away = (class - predicted).rem_euclid(radix as i64);
                self.counts[sign][k][away as usize] += 1;
            }
        }
        self.count_cost += started.elapsed();
    }

    /// Whether B1u has been measured for long enough.
    fn done(&self) -> bool {
        self.seen >= ROTATION_SYMBOLS || self.first + self.seen as usize >= self.predicted.len()
    }

    /// The k = 3 position, folded back to classes: for every candidate **index**
    /// offset `offset`, the half-step histogram is shifted by it and halved
    /// (`h -> h - offset -> floor(./2) mod Mi`, which is `index_to_ki_class` of
    /// `eta - offset` exactly), and the bar in [`Self::measure`] is applied to
    /// the folded class histogram -- the same bar, unchanged, on the right
    /// quantity.
    ///
    /// The candidate that clears with the most samples wins, and on a tie the
    /// smallest offset does, so the identity wins whenever the class grid
    /// already had a mode: a line whose rotation is an even number of steps
    /// reads exactly what it read before this, and only a line the class grid
    /// split (`docs/136`: 40 of 40 split `22/18`) moves. The offsets are not
    /// searched for a better-looking margin -- `docs/136 §3`'s control that
    /// `g4`'s `offset = 2` folds back to `22/18` is preserved by this rule,
    /// because `offset = 1` clears first.
    fn folded(&self, union: bool, sign: usize) -> Position {
        // The half-step grid is `2 * Mi` wide, and `Mi` here is the modulus the
        // k = 3 samples were measured in -- not the widest modulus on the rung.
        let radix = self.radix[TRELLIS_FRAME - 1].max(1);
        let width = 2 * radix;
        let half: Vec<u32> = (0..width)
            .map(|step| {
                if union {
                    u32::from(self.half[0][step]) + u32::from(self.half[1][step])
                } else {
                    u32::from(self.half[sign][step])
                }
            })
            .collect();
        let samples: u32 = half.iter().copied().sum();
        let mut best: Option<Position> = None;
        // The best mode *any* offset reached, clearing the bar or not. Without
        // it a position whose histogram has no mode prints the same `0/0` as
        // one nothing ever reached, and the line then reports a vote as a tie
        // when there was no vote at all: `docs/299` sec 2. A zero margin now
        // means exactly one thing -- no samples reached this position.
        let mut attempt: Option<(u32, u32)> = None;
        for offset in 0..width {
            let mut bins: Vec<u32> = (0..radix).map(|_| 0).collect();
            for (step, &count) in half.iter().enumerate() {
                let shifted = (step as i64 - offset as i64).rem_euclid(width as i64) as usize;
                bins[(shifted / 2) % radix] += count;
            }
            let mut ranked: Vec<(u32, usize)> =
                bins.iter().enumerate().map(|(class, &count)| (count, class)).collect();
            ranked.sort_unstable_by(|a, b| b.cmp(a));
            let top = ranked[0];
            let next = ranked.get(1).copied().unwrap_or((0, 0));
            let clear = top.0 > 0 && top.0 * 2 >= samples && top.0 >= next.0 * 2;
            if attempt.is_none_or(|(reached, _)| top.0 > reached) {
                attempt = Some((top.0, next.0));
            }
            if clear && best.as_ref().is_none_or(|&won| top.0 > won.best as u32) {
                best = Some(Position {
                    rotation: Some(top.1 as i64),
                    best: top.0.min(u32::from(u16::MAX)) as u16,
                    rival: next.0.min(u32::from(u16::MAX)) as u16,
                    no_mode: false,
                    shift: offset as i32,
                });
            }
        }
        best.unwrap_or_else(|| {
            // No verdict, but the evidence stays on the line.
            let (reached, next) = attempt.unwrap_or((0, 0));
            Position {
                rotation: None,
                best: reached.min(u32::from(u16::MAX)) as u16,
                rival: next.min(u32::from(u16::MAX)) as u16,
                no_mode: reached * 2 < samples,
                shift: 0,
            }
        })
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
    fn measure(&self) -> Measurement {
        // Which reading of the model this line is on: the one whose four
        // positions agree best. Ties go to the first, and a tie is not a
        // measurement anyway -- the per-position bar below is what decides
        // whether anything is taken from it.
        let agreement = |sign: usize| -> u32 {
            self.counts[sign]
                .iter()
                .enumerate()
                .map(|(k, counts)| {
                    // The k = 3 position is counted on the half-step grid; its
                    // agreement with the model is still the largest bin, which
                    // is the same number the class histogram gave when the
                    // rotation was even.
                    if k == TRELLIS_FRAME - 1 {
                        u32::from(self.half[sign].iter().copied().max().unwrap_or(0))
                    } else {
                        u32::from(counts.iter().copied().max().unwrap_or(0))
                    }
                })
                .sum()
        };
        let (first, second) = (agreement(0), agreement(1));
        let sign = usize::from(second > first);
        // One reading, when the line is on one of them: at 24 000 an Agere
        // reads 40 of 40 under the complemented model and about 14 of 40 under
        // the other, so the wrong one is a flat histogram and nothing but a
        // rival count. Mixing it in (the union below) does not move the mode
        // but it does inflate the runner-up, which is exactly the number the
        // margin is made of -- measured here: 40 against 0 under the reading
        // that read the line, 40 against 14 under the union.
        //
        // Both readings, when neither explains the line alone: at 32 000 the
        // far end reflects whole frames on a chain of its own, so a frame reads
        // against one reading or against the other and neither alone is a mode
        // (`docs/74`). The rotation is a property of the far end's index map,
        // which does not alternate, so it is the union that has to be sharp --
        // and it is: 90 of 90 classes a position at 32 000, against 36 and 54
        // taken separately.
        let union = (first as u64) * 2 <= (second as u64) * 3 && (second as u64) * 2 <= (first as u64) * 3
            || first.max(second) == 0;
        let mut positions = [Position::default(); TRELLIS_FRAME];
        for (k, counts) in self.counts[0].iter().enumerate() {
            if k == TRELLIS_FRAME - 1 {
                positions[k] = self.folded(union, sign);
                continue;
            }
            // As wide as the widest modulus this position's samples were
            // measured in, so that a `Mi = 8` interval is not scored in
            // radix 16 (`docs/301` sec 2).
            let histogram: Vec<u32> = (0..self.radix[k].max(1))
                .map(|class| {
                    if union {
                        u32::from(counts[class]) + u32::from(self.counts[1][k][class])
                    } else {
                        u32::from(self.counts[sign][k][class])
                    }
                })
                .collect();
            let samples: u32 = counts.iter().map(|&count| u32::from(count)).sum();
            let mut ranked: Vec<(u32, usize)> =
                histogram.iter().enumerate().map(|(class, &count)| (count, class)).collect();
            ranked.sort_unstable_by(|a, b| b.cmp(a));
            let best = ranked[0];
            let next = ranked.get(1).copied().unwrap_or((0, 0));
            let clear = best.0 > 0 && best.0 * 2 >= samples && best.0 >= next.0 * 2;
            positions[k] = Position {
                rotation: clear.then_some(best.1 as i64),
                best: best.0.min(u32::from(u16::MAX)) as u16,
                rival: next.0.min(u32::from(u16::MAX)) as u16,
                no_mode: false,
                shift: 0,
            };
        }
        Measurement { positions, samples: self.seen, first: self.first, union, reading: sign }
    }
}

/// One position's verdict, and the two counts it was taken on.
#[derive(Clone, Copy, Debug, Default)]
struct Position {
    /// The rotation this position reads, or `None` when the vote was not clear
    /// enough to be one and the constant is to be kept instead.
    rotation: Option<i64>,
    /// Samples that voted for [`Self::rotation`].
    best: u16,
    /// Samples that voted for the closest rival class.
    rival: u16,
    /// Whether this position came back with samples and **no mode at all** --
    /// no class holding even half of them, at any offset at k = 3 -- as
    /// opposed to two live hypotheses (a tie), which is evidence of a
    /// disagreement and is never overridden. Only `folded` sets it: the class
    /// histogram at k = 0, 1, 2 always names a best and a runner-up, and a
    /// position there that does not clear the bar has two candidates, not
    /// none.
    no_mode: bool,
    /// At k = 3 only: the **index** offset the histograms were folded by, in
    /// steps of 1, before the class map halved them. Zero at the other three
    /// positions, where a class offset is already the whole story and
    /// `PAIR_FIRST_INDEX_OFFSET` is applied by `index_to_ki` itself.
    ///
    /// This is the half of the fix `docs/136 §4.3` names: a rotation that is
    /// an odd number of index steps cannot be written as a `Ki` difference at
    /// all, so `push_etas` has to apply it *before* `index_to_ki_class` halves
    /// the index, and the class part (`Self::rotation`) after.
    shift: i32,
}

/// What one measurement of B1u decided, with the margin every position was
/// decided on.
#[derive(Clone, Debug)]
struct Measurement {
    positions: [Position; TRELLIS_FRAME],
    samples: u64,
    /// The model symbol the measurement started at, so that a log line can say
    /// which part of B1u it read.
    first: usize,
    /// Whether both readings of the model were needed (`true`) or one
    /// explained the line (`false`).
    union: bool,
    /// Which reading, when only one was used.
    reading: usize,
}

impl Measurement {
    /// The one rotation the positions that did carry evidence agree on, when
    /// at least three of the four did and they are the same number.
    ///
    /// This is what a position with no mode of its own falls back to, and the
    /// reason it is not the constant: k = 3's vote is the only one taken on
    /// the **half-step index grid** (`Self::folded`, and `Measuring::half`),
    /// whose class is a single level wide -- 6.4.2's class there is
    /// `((eta - p) / 2) mod Mi`, so its two members are the adjacent indices
    /// `2 Ki` and `2 Ki + 1`, where at k = 0, 1, 2 the members are eight
    /// levels apart. A level table that is close but not exact therefore reads
    /// k = 0, 1, 2 unanimously and cannot separate k = 3's pairs, which is
    /// `docs/299` sec 2's `40/0, 40/0, 40/0, 0/0` exactly. Three unanimous
    /// votes are a measurement of the far end's index map; the constant is a
    /// measurement of *another* far end, and keeping it in place of an
    /// absent vote is the thing `docs/287`'s `unpaired=0` and `docs/298`'s
    /// residual-as-fit have in common. So the fourth position takes the
    /// number the other three hold, and `line` marks it derived.
    fn unanimous(&self) -> Option<i64> {
        let measured: Vec<i64> = self.positions.iter().filter_map(|position| position.rotation).collect();
        if measured.len() < TRELLIS_FRAME - 1 || measured.iter().any(|&rotation| rotation != measured[0]) {
            return None;
        }
        measured.first().copied()
    }

    /// The rotation in force at each position: the measurement where it was
    /// clear enough, the other three's number where it was not and they agree,
    /// and nothing otherwise.
    fn table(&self) -> [Option<i64>; TRELLIS_FRAME] {
        let held = self.unanimous();
        self.positions.map(|position| match (position.rotation, position.no_mode, held) {
            (Some(rotation), _, _) => Some(rotation),
            // No vote of its own and no mode to vote on: the rotation the
            // other three measured is this position's too, because the ratio
            // is a property of the far end's index map and not of one trellis
            // position. A *tie* is not this case and keeps the constant.
            (None, true, Some(rotation)) => Some(rotation),
            (None, _, _) => None,
        })
    }

    /// The index offset in force at each position: the half-step part of the
    /// k = 3 rotation, zero everywhere a class offset is the whole story.
    fn shifts(&self) -> [i32; TRELLIS_FRAME] {
        self.positions.map(|position| position.shift)
    }

    /// Whether every position read clearly enough to be taken.
    fn confident(&self) -> bool {
        self.positions.iter().all(|position| position.rotation.is_some())
    }

    /// One line: what was measured, over which part of B1u, and how clearly.
    ///
    /// This is the line a 547 of 576 has to be diagnosable from. A reading that
    /// is nearly right is the state that is hardest to see: the frame phase is
    /// there (so the boundary is not the trouble) and the classes are almost
    /// there (so the rotation is nearly right), and outside this line nothing
    /// in the transcript says which of the two is short or by how much.
    fn line(&self) -> String {
        let held = self.unanimous();
        let table: Vec<String> = self
            .positions
            .iter()
            .map(|position| match (position.rotation, position.no_mode, held) {
                (Some(rotation), _, _) => rotation.to_string(),
                (None, true, Some(rotation)) => format!("{rotation}*"),
                (None, _, _) => "-".to_string(),
            })
            .collect();
        let margin: Vec<String> =
            self.positions.iter().map(|position| format!("{}/{}", position.best, position.rival)).collect();
        let verdict = if self.confident() {
            "confident"
        } else if self.positions.iter().any(|position| position.no_mode) && held.is_some() {
            // `*`: taken from the positions that did carry evidence, not from
            // the constant, and not a tie.
            "derived where no mode"
        } else {
            "ambiguous, constant kept where unclear"
        };
        format!(
            "rotation {} margin {} ({}) {} over {} symbols from B1u symbol {} ({})",
            table.join(","),
            margin.join(","),
            verdict,
            if self.union { "both readings" } else if self.reading == 1 { "complemented model" } else { "model as printed" },
            self.samples,
            self.first,
            if self.union { "union" } else { "single reading" }
        )
    }
}

/// `V92_ROT_TRACE`: what one measurement saw and what it accepted. Diagnostic,
/// written once per decoder that finished one; nothing in the data path reads
/// it.
fn trace_rotation(measuring: &Measuring, measurement: &Measurement) {
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
        let half: Vec<String> = measuring.half[sign].iter().map(u16::to_string).collect();
        let _ = writeln!(file, "sign {sign} k 3 half [{}]", half.join(" "));
        for (k, counts) in positions.iter().enumerate() {
            let row: Vec<String> = counts.iter().map(u16::to_string).collect();
            let accepted = u8::from(measurement.positions[k].rotation.is_some());
            let _ = writeln!(file, "sign {sign} k {k} [{}] accepted {}", row.join(" "), accepted);
        }
    }
    let _ = writeln!(file, "measure {}", measurement.line());
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

/// `V92_KI_TRACE`: every data frame's twelve Ki, in order, one frame a line --
/// the bit layer's own input, before the mixed radix, the differential sign
/// step and the descrambler touch it.
///
/// It exists because the two things a wrong B1u reading can be are not
/// distinguishable from the bits alone: a class map that is off at some
/// position gives the wrong Ki, and a correct Ki stream that comes out as the
/// wrong bits is a fault in 6.4.1 or in the descrambler's state. Writing the
/// Ki and the bits together is what separates them, and it is the only
/// instrument that says which of the two a 547 of 576 is.
fn trace_ki(labels: &[u8; UP_INTERVALS]) {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    let file = FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_KI_TRACE")?);
        Some(Arc::new(Mutex::new(std::fs::File::create(path).ok()?)))
    });
    let Some(file) = file else { return };
    let Ok(mut file) = file.lock() else { return };
    use std::io::Write;
    let text: Vec<String> = labels.iter().map(u8::to_string).collect();
    let _ = writeln!(file, "{}", text.join(" "));
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

/// One trellis frame held in the decoder's delay line: the four symbols' own
/// slice tables, the 4D branch metrics built from them, and which frame of B1u
/// it is.
#[derive(Debug, Clone, Copy)]
struct Stage {
    /// `table[out * 16 + y]`: the cost of the best four indices for that
    /// (Y0, y), or infinite when no indices make it. Only [`build`] fills this,
    /// and [`build`] only runs for [`TrellisMode::Viterbi`]; the slicing
    /// decoder leaves it infinite and never reads it.
    table: [f64; 32],
    /// Which residue class each of the four symbols was taken from, per
    /// (out, y), so that a decision names indices and not classes.
    residues: [[(i32, i32, i32, i32); 16]; 2],
    /// The per-symbol tables the indices are read off.
    bests: [Bests; TRELLIS_FRAME],
    /// Which trellis frame this is, counted from the first symbol of B1u.
    /// Three of them make a data frame (Figure 1), so only every third may
    /// start one -- a decision that begins part way through a data frame has
    /// nothing to give.
    index: u64,
}

impl Stage {
    /// A stage with no branch metrics: what [`TrellisMode::Slice`] keeps,
    /// since a decision taken from four nearest points reads none of them.
    fn sliced(bests: [Bests; TRELLIS_FRAME], index: u64) -> Self {
        Self { table: [f64::MAX; 32], residues: [[(0, 0, 0, 0); 16]; 2], bests, index }
    }
}

/// The upstream decoder: one equalised level a symbol in, bits out every
/// twelve.
#[derive(Debug, Clone)]
pub struct Decoder {
    moduli: Moduli12,
    code: Code,
    /// Which of the two decisions a frame gets; see [`TrellisMode`].
    mode: TrellisMode,
    /// Path metrics, one per convolutional state, live only in
    /// [`TrellisMode::Viterbi`].
    metrics: Vec<f64>,
    params: Parameters,
    descrambler: Scrambler,
    modulus: modulus::Decoder,
    bits_per_frame: u32,
    /// The symbol count since the first symbol of B1u (8.7.1).
    n: u64,
    frame: [Bests; TRELLIS_FRAME],
    filled: usize,
    stages: VecDeque<Stage>,
    /// `back[i][state]` is `(where the state at frame i + 1 came from, the
    /// inputs that took it there)`, so a traceback can name the branch as well
    /// as the state. Parallel to `stages`, and empty in
    /// [`TrellisMode::Slice`].
    back: VecDeque<Vec<(u8, u8)>>,
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
    /// The measurement's **index** offset at each position: zero except at
    /// k = 3, where `docs/136` measured that the far end's rotation is an odd
    /// number of index steps and a class offset cannot express it. Applied to
    /// the index before `index_to_ki` halves it (`push_etas`).
    shift: [i32; TRELLIS_FRAME],
    /// What the measurement decided, in one line, once it has run: the rotation
    /// per position, the count behind each of them against its best rival, and
    /// whether they were clear enough to be taken. `None` before the measurement
    /// finishes and on a decoder nobody asked to measure.
    ///
    /// It is kept rather than printed because the measurement happens inside
    /// the boundary gate's own decoders, thirteen of them, and what has to be
    /// logged is the one the gate believed -- not thirteen lines, one of which
    /// is right.
    rotation_line: Option<String>,
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
    /// Every decided frame's four indices, youngest last, while an offline
    /// instrument has asked for them ([`Self::keep_decisions`]). Empty by
    /// default: the bit stream is what the modem wants and the indices are what
    /// a *measurement* wants, and only one of the two belongs in a call.
    decisions: Vec<(u64, [i32; TRELLIS_FRAME])>,
    /// Whether the above is being filled.
    keeping: bool,
    /// The design's own constellation sets, kept un-scaled so that a decision
    /// table at one `k` can be rebuilt from the design rather than from another
    /// table (`docs/114` R4).
    design: Vec<Vec<u16>>,
    /// The scale `k` the decision table is built at, per `[8.7.3]`'s CPu mask
    /// ([`codegrid::estimate_from_mask`]) or the far end's own level. One --
    /// the design's own codes -- until [`Self::slice_at`] is told otherwise.
    scale: f64,
}

impl Decoder {
    /// A decoder for a far end sending these parameters, starting at the first
    /// symbol of B1u: every memory zero and n = 0, exactly as the encoder
    /// starts (8.7.1).
    pub fn new(parameters: &Parameters) -> Self {
        Self::with_trellis(parameters, TrellisMode::from_env())
    }

    /// The same decoder, with the trellis decision named instead of read off
    /// `V92_TRELLIS`. Offline instruments and tests use this so that both
    /// decisions can be measured in one process, where an environment variable
    /// cannot be one thing for one decoder and another for the next.
    pub fn with_trellis(parameters: &Parameters, mode: TrellisMode) -> Self {
        let code = code_for(parameters.trellis);
        // The maximum-likelihood start to an unknown state: every state equal.
        // `V92_B1U_STATE` pins one instead (a sweep, not a default).
        let mut metrics = vec![0.0f64; code.states()];
        if let Some(state) = start_state(code.states()) {
            metrics.iter_mut().for_each(|metric| *metric = f64::MAX);
            metrics[state] = 0.0;
        }
        Self {
            moduli: Moduli12::new(parameters.moduli),
            code,
            mode,
            metrics,
            params: parameters.clone(),
            descrambler: Scrambler::new(crate::v32::Mode::Answer),
            modulus: modulus::Decoder::new(),
            bits_per_frame: up_bits(parameters.drn),
            n: 0,
            frame: [Bests::default(); TRELLIS_FRAME],
            filled: 0,
            stages: VecDeque::new(),
            back: VecDeque::new(),
            labels: Vec::with_capacity(UP_INTERVALS),
            bits: Vec::new(),
            decided: 0,
            measuring: None,
            rotation: [None; TRELLIS_FRAME],
            shift: [0; TRELLIS_FRAME],
            rotation_line: None,
            by_content: false,
            pushed: 0,
            decisions: Vec::new(),
            keeping: false,
            design: parameters.sets.clone(),
            scale: 1.0,
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
        // The symbol's own interval, the same number `push_etas` names its Ki
        // with for this symbol ([`interval_at`], one source of truth).
        let interval = interval_at(self.n / TRELLIS_FRAME as u64, (self.n % TRELLIS_FRAME as u64) as usize);
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
                // Measured once and frozen: the data mode has no known payload
                // to re-measure against, and re-measuring after the first
                // decision would put two rotations in one B1u.
                let measurement = measuring.measure();
                trace_rotation(measuring, &measurement);
                self.rotation = measurement.table();
                self.shift = measurement.shifts();
                self.rotation_line = Some(measurement.line());
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

    /// Slice against the table the far end actually emits, `round(k a(eta))`,
    /// instead of the design's own `a(eta)` (`docs/114` R4).
    ///
    /// `k` is the scale between the design's codes and the levels on the wire:
    /// one when the far end transmits the design's own code values -- which is
    /// what a code-grid design asks for and what a code-transparent far end
    /// does -- and something else when the far end is at its own level, when
    /// `[8.7.3]`'s CPu mask is where `k` is measured ([`codegrid::estimate_from_mask`]).
    ///
    /// It is a scale on the *decisions* and not on the design: the levels rise
    /// with the index at any `k > 0`, so every distance moves with the
    /// thresholds and a `k` away from one costs spacing rather than adding a
    /// constant. What it is not is free: the design's gap is the finest thing
    /// the rung has, and a table left at one while the wire is at `k` decides
    /// the neighbouring code whenever `|k - 1| a(eta)` passes half a gap --
    /// which at LC = 64 is 8 LSB (`docs/253`).
    pub fn slice_at(&mut self, k: f64) {
        if !(k.is_finite() && k > 0.0) {
            return;
        }
        self.scale = k;
        self.params.sets = if (k - 1.0).abs() < f64::EPSILON {
            self.design.clone()
        } else {
            self.design.iter().map(|set| codegrid::scaled(set, k)).collect()
        };
    }

    /// The `k` the table is built at; one until [`Self::slice_at`] moves it.
    pub fn scale(&self) -> f64 {
        self.scale
    }

    /// Symbols fed since the first symbol of B1u.
    pub fn symbols(&self) -> u64 {
        self.n
    }

    /// Trellis frames decided on so far.
    pub fn decided(&self) -> u64 {
        self.decided
    }

    /// Which decision this decoder takes, and at which code's rate: what a
    /// bench log or an offline instrument needs to say which of the two it
    /// measured.
    pub fn trellis_mode(&self) -> TrellisMode {
        self.mode
    }

    /// Keep every decided frame's four indices, for [`Self::take_decisions`].
    ///
    /// **Off unless asked for, and it stays off**: this is the instrument an
    /// offline measurement of the two decisions needs (`examples/
    /// v92_trellis_gain.rs`), and it costs a `Vec` that grows for as long as
    /// nobody takes it. A call runs without it. `V92_ETA_TRACE` is the same
    /// data on disk, for one decoder at a time.
    pub fn keep_decisions(&mut self, on: bool) {
        self.keeping = on;
    }

    /// The frames decided since the last call, oldest first, each with the four
    /// indices the decision named.
    pub fn take_decisions(&mut self) -> Vec<[i32; TRELLIS_FRAME]> {
        std::mem::take(&mut self.decisions).into_iter().map(|(_, etas)| etas).collect()
    }

    /// The same frames, each with the trellis frame it is: `index` 0 is B1u's
    /// first four symbols (8.7.1). A measurement that has to line a decision up
    /// with what the far end sent needs the index and not the position in the
    /// list, because a frame the slice drops takes its place with it.
    pub fn take_decisions_indexed(&mut self) -> Vec<(u64, [i32; TRELLIS_FRAME])> {
        std::mem::take(&mut self.decisions)
    }

    /// The rate this decoder was built for, in bit/s.
    pub fn rate(&self) -> u32 {
        self.bits_per_frame * 8000 / UP_INTERVALS as u32
    }

    /// The rung of 6.1's ladder this decoder was built for.
    pub fn drn(&self) -> u8 {
        (1..=UP_RATES as u8).find(|&drn| up_bits(drn) == self.bits_per_frame).unwrap_or(0)
    }

    /// The twelve moduli 6.4.1 is being decoded with, interval 0 first.
    ///
    /// This is the decoder's own copy of the array, and the only reason it is
    /// public is the lock next to `Cpd::from`: `docs/270` sec 1.2's claim that
    /// these cannot drift from the declaration is a property of *construction*
    /// (one `Parameters`, two consumers), and a property of construction is
    /// worth one assertion that fails if a future reader ever hands the decoder
    /// a second source for them (`docs/272` sec 3).
    pub fn moduli(&self) -> [u8; UP_INTERVALS] {
        *self.moduli.values()
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
        // On unless `V92_ROT=0`, at every rung: above 24 000 because
        // `PAIR_FIRST_INDEX_OFFSET` is what an Agere's Mi = 8 line was measured
        // at and at Mi = 16 that constant is not the far end's (`docs/74`), and
        // at 24 000 because that constant was measured on *one* line of that
        // far end and the next call need not agree (see [`rotation_wanted`]:
        // prime-a measures `(0, 0, 0, 0)` where prime-b measures `(3, 0, 3, 0)`).
        // A call where the far end does agree leaves the measurement reporting
        // the constant and nothing moves.
        if !rotation_wanted() || self.measuring.is_some() {
            return;
        }
        self.measuring = Measuring::new(self.bits_per_frame, self.params.moduli, first);
    }

    /// What [`Self::measure_rotation_from`] decided, in one line, once it has
    /// finished. `None` before then, and on a decoder that was not asked.
    pub fn rotation_line(&self) -> Option<&str> {
        self.rotation_line.as_deref()
    }

    /// This frame's four slices, one decision step, and one frame of delay.
    ///
    /// **In `slice` the decision is the slice's own.** The 4D trellis of 6.4.3
    /// was tried as a hard constraint here and it cannot be one: the index
    /// sequence this far end sends does not lie on the code's path (a hundred
    /// and forty of B1u's hundred and forty-three trellis frames, at best, need
    /// a Y0 inversion to reach it -- see `docs/96` §25), so a joint decision
    /// over the code moves symbols off lattice points they sit 0.15 half-cells
    /// from and onto points 1.9 half-cells away. It is the same 6.4.2 class
    /// either way, and the class is all the bit layer needs: US6414989B1 col.
    /// 14, "the receiver need only be concerned with the coset received". The
    /// window stays because a decision cannot be re-taken -- the measurement
    /// above takes forty symbols a position, and emitting bits early would put
    /// two rotations in one B1u -- not because anything is being waited for.
    ///
    /// **In `viterbi` it is the cheapest path's**, and `docs/96` §25's reason
    /// for deleting it is gone: that measurement built its candidate set with
    /// the 2D pair's indices the wrong way round (`docs/110`), which is the
    /// same wrong constraint that made the far end's own stream look
    /// unreachable. With [`inverse_map`] corrected the branch constraint is the
    /// one the far end obeys.
    fn fill(&mut self) {
        let index = self.n / TRELLIS_FRAME as u64 - 1;
        match self.mode {
            TrellisMode::Slice => {
                let stage = Stage::sliced(self.frame, index);
                self.stages.push_back(stage);
            }
            TrellisMode::Viterbi => {
                if !self.step(index) {
                    return;
                }
                // The measurement has to be finished before the first frame is
                // decided, or one B1u would be read on two rotations. It is in
                // a hundred and sixty symbols and the first decision is at a
                // hundred and sixty-four, so nothing is ever held here.
                if self.measuring.is_some() {
                    return;
                }
            }
        }
        while self.stages.len() > SURVIVOR_DEPTH {
            self.decide();
        }
    }

    /// One trellis step of the maximum-likelihood path: this frame's four
    /// branch metrics, the metrics they leave every state with, and the back
    /// pointers that say where each one came from.
    ///
    /// Returns false when no state is reachable, which is the signal to hold
    /// what is left of the window rather than decide on noise.
    fn step(&mut self, index: u64) -> bool {
        let mut stage = build(&self.frame, self.code);
        stage.index = index;
        let states = self.code.states();
        let mut next = vec![f64::MAX; states];
        let mut from = vec![(0u8, 0u8); states];
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
                    from[to] = (state as u8, y as u8);
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
            return false;
        }
        for metric in &mut next {
            if *metric != f64::MAX {
                *metric -= floor;
            }
        }
        self.metrics = next;
        self.stages.push_back(stage);
        self.back.push_back(from);
        true
    }

    /// The oldest frame in the window, decided by the switch in [`TrellisMode`].
    fn decide(&mut self) {
        let decided = match self.mode {
            TrellisMode::Slice => self.decide_from_its_own_slice(),
            TrellisMode::Viterbi => self.decide_from_the_path(),
        };
        let Some((index, etas)) = decided else { return };
        self.decided += 1;
        if self.keeping {
            self.decisions.push((index, etas));
        }
        trace_etas(self, index, etas);
        // 8.7.1 and Figure 1: a data frame is three trellis frames, and the
        // first symbol of B1u begins one. Which of the twelve intervals this
        // frame's four symbols are is a function of the frame's own index
        // ([`interval_at`]), so the index goes in and the group decides for
        // itself whether it is filling or waiting: a decision that lands in
        // the middle of a data frame -- which is every decision until the
        // window has passed -- has no whole data frame's worth of intervals
        // and is dropped, and so is one that lands after a hole.
        self.push_etas(index, etas);
    }

    /// The oldest frame in the window, decided from its own four slices.
    fn decide_from_its_own_slice(&mut self) -> Option<(u64, [i32; TRELLIS_FRAME])> {
        let stage = self.stages.pop_front().expect("a stage is held before one is decided");
        let mut etas = [0i32; TRELLIS_FRAME];
        for (k, bests) in stage.bests.iter().enumerate() {
            let Some(eta) = bests.nearest() else {
                // Nothing in the constellation is anywhere near the line:
                // drop this frame rather than decide on noise, and keep the
                // delay line's own alignment.
                return None;
            };
            etas[k] = eta;
        }
        Some((stage.index, etas))
    }

    /// The oldest frame in the window, decided by tracing the cheapest path
    /// through the whole window back from the best final state.
    ///
    /// The traceback follows the survivor's **own** branch, not the cheapest
    /// branch at the state it passes through: `docs/96` §25 measured both and
    /// they differ (532 against 545 of 576), because the two are the same only
    /// where the window's best path is the best path to that state. Each back
    /// pointer therefore carries the inputs as well as the predecessor, and the
    /// frame's indices are read off the branch metric table for the state the
    /// branch left.
    fn decide_from_the_path(&mut self) -> Option<(u64, [i32; TRELLIS_FRAME])> {
        let state = self
            .metrics
            .iter()
            .enumerate()
            .filter(|(_, metric)| **metric != f64::MAX)
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(state, _)| state as u8)?;
        let stage = self.stages.pop_front().expect("a stage is held before one is decided");
        // The oldest frame's own pointers, kept aside: the walk below ends on
        // the state the oldest frame *left* in, and this is what it did.
        let oldest = self.back.pop_front().expect("a stage has a back pointer in the viterbi mode");
        // Walking every pointer in what is left of the window, newest first,
        // lands on the state the oldest frame left in -- which is where the
        // oldest frame's own pointer is read.
        let mut state = state;
        for step in self.back.iter().rev() {
            state = step[usize::from(state)].0;
        }
        let (from, y) = oldest[usize::from(state)];
        // 6.4.2's k = 3 parity is over indices and not over labels, and the
        // branch's indices are the cheapest ones for its (Y0, y): Y0 is the
        // output of the state the branch left.
        let out = usize::from(self.code.output(from));
        let (r0, r1, r2, r3) = stage.residues[out][usize::from(y)];
        let mut etas = [0i32; TRELLIS_FRAME];
        for (k, residue) in [r0, r1, r2, r3].iter().enumerate() {
            etas[k] = stage.bests[k].eta[residue.rem_euclid(4) as usize];
        }
        debug_assert_eq!(
            inverse_map(etas) & self.code.inputs(),
            y & self.code.inputs(),
            "the indices decided on do not carry the inputs the path took"
        );
        Some((stage.index, etas))
    }

    /// One frame's indices to Ki, and twelve symbols' worth to bits.
    ///
    /// `index` is the frame's own position in B1u's trellis numbering, which is
    /// what names its four symbols' intervals ([`interval_at`]).
    fn push_etas(&mut self, index: u64, etas: [i32; TRELLIS_FRAME]) {
        // The group this frame continues, from the symbol position and not
        // from `labels.len()` (see [`interval_at`]). A frame that does not
        // follow the previous one -- the window declined it, or the window was
        // restarted rather than decide on noise -- leaves the group short, and
        // a data frame is twelve *contiguous* symbols: 8.7.1's three trellis
        // frames, 6.4.1's mixed radix over the twelve Ki. A group stapled
        // across a hole is a frame of wrong bits and an interval offset for
        // the rest of B1u, so the partial group is dropped and the next
        // boundary starts a new one.
        let interval = interval_at(index, 0);
        if interval != self.labels.len() {
            self.labels.clear();
            if interval != 0 {
                // Not a boundary: no group can start here. Wait for one.
                return;
            }
        }
        for (k, &eta) in etas.iter().enumerate() {
            let modulus = self.moduli.modulus(interval_at(index, k));
            // The index offset the measurement folded this position by, taken
            // off the index **before** `index_to_ki` applies the class map.
            // That is the whole of the k = 3 half-step fix: at k = 3 the map
            // halves `eta` (`precoder.rs:430`), so a rotation of an odd number
            // of index steps is not a class difference at all and the `Ki`
            // correction below cannot express it -- it would have to round
            // half a step, which is the `22/18` split `docs/136` measured.
            // Removing the offset from the index first makes the class map
            // name the right class whether the offset is even or odd, and the
            // class part (`want`, below) then cancels what is left.
            //
            // Cost: one integer subtraction per symbol (`shift[k]` is zero at
            // the other three positions, so they are unchanged by an identity
            // subtract the CPU does not branch on), and no new branch: the
            // `shift` array is a load of the same hot structure `rotation` and
            // `named_rotation` are already loaded from.
            let mut ki = index_to_ki(eta - self.shift[k], modulus, k);
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
                trace_ki(&labels);
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
///
/// **The label chain is the one `docs/110` found**, and that is the whole
/// difference between this table and the one `docs/96` §25 measured as
/// hopeless: 6.4.3's pair has its real part *second*, so `label_of` is fed
/// `(eta of the second symbol, eta of the first)`. Read the other way round,
/// the far end's own index stream is on no path of any 9.6.3.2 code at any
/// initial state; read this way it is on the 16-state code's path in 218 900 of
/// 218 900 frames (`docs/110` §3).
fn build(bests: &[Bests; TRELLIS_FRAME], code: Code) -> Stage {
    let mut stage = Stage {
        table: [f64::MAX; 32],
        residues: [[(0, 0, 0, 0); 16]; 2],
        bests: *bests,
        index: 0,
    };
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
                    let first = label_of(r1 as i32, r0 as i32);
                    for r2 in 0..4usize {
                        if bests[2].cost[r2] == f64::MAX {
                            continue;
                        }
                        let parity = (r0 + r1 + r2 + out) % 2;
                        for r3 in (parity..4).step_by(2) {
                            if bests[3].cost[r3] == f64::MAX {
                                continue;
                            }
                            let second = label_of(r3 as i32, r2 as i32);
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
    use crate::v92::precoder::{Chain, Class};
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
            levelset: crate::v92::LevelSet::Uniform,
        law: None, }
    }

    /// **Is 6.4.2's `k = 3` class taken against the declared points?**
    ///
    /// `[quoted]` `v92.txt:452-459`: "Let the N constellation points be denoted
    /// by `a(eta)`, `-N/2 <= eta < N/2`, where the indices are in the same order
    /// as the levels ... E(Ki) = {a(eta_k) | eta_k = 2 Ki + 2 z_k Mi + (eta_0 +
    /// eta_1 + eta_2 + Y0) mod 2, z_k an integer} for k = 3."
    ///
    /// The class is a **congruence on the index**: `eta -> ((eta - eta mod 2) /
    /// 2) mod Mi`, which is [`index_to_ki_class`] and is what the decoder runs
    /// (`decoder.rs`, `index_to_ki` at the Ki of every decided frame). The level
    /// geometry enters only through `a(eta)`, so the *boundary* between two
    /// adjacent declared points is wherever the declaration puts them -- their
    /// midpoint -- and a boundary table stepped out of `min_gap` is not it,
    /// because at `drn 1` the declared gaps are `256 ... 352` LSB, widest over
    /// narrowest `1.375`.
    ///
    /// The uniform arm is the control: there the declared gaps *are* one number,
    /// so its midpoints stay equally spaced and its behaviour is unchanged.
    ///
    /// The set is the `drn 1` code arm's declaration as `docs/314` sec 1
    /// measured it; every expectation below is computed from that set and from
    /// the clause, never tabulated.
    #[test]
    fn the_class_at_k3_is_taken_against_the_declared_points() {
        // `drn 1` is 36 bits over 12 intervals, 3 bits each, so every Mi is 8
        // and the `k = 3` class steps by 2 x Mi = 16 = N: one member a class.
        let modulus = 1u8 << (up_bits(1) / UP_INTERVALS as u32);
        assert_eq!(modulus, 8);

        // CPd's LC ascending "linear value" words, `docs/314` sec 1.
        let declared: [u16; 8] = [324, 652, 988, 1308, 1628, 1980, 2236, 2492];
        // The uniform arm as it ships: 6.4.2's odd lattice a(eta) = 2 eta + 1.
        let uniform: Vec<u16> = (0..8).map(|i| (2 * i + 1) as u16).collect();

        let decide = |set: &[u16], level: f64| -> i32 {
            Bests::new(&Constellation::new(set), level)
                .nearest()
                .expect("a declared point is nearest to every level")
        };
        let class = |index: i32| index_to_ki_class(index, modulus, TRELLIS_FRAME - 1);

        // The gaps of the declaration are not one number, which is the whole
        // reason a lattice-stepped boundary cannot be the declared one.
        let mut gaps: Vec<u16> = declared.windows(2).map(|w| w[1] - w[0]).collect();
        gaps.sort_unstable();
        assert_eq!((gaps[0], gaps[gaps.len() - 1]), (256, 352));

        // **The property, on both arms, computed from the declaration.**
        for set in [&declared[..], uniform.as_slice()] {
            let points: Vec<f64> = set.iter().map(|&p| f64::from(p)).collect();
            for i in 0..points.len() - 1 {
                let mid = 0.5 * (points[i] + points[i + 1]);
                let below = decide(set, mid - 1e-6);
                let above = decide(set, mid + 1e-6);
                // Below the midpoint the decision is index `i`, above it `i+1`:
                // the boundary is where the declaration puts it.
                assert_eq!(below, i as i32, "{set:?}: just below {mid}");
                assert_eq!(above, i as i32 + 1, "{set:?}: just above {mid}");
                // 6.4.2 at k = 3 pairs a(i) with a(i+1) inside one class, so
                // the class changes at the declared midpoints whose lower index
                // is odd and at no other -- the positions still come from the
                // declaration.
                assert_eq!(class(below) != class(above), i % 2 == 1, "{set:?} i {i}");
            }
        }

        // **Not a lattice.** `min_gap` is 256 here; a table stepped out of it
        // lands between two declared points (`324 + 256 = 580`), where the
        // decision does not change, and misses 820, which does.
        assert_eq!(codegrid::min_gap(&declared), 256.0);
        assert_eq!(decide(&declared, 580.0 - 1e-6), 1);
        assert_eq!(decide(&declared, 580.0 + 1e-6), 1);
        assert_eq!(decide(&declared, 820.0 - 1e-6), 1);
        assert_eq!(decide(&declared, 820.0 + 1e-6), 2);

        // The control: one declared gap, equally spaced boundaries, the arm's
        // behaviour exactly as it was.
        let uniform_gap = codegrid::min_gap(&uniform);
        assert_eq!(uniform_gap, 2.0);
        let boundaries: Vec<f64> = uniform
            .windows(2)
            .map(|w| 0.5 * (f64::from(w[0]) + f64::from(w[1])))
            .collect();
        for (i, boundary) in boundaries.iter().enumerate() {
            assert_eq!(decide(&uniform, boundary - 1e-6), i as i32);
            assert_eq!(decide(&uniform, boundary + 1e-6), i as i32 + 1);
        }
        let steps: Vec<f64> = boundaries.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(steps.iter().all(|&step| step == uniform_gap), "{steps:?}");

        // The zero-margin fact, counted against the declared alphabet rather
        // than assumed: `2 x Mi = 16 = N` at `drn 1`, so a `k = 3` class holds
        // exactly one index per trellis parity and that index reads back as its
        // own Ki. One member is no slack, and it is legal (`N >= 2 Mi`).
        for ki in 0..=7u8 {
            for parity in [false, true] {
                let class_span = Class::for_interval(ki, modulus, TRELLIS_FRAME - 1, parity);
                let members: Vec<i32> = (-8..=7).filter(|&eta| class_span.holds(eta)).collect();
                assert_eq!(members.len(), 1, "Ki {ki} parity {parity}");
                assert_eq!(class(members[0]), ki, "Ki {ki} parity {parity}");
            }
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

    /// The same round trip with [`TrellisMode::Viterbi`] deciding the frames.
    ///
    /// Over a channel with nothing in it the two decisions have to agree
    /// everywhere, and both have to come back with every bit: the branch
    /// constraint 6.4.3's parity puts on a frame is one our own transmitter
    /// satisfies by construction, so the joint decision has nothing to move.
    /// This is the self-loop `docs/110` §7 asks for before the Viterbi is
    /// trusted with anything: it says the two ends of *our* chain still agree
    /// after the pair order changed under them.
    ///
    /// It is not evidence about the far end, and says so: both ends run the
    /// same tables, so a convention error made once passes here. `docs/110` is
    /// the far-end evidence.
    #[test]
    fn the_viterbi_and_the_slice_agree_over_a_channel_with_nothing_in_it() {
        for trellis in [Trellis::Sixteen, Trellis::ThirtyTwo, Trellis::SixtyFour] {
            let parameters = parameters(1, trellis);
            let mut encoder = UpEncoder::new(&parameters).expect("feasible");
            let mut sliced = Decoder::with_trellis(&parameters, TrellisMode::Slice);
            let mut viterbi = Decoder::with_trellis(&parameters, TrellisMode::Viterbi);
            let mut random = Random::new(0x5eed);
            let mut sent = Vec::new();
            let mut from_slice = Vec::new();
            let mut from_viterbi = Vec::new();
            for _ in 0..(SURVIVOR_DEPTH * 6 * UP_INTERVALS) {
                let symbol = encoder.next_symbol(|| {
                    let bit = random.next();
                    sent.push(bit);
                    bit
                });
                sliced.feed(symbol.u);
                viterbi.feed(symbol.u);
                from_slice.extend(sliced.take_bits());
                from_viterbi.extend(viterbi.take_bits());
            }
            assert_eq!(viterbi.trellis_mode(), TrellisMode::Viterbi);
            assert_eq!(sliced.decided(), viterbi.decided(), "{trellis:?}: the two decided a different number of frames");
            assert_eq!(from_slice, from_viterbi, "{trellis:?}: the two decisions differ over a perfect channel");
            let (skipped, errors) = align(&sent, &from_viterbi);
            assert!(skipped.is_some(), "{trellis:?}: the streams do not line up");
            assert_eq!(errors, 0, "{trellis:?}: {errors} bits wrong through the viterbi");
        }
    }

    /// The decision table has to be built at the far end's `k`, and the rung
    /// that proves it is the finest one (docs/253).
    ///
    /// A far end whose levels are `k` times the design's codes cannot be sliced
    /// against the design's own codes: the levels rise with the index at any
    /// `k > 0`, so the nearest design point to `k a(eta)` is the neighbour as
    /// soon as `|k - 1| a(eta)` passes half a gap. At LC = 64 that is 8 LSB, so
    /// two per cent of scale cost codes across the whole upper half of the set.
    /// [`Decoder::slice_at`] moves the table with the levels and the same stream
    /// comes back whole, which is `docs/114` R4 in one assertion and the reason
    /// `codegrid::scaled` is on the decision path.
    #[test]
    fn a_stream_at_k_needs_the_table_at_k() {
        let mut parameters = parameters(19, Trellis::Sixteen);
        parameters.sets = vec![codegrid::grid(crate::v90::ucode::Law::Mu, 64).expect("a 64-point code set")];
        parameters.moduli = [64u8; UP_INTERVALS];
        parameters.levelset = crate::v92::LevelSet::Code;
        let point = |eta: i32| -> f64 {
            let set = &parameters.sets[0];
            let index = if eta >= 0 { eta as usize } else { (-eta - 1) as usize };
            let value = f64::from(set[index]);
            if eta >= 0 { value } else { -value }
        };
        let mut encoder = UpEncoder::new(&parameters).expect("feasible");
        let sent = encoder.b1u();
        let k = 1.02;
        let wrong = |decoder: &mut Decoder| -> usize {
            decoder.keep_decisions(true);
            let mut wrong = 0;
            for symbol in &sent {
                decoder.feed(symbol.u * k);
                for (index, etas) in decoder.take_decisions_indexed() {
                    for (position, &eta) in etas.iter().enumerate() {
                        let at = index as usize * TRELLIS_FRAME + position;
                        if at < sent.len() && eta != sent[at].eta {
                            wrong += 1;
                        }
                    }
                    let _ = point;
                }
            }
            wrong
        };
        // Where the threshold is, measured on this set and this stream: 1.01
        // costs nothing and 1.02 costs 7.9 % of the codes, because the first
        // point the shift carries past half its own gap is the 428 LSB one
        // (gap 32) and `0.02 x 428 = 8.6` is past 16. The 64-point grid's
        // finest steps are its lowest ones -- 24, 24, 24, 32 LSB at 24 ... 148
        // LSB -- so a scale error bites at small `a` first and everywhere
        // above it after that.
        let mut near_one = Decoder::with_trellis(&parameters, TrellisMode::Slice);
        near_one.slice_at(1.01);
        assert_eq!(wrong(&mut near_one), 0, "1 % of scale is inside this set's gaps");
        let mut at_one = Decoder::with_trellis(&parameters, TrellisMode::Slice);
        assert_eq!(at_one.scale(), 1.0);
        let design = wrong(&mut at_one);
        assert_eq!(design, 33, "a stream at 1.02 must not slice right at k = 1: 33 of 416");
        let mut at_k = Decoder::with_trellis(&parameters, TrellisMode::Slice);
        at_k.slice_at(k);
        assert!((at_k.scale() - k).abs() < 1e-12);
        assert_eq!(wrong(&mut at_k), 0, "the table at k must slice its own stream");
        // ... and back, so that a scale is a property of the table and not a
        // history: the same decoder told k = 1 again is the design's decoder.
        at_k.slice_at(1.0);
        assert_eq!(at_k.params.sets, parameters.sets, "k = 1 is the design itself");
    }

    /// `V92_ROT`'s measurement, pinned on lines that have no capture in them.
    ///
    /// Three shapes, which are the three a real call can have:
    ///
    /// * a rotation that happens to be the constant, `3, 0, 3, 0` -- the Agere's
    ///   `prime-b`, where the measurement agrees with `PAIR_FIRST_INDEX_OFFSET`
    ///   and therefore moves nothing;
    /// * a rotation that is not, `0, 0, 0, 0` -- the same far end on the same
    ///   bench one call later (`prime-a`), where the constant reads 399 of 576
    ///   and this reads every bit;
    /// * a line that does not hold still, where the mode is a tie. That is the
    ///   case the old bar was one comparison short of: "at least half the
    ///   samples" alone accepts a 20/20 split, and a rotation taken off a tie
    ///   is a wrong rotation, which is about half the bits and reads as noise
    ///   rather than as a fault.
    ///
    /// The line is built here rather than captured: 8.7.1's data frames of ones
    /// through the encoder this crate sends with, and the far end's index map
    /// applied to the level on the way out -- a rotation of `r` classes is an
    /// index shift of `r` at k = 0, 1, 2 and of `2r` at k = 3, where 6.4.2's
    /// class steps by `2 Mi`. Nothing else about the far end is modelled.
    #[test]
    fn the_rotation_is_measured_off_b1u_and_its_margin_is_reported() {
        for table in [[3i64, 0, 3, 0], [0, 0, 0, 0]] {
            let (decoded, line) = b1u_over_a_rotated_line(|k, _frame| table[k]);
            let ones = decoded.iter().filter(|bit| **bit).count();
            assert!(ones > 0, "{table:?}: nothing decoded");
            assert_eq!(ones, decoded.len(), "{table:?}: {} bits of {} read as ones\n{line}", decoded.len() - ones, decoded.len());
            assert!(
                line.contains(&format!("rotation {}", table.map(|r| r.to_string()).join(","))),
                "{table:?} was not the measured rotation: {line}"
            );
            assert!(line.contains("(confident)"), "{table:?}: {line}");
            // The margin is the ones count of the chosen hypothesis against the
            // best rival, per position: forty samples a position, one rotation
            // applied to all of them, so forty to nothing every time. A bar
            // without the runner-up half could not tell this from a tie.
            assert!(line.contains("margin 40/0,40/0,40/0,40/0"), "{table:?}: {line}");
        }
        // Half the frames at one rotation and half at another, at k = 0 only:
        // the other three positions are still unanimous, so the line has to say
        // which position it would not take rather than throw the call away.
        let (_, line) = b1u_over_a_rotated_line(|k, frame| if k != 0 { 0 } else if frame % 2 == 1 { 0 } else { 3 });
        assert!(line.contains("ambiguous"), "{line}");
        assert!(line.contains("rotation -,0,0,0"), "the tie was not the one reported: {line}");
        // The three positions that did hold still are still taken: an ambiguous
        // position costs the call that position's rotation and not the three
        // that were clear.
        assert!(line.contains("margin 21/19,40/0,40/0,40/0"), "{line}");
    }

    /// `docs/299` sec 2 and sec 4. The fourth position's vote is the only one
    /// taken on the half-step **index** grid, whose class is one level wide at
    /// k = 3 (`((eta - p) / 2) mod Mi`, so its members are the adjacent indices
    /// `2 Ki` and `2 Ki + 1`) where k = 0, 1, 2's members are eight levels
    /// apart. A level table that is close but not exact reads the first three
    /// unanimously and cannot separate the fourth's pairs, so the fourth comes
    /// back with samples and no mode -- the call's own
    /// `40/0,40/0,40/0,0/0`, one ticket after the fit reached 33.7 dB.
    ///
    /// Two things are asserted, and the first is the one that was wrong: a
    /// position with samples and no mode must not print the same margin as one
    /// nothing reached (`0/0` is an absence of evidence and never a tie), and
    /// the rotation it cannot measure it takes from the three that can, marked
    /// `*`, instead of keeping the constant -- which is a measurement of some
    /// other far end.
    #[test]
    fn a_k3_position_with_no_mode_takes_the_rotation_the_other_three_hold() {
        let parameters = parameters(1, Trellis::Sixteen);
        let mut measuring = Measuring::new(up_bits(parameters.drn), parameters.moduli, 0)
            .expect("a measurement at drn 1");
        let predicted = measuring.predicted.clone();
        for index in 0..ROTATION_SYMBOLS as usize {
            let k = index % TRELLIS_FRAME;
            let model = i64::from(predicted[index]);
            let frame = (index / TRELLIS_FRAME) as i64;
            // k = 0, 1, 2: the far end's class map is `model + 4`, one index
            // step, so every sample votes the same.
            // k = 3: the indices are drawn from the class grid one frame at a
            // time, which is a far end the half-step fold cannot name.
            let eta = if k == TRELLIS_FRAME - 1 {
                (2 * ((model + frame) % 8) + frame % 2) as i32
            } else {
                ((model + 4) % 8) as i32
            };
            measuring.observe(parameters.moduli[index % UP_INTERVALS], eta);
        }
        assert!(measuring.done(), "the measurement did not fill");
        let measurement = measuring.measure();
        let line = measurement.line();
        assert!(
            measurement.positions[..TRELLIS_FRAME - 1].iter().all(|position| position.rotation == Some(4)),
            "the three class-grid positions did not read the rotation: {line}"
        );
        assert_eq!(measurement.positions[TRELLIS_FRAME - 1].rotation, None, "{line}");
        assert!(
            measurement.positions[TRELLIS_FRAME - 1].best > 0,
            "the absence of a vote was printed as no evidence at all: {line}"
        );
        assert!(line.contains("rotation 4,4,4,4*"), "the fourth vote was not derived: {line}");
        assert!(line.contains("derived where no mode"), "{line}");
        assert!(!line.contains("margin 40/0,40/0,40/0,0/0"), "{line}");
    }

    /// B1u over a synthetic far end whose class map is rotated per trellis
    /// position: what the decoder read, and what it measured.
    ///
    /// `rotation(k, frame)` is that far end's rotation at trellis position `k`
    /// of data frame `frame`, in classes. The line is built by stepping this
    /// crate's own chain with `Ki + rotation` rather than by moving levels
    /// about, and no other way would do: the rotation is part of the far end's
    /// `Ki -> index` map, the trellis is applied to the indices it produces, so
    /// a line whose levels were rotated after the encoder ran would be a line
    /// no conforming far end can send -- and one the receiver is right to
    /// deviate on.
    fn b1u_over_a_rotated_line(rotation: impl Fn(usize, usize) -> i64) -> (Vec<bool>, String) {
        let parameters = parameters(1, Trellis::Sixteen);
        let bits = up_bits(parameters.drn);
        let classes = b1u_classes(bits, &parameters.moduli);
        let mut chain = Chain::new(&parameters).expect("the chain these parameters describe");
        let mut decoder = Decoder::new(&parameters);
        decoder.measure_rotation_from(0);
        // B1u, and the survivor window's worth over, so that the last frame the
        // decoder can decide is still one of B1u's.
        let fed = B1U_FRAMES * UP_INTERVALS + TRELLIS_FRAME * (SURVIVOR_DEPTH + 2);
        let mut decoded = Vec::new();
        for n in 0..fed {
            let k = n % TRELLIS_FRAME;
            let want = rotation(k, n / UP_INTERVALS);
            // The chain this crate sends with already carries the constant at
            // k = 0 and 2, and that constant is not part of the far end being
            // synthesised here: it is what the *measurement* is compared
            // against. So the class handed to the chain is the model's plus the
            // far end's rotation minus the constant the chain will add back.
            let constant = if opens_a_pair(k) { pair_first_index_offset() } else { 0 };
            let ki = (i64::from(classes[n % classes.len()]) + want - constant).rem_euclid(
                i64::from(*parameters.moduli.iter().max().expect("twelve moduli")),
            ) as u8;
            let symbol = chain.step(ki);
            decoder.feed(symbol.u);
            decoded.extend(decoder.take_bits());
        }
        // Keep only the bits that are B1u's: the decoder decides its first
        // frame a survivor window in, so the last `SURVIVOR_DEPTH + 1` frames
        // of the feed are carried by symbols B1u never sent.
        let keep = (B1U_FRAMES * UP_INTERVALS / TRELLIS_FRAME - (SURVIVOR_DEPTH + 1)) * (bits as usize / TRELLIS_FRAME);
        decoded.truncate(keep.min(decoded.len()));
        (decoded, decoder.rotation_line().unwrap_or_default().to_string())
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
        let _ = decoder.take_bits();
    }

    /// A level that sits on its own lattice point is decided by that point,
    /// and not by the 4D code.
    ///
    /// B1u's thirty-one missing bits were seven of its hundred and ninety-two
    /// decided Ki, each off by exactly one class, at symbols whose levels sat
    /// **0.146 to 0.308 half-cells** from the lattice point that carries the
    /// right index -- a hundred times closer to it than to the neighbour a
    /// joint decision would have to move them to. The moves were forced: the
    /// four indices this far end sent in those frames do not lie on the 4D
    /// code's path, so a Viterbi over that code has to move one of the four,
    /// and it moves the one that costs least.
    ///
    /// This builds exactly that sequence out of synthetic levels: B1u's own
    /// classes through the production chain, every level displaced by one of
    /// the residuals measured on the line, and one symbol displaced by a whole
    /// lattice step so that the four indices of its frame are off the code's
    /// path, as B1u's were. What has to come back is the slice's own reading at
    /// every position -- the same bits the displaced classes decode to -- and
    /// not a code-consistent reading of them.
    #[test]
    fn a_level_on_its_own_lattice_point_is_not_moved_by_the_code() {
        // The residuals measured on the calls that read 547 of 576, in
        // half-cells, and the symbol that was a whole step out (the equaliser
        // had not settled: B1u's first data frames).
        const RESIDUALS: [f64; 7] = [0.146, 0.174, 0.263, 0.308, 0.173, 0.156, 0.144];
        const SHIFTED: usize = 30;
        let parameters = parameters(1, Trellis::Sixteen);
        let bits = up_bits(parameters.drn);
        let moduli = Moduli12::new(parameters.moduli);
        let classes = b1u_classes(bits, &parameters.moduli);
        let mut chain = Chain::new(&parameters).expect("the chain these parameters describe");
        let mut decoder = Decoder::new(&parameters);
        decoder.measure_rotation_from(0);
        let fed = B1U_FRAMES * UP_INTERVALS + TRELLIS_FRAME * (SURVIVOR_DEPTH + 2);
        // The indices the *levels* fall into, and the bits they carry.
        let mut labels: Vec<u8> = Vec::new();
        let mut want: Vec<bool> = Vec::new();
        let mut modulus = modulus::Decoder::new();
        let mut scrambler = Scrambler::new(crate::v32::Mode::Answer);
        let mut decoded = Vec::new();
        for n in 0..fed {
            let k = n % TRELLIS_FRAME;
            let constant = if opens_a_pair(k) { pair_first_index_offset() } else { 0 };
            let modulus_now = moduli.modulus(n % UP_INTERVALS);
            let ki = (i64::from(classes[n % classes.len()]) - constant).rem_euclid(i64::from(modulus_now.max(1))) as u8;
            let symbol = chain.step(ki);
            let mut level = symbol.u;
            if n < B1U_FRAMES * UP_INTERVALS {
                level += if n % 2 == 0 { RESIDUALS[n % RESIDUALS.len()] } else { -RESIDUALS[n % RESIDUALS.len()] };
                if n == SHIFTED {
                    level -= 2.0;
                }
            }
            decoder.feed(level);
            decoded.extend(decoder.take_bits());
            if n < B1U_FRAMES * UP_INTERVALS {
                // What the slice says this level is: the same table the
                // decoder slices with, read the same way.
                let set = parameters.set_for(n % UP_INTERVALS).expect("a set");
                let eta = Bests::new(&Constellation::new(set), level).nearest().expect("a nearest point");
                labels.push(index_to_ki_class(eta, modulus_now, k));
                if labels.len() == UP_INTERVALS {
                    let frame: [u8; UP_INTERVALS] = labels.drain(..).collect::<Vec<u8>>().try_into().expect("twelve");
                    let frame = modulus.decode(&frame, &moduli, bits).expect("the slice's own frame decodes");
                    want.extend(frame.bits().map(|bit| scrambler.descramble(bit)));
                }
            }
        }
        let (at, errors) = align(&want, &decoded);
        assert_eq!(errors, 0, "the decoded bits are not the ones the levels carry (offset {at:?})");
        assert_eq!(at, Some(0), "the decoder lost the frame's place");
        // And the shifted symbol is the reason this is not a round trip of a
        // conforming line: the four indices of its frame are off the code's path.
    }

    /// **A rotation at k = 3 is an index offset, and is measured on the index
    /// grid** (`docs/136`).
    ///
    /// At k = 3 6.4.2's class is `((eta - (eta mod 2)) / 2) mod Mi`, so one
    /// index step is *half* a class. A far end whose rotation there is an odd
    /// number of index steps therefore lands on two classes, split by the
    /// transmitted parity bit, and the class-grid measurement reads that split
    /// as `22/18` -- not a mode, so nothing is taken (`g4`, `docs/136 §1`).
    ///
    /// Two synthetic lines, both carrying B1u's own class sequence:
    ///
    /// * the **odd** one sends `eta = 2 * model - (1 - p)` at k = 3 with `p`
    ///   the frame's parity bit, i.e. an index offset of one step: half the
    ///   symbols land a class low (h = 7), half a class high (h = 0), exactly
    ///   `g4`'s `[18 ... 22]`. On the class grid that is `20/20` and no mode;
    ///   folded by the one index step it is `40/0` and reads **7**, which is
    ///   what this line's k = 1 reads too -- `docs/132`'s "k = 1 is an
    ///   outlier" puzzle, dissolved;
    /// * the **even** one sends `eta = 2 * model + p`: an index offset of zero,
    ///   which the class grid already names (`a6`'s `40/0`). It must read 0
    ///   with no index offset, i.e. it must not move at all.
    ///
    /// The first case is also the one that fails on the code before the fix: it
    /// returns no rotation at k = 3, and the decoder falls back to the constant
    /// there for the whole of B1u.
    #[test]
    fn a_half_step_rotation_at_k3_is_measured_on_the_index_grid() {
        let parameters = parameters(1, Trellis::Sixteen);
        let bits = up_bits(parameters.drn);
        assert!(parameters.moduli.iter().all(|&modulus| modulus == 8), "eight classes");
        let classes = b1u_classes(bits, &parameters.moduli);
        let set = Constellation::new(&parameters.sets[0]);
        // What the other three positions read: a class offset each, and the
        // pair-second slot at 7 for the odd line, as `g4` reads it.
        for (classed, odd, want) in [
            ([6, 7, 6], false, [6, 7, 6, 0]),
            ([6, 7, 6], true, [6, 7, 6, 7]),
        ] {
            let mut measuring = Measuring::new(bits, parameters.moduli, 0).expect("eight classes");
            for n in 0..ROTATION_SYMBOLS as usize {
                let k = n % TRELLIS_FRAME;
                let model = i64::from(classes[n]);
                // The frame's own parity bit, transmitted and free (6.4.2): it
                // is *not* the symbol parity, which is why an odd index offset
                // splits the class histogram in two.
                let parity = ((n / TRELLIS_FRAME) % 2) as i64;
                let eta = if k == TRELLIS_FRAME - 1 {
                    // `parity` is the free bit of 6.4.2, and `odd` is the one
                    // index step this far end rotates by: the offset lands the
                    // even-parity symbol a class low (h = 15) and the odd one on
                    // the model (h = 0) -- `g4`'s `[18 ... 22]`, bin for bin.
                    // Sixteen steps up is the same class and the same half-step
                    // bin, and keeps the level inside the set.
                    2 * model + parity - i64::from(odd) + 16
                } else {
                    model + classed[k]
                };
                let level = set.level(eta as i32).expect("every class has a member in the set");
                let nearest = Bests::new(&set, level).nearest().expect("the set is not empty");
                measuring.observe(parameters.moduli[n % UP_INTERVALS], nearest);
            }
            let measurement = measuring.measure();
            assert_eq!(
                measurement.table(),
                want.map(Some),
                "class offsets {classed:?}, odd index step at k = 3 = {odd}"
            );
            assert_eq!(
                measurement.shifts()[TRELLIS_FRAME - 1],
                i32::from(odd),
                "the k = 3 fold has to be the index offset it was measured by, not a class offset"
            );
            assert_eq!(
                measurement.positions[TRELLIS_FRAME - 1].best,
                ROTATION_SYMBOLS as u16 / TRELLIS_FRAME as u16,
                "a half-step rotation has to fold to one mode, not to the 22/18 split"
            );
        }
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
                    measuring.measure().table(),
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
        assert_eq!(measuring.measure().table(), [None; TRELLIS_FRAME], "noise was taken for a rotation");
    }
    /// `docs/301` sec 1. One dropped frame in B1u must not offset the twelve
    /// interval group for the rest of it.
    ///
    /// The group is twelve *contiguous* symbols: 8.7.1's three trellis frames,
    /// 6.4.1's mixed radix over their twelve `Ki`. A frame the window declines
    /// (`decide_from_its_own_slice`'s `nearest()` on noise, or `step`'s no
    /// reachable state) pushes no labels, so a group assembled by counting
    /// labels is short by four and the two sides of the hole are stapled into
    /// one frame: a frame's worth of wrong bits *and* every later symbol named
    /// with the wrong interval's modulus, for the rest of B1u. That is the
    /// call's shape -- the first 160 symbols carry the right digits and the
    /// call still reads 362/576.
    ///
    /// The rung is Agere's `drn 2` (`[16, 16, 8 x 10]`, `docs/270` sec 1.2) so
    /// that a mis-attributed interval is visible in the `Ki` itself: index 12
    /// is `12 mod 16 = 12` on an `Mi = 16` interval and `12 mod 8 = 4` on an
    /// `Mi = 8` one.
    #[test]
    fn a_dropped_frame_does_not_offset_the_twelve_interval_group() {
        let moduli = [16u8, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8];
        let mut parameters = parameters(2, Trellis::Sixteen);
        assert_eq!(up_bits(2), 38, "drn 2's data frame is 38 bits");
        parameters.moduli = moduli;
        let mut decoder = Decoder::new(&parameters);
        let eta = 12;
        // What frame `frame`'s four labels are when each symbol is read with the
        // modulus of the interval it actually sits in.
        let own_intervals = |frame: u64| -> Vec<u8> {
            (0..TRELLIS_FRAME)
                .map(|k| index_to_ki(eta, moduli[(frame as usize * TRELLIS_FRAME + k) % UP_INTERVALS], k))
                .collect()
        };
        // B1u's first data frame: frames 0, 1, 2 are intervals 0 .. 11.
        for index in 0..3u64 {
            decoder.push_etas(index, [eta; TRELLIS_FRAME]);
        }
        assert!(decoder.labels.is_empty(), "the first group decoded: {:?}", decoder.labels);
        // Frames 3 and 4 arrive, filling intervals 0 .. 3 and 4 .. 7.
        for index in 3..5u64 {
            decoder.push_etas(index, [eta; TRELLIS_FRAME]);
        }
        assert_eq!(decoder.labels, own_intervals(3).into_iter().chain(own_intervals(4)).collect::<Vec<_>>());
        // Frame 5 is dropped. Frame 6's first symbol is interval 0 again, so
        // the group restarts there on twelve contiguous symbols of their own
        // intervals -- the alternative, which is what the code did, is a
        // twelve-label "frame" stapled across the hole.
        decoder.push_etas(6, [eta; TRELLIS_FRAME]);
        assert_eq!(
            decoder.labels,
            own_intervals(6),
            "the group restarted at frame 6's interval 0 -- the symbols after the \
             hole are on their own intervals, not stitched to the four before it"
        );
        decoder.push_etas(7, [eta; TRELLIS_FRAME]);
        assert_eq!(decoder.labels.len(), 8, "{:?}", decoder.labels);
        decoder.push_etas(8, [eta; TRELLIS_FRAME]);
        assert!(decoder.labels.is_empty(), "frames 6, 7, 8 are one group: {:?}", decoder.labels);
        // ... and the group after it is on its own intervals too, which is the
        // part a container-derived interval gets wrong for the rest of B1u.
        decoder.push_etas(9, [eta; TRELLIS_FRAME]);
        assert_eq!(decoder.labels, own_intervals(9), "frame 9 begins interval 0");
        // A frame that does not begin a group is not pushed at all, rather than
        // starting one mid-interval: 8.7.1's data frame begins at interval 0.
        decoder.labels.clear();
        decoder.push_etas(10, [eta; TRELLIS_FRAME]);
        assert!(decoder.labels.is_empty(), "frame 10 is intervals 4 .. 7, no boundary: {:?}", decoder.labels);
    }

    /// `docs/301` sec 2. A class is binned modulo the **interval's own `Mi`**,
    /// not modulo the widest modulus on the rung.
    ///
    /// The class map is `Ki = eta mod Mi` at k = 0, 1, 2 and
    /// `((eta - p) / 2) mod Mi` at k = 3 (`precoder.rs:430-437`, 6.4.1/6.4.2),
    /// so `Mi` is the period the difference lives in. Binning it mod
    /// `max(moduli)` is invisible where the moduli are equal -- both verified
    /// far ends at 24 000, and HSF at any rung it uses -- and wrong wherever
    /// they are mixed: at Agere's `drn 2` the ten `Mi = 8` intervals have their
    /// samples split eight bins apart, because a difference of `-1` is `7` in
    /// radix 8 and `15` in radix 16. That is the `+-Mi/2` lobe `docs/270` sec
    /// 1.3 printed; `docs/272`/`docs/275` traced it to the analysis tool's
    /// radix and fixed it there (`b1d8223`), and this was the other instance --
    /// the receiver's own `Measuring` (`docs/301` sec 2).
    #[test]
    fn a_class_is_binned_in_its_own_intervals_radix() {
        let moduli = [16u8, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8];
        let bits = up_bits(2);
        let mut measuring = Measuring::new(bits, moduli, 0).expect("Mi <= 16");
        let predicted = measuring.predicted.clone();
        assert_eq!(predicted.len(), B1U_FRAMES * UP_INTERVALS);
        // One rotation for the whole line, seven index steps, which is the
        // shape that splits: on an `Mi = 8` interval it is `7` one way and `-1`
        // the other, and those are one bin apart in radix 8 and eight apart in
        // radix 16.
        const ROTATION: i32 = 7;
        for index in 0..ROTATION_SYMBOLS as usize {
            let k = index % TRELLIS_FRAME;
            let modulus = moduli[index % UP_INTERVALS];
            let model = i32::from(predicted[index]);
            assert!(model < i32::from(modulus), "a class is inside its own interval's Mi");
            let class = (model + ROTATION).rem_euclid(i32::from(modulus));
            // Any member of the class will do at k = 0, 1, 2; at k = 3 the
            // class's members are the adjacent indices `2 Ki`, `2 Ki + 1`.
            let eta = if k == TRELLIS_FRAME - 1 { 2 * class } else { class };
            measuring.observe(modulus, eta);
        }
        // Every sample of every position is that one rotation, so the reading
        // is the rotation at all four and the margin is everything to nothing.
        // Scored in radix 16 the `Mi = 8` samples split, so no position has a
        // mode at all and nothing is taken: that is the cost of the defect.
        assert_eq!(
            measuring.measure().table(),
            [Some(ROTATION as i64); TRELLIS_FRAME],
            "one rotation over a mixed rung must read at every position"
        );
        // And the lobe itself: no sample of any position sits eight bins away
        // from that position's mode.
        for (sign, rows) in measuring.counts.iter().enumerate() {
            for (k, row) in rows.iter().enumerate() {
                if k == TRELLIS_FRAME - 1 {
                    continue;
                }
                let away = (ROTATION as usize + 8) % 16;
                assert_eq!(
                    row[away], 0,
                    "sign {sign} position {k}: a class eight bins from its own mode, which is a \
                     Mi = 8 interval scored in radix 16: {row:?}"
                );
            }
        }
        // The uniform rungs cannot move: where all twelve `Mi` are equal, the
        // interval's radix *is* `max(moduli)`, which is the arithmetic
        // `docs/46` (`Mi = 8`) and `docs/74` (`Mi = 16`) were measured with.
        // The regression that would matter -- a known-good call moving -- is
        // `a_rotation_that_is_not_the_constant_is_measured_off_b1u` above,
        // untouched by this and still green at `drn 1`.
        for (drn, mi) in [(1u8, 8u8), (7, 16)] {
            let mut uniform = Measuring::new(up_bits(drn), [mi; UP_INTERVALS], 0).expect("Mi <= 16");
            for index in 0..ROTATION_SYMBOLS as usize {
                let model = i32::from(uniform.predicted[index]);
                let eta = if index % TRELLIS_FRAME == TRELLIS_FRAME - 1 { 2 * model } else { model };
                uniform.observe(mi, eta);
            }
            assert_eq!(uniform.radix, [usize::from(mi); TRELLIS_FRAME], "drn {drn}: the only modulus is the radix");
            assert_eq!(uniform.measure().table(), [Some(0); TRELLIS_FRAME], "drn {drn}: no rotation reads as no rotation");
        }
    }

}

