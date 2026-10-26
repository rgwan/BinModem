//! `docs/250`: the closed loop, the ideal case, and the tolerance sweep.
//!
//! The chain is `docs/248`'s generator with three additions, and no new
//! transmitter:
//!
//! ```text
//! bits -> UpEncoder (6.3, 6.4.1-6.4.4) -> G x v(n)  [Symbol::out = the line, pre-tap]
//!      -> far end applies FarEndParams (pre-emphasis, fractional instant, level)
//!      -> the CO's own sampling instant and one post-cursor tap (the channel)
//!      -> G.711 quantiser -> codewords
//!      -> our receiver: Decoder (B1u in the bench's terms) + per-chord statistic
//! ```
//!
//! `--mode closed` runs the loop: the far end transmits with a line it chose,
//! our end estimates the delay, the level and the tap off TRN1u
//! (`farend::estimate`), and the far end re-transmits with
//! `Estimate::correction`'s parameter set. `--mode ideal` sets the parameters to
//! their exact values -- zero instant error, the exact level, and the exact
//! three-tap inverse of the channel -- which is the bound, not the bench.
//! `--mode sweep` puts a known error on one parameter at a time.
//!
//! ```text
//! cargo run --release -p datapump --example v92_closed_loop -- \
//!     --mode ideal --drn 0 --levelset code --chords
//! ```
//!
//! `--drn 0` walks the whole ladder, `drn 1 .. 19` (24 000 .. 48 000).

use datapump::v90::ucode::{self, Law, UCODES};
use datapump::v92::decoder::Decoder;
use datapump::v92::design::{self, Channel};
use datapump::v92::farend::{self, FarEndParams};
use datapump::v92::receiver::Receiver;
use datapump::v92::rung::{self, Reading};
use datapump::v92::up_signals::Trn1u;
use datapump::v92::upstream::UpEncoder;
use datapump::v92::{FilterLimits, LevelSet, Parameters, Trellis, UP_INTERVALS, upchoice};

/// A deterministic bit source. No crate, same stream every run.
struct Random(u64);

impl Random {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }
    fn next(&mut self) -> bool {
        self.word() >> 63 == 0
    }
    fn word(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

/// The rung's parameters **as the design path emits them** (`docs/252`), not as
/// the harness would like them.
///
/// This is the whole of `docs/254` step 1: `G` is computed where CPd bits 35:50
/// are computed -- `design::design` normalises the section to unit energy and
/// `upchoice::build_with` divides the set's gain by `sqrt(sent_prefilter_energy)`
/// -- so the number the far end is driven at below is the one on the wire, not a
/// copy of it. The design is taken against the harness's neutral line (one tap,
/// nothing else), so `z2` is the identity to the last Q0.15 code and the
/// operating point this buys is `1/G` of the emitted gain.
fn parameters(drn: u8, trellis: Trellis, levelset: LevelSet) -> Parameters {
    let limits = FilterLimits::from_info1a(3, 3, 3);
    let designed = design::design(
        &Channel { taps: vec![1.0], noise: 0.0, signal: 1.0 },
        &limits,
    )
    .expect("the harness's neutral channel is designable");
    upchoice::pin_with(&designed, drn, trellis, 0.0, levelset)
        .unwrap_or_else(|| panic!("drn {drn} is not built by the design path"))
        .parameters
}

/// The G.711 quantiser the central office is.
fn quantise(law: Law, lsb: f64) -> i32 {
    let full = lsb.clamp(-32768.0, 32767.0).round() as i32;
    let (code, negative) = ucode::nearest(law, full.abs().min(32767));
    let level = ucode::linear(law, code).abs();
    if negative || full < 0 { -level } else { level }
}

/// The chord ladder: for each chord, its step in LSB and the codes in it -- the
/// same table `tools/ucode_chords.py` derives, derived here so that the two can
/// be compared on one file.
struct Ladder {
    step: Vec<i32>,
}

impl Ladder {
    fn new(law: Law) -> Self {
        let mut step = vec![0i32; 8];
        for chord in 0..8u16 {
            let mut mags: Vec<i32> = (0..UCODES as u8)
                .filter(|&code| (code >> 4) == chord as u8)
                .map(|code| ucode::linear(law, code).abs())
                .collect();
            mags.sort_unstable();
            mags.dedup();
            step[chord as usize] = mags.windows(2).map(|p| p[1] - p[0]).min().unwrap_or(0);
        }
        Self { step }
    }

    /// The chord a sample belongs to, its error against the nearest code, and
    /// whether that error crossed half the chord's own minimum gap.
    fn read(&self, law: Law, x: f64) -> Option<(usize, f64, f64)> {
        let full = x.clamp(-32768.0, 32767.0).round() as i32;
        if full == 0 {
            return None;
        }
        let (code, negative) = ucode::nearest(law, full.abs().min(32767));
        let level = ucode::linear(law, code).abs();
        let chord = (code >> 4) as usize;
        let step = f64::from(self.step[chord]);
        let _ = negative;
        Some((chord, step, (f64::from(full.abs()) - f64::from(level)).abs()))
    }
}

/// One chord's statistics over a stream, which is `ucode_chords.py`'s row.
#[derive(Default, Clone, Copy)]
struct Row {
    n: u64,
    sum_sq: f64,
    max: f64,
}

/// The whole statistic: per chord, and the population-weighted crossing at a
/// gap in LSB.
struct Stats {
    rows: [Row; 8],
    total: u64,
}

impl Stats {
    fn scan(law: Law, ladder: &Ladder, xs: &[f64]) -> Self {
        let mut stats = Self { rows: [Row::default(); 8], total: 0 };
        for &x in xs {
            let Some((chord, step, error)) = ladder.read(law, x) else { continue };
            let _ = step;
            let row = &mut stats.rows[chord];
            row.n += 1;
            row.sum_sq += error * error;
            row.max = row.max.max(error);
            stats.total += 1;
        }
        stats
    }

    fn pop(&self, chord: usize) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        100.0 * self.rows[chord].n as f64 / self.total as f64
    }

    fn rms_half(&self, chord: usize, ladder: &Ladder) -> f64 {
        let row = &self.rows[chord];
        if row.n == 0 {
            return 0.0;
        }
        (row.sum_sq / row.n as f64).sqrt() / (f64::from(ladder.step[chord]) / 2.0)
    }

    fn support(&self, chord: usize, ladder: &Ladder) -> f64 {
        self.rows[chord].max / (f64::from(ladder.step[chord]) / 2.0)
    }

    /// The crossing at `gap` LSB, weighted by each chord's own population: the
    /// number docs/244's table carries, in per cent of symbols.
    fn crossing(&self, ladder: &Ladder, xs: &[f64], law: Law, gap: f64) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        let mut hit = 0.0;
        for &x in xs {
            let Some((_, step, error)) = ladder.read(law, x) else { continue };
            if error > (gap.max(step) / 2.0) {
                hit += 1.0;
            }
        }
        let _ = ladder;
        100.0 * hit / self.total as f64
    }
}

/// The landing, measured **before** the codec.
///
/// `Stats::scan` reads the quantiser's *codewords*, and a codeword is by
/// construction one of `ucode::linear`'s values, so its error against the
/// nearest code is zero for any level and any alphabet: `rms / half` on the
/// codewords is a tautology (`docs/255` S1.1). This reads the far end's own
/// line, `continuous`, where a level or a time error is visible.
struct Landing {
    rows: [Row; 8],
    total: u64,
}

impl Landing {
    fn scan(law: Law, ladder: &Ladder, xs: &[f64]) -> Self {
        let mut out = Self { rows: [Row::default(); 8], total: 0 };
        for &x in xs {
            let full = x.abs();
            if full < 0.5 {
                continue;
            }
            let (code, _) = ucode::nearest(law, full.min(32767.0).round() as i32);
            let level = f64::from(ucode::linear(law, code).abs());
            let chord = ((code >> 4) as usize).min(7);
            let error = (full - level).abs();
            let row = &mut out.rows[chord];
            row.n += 1;
            row.sum_sq += error * error;
            row.max = row.max.max(error);
            out.total += 1;
        }
        let _ = ladder;
        out
    }

    fn rms_half(&self, chord: usize, ladder: &Ladder) -> f64 {
        let row = &self.rows[chord];
        if row.n == 0 {
            return 0.0;
        }
        (row.sum_sq / row.n as f64).sqrt() / (f64::from(ladder.step[chord]) / 2.0)
    }

    fn support(&self, chord: usize, ladder: &Ladder) -> f64 {
        self.rows[chord].max / (f64::from(ladder.step[chord]) / 2.0)
    }

    fn pop(&self, chord: usize) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        100.0 * self.rows[chord].n as f64 / self.total as f64
    }

    /// The population-weighted `rms / half` over the whole stream, which is one
    /// number a log line can carry.
    fn weighted(&self, ladder: &Ladder) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        let mut sum = 0.0;
        for chord in 0..8 {
            if self.rows[chord].n == 0 {
                continue;
            }
            let share = self.rows[chord].n as f64 / self.total as f64;
            sum += share * self.rms_half(chord, ladder);
        }
        sum
    }
}

#[derive(Clone)]
struct Settings {
    drn: u8,
    frames: usize,
    levelset: LevelSet,
    law: Law,
    seed: u64,
    mode: String,
    /// The line the far end's transmitter is on: the sampling instant the CO
    /// uses, one post-cursor tap, and the level it transmits at.
    timing: f64,
    tap: f64,
    lu: f64,
    /// The level our receiver's decoder is fed at, 8.8.3's mean-square-1 point.
    target: f64,
    chords: bool,
    /// What the slicing table is built at: `one` is the design's own codes (the
    /// default and what every `docs/250` number used), `auto` is the far end's
    /// own scale, `lu x G`, which is what a receiver measuring `K` off the CPu
    /// mask would build (`docs/114` R4, `docs/253`).
    slice: String,
    receiver: bool,
    /// `--mode sweep`: the error put on one parameter and nothing else.
    err_delay: Option<f64>,
    err_level: Option<f64>,
    err_tap: Option<f64>,
    /// Whether the CO's sampling instant is a second interpolation kernel
    /// (docs/250-254's convention) or the point sample of a continuous line it
    /// physically is (`docs/255` S1).
    instant: Instant,
    /// `docs/255`'s freeze-and-sweep: in `--mode closed`, these components of
    /// the estimate are replaced by their exact values, so the run carries that
    /// component's error and no other's.
    freeze: Vec<String>,
    /// Whether to print the landing measured *before* the codec, which is the
    /// only form of it that can fail.
    landing: bool,
    /// How many taps of the far end's pre-emphasis to keep: `farend::EQ_DEPTH`
    /// unless `--depth` says otherwise. Still computed from the *estimated*
    /// post-cursor, so the depth axis is swept without handing the loop the
    /// answer (`docs/255` S3).
    depth: usize,
    /// `docs/261`: the level error the **generator** puts on the far end's
    /// emitted level, in per cent of `1/G`.
    ///
    /// This is the far end's own realisation error, applied inside `transmit`
    /// after every parameter has been applied: the far end is told a level and
    /// puts `(1 + far_level/100)` times it on the line. The loop is not told --
    /// it sees the stream and estimates from it, and the receiver's only lever
    /// is the scale `K` it measures (`--slice est`). V.92 has no field that
    /// re-commands the level in data mode -- `G` went out once in CPd bits
    /// 35:50 (`v92.txt:1824-1826`) -- so with `--hold-level` the commanded
    /// level stays at the design's `1/G` and the far end's realisation error
    /// persists for the whole call.
    far_level: f64,
    /// Whether our end may re-command the far end's level. `false` is the
    /// physical case.
    hold_level: bool,
}

/// How the chain turns the far end's fractional instant and the CO's sampling
/// instant into samples.
#[derive(Clone, Copy, PartialEq)]
enum Instant {
    /// One sample of the continuous line: `S x(n) p((m - n)T + delta)`, i.e.
    /// one kernel at the *difference* of the two positions. This is what a
    /// point sample of a DAC's output does.
    Single,
    /// Two kernels in series: the far end's windowed sinc, then the CO's,
    /// which composes them to `H(f)^2` and is `docs/250`-`docs/254`'s
    /// convention.
    Double,
}

fn parse() -> Settings {
    let mut s = Settings {
        drn: 0,
        frames: 1_200,
        levelset: LevelSet::Code,
        law: Law::Mu,
        seed: 1,
        mode: "ideal".to_owned(),
        timing: 0.0,
        tap: 0.0,
        lu: 900.0,
        target: 900.0,
        chords: false,
        slice: "one".to_owned(),
        receiver: false,
        err_delay: None,
        err_level: None,
        err_tap: None,
        instant: Instant::Single,
        freeze: Vec::new(),
        landing: false,
        depth: farend::EQ_DEPTH,
        far_level: 0.0,
        hold_level: false,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().expect("a value after the option");
        match arg.as_str() {
            "--drn" => s.drn = value().parse().expect("drn"),
            "--frames" => s.frames = value().parse().expect("frames"),
            "--law" => s.law = if value() == "a" { Law::A } else { Law::Mu },
            "--seed" => s.seed = value().parse().expect("seed"),
            "--mode" => s.mode = value(),
            "--timing" => s.timing = value().parse().expect("timing"),
            "--tap" => s.tap = value().parse().expect("tap"),
            "--lu" => s.lu = value().parse().expect("lu"),
            "--target" => s.target = value().parse().expect("target"),
            "--levelset" => {
                s.levelset = match value().as_str() {
                    "code" | "kode" => LevelSet::Code,
                    _ => LevelSet::Uniform,
                }
            }
            "--err-delay" => s.err_delay = Some(value().parse().expect("err-delay")),
            "--err-level" => s.err_level = Some(value().parse().expect("err-level")),
            "--err-tap" => s.err_tap = Some(value().parse().expect("err-tap")),
            "--chords" => s.chords = true,
            "--slice" => s.slice = value(),
            "--receiver" => s.receiver = true,
            "--instant" => {
                s.instant = match value().as_str() {
                    "single" => Instant::Single,
                    "double" => Instant::Double,
                    other => panic!("instant {other}"),
                }
            }
            "--freeze" => {
                s.freeze = value().split(',').map(str::to_owned).collect();
            }
            "--landing" => s.landing = true,
            "--depth" => s.depth = value().parse().expect("depth"),
            "--far-level" => s.far_level = value().parse().expect("far-level"),
            "--hold-level" => s.hold_level = true,
            other => panic!("unknown option {other}"),
        }
    }
    s
}

/// The far end's continuous stream in LSB with `params` applied, then the
/// channel: the CO's own sampling instant, and one post-cursor tap.
fn transmit(
    symbols: &[f64],
    params: &FarEndParams,
    timing: f64,
    tap: f64,
    law: Law,
    instant: Instant,
    far: f64,
) -> (Vec<f64>, Vec<i32>) {
    let sampled = match instant {
        Instant::Single if params.delay != 0.0 => {
            // The far end's instant and the CO's are two positions on one
            // continuous line, so they compose to a single kernel at their
            // difference. Applying two truncated sincs in series squares the
            // interpolation error instead (`docs/255` S1).
            let shaped = params.eq.filter(symbols);
            let placed = farend::shift_centred(&shaped, params.delay + timing);
            placed.iter().map(|&v| v * params.level_lsb).collect()
        }
        _ => {
            let placed = params.apply(symbols);
            if timing == 0.0 { placed } else { farend::shift_centred(&placed, timing) }
        }
    };
    // `docs/261`: the far end's own realisation error on the level it was told,
    // applied by the *generator*, after every parameter and before the channel.
    // `far == 1.0` is bit-for-bit the old path.
    let sampled: Vec<f64> =
        if far == 1.0 { sampled } else { sampled.iter().map(|&v| v * far).collect() };
    let mut continuous = Vec::with_capacity(sampled.len());
    let mut previous = 0.0;
    for &x in &sampled {
        let y = x + tap * previous;
        previous = x;
        continuous.push(y.clamp(-32768.0, 32767.0).round());
    }
    let codewords = continuous.iter().map(|&z| quantise(law, z)).collect();
    (continuous, codewords)
}

/// The whole upstream stream, in `out` units, plus the ideal TRN1u slice the
/// estimator is given.
fn stream(s: &Settings) -> (Parameters, Vec<f64>, Vec<f64>, usize, Vec<i32>) {
    let parameters = parameters(s.drn, Trellis::Sixteen, s.levelset);
    let mut encoder = UpEncoder::new(&parameters).expect("feasible parameters");
    let mut random = Random::new(s.seed);
    let mut out = Vec::new();
    let mut trn = Trn1u::new();
    let mut ideal_trn = Vec::new();
    for _ in 0..2_040 {
        let symbol = trn.next_symbol();
        out.push(symbol);
        ideal_trn.push(symbol);
    }
    let b1u_at = out.len();
    // The indices B1u was sent with. 6.4.2's y(n) is the number the far end's
    // decoder has to recover, so it is what a code (bin) error is measured
    // against -- not the level, which the codec rounds and the tap moves.
    let mut b1u_etas = Vec::new();
    for symbol in encoder.b1u() {
        out.push(symbol.out);
        b1u_etas.push(symbol.eta);
    }
    for _ in 0..s.frames * UP_INTERVALS {
        out.push(encoder.next_symbol(|| random.next()).out);
    }
    (parameters, out, ideal_trn, b1u_at, b1u_etas)
}

/// The B1u count in the bench's own terms: bits decoded, how many of them are
/// ones, and the class rotation the decoder measured.
fn b1u(parameters: &Parameters, codewords: &[i32], at: usize, lu: f64, k: Option<f64>) -> (usize, usize, String) {
    let mut decoder = Decoder::new(parameters);
    if let Some(k) = k {
        decoder.slice_at(k);
    }
    decoder.measure_rotation_from(0);
    let mut ones = 0;
    let mut bits = 0;
    for &code in codewords.iter().skip(at).take(UP_INTERVALS * 48) {
        decoder.feed(f64::from(code) / lu);
        for bit in decoder.take_bits() {
            bits += 1;
            ones += usize::from(bit);
        }
    }
    let rotation = decoder.rotation_line().unwrap_or("-").to_owned();
    (ones, bits, rotation)
}

/// The **code (bin) error rate**: the fraction of B1u's symbols whose decided
/// code differs from the code that was sent, per chord and weighted by each
/// chord's population (`docs/253` §2).
///
/// This is the correction to the instrument, not an addition to it. The
/// per-chord half-step crossing (`Stats::crossing`) counts a symbol only when it
/// moves past half of *its chord's own step*, which at 64 points is 8 LSB in
/// chord 0 and 128 LSB in chord 4 -- one number that is neither the design's gap
/// nor the thing a slicer decides. `docs/250` §4 is what that costs: the crossing
/// reads exactly 0.0000 % across a sweep in which `B1u` loses 41 points, because
/// the decision that fails is the *adjacent bin*, one code step away, and never a
/// half-step boundary. Both are kept, because they bind at different rungs: the
/// crossing binds where the noise is a fraction of the step (drn 1 ... 8), the bin
/// error where the gap has come down to the codec's own step (drn 14 ... 19).
#[derive(Default, Clone, Copy)]
struct Bins {
    /// Per chord: the symbols sent, and the ones decided as another code.
    sent: [u64; 8],
    wrong: [u64; 8],
    total: u64,
    total_wrong: u64,
    /// Of the errors, the ones that landed on the adjacent index -- the failure
    /// mode a bin has and a half-step crossing does not describe.
    adjacent: u64,
    /// The scale the table was built at, and whether every decided frame found
    /// its transmitted indices (a dropped frame would shift the pairing).
    k: f64,
    aligned: bool,
}

impl Bins {
    /// Slice B1u with `Decoder` and compare each decided index with the one that
    /// was sent. `k` is the table's scale: `None` leaves the design's own codes.
    fn measure(
        parameters: &Parameters,
        sent_etas: &[i32],
        codewords: &[i32],
        at: usize,
        lu: f64,
        k: Option<f64>,
        ladder: &Ladder,
        law: Law,
    ) -> Self {
        let mut bins = Self { k: k.unwrap_or(1.0), aligned: true, ..Self::default() };
        let mut decoder = Decoder::new(parameters);
        if let Some(k) = k {
            decoder.slice_at(k);
        }
        decoder.keep_decisions(true);
        let set = &parameters.sets[0];
        let chord_of = |eta: i32| -> usize {
            let index = if eta >= 0 { eta as usize } else { (-eta - 1) as usize };
            let value = f64::from(set.get(index).copied().unwrap_or(0));
            let full = value.clamp(0.0, 32767.0).round() as i32;
            let (code, _) = ucode::nearest(law, full);
            ((code >> 4) as usize).min(7)
        };
        let _ = ladder;
        for (n, &code) in codewords.iter().skip(at).take(UP_INTERVALS * 48).enumerate() {
            decoder.feed(f64::from(code) / lu);
            for (index, etas) in decoder.take_decisions_indexed() {
                for (position, &eta) in etas.iter().enumerate() {
                    let symbol = index as usize * 4 + position;
                    let Some(&sent) = sent_etas.get(symbol) else {
                        bins.aligned = false;
                        continue;
                    };
                    let _ = n;
                    let chord = chord_of(sent);
                    bins.sent[chord] += 1;
                    bins.total += 1;
                    if eta != sent {
                        bins.wrong[chord] += 1;
                        bins.total_wrong += 1;
                        if (eta - sent).abs() == 1 {
                            bins.adjacent += 1;
                        }
                    }
                }
            }
        }
        bins
    }

    fn pct(&self) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        100.0 * self.total_wrong as f64 / self.total as f64
    }

    /// The population-weighted bin error rate: each chord's own error rate times
    /// the share of the symbols it carries, which is the number that is
    /// comparable across rungs whose chord populations differ.
    fn weighted(&self) -> f64 {
        let mut sum = 0.0;
        for chord in 0..8 {
            if self.sent[chord] == 0 {
                continue;
            }
            let share = self.sent[chord] as f64 / self.total.max(1) as f64;
            sum += share * (self.wrong[chord] as f64 / self.sent[chord] as f64);
        }
        100.0 * sum
    }

    fn chord_pct(&self, chord: usize) -> f64 {
        if self.sent[chord] == 0 {
            return 0.0;
        }
        100.0 * self.wrong[chord] as f64 / self.sent[chord] as f64
    }

    fn chord_pop(&self, chord: usize) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        100.0 * self.sent[chord] as f64 / self.total as f64
    }
}

fn main() {
    let s = parse();
    let ladder = Ladder::new(s.law);
    let rungs: Vec<u8> = if s.drn == 0 { (1..=19).collect() } else { vec![s.drn] };
    println!(
        "mode={} levelset={} law={} lu={} target={} timing={} tap={} frames={} seed={}",
        s.mode,
        s.levelset.name(),
        if s.law == Law::Mu { "mu" } else { "a" },
        s.lu,
        s.target,
        s.timing,
        s.tap,
        s.frames,
        s.seed
    );
    for drn in rungs {
        let rung = datapump::v92::rung::Rung::of(drn).expect("a rung");
        let gap = datapump::v92::rung::gap_lsb(datapump::v92::rung::points(drn)).unwrap_or(0.0);
        let params = parameters(drn, Trellis::Sixteen, s.levelset);
        let points = datapump::v92::rung::points(drn);
        // The alphabet's own structure: how many codes each chord carries, and
        // the smallest gap between neighbours inside a chord.
        let mut per_chord = [0usize; 8];
        let set = &params.sets[0];
        if s.levelset == LevelSet::Code {
            for &point in set {
                per_chord[usize::from((point & 0x7F) >> 4)] += 1;
            }
        }
        let least = if s.levelset == LevelSet::Code {
            set.windows(2).map(|p| i32::from(p[1]) - i32::from(p[0])).min().unwrap_or(0)
        } else {
            2
        };

        // `docs/261`: how much higher the far end's DAC sits than the level it
        // was told, as a multiplier on top of every `FarEndParams::level_lsb`.
        let far = 1.0 + s.far_level / 100.0;
        let (parameters, out, ideal_trn, b1u_at, b1u_etas) =
            stream(&Settings { drn, ..s.clone() });
        // `--lu 0` is 8.8.3's own operating point: the level at which one unit
        // of `G x v` arrives as one LSB, `1 / G`. Every design point is then a
        // code value on the line, so the codec's quantiser is the no-op the
        // code level set exists to make it -- which is the *only* level at
        // which an "ideal case" is ideal (docs/253 sec 1).
        let s = if s.lu == 0.0 { Settings { lu: 1.0 / parameters.gain, ..s.clone() } } else { s.clone() };
        let line_lsb: Vec<f64> = out.iter().map(|&v| v * s.lu).collect();
        let _ = &line_lsb;

        // --- the loop -----------------------------------------------------
        let (used, estimated) = match s.mode.as_str() {
            "closed" => {
                // Pass 1: the far end is on the line it chose.
                let start = FarEndParams::bypass(s.lu);
                let (_, first) = transmit(&out, &start, s.timing, s.tap, s.law, s.instant, far);
                let observed: Vec<f64> = first.iter().map(|&q| f64::from(q)).collect();
                let e = farend::estimate(
                    &ideal_trn,
                    &observed[..ideal_trn.len()],
                    0..8,
                );
                let correction = e.correction(s.lu, s.lu);
                println!(
                    "  estimate drn={drn} delay={:+.4} level={:.1} tap={:+.4} nmse={:.5} -> \
                     params delay={:+.4} level={:.1} eq=[{}]",
                    e.delay,
                    e.level,
                    e.tap,
                    e.nmse,
                    correction.delay,
                    correction.level_lsb,
                    correction
                        .eq
                        .taps
                        .iter()
                        .map(|c| format!("{c:+.3}"))
                        .collect::<Vec<_>>()
                        .join(",")
                );
                // docs/255's freeze-and-sweep: put the exact value back into
                // one named component, so the run carries the others' error and
                // nothing else's. The exact values are the truth of the line the
                // far end is on, not the answer to the estimator's question.
                let mut frozen = correction;
                if s.depth != farend::EQ_DEPTH {
                    frozen.eq = farend::EqTaps::inverse_of_depth(frozen.eq.taps[1].abs(), s.depth);
                }
                if s.hold_level {
                    // The level was named once (`G`, CPd 35:50) and no field in
                    // data mode re-commands it: the far end's realisation error
                    // is measured, not removed. This is not `--freeze level`,
                    // which puts the *answer* back; here the command is `1/G`
                    // and the answer is unknown to the loop.
                    frozen.level_lsb = s.lu;
                }
                for name in &s.freeze {
                    match name.as_str() {
                        "delay" => frozen.delay = -s.timing,
                        "level" => frozen.level_lsb = s.lu,
                        "eq" => frozen.eq = farend::EqTaps::inverse_of(s.tap),
                        other => panic!("freeze {other}"),
                    }
                }
                (frozen, Some(e))
            }
            "sweep" => {
                // The exact parameters, plus one named error and nothing else.
                let mut p = FarEndParams::bypass(s.lu);
                if let Some(e) = s.err_delay {
                    p.delay = e;
                }
                if let Some(e) = s.err_level {
                    p.level_lsb = s.lu + e;
                }
                if let Some(e) = s.err_tap {
                    p.eq = farend::EqTaps::inverse_of(s.tap + e);
                }
                (p, None)
            }
            _ => {
                // The ideal case: exact values, which for a channel with no tap
                // and no instant error is the bypass itself.
                let mut p = FarEndParams::bypass(s.lu);
                if s.tap != 0.0 {
                    p.eq = farend::EqTaps::inverse_of(s.tap);
                }
                if s.err_delay.is_some() || s.err_level.is_some() || s.err_tap.is_some() {
                    if let Some(e) = s.err_delay {
                        p.delay = e;
                    }
                    if let Some(e) = s.err_level {
                        p.level_lsb += e;
                    }
                    if let Some(e) = s.err_tap {
                        p.eq = farend::EqTaps::inverse_of(s.tap + e);
                    }
                }
                (p, None)
            }
        };
        let _ = estimated;

        let (continuous, codewords) =
            transmit(&out, &used, s.timing, s.tap, s.law, s.instant, far);
        let rms = (continuous.iter().map(|v| v * v).sum::<f64>() / continuous.len() as f64).sqrt();
        let peak = continuous.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let out_rms = (out.iter().map(|v| v * v).sum::<f64>() / out.len() as f64).sqrt();
        let rx: Vec<f64> = codewords.iter().map(|&q| f64::from(q)).collect();
        let stats = Stats::scan(s.law, &ladder, &rx);
        let pct = 0.0;
        // The far end's own scale: the line carries `G x v` at `level_lsb` LSB,
        // so the levels arriving are `lu x G` times the design's codes.
        // `k` is how much more the far end transmits than the receiver assumes
        // it does: the receiver converts LSB back to `G x v` with the level it
        // believes (`s.lu`), so a far end on a different level arrives at the
        // slicer scaled by the ratio. In the ideal case that ratio is one, and
        // it is `level_lsb` against `lu` in the closed loop -- 1.055 at
        // `docs/250`'s drn 19 run, which is the LC = 64 alphabet's own
        // half-gap's worth of scale and more.
        let k = match s.slice.as_str() {
            "auto" => Some(used.level_lsb / s.lu),
            // `docs/261`: the scale our *own* estimator measured off TRN1u, in
            // units of `1/G` -- the `K` a receiver builds its table at
            // (`docs/114` R4). Nothing here reads `far`.
            "est" => Some(match estimated {
                Some(e) => e.level * parameters.gain,
                None => 1.0,
            }),
            "one" => None,
            other => Some(other.parse().expect("a scale")),
        };
        let (ones, bits, rotation) = b1u(&parameters, &codewords, b1u_at, s.lu, k);
        let pct = if bits > 0 { 100.0 * ones as f64 / bits as f64 } else { 0.0 };
        let bins = Bins::measure(&parameters, &b1u_etas, &codewords, b1u_at, s.lu, k, &ladder, s.law);
        let crossing = stats.crossing(&ladder, &rx, s.law, gap);
        let landing = Landing::scan(s.law, &ladder, &continuous);
        println!(
            "RUNG drn={drn} rate={} lc={} points={points} gap={gap:.0} leastGap={least} \
             eq=[{}] delay={:+.4} level={:.1} G={:.5} out_rms={out_rms:.4} \
             rms={rms:.1} peak={peak:.0} \
             xing={crossing:.4}% bin={bin:.2}% adj={adj} k={kb:.4} farlev={flev:.2} \
             b1u={ones}/{bits}={pct:.1}% land={land:.3} \
             G9={g9:.9} energy={energy:.6} invG={invg:.3}",
            rung.rate,
            rung.lc,
            used.eq
                .taps
                .iter()
                .map(|c| format!("{c:+.3}"))
                .collect::<Vec<_>>()
                .join(","),
            used.delay,
            used.level_lsb,
            parameters.gain,
            land = landing.weighted(&ladder),
            bin = bins.pct(),
            adj = if bins.aligned { bins.adjacent.to_string() } else { "unaligned".to_owned() },
            kb = bins.k,
            flev = used.level_lsb * far,
            g9 = parameters.gain,
            energy = datapump::v92::sent_prefilter_energy(&parameters.filters.z2),
            invg = 1.0 / parameters.gain,
        );
        // docs/254 step 2: the rung's decision, taken on the instrument that
        // binds at that rung -- the crossing below drn 9, the bin error rate at
        // and above it -- with the level precondition printed beside it,
        // because a grant is conditional on a far end obeying the `G` we send.
        let min_gap = if s.levelset == LevelSet::Code {
            f64::from(least)
        } else {
            rung::gap_lsb(points).unwrap_or(f64::from(least))
        };
        let reading = Reading {
            drn,
            lc: rung.lc,
            min_gap_lsb: min_gap,
            level_lsb: used.level_lsb * far,
            gain: parameters.gain,
            bin: bins.pct() / 100.0,
            crossing: crossing / 100.0,
            symbols: bins.total,
        };
        println!("  {}", reading.line(rung::admit(&reading)));
        if s.chords {
            for chord in (0..8).rev() {
                if stats.rows[chord].n == 0 {
                    continue;
                }
                println!(
                    "  CHORD drn={drn} chord={chord} step={} k_c={} n={} pop={:.1}% \
                     rms/half={:.3} support={:.2} half={:.0} bin={:.1}% nbin={} \
                     land_rms/half={:.3} land_support={:.2} land_pop={:.1}%",
                    ladder.step[chord],
                    per_chord[chord],
                    stats.rows[chord].n,
                    stats.pop(chord),
                    stats.rms_half(chord, &ladder),
                    stats.support(chord, &ladder),
                    f64::from(ladder.step[chord]) / 2.0,
                    bins.chord_pct(chord),
                    bins.sent[chord],
                    landing.rms_half(chord, &ladder),
                    landing.support(chord, &ladder),
                    landing.pop(chord),
                );
            }
        }
        if s.receiver {
            let mut rx = Receiver::new();
            let mut heard = 0;
            for &code in &codewords {
                rx.feed(f64::from(code) / s.lu);
                while rx.heard().is_some() {
                    heard += 1;
                }
            }
            println!(
                "  RECEIVER drn={drn} events={heard} trained={} snr={:.1} dB taps={} \
                 forward={:.4} feedback={:.4}",
                rx.is_trained(),
                rx.snr_db(),
                rx.taps().len(),
                rx.taps().iter().filter(|t| t.abs() > 1e-9).count(),
                rx.feedback().len(),
            );
        }
    }
}
