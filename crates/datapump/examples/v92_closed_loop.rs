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
use datapump::v92::farend::{self, FarEndParams};
use datapump::v92::receiver::Receiver;
use datapump::v92::up_signals::Trn1u;
use datapump::v92::upstream::UpEncoder;
use datapump::v92::{
    CONSTELLATION_FRAME, Filters, LevelSet, Parameters, Trellis, UP_INTERVALS, up_bits,
};

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

/// The rung's parameters, exactly as `docs/248`'s generator builds them, with
/// the level set named instead of fixed: `docs/250` §2's gap is this argument.
fn parameters(drn: u8, trellis: Trellis, levelset: LevelSet) -> Parameters {
    let bits = up_bits(drn);
    let each = bits / UP_INTERVALS as u32;
    let spare = bits % UP_INTERVALS as u32;
    let mut moduli = [0u8; UP_INTERVALS];
    let mut levels = 1usize;
    for i in 0..UP_INTERVALS {
        let width = each + u32::from((i as u32) < spare);
        let modulus = 1u8 << width;
        moduli[i] = modulus;
        let need = if i % 4 == 3 { usize::from(modulus) } else { usize::from(modulus).div_ceil(2) };
        levels = levels.max(need);
    }
    let set = levelset.points(levels).expect("a level set for this rung");
    let gain = levelset.gain(&set);
    Parameters {
        drn,
        trellis,
        extend_e2u: false,
        gain,
        moduli,
        filters: Filters { z2: vec![1.0], ..Filters::default() },
        sets: vec![set],
        indices: [0; CONSTELLATION_FRAME],
        levelset,
    }
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
    receiver: bool,
    /// `--mode sweep`: the error put on one parameter and nothing else.
    err_delay: Option<f64>,
    err_level: Option<f64>,
    err_tap: Option<f64>,
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
        receiver: false,
        err_delay: None,
        err_level: None,
        err_tap: None,
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
            "--receiver" => s.receiver = true,
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
) -> (Vec<f64>, Vec<i32>) {
    let placed = params.apply(symbols);
    let sampled =
        if timing == 0.0 { placed.clone() } else { farend::shift_centred(&placed, timing) };
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
fn stream(s: &Settings) -> (Parameters, Vec<f64>, Vec<f64>, usize) {
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
    for symbol in encoder.b1u() {
        out.push(symbol.out);
    }
    for _ in 0..s.frames * UP_INTERVALS {
        out.push(encoder.next_symbol(|| random.next()).out);
    }
    (parameters, out, ideal_trn, b1u_at)
}

/// The B1u count in the bench's own terms: bits decoded, how many of them are
/// ones, and the class rotation the decoder measured.
fn b1u(parameters: &Parameters, codewords: &[i32], at: usize, lu: f64) -> (usize, usize, String) {
    let mut decoder = Decoder::new(parameters);
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

        let (parameters, out, ideal_trn, b1u_at) = stream(&Settings { drn, ..s.clone() });
        let line_lsb: Vec<f64> = out.iter().map(|&v| v * s.lu).collect();
        let _ = &line_lsb;

        // --- the loop -----------------------------------------------------
        let (used, estimated) = match s.mode.as_str() {
            "closed" => {
                // Pass 1: the far end is on the line it chose.
                let start = FarEndParams::bypass(s.lu);
                let (_, first) = transmit(&out, &start, s.timing, s.tap, s.law);
                let observed: Vec<f64> = first.iter().map(|&q| f64::from(q)).collect();
                let e = farend::estimate(
                    &ideal_trn,
                    &observed[..ideal_trn.len()],
                    0..8,
                );
                let correction = e.correction(s.lu, s.lu);
                println!(
                    "  estimate drn={drn} delay={:+.4} level={:.1} tap={:+.4} nmse={:.5} -> \
                     params delay={:+.4} level={:.1} eq=[{:.3},{:+.3},{:+.3}]",
                    e.delay,
                    e.level,
                    e.tap,
                    e.nmse,
                    correction.delay,
                    correction.level_lsb,
                    correction.eq.taps[0],
                    correction.eq.taps[1],
                    correction.eq.taps[2]
                );
                (correction, Some(e))
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

        let (continuous, codewords) = transmit(&out, &used, s.timing, s.tap, s.law);
        let rms = (continuous.iter().map(|v| v * v).sum::<f64>() / continuous.len() as f64).sqrt();
        let peak = continuous.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let out_rms = (out.iter().map(|v| v * v).sum::<f64>() / out.len() as f64).sqrt();
        let rx: Vec<f64> = codewords.iter().map(|&q| f64::from(q)).collect();
        let stats = Stats::scan(s.law, &ladder, &rx);
        let (ones, bits, rotation) = b1u(&parameters, &codewords, b1u_at, s.lu);
        let pct = if bits > 0 { 100.0 * ones as f64 / bits as f64 } else { 0.0 };
        let crossing = stats.crossing(&ladder, &rx, s.law, gap);
        println!(
            "RUNG drn={drn} rate={} lc={} points={points} gap={gap:.0} leastGap={least} \
             eq=[{:.3},{:+.3},{:+.3}] delay={:+.4} level={:.1} G={:.5} out_rms={out_rms:.4} \
             rms={rms:.1} peak={peak:.0} pop1024={pop1024:.1} pop512={pop512:.1} \
             xing={crossing:.4}% b1u={ones}/{bits}={pct:.1}% rot={rotation}",
            rung.rate,
            rung.lc,
            used.eq.taps[0],
            used.eq.taps[1],
            used.eq.taps[2],
            used.delay,
            used.level_lsb,
            parameters.gain,
            pop1024 = stats.pop(7),
            pop512 = stats.pop(6),
        );
        if s.chords {
            for chord in (0..8).rev() {
                if stats.rows[chord].n == 0 {
                    continue;
                }
                println!(
                    "  CHORD drn={drn} chord={chord} step={} k_c={} n={} pop={:.1}% \
                     rms/half={:.3} support={:.2} half={:.0}",
                    ladder.step[chord],
                    per_chord[chord],
                    stats.rows[chord].n,
                    stats.pop(chord),
                    stats.rms_half(chord, &ladder),
                    stats.support(chord, &ladder),
                    f64::from(ladder.step[chord]) / 2.0,
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
