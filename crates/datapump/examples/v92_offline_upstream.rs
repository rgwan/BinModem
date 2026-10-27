//! The analogue modem's upstream, generated statically, put through a modelled
//! central office, and written as `s16le` at 8000 Hz -- the same shape of file
//! the bench writes as `agr-tx.raw`, so every tool that reads a capture reads
//! this one too (`tools/ucode_chords.py`, `tools/ucode_margin.py`, the
//! `v92_capture` diagnostics).
//!
//! ```text
//! bits --GPA--> modulus encoder --> precoder/prefilter/trellis (G x v(n))
//!      --> fractional sampling instant --> level --> residual ISI tap
//!      --> G.711 quantiser (mu/A) --> s16le
//! ```
//!
//! **The transmitter is `v92::upstream::UpEncoder`, not a new one.** It is the
//! one place in this crate where 6.3's scrambler, 6.4.1's twelve-interval
//! modulus encoder and 6.4.2-6.4.4's precoder, equivalence classes and 4D code
//! are strung together in the order the Recommendation prints them
//! (`upstream.rs:1-15`), so driving it is driving the standard's chain and not a
//! second implementation of it. Nothing here is derived from the receiver.
//!
//! Usage:
//! ```text
//! cargo run --release -p datapump --example v92_offline_upstream -- \
//!     --drn 1 --frames 4000 --timing 0.3 --tap 0.24 --level 0.0 \
//!     --law mu --raw /tmp/syn.raw
//! ```

use datapump::v92::upstream::UpEncoder;
use datapump::v90::ucode::{self, Law};
use datapump::v92::{CONSTELLATION_FRAME, Filters, Parameters, Trellis, UP_INTERVALS, up_bits};
use datapump::v92::{fracdelay, up_signals::Trn1u};

/// A deterministic bit source and noise source. No crate, same stream every run.
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
    fn uniform(&mut self) -> f64 {
        ((self.word() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// The rung's parameters, in the shape `v92_trellis_gain` uses: every interval
/// on its own modulus (6.4.1), `LC` positive points on the odd lattice
/// `a(eta) = 2 eta + 1` (6.4.2's "indices in the same order as the levels"), a
/// prefilter of unity gain and no precoder feedback -- the far end's own
/// prefilter is the *digital* modem's design and is dialled separately here.
fn parameters(drn: u8, trellis: Trellis) -> Parameters {
    let bits = up_bits(drn);
    let each = bits / UP_INTERVALS as u32;
    let spare = bits % UP_INTERVALS as u32;
    let mut moduli = [0u8; UP_INTERVALS];
    let mut lc = 1usize;
    for i in 0..UP_INTERVALS {
        let width = each + u32::from((i as u32) < spare);
        let modulus = 1u8 << width;
        moduli[i] = modulus;
        let need = if i % 4 == 3 { usize::from(modulus) } else { usize::from(modulus).div_ceil(2) };
        lc = lc.max(need);
    }
    Parameters {
        drn,
        trellis,
        extend_e2u: false,
        gain: 8.0 / (2.0 * lc as f64 - 1.0),
        moduli,
        filters: Filters { z2: vec![1.0], ..Filters::default() },
        sets: vec![(0..lc).map(|index| (2 * index + 1) as u16).collect()],
        indices: [0; CONSTELLATION_FRAME],
        levelset: datapump::v92::LevelSet::Uniform,
        // No `CPu` has named a companding law on this run, and none can: this
        // model builds the odd lattice `a(eta) = 2 eta + 1` itself, so its set
        // is levels in index units and not G.711 codes. `None` is the state
        // "no law declared" (`docs/313` sec 1), which is the truth here -- it is
        // not a µ-law default, and `LevelSet::Uniform` needs no law.
        law: None,
    }
}

struct Settings {
    drn: u8,
    frames: usize,
    timing: f64,
    tap: f64,
    level_db: f64,
    /// `out` unit (mean square 1, 3.8) in the codec's 16-bit LSB.
    lu_lsb: f64,
    noise_lsb: f64,
    law: Law,
    seed: u64,
    raw: Option<String>,
    rx_raw: Option<String>,
    trellis: bool,
}

fn parse() -> Settings {
    let mut s = Settings {
        drn: 1,
        frames: 4_000,
        timing: 0.3,
        tap: 0.24,
        level_db: 0.0,
        lu_lsb: 1_600.0,
        noise_lsb: 0.0,
        law: Law::Mu,
        seed: 1,
        raw: None,
        rx_raw: None,
        trellis: true,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().expect("a value after the option");
        match arg.as_str() {
            "--drn" => s.drn = value().parse().expect("drn"),
            "--frames" => s.frames = value().parse().expect("frames"),
            "--timing" => s.timing = value().parse().expect("timing"),
            "--tap" => s.tap = value().parse().expect("tap"),
            "--level" => s.level_db = value().parse().expect("level dB"),
            "--lu" => s.lu_lsb = value().parse().expect("LU in LSB"),
            "--noise" => s.noise_lsb = value().parse().expect("noise LSB"),
            "--law" => s.law = if value() == "a" { Law::A } else { Law::Mu },
            "--seed" => s.seed = value().parse().expect("seed"),
            "--raw" => s.raw = Some(value()),
            "--rx-raw" => s.rx_raw = Some(value()),
            "--no-trellis" => s.trellis = false,
            other => panic!("unknown option {other}"),
        }
    }
    s
}

/// The G.711 quantiser the central office is: nearest code of `law`, then that
/// code's Table 1 amplitude back in LSB. This is the only nonlinear element and
/// it is not an impairment -- `docs/244` settles it as physical.
fn quantise(law: Law, lsb: f64) -> i32 {
    let full = lsb.clamp(-32768.0, 32767.0).round() as i32;
    let (code, negative) = ucode::nearest(law, full.abs().min(32767));
    let level = ucode::linear(law, code).abs();
    if negative || full < 0 { -level } else { level }
}

fn main() {
    let s = parse();
    // V.92 always codes: CPd bits 27:28 name a code and there is no "off" (mod.rs:405-411).
    let trellis = if s.trellis { Trellis::Sixteen } else { Trellis::Sixteen };
    let parameters = parameters(s.drn, trellis);
    let mut encoder = UpEncoder::new(&parameters).expect("feasible parameters");
    let mut random = Random::new(s.seed);

    // 8.6.2's TRN1u first, then B1u (8.7.1), then data frames: the order a call
    // presents them, so the receiver takes the stream it takes from a wire.
    let mut out: Vec<f64> = Vec::new();
    let mut trn = Trn1u::new();
    for _ in 0..2_040 {
        out.push(trn.next_symbol());
    }
    for symbol in encoder.b1u() {
        out.push(symbol.out);
    }
    for _ in 0..s.frames * UP_INTERVALS {
        out.push(encoder.next_symbol(|| random.next()).out);
    }

    // The CO's sampling instant, in fractions of a symbol: the one impairment
    // that decides whether the far end lands on the instant at all.
    let sampled = fracdelay::replay(&out, s.timing);

    let scale = s.lu_lsb * 10f64.powf(s.level_db / 20.0);
    let mut lsb: Vec<f64> = Vec::with_capacity(sampled.len());
    let mut previous = 0.0;
    for &x in &sampled {
        // Residual ISI at the sampling instant: one dialable tap, `tap` of the
        // previous symbol added to this one. `0.0` is a pure quantiser.
        let y = x + s.tap * previous;
        previous = x;
        let mut z = y * scale;
        if s.noise_lsb > 0.0 {
            z += s.noise_lsb * random.normal();
        }
        lsb.push(z);
    }
    // The far end's own transmit stream: continuous, and what `docs/244`
    // measures its residual against (`agr-tx.raw` is exactly this, not the
    // codec's output). Then the codec's output, which is what the receiver gets.
    let continuous: Vec<i32> = lsb
        .iter()
        .map(|&z| z.clamp(-32768.0, 32767.0).round() as i32)
        .collect();
    let quantised: Vec<i32> = lsb.iter().map(|&z| quantise(s.law, z)).collect();

    let rms = (continuous.iter().map(|&q| f64::from(q).powi(2)).sum::<f64>()
        / continuous.len() as f64)
        .sqrt();
    eprintln!(
        "{} symbols (2040 TRN1u + 576 B1u + {} data), drn {} K {} moduli {:?} G {:.4}",
        quantised.len(),
        s.frames * UP_INTERVALS,
        s.drn,
        up_bits(s.drn),
        parameters.moduli,
        parameters.gain
    );
    eprintln!(
        "channel: timing {:+.3} symbol, tap {}, level {:+.1} dB, LU {} LSB, {} , noise {} LSB",
        s.timing,
        s.tap,
        s.level_db,
        s.lu_lsb,
        if s.law == Law::Mu { "mu-law" } else { "A-law" },
        s.noise_lsb
    );
    eprintln!(
        "transmit rms {rms:.2} LSB, peak {}",
        continuous.iter().map(|q| q.abs()).max().unwrap_or(0)
    );

    if let Some(path) = &s.rx_raw {
        let mut bytes = Vec::with_capacity(quantised.len() * 2);
        for q in &quantised {
            bytes.extend_from_slice(&(i16::try_from(*q).unwrap_or(if *q < 0 { i16::MIN } else { i16::MAX })).to_le_bytes());
        }
        std::fs::write(path, &bytes).expect("write");
        eprintln!("wrote {} bytes to {path}", bytes.len());
    }
    if let Some(path) = &s.raw {
        let mut bytes = Vec::with_capacity(continuous.len() * 2);
        for q in &continuous {
            bytes.extend_from_slice(&(i16::try_from(*q).unwrap_or(if *q < 0 { i16::MIN } else { i16::MAX })).to_le_bytes());
        }
        std::fs::write(path, &bytes).expect("write");
        eprintln!("wrote {} bytes to {path}", bytes.len());
    }
}
