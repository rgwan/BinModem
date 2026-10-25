//! What the 4D code is worth on the channel we measured: Viterbi against slice.
//!
//! `docs/108` puts the residual at the slicer at **σ = 21.3 LSB** (0.122 of the
//! 24 000 half-grid, one half-grid being 174.5 LSB) and names **σ = 8 LSB**
//! (0.046 half-grids) as the post-code-centre target. `docs/96` §25.3 and
//! `docs/108` §7 both say the same thing about what would settle 40 000: a
//! coding gain of 3-4 dB. This measures the only candidate that is already
//! written down in the spec, 9.6.3.2's 16-state code, on a synthetic channel
//! with exactly that noise.
//!
//! The channel is our own transmitter into two of our own receivers:
//!
//! ```text
//! bits -> UpEncoder -> G x a(eta) -> + N(0, sigma) on the u scale -> both decoders
//! ```
//!
//! σ is a fixed *absolute* displacement of the wire level, because that is what
//! `docs/108` measures: 21.3 LSB is a property of the line, and a rung with more
//! levels puts its levels closer together. That is 0.06 of a level step at
//! 24 000 and 0.52 of one at 48 000 -- which is the whole reason `docs/108` §7
//! says what it says about the top two rungs.
//!
//! Two numbers per rung and per σ:
//!
//! * **FER** -- the fraction of trellis frames (four symbols) whose four decided
//!   indices are not the four that were sent. This is the frame the code works
//!   on, so it is where a coding gain shows up undiluted.
//! * **BER** -- the same over indices rather than frames.
//!
//! And the gain itself, for σ = 21.3 LSB: the σ at which the **slicing**
//! decoder's FER equals the **Viterbi**'s FER at that σ, in dB. That is the
//! number `docs/108` §7 asks for, in the units it asks for it.
//!
//! Usage: `cargo run --release -p datapump --example v92_trellis_gain`
//!
//! **What this cannot say.** It is a synthetic channel with white Gaussian noise
//! and no intersymbol interference: it prices the code, not the call. A real
//! line's residual is not white (docs/108 measures it per node and per symbol),
//! and the equaliser's own error propagation is not in here at all.

use datapump::v92::decoder::{Decoder, TrellisMode};
use datapump::v92::upstream::UpEncoder;
use datapump::v92::{CONSTELLATION_FRAME, Filters, Parameters, TRELLIS_FRAME, Trellis, UP_INTERVALS, up_bits};

/// `docs/108` §0: the measured residual, in LSB level units.
const SIGMA_MEASURED_LSB: f64 = 21.3;
/// `docs/108`: the post-code-centre target.
const SIGMA_TARGET_LSB: f64 = 8.0;
/// `docs/108` §1: the 24 000 lattice step is **349 LSB**, and in the domain the
/// chain's `out` is in, a 24 000 level step is `2G = 16/15` (eight positive
/// levels, `G = 8/15`). So one LSB is `(16/15)/349` of a wire unit, and that
/// conversion is **rung-independent**: the noise is the line's, not the
/// constellation's. What changes across the rungs is the level step, which
/// shrinks in those same absolute units as the rung carries more bits -- eight
/// positive levels and a step of `16/15` at 24 000, sixty-four of them and a
/// step of `16/127` at 48 000 -- so `docs/108`'s 21.3 LSB is 0.06 of a level
/// step at 24 000 and 0.52 of one at 48 000.
const LSB_PER_STEP: f64 = 349.0;
const STEP_AT_24000: f64 = 16.0 / 15.0;

/// `docs/108`'s LSB figure on the wire scale the chain's `out` is in.
fn lsb_to_wire(lsb: f64) -> f64 {
    lsb / LSB_PER_STEP * STEP_AT_24000
}

/// How many trellis frames of payload a measurement averages over. 40 000 of
/// them is 160 000 symbols: forty frame errors at an FER of 1e-3, which is what
/// a shape-only number needs, and a couple of seconds a point.
const FRAMES: usize = 40_000;

/// The frame error rates the coding gain is quoted at: the two an outer
/// retransmission layer would care about.
const GAIN_TARGETS: [f64; 2] = [1e-2, 1e-3];

/// Parameters for the rung: `levels = Mi` positive points spaced by two, every
/// interval on modulus `Mi`, no precoder and a prefilter of unity gain, so the
/// wire carries `G a(eta)` and nothing else. `G` is the largest the chain's own
/// output limit allows, which keeps the transmission out of the clamp.
fn parameters(drn: u8, levels: usize) -> Parameters {
    #[allow(clippy::cast_possible_truncation)]
    let modulus = 1u8 << (up_bits(drn) / UP_INTERVALS as u32);
    Parameters {
        drn,
        trellis: Trellis::Sixteen,
        extend_e2u: false,
        gain: 8.0 / (2.0 * levels as f64 - 1.0),
        moduli: [modulus; UP_INTERVALS],
        filters: Filters { z2: vec![1.0], ..Filters::default() },
        sets: vec![(0..levels).map(|index| (2 * index + 1) as u16).collect()],
        indices: [0; CONSTELLATION_FRAME],
        levelset: datapump::v92::LevelSet::Uniform,
    }
}

/// A deterministic source of bits and of Gaussian noise: no crate, and the same
/// channel on every run, so the two decoders are always compared on the same
/// noise.
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

    /// A uniform in (0, 1), which is all Box-Muller needs.
    fn uniform(&mut self) -> f64 {
        ((self.word() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// One standard normal deviate, Box-Muller.
    fn normal(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// The two decoders' decisions over one noisy channel.
struct Measured {
    /// Fraction of trellis frames whose four indices are not the four sent.
    frame_error: f64,
    /// Fraction of indices that are not the index sent.
    index_error: f64,
    frames: usize,
}

/// Send `FRAMES` trellis frames through the chain and both decisions, with `σ`
/// on the `u` scale.
fn measure(drn: u8, levels: usize, sigma: f64, seed: u64, mode: TrellisMode) -> Measured {
    let parameters = parameters(drn, levels);
    let mut encoder = UpEncoder::new(&parameters).expect("the parameters are feasible");
    let mut decoder = Decoder::with_trellis(&parameters, mode);
    decoder.keep_decisions(true);
    let mut random = Random::new(seed);
    let mut sent: Vec<[i32; TRELLIS_FRAME]> = Vec::with_capacity(FRAMES);
    // B1u first, so that the decoder is fed from the first symbol of it and the
    // memories are the ones a call's are (8.7.1); then the data frames.
    let mut symbols = encoder.b1u();
    for _ in 0..FRAMES * TRELLIS_FRAME {
        symbols.push(encoder.next_symbol(|| random.next()));
    }
    for symbol in &symbols {
        // The noise is added on the wire, after the gain: a level there is
        // `G a(eta)` and `sigma` is an absolute displacement of it.
        decoder.feed(parameters.gain * symbol.u + sigma * random.normal());
    }
    // The transmitted frames, in the order the encoder produced them.
    for frame in 0..symbols.len() / TRELLIS_FRAME {
        let at = frame * TRELLIS_FRAME;
        sent.push([symbols[at].eta, symbols[at + 1].eta, symbols[at + 2].eta, symbols[at + 3].eta]);
    }
    let decided = decoder.take_decisions();
    assert!(!decided.is_empty(), "nothing was decided");
    // The decisions are the frames this decoder was handed, oldest first: the
    // window only delays which frame is decided *when*, not the order.
    let mut bad_frames = 0usize;
    let mut bad_indices = 0usize;
    let mut indices = 0usize;
    for (frame, decided) in decided.iter().enumerate() {
        let Some(want) = sent.get(frame) else { break };
        let wrong = (0..TRELLIS_FRAME).filter(|&k| decided[k] != want[k]).count();
        if wrong > 0 {
            bad_frames += 1;
        }
        bad_indices += wrong;
        indices += TRELLIS_FRAME;
    }
    Measured {
        frame_error: bad_frames as f64 / decided.len() as f64,
        index_error: bad_indices as f64 / indices as f64,
        frames: decided.len(),
    }
}

fn main() {
    // drn 1, 7, 13 and 19 are 24 000, 32 000, 40 000 and 48 000 bit/s
    // (`up_bits`), and Mi = K/12 is their positive level count.
    let rungs = [(1u8, 24_000u32), (7, 32_000), (13, 40_000), (19, 48_000)];
    println!("noise is docs/108's, an absolute displacement of the wire level (349 LSB to a 24 000 step)");
    println!("{FRAMES} trellis frames per point; the two decisions see the same noise everywhere\n");
    for (drn, rate) in rungs {
        let levels = 1usize << (up_bits(drn) / UP_INTERVALS as u32);
        println!(
            "{rate} bit/s (drn {drn}): K = {}, Mi = {}, {levels} positive levels ({} points), level step {:.3} wire units",
            up_bits(drn),
            1u32 << (up_bits(drn) / UP_INTERVALS as u32),
            2 * levels,
            16.0 / (2.0 * levels as f64 - 1.0)
        );
        println!("  {:>9}  {:>9}  {:>9}  {:>9}  {:>9}", "sigma LSB", "slice FER", "vit FER", "slice BER", "vit BER");
        for (name, lsb) in [("measured", SIGMA_MEASURED_LSB), ("target", SIGMA_TARGET_LSB)] {
            let sigma = lsb_to_wire(lsb);
            let sliced = measure(drn, levels, sigma, 0x5EED_1234, TrellisMode::Slice);
            let viterbi = measure(drn, levels, sigma, 0x5EED_1234, TrellisMode::Viterbi);
            assert_eq!(sliced.frames, viterbi.frames, "the two decided a different number of frames");
            assert!(viterbi.frames >= FRAMES, "only {} frames were decided", viterbi.frames);
            println!(
                "  {:>9}  {:>9.5}  {:>9.5}  {:>9.6}  {:>9.6}   {name}",
                format!("{lsb:.1}"), sliced.frame_error, viterbi.frame_error, sliced.index_error, viterbi.index_error
            );
        }
        // The coding gain: the noise each decision needs to reach the same
        // frame error rate, in dB. A quarter of a level step is where the
        // slice starts to fail, so a target it can never reach is reported as
        // such rather than as a number the bisection invented.
        for target in GAIN_TARGETS {
            let sliced = noise_for(drn, levels, target, TrellisMode::Slice);
            let viterbi = noise_for(drn, levels, target, TrellisMode::Viterbi);
            match (sliced, viterbi) {
                (Some(sliced), Some(viterbi)) => println!(
                    "  FER {target:>6.0e}: slice needs {:.1} LSB of noise, viterbi {:.1} LSB -> coding gain {:+.2} dB",
                    sliced / STEP_AT_24000 * LSB_PER_STEP,
                    viterbi / STEP_AT_24000 * LSB_PER_STEP,
                    // The signal is the same at both points and only the noise
                    // moved, so a ratio of sigmas is a ratio of amplitudes and
                    // 20 log10 of it is decibels of SNR.
                    20.0 * (viterbi / sliced).log10()
                ),
                (slice, viterbi) => println!(
                    "  FER {target:>6.0e}: not reachable on this channel (slice {}, viterbi {})",
                    if slice.is_some() { "yes" } else { "no" },
                    if viterbi.is_some() { "yes" } else { "no" }
                ),
            }
        }
        println!();
    }
}

/// The wire noise at which `mode` reaches `target` frame error rate, by
/// bisection on a bracket grown until it holds the crossing.
///
/// `None` when the decision cannot reach the target at all: the Viterbi at an
/// FER of 1e-3 on a 128-point set would need a channel that never misses, and
/// the bisection would otherwise return the edge of its bracket as if it meant
/// something.
fn noise_for(drn: u8, levels: usize, target: f64, mode: TrellisMode) -> Option<f64> {
    // A channel of one LSB is the quietest this measures: a decision that
    // still misses the target there cannot reach it at all.
    let mut low = lsb_to_wire(1.0);
    if measure(drn, levels, low, 0x5EED_1234, mode).frame_error > target {
        return None;
    }
    // Grow the bracket until the rate is over the target. It always is at some
    // noise, because a level step's worth of noise leaves nothing decidable.
    let mut high = low;
    loop {
        high *= 2.0;
        if high > lsb_to_wire(4_000.0) {
            return None;
        }
        if measure(drn, levels, high, 0x5EED_1234, mode).frame_error > target {
            break;
        }
        low = high;
    }
    for _ in 0..12 {
        let middle = 0.5 * (low + high);
        if measure(drn, levels, middle, 0x5EED_1234, mode).frame_error <= target {
            low = middle;
        } else {
            high = middle;
        }
    }
    Some(0.5 * (low + high))
}
