//! A fixed fractional delay line, ahead of the equaliser.
//!
//! Why it exists: the far end's transmitter shapes its spectrum and the loop
//! may contain a resampler, so the instant that best opens the eye is in
//! general *not* a whole symbol. Until `docs/80` this was argued from a
//! measured phase window; the window turned out to be two defects of ours, so
//! the remaining reason is quality rather than connectivity (`docs/81`,
//! `docs/82`): across the phases a loop can present, the two-point stream reads
//! between 0.0827 and 0.1034 half-cells at 24000, 0.1804 and 0.2404 at 32000,
//! 0.3030 and 0.3835 at 40000. A converter pins the instant to the good end of
//! that spread instead of leaving it where the far end happened to put it.
//!
//! Shape of the thing, fixed by the measurements rather than chosen:
//!
//! * **127 taps.** `docs/79` derived the length from a specification -- group
//!   delay within 0.01 sample and amplitude within 0.1 dB of the requested
//!   delay, worst case over every delta in [0, 1). The length is set by the
//!   **band edge**: the stream reaches 3816 Hz where a real-coefficient FIR
//!   must already be going complex-blind at the 4000 Hz fold, and 3.8 kHz needs
//!   115 taps while 3.82 kHz needs 127. A 65-tap filter measures as costing
//!   1.45 dB, all of it in the band edge: at delta = 0.181 it leaves the group
//!   delay at 3.7 kHz at 0.085 where 0.181 was asked for.
//! * **Kaiser, beta = 8.6.** `docs/79` compared it against Dolph-Chebyshev (113
//!   taps) and a length-tuned Kaiser (119) and kept the fixed beta for its 1.6x
//!   margin and ~90 dB of stopband.
//! * **Fixed, not tracking.** The symbol rate is locked to the network clock,
//!   so there is no frequency offset and no ppm term; the loop delay is a fixed
//!   non-integer number of symbols. The delay is therefore estimated once, at
//!   the first TRN1u, installed, and held. Slow physical drift is left to the
//!   equaliser's own adaptation.
//!
//! At delta = 0 the taps are exactly a unit impulse, so the line is a pure
//! 63-symbol delay and nothing else: `docs/79` showed that integer part is
//! absorbed by the chain, and it is what makes the disabled case and the
//! zero-delta case separately checkable.

use std::collections::VecDeque;

/// Half length in symbols, from `docs/79`. The filter is `2 * FE_HALF + 1` taps.
pub const FE_HALF: usize = 63;

/// Taps in the delay line.
pub const FE_TAPS: usize = 2 * FE_HALF + 1;

/// Kaiser's beta, from `docs/79`.
const BETA: f64 = 8.6;

/// Zeroth-order modified Bessel function of the first kind, by its series. Only
/// ever called with the window's argument, at most `BETA`.
fn i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let half = x / 2.0;
    for k in 1..40 {
        term *= (half / k as f64) * (half / k as f64);
        sum += term;
        if term < 1e-12 * sum {
            break;
        }
    }
    sum
}

/// The windowed sinc for a delay of `delta` symbols: `n - delta` in the
/// argument, so that `delta = 0` is the unit impulse exactly.
#[must_use]
pub fn taps(delta: f64) -> Vec<f64> {
    let norm = i0(BETA);
    let mut h = Vec::with_capacity(FE_TAPS);
    let mut sum = 0.0;
    for k in 0..FE_TAPS {
        let n = k as f64 - FE_HALF as f64;
        let x = n - delta;
        let sinc = if x.abs() < 1e-12 {
            1.0
        } else {
            let a = std::f64::consts::PI * x;
            a.sin() / a
        };
        let r = n / FE_HALF as f64;
        let w = i0(BETA * (1.0 - r * r).max(0.0).sqrt()) / norm;
        let v = sinc * w;
        h.push(v);
        sum += v;
    }
    // Unity gain at DC, which is what keeps a two-point stream two-pointed.
    if sum != 0.0 {
        for v in &mut h {
            *v /= sum;
        }
    }
    h
}

/// Convolve a whole run of samples with the line, zero padding on the left.
///
/// This is what the aim search uses to ask "what would this run look like at
/// another delay" without disturbing the live stream, so it must agree with
/// [`FracDelay::push`] sample for sample.
#[must_use]
pub fn replay(samples: &[f64], delta: f64) -> Vec<f64> {
    let h = taps(delta);
    let mut out = Vec::with_capacity(samples.len());
    for n in 0..samples.len() {
        let mut y = 0.0;
        for k in 0..FE_TAPS {
            if let Some(i) = n.checked_sub(k) {
                y += h[k] * samples[i];
            }
        }
        out.push(y);
    }
    out
}

/// The live line: one sample in, one sample out, at symbol rate.
#[derive(Debug, Clone)]
pub struct FracDelay {
    history: VecDeque<f64>,
    h: Vec<f64>,
    delta: f64,
    /// How many raw samples to keep, so that the aim search can replay the same
    /// indices the receiver's own history uses.
    keep: usize,
}

impl FracDelay {
    /// A line keeping `keep` raw samples, starting at zero delay.
    #[must_use]
    pub fn new(keep: usize) -> Self {
        Self {
            history: VecDeque::with_capacity(keep + FE_TAPS),
            h: taps(0.0),
            delta: 0.0,
            keep,
        }
    }

    /// One symbol in, delayed by `FE_HALF + delta` symbols out.
    pub fn push(&mut self, level: f64) -> f64 {
        self.history.push_back(level);
        let mut y = 0.0;
        for (i, x) in self.history.iter().rev().enumerate() {
            if i >= FE_TAPS {
                break;
            }
            y += self.h[i] * x;
        }
        while self.history.len() > self.keep {
            self.history.pop_front();
        }
        y
    }

    /// The raw stream as the aim search sees it: oldest first, the same
    /// indices as the receiver's own history once it is full.
    #[must_use]
    pub fn raw(&self) -> Vec<f64> {
        self.history.iter().copied().collect()
    }

    /// The delay in symbols being applied.
    #[must_use]
    pub fn delta(&self) -> f64 {
        self.delta
    }

    /// Install a delay and hold it. The taps are rebuilt; at zero they are a
    /// unit impulse, so this is exact rather than approximate.
    pub fn install(&mut self, delta: f64) {
        self.delta = delta;
        self.h = taps(delta);
    }
}
