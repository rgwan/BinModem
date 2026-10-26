//! The closed loop's parameter set: **our end measures, the far end applies**.
//!
//! Everything the digital modem can ask an analogue modem to change about its
//! *transmitter* is, in this note, one small named struct. It exists because
//! the measurement (`docs/248`) says the residual is dominated by two things
//! that are the far end's to get right -- the fractional sampling instant and
//! the transmit level -- and by one thing that is the channel's but the far end
//! can pre-correct, its short post-cursor ISI. Naming them together, with their
//! units, makes the loop testable: estimator -> [`FarEndParams`] -> transmitter
//! -> codewords -> receiver -> the same per-chord statistic it started from.
//!
//! # What is confirmed and what is not
//!
//! V.92 has fields for the *constellation* (CPd's points, `G`, the moduli) and
//! none of them for a delay, a level or an equaliser tap. The delay in
//! particular: `docs/249` (in flight) is the note that establishes which field,
//! if any, carries it, and this module **does not wait for it and does not
//! invent one**. Each parameter below therefore states its own evidence tier:
//! what the clause fixes is quoted, what this crate already sends is
//! [verified], and what has no field yet is **[our choice] / unconfirmed**.
//! Nothing here is a protocol change: the struct is a model of what the far end
//! adjusts, not a bit layout.
//!
//! # The bypass rule
//!
//! The equaliser's verdict on this bench is that it is not what the residual is
//! made of, but the equaliser stays in the code path with the data path
//! bypassing it (the user's design rule). [`EqTaps::bypass`] is that bypass,
//! and it is **bit-exact**: [`FarEndParams::apply`] with a bypassing parameter
//! set is a multiply by the level and nothing else, and [`EqTaps::gain`] is
//! `1.0` and [`EqTaps::forward`] is `0.0` by construction, so a check that
//! compares a bypassed run against a no-equaliser reference compares bit
//! patterns and not a tolerance.

use super::fracdelay::{self, FE_HALF, FE_TAPS};

/// The evidence tier of one parameter, kept as data so the deliverable's table
/// is generated from the same strings the code carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tier {
    /// A clause of the Recommendation fixes it, with the line number.
    Clause,
    /// The crate already sends or uses it.
    Verified,
    /// Ours, with no field established yet (docs/249 pending).
    OurChoiceUnconfirmed,
}

impl Tier {
    /// The marker the note prints for this tier.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Tier::Clause => "[quoted]",
            Tier::Verified => "[verified]",
            Tier::OurChoiceUnconfirmed => "[our choice] / unconfirmed",
        }
    }
}

/// One row of the parameter table: meaning, physical unit, the range a real far
/// end could implement, the field or clause it would ride in, and its tier.
#[derive(Clone, Copy, Debug)]
pub struct FieldSpec {
    /// The member of [`FarEndParams`] this row describes.
    pub name: &'static str,
    /// What it means, in one sentence.
    pub meaning: &'static str,
    /// Its physical unit.
    pub unit: &'static str,
    /// What a far end could implement -- a hardware bound, not a design goal.
    pub range: &'static str,
    /// The clause or patent field it would be carried in, or `None` when the
    /// clause is not established (`docs/249`).
    pub field: Option<&'static str>,
    /// How well established the row is.
    pub tier: Tier,
}

/// The three rows of the parameter table, as data.
///
/// The order is the order of the members of [`FarEndParams`].
pub const FIELDS: [FieldSpec; 3] = [
    FieldSpec {
        name: "delay",
        meaning: "fractional position of the far end's symbol clock against the digital \
                  modem's sampling instant; positive is later",
        unit: "symbol (T)",
        range: "-0.5 .. +0.5; beyond half a symbol the adjacent instant is closer",
        field: None,
        tier: Tier::OurChoiceUnconfirmed,
    },
    FieldSpec {
        name: "level_lsb",
        meaning: "transmit level that makes one unit of `G x v(n)` (8.8.3's mean square 1) \
                  arrive as this many G.711 LSB",
        unit: "LSB (16-bit linear, G.711 Table 1)",
        range: "300 .. 3000 LSB about a 900 LSB reference (-9.5 .. +10.5 dB); a codec's \
                own range is 0 .. 32124",
        field: Some("8.8.3's power convention -- `ms(v G) = 1` is the desired power -- \
                     with CPd's `G` (bits 35:50) scaling it"),
        tier: Tier::Verified,
    },
    FieldSpec {
        name: "eq",
        meaning: "3-tap pre-emphasis the far end applies to `G x v(n)` before its DAC -- \
                  the inverse of the channel's short post-cursor response",
        unit: "dimensionless tap weights, T spaced",
        range: "|c_k| <= 1.0 and DC gain 0.5 .. 2.0; three taps is what the measured ISI \
                needs and what a transmitter's shaping filter has room for",
        field: None,
        tier: Tier::OurChoiceUnconfirmed,
    },
];

/// What the far end's transmitter pre-applies: `y(n) = c0 x(n) + c1 x(n-1) +
/// c2 x(n-2)`.
///
/// Three taps because the residual ISI is one post-cursor tap (`docs/248`'s
/// `--tap`), and the best three-tap inverse of `1 + a z^-1` is
/// `1 - a z^-1 + a^2 z^-2`: its product with the channel is `1 + a^3 z^-3`, so
/// a one-tap channel is cancelled down to `a^3` -- 1.4 % of the tap at
/// `a = 0.24`, and nothing at the level the quantiser works at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EqTaps {
    /// `c0, c1, c2`.
    pub taps: [f64; 3],
}

impl EqTaps {
    /// The extreme case the user's design rule is about: no equaliser at all.
    pub const BYPASS: Self = Self { taps: [1.0, 0.0, 0.0] };

    /// The three-tap inverse of the channel `1 + a z^-1`, as the far end would
    /// pre-apply it: `1 - a z^-1 + a^2 z^-2`, whose product with the channel is
    /// `1 + a^3 z^-3`.
    #[must_use]
    pub fn inverse_of(a: f64) -> Self {
        Self { taps: [1.0, -a, a * a] }
    }

    /// Whether the data path costs nothing: this is the predicate the
    /// activation census is taken against.
    #[must_use]
    pub fn is_bypass(&self) -> bool {
        self.taps == Self::BYPASS.taps
    }

    /// DC gain, `c0 + c1 + c2`: `1.0` for the bypass, and the number a check
    /// compares first.
    #[must_use]
    pub fn gain(&self) -> f64 {
        self.taps.iter().sum()
    }

    /// The weight at non-zero lags, `c1 + c2`: `0.0` for the bypass, and the
    /// second number a check compares.
    #[must_use]
    pub fn forward(&self) -> f64 {
        self.taps[1] + self.taps[2]
    }

    /// `y(n) = c0 x(n) + c1 x(n-1) + c2 x(n-2)`, causal, memory zeroed before
    /// the first symbol (the far end starts from B1u's own reset, 8.7.1).
    #[must_use]
    pub fn filter(&self, symbols: &[f64]) -> Vec<f64> {
        if self.is_bypass() {
            return symbols.to_vec();
        }
        let mut out = Vec::with_capacity(symbols.len());
        for n in 0..symbols.len() {
            let mut y = 0.0;
            for (k, &c) in self.taps.iter().enumerate() {
                if let Some(i) = n.checked_sub(k) {
                    y += c * symbols[i];
                }
            }
            out.push(y);
        }
        out
    }

    /// What the pre-emphasis spends in the worst case: the sum of the tap
    /// magnitudes, which bounds its gain to any input.
    #[must_use]
    pub fn peak(&self) -> f64 {
        self.taps.iter().map(|c| c.abs()).sum()
    }
}

/// What our end tells the far end to do. Units are the [`FIELDS`] table's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FarEndParams {
    /// Fractional symbol shift the far end applies to its own transmit clock;
    /// positive is later. This is the *correction*, so the value that cancels a
    /// measured sampling instant `d` is `-d`.
    pub delay: f64,
    /// `out` unit -> LSB: the far end's transmit level at 8.8.3's
    /// mean-square-1 point.
    pub level_lsb: f64,
    /// The pre-emphasis the far end pre-applies.
    pub eq: EqTaps,
}

impl FarEndParams {
    /// The parameter set that changes nothing: the level is the reference and
    /// the equaliser is bypassed. Applying it is a multiply and nothing else.
    #[must_use]
    pub fn bypass(level_lsb: f64) -> Self {
        Self { delay: 0.0, level_lsb, eq: EqTaps::BYPASS }
    }

    /// Whether the data path has neither a delay nor a filter in it.
    #[must_use]
    pub fn is_bypass(&self) -> bool {
        self.delay == 0.0 && self.eq.is_bypass()
    }

    /// The far end's transmit stream in LSB: pre-emphasis, then the fractional
    /// shift, then the level.
    ///
    /// The shift is **centred** and so has no constant delay of its own: the
    /// taps of `fracdelay::taps(delay)` are laid around the sample they belong
    /// to, `y(n) = sum_m h(m) x(n - m)` with `m` in `-FE_HALF .. FE_HALF`. A
    /// generator has the whole frame before its DAC; paying `FE_HALF` symbols of
    /// latency for a fractional shift would be a modelling artefact and not a
    /// transmitter property.
    #[must_use]
    pub fn apply(&self, symbols: &[f64]) -> Vec<f64> {
        let shaped = self.eq.filter(symbols);
        // The bypass must be arithmetic-free: no windowed sinc, no rounding.
        let placed = if self.delay == 0.0 { shaped } else { shift_centred(&shaped, self.delay) };
        placed.iter().map(|&v| v * self.level_lsb).collect()
    }
}

/// `y(n) = sum_m h(m) x(n - m)` with `h` a windowed sinc peaking at `m = delta`
/// and support `-FE_HALF .. FE_HALF`: `x` shifted *later* by `delta` with no net
/// delay.
#[must_use]
pub fn shift_centred(samples: &[f64], delta: f64) -> Vec<f64> {
    let h = fracdelay::taps(delta);
    let half = FE_HALF as isize;
    let mut out = Vec::with_capacity(samples.len());
    for n in 0..samples.len() {
        let mut y = 0.0;
        for (k, &coeff) in h.iter().enumerate().take(FE_TAPS) {
            let m = k as isize - half;
            let i = n as isize - m;
            if i >= 0 && (i as usize) < samples.len() {
                y += coeff * samples[i as usize];
            }
        }
        out.push(y);
    }
    out
}

/// What our end measured: the fractional sampling instant, the scale, and the
/// channel's single post-cursor tap. These are the *observations*; the
/// parameter set is their negation ([`Estimate::correction`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    /// The fractional part of the received stream's delay against the ideal
    /// symbol stream, in symbols, in `-0.5 .. +0.5`.
    pub delay: f64,
    /// `observed / ideal` at the best fit: the level the line carried, in LSB
    /// per `out` unit.
    pub level: f64,
    /// The channel's first post-cursor coefficient, `y(n) = x(n) + a x(n-1)`,
    /// at the best fit.
    pub tap: f64,
    /// The fit's normalised mean square error at the best point, which is what
    /// says the estimate was taken on a line that had an answer.
    pub nmse: f64,
}

impl Estimate {
    /// The parameter set that cancels this estimate, at the level the receiver
    /// wants to see.
    ///
    /// Three corrections and no more: the delay is negated, the level is scaled
    /// so that what arrives is `target_lsb`, and the pre-emphasis is the exact
    /// three-tap inverse of the measured tap.
    #[must_use]
    pub fn correction(&self, current_level_lsb: f64, target_lsb: f64) -> FarEndParams {
        let scale = if self.level > 0.0 { target_lsb / self.level } else { 1.0 };
        FarEndParams {
            delay: -self.delay,
            level_lsb: current_level_lsb * scale,
            eq: EqTaps::inverse_of(self.tap),
        }
    }
}

/// The integer lag a far end's transmit stream arrives at, for callers that
/// want the estimator's own alignment window: the far end is aligned to the
/// ideal symbol stream and the fractional shift is centred, so the window is
/// zero and a few samples of slack either side.
pub const FE_HALF_LAG: usize = FE_HALF;

/// The coarse stage of the fractional search: `1/64` of a symbol.
pub const COARSE: usize = 64;
/// The fine stage: `1/1024` of a symbol, which is finer than any requirement
/// `docs/250` §4 states.
pub const FINE: usize = 1024;

/// Our end's estimator: a matched fit of `observed` against the *ideal* symbol
/// stream the far end was asked to send.
///
/// The ideal stream is known during TRN1u -- it is the one training sequence
/// with no scrambler seed in it, so a known-sequence fit is what a real
/// acquisition has -- and this is one: an integer-lag correlation for the
/// alignment, then a two-parameter (level, tap) least squares at each fractional
/// lag on a two-stage grid. Nothing here reads the channel, the noise or the
/// receiver; the inputs are `observed` and `ideal` and nothing else.
#[must_use]
pub fn estimate(ideal: &[f64], observed: &[f64], lags: std::ops::Range<usize>) -> Estimate {
    let n = ideal.len().min(observed.len());
    if n < 4 {
        return Estimate { delay: 0.0, level: 0.0, tap: 0.0, nmse: f64::INFINITY };
    }
    // 1. The integer lag: the largest normalised correlation over the window the
    // caller allows.
    let mut best_lag = lags.start;
    let mut best_corr = f64::NEG_INFINITY;
    for lag in lags {
        let (mut num, mut den) = (0.0, 0.0);
        for i in lag..n {
            let j = i - lag;
            num += observed[i] * ideal[j];
            den += ideal[j] * ideal[j];
        }
        let corr = if den > 0.0 { num / den.sqrt() } else { f64::NEG_INFINITY };
        if corr > best_corr {
            best_corr = corr;
            best_lag = lag;
        }
    }
    // 2. The fractional part, coarse then fine, each point a least-squares
    // (level, tap) fit against the shifted ideal.
    let fit_at = |delta: f64| -> (f64, f64, f64) {
        let shifted = shift_centred(ideal, delta);
        // Two regressors: the shifted stream, and the same stream one symbol
        // late -- the channel this model has.
        let (mut xx, mut xz, mut zz) = (0.0, 0.0, 0.0);
        let (mut yx, mut yz) = (0.0, 0.0);
        for i in best_lag..n {
            let j = i - best_lag;
            let x = shifted[j];
            let z = if j > 0 { shifted[j - 1] } else { 0.0 };
            let y = observed[i];
            xx += x * x;
            xz += x * z;
            zz += z * z;
            yx += y * x;
            yz += y * z;
        }
        // The two normal equations in (g, g a): the level multiplies both
        // regressors, so the tap comes out as the ratio.
        let det = xx * zz - xz * xz;
        let (g, ga) = if det.abs() > 1e-12 {
            ((yx * zz - yz * xz) / det, (yz * xx - yx * xz) / det)
        } else {
            (if xx > 0.0 { yx / xx } else { 0.0 }, 0.0)
        };
        let a = if g.abs() > 1e-12 { ga / g } else { 0.0 };
        let mut err = 0.0;
        let mut pow = 0.0;
        for i in best_lag..n {
            let j = i - best_lag;
            let z = if j > 0 { shifted[j - 1] } else { 0.0 };
            let model = g * shifted[j] + ga * z;
            err += (observed[i] - model).powi(2);
            pow += observed[i] * observed[i];
        }
        (g, a, if pow > 0.0 { err / pow } else { f64::INFINITY })
    };
    let mut best = (0.0f64, f64::INFINITY, 0.0, 0.0);
    let step = 1.0 / COARSE as f64;
    for k in 0..COARSE {
        let (g, a, nmse) = fit_at(-0.5 + k as f64 * step);
        if nmse < best.1 {
            best = (-0.5 + k as f64 * step, nmse, g, a);
        }
    }
    let centre = best.0;
    let step = 2.0 / COARSE as f64 / FINE as f64;
    for k in 0..=FINE {
        let delta = centre - 1.0 / COARSE as f64 + k as f64 * step;
        if delta.abs() > 0.5 {
            continue;
        }
        let (g, a, nmse) = fit_at(delta);
        if nmse < best.1 {
            best = (delta, nmse, g, a);
        }
    }
    Estimate { delay: best.0, level: best.2, tap: best.3, nmse: best.1 }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The user's design rule, as a check: the bypass is a multiply by the
    /// level and nothing else, and its two cardinal numbers are 1 and 0.
    #[test]
    fn the_bypass_is_bit_exact_and_its_gain_and_forward_are_one_and_zero() {
        let params = FarEndParams::bypass(900.0);
        assert!(params.is_bypass());
        assert_eq!(params.eq.gain(), 1.0);
        assert_eq!(params.eq.forward(), 0.0);
        let symbols: Vec<f64> = (0..97).map(|n| ((n * 37) % 19) as f64 - 9.0).collect();
        let out = params.apply(&symbols);
        // A no-equaliser reference is the same stream at the same level.
        let reference: Vec<f64> = symbols.iter().map(|&x| x * 900.0).collect();
        for (a, b) in out.iter().zip(&reference) {
            assert_eq!(a.to_bits(), b.to_bits(), "{a} against {b}: the bypass is not bit-exact");
        }
        // And the filter alone is the identity, not a near-identity.
        for (a, b) in params.eq.filter(&symbols).iter().zip(&symbols) {
            assert_eq!(a.to_bits(), b.to_bits());
        }
    }

    /// Three taps cancel a one-tap channel down to `a^3`, and no further.
    #[test]
    fn the_three_tap_inverse_cancels_a_one_tap_channel() {
        for a in [0.0, 0.1, 0.24, 0.3] {
            let symbols: Vec<f64> = (0..64).map(|n| ((n * 29) % 23) as f64 - 11.0).collect();
            let pre = EqTaps::inverse_of(a).filter(&symbols);
            let mut after = Vec::new();
            let mut previous = 0.0;
            for &x in &pre {
                after.push(x + a * previous);
                previous = x;
            }
            // What is left is `a^3 x(n-3)`, which is the arithmetic and not a
            // modelling choice.
            let bound = a * a * a * 11.0 + 1e-12;
            for (got, want) in after.iter().zip(&symbols) {
                assert!((got - want).abs() <= bound, "a {a}: {got} against {want}");
            }
        }
    }

    /// The estimator recovers a delay, a level and a tap that were put on the
    /// line, on a stream with no quantiser in it.
    #[test]
    fn the_estimator_recovers_a_delay_a_level_and_a_tap() {
        // A deterministic broadband source: the fractional fit needs a signal
        // that is not periodic, or the lag search has more than one answer.
        let mut state = 0x2545_F491_4F6C_DD1Du64;
        let mut next = || {
            state ^= state >> 12;
            state ^= state << 25;
            state ^= state >> 27;
            ((state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40) as f64) / 65536.0 - 1.0
        };
        // Smoothed, because a symbol stream's own bandwidth is what makes the
        // fractional fit well posed: a white sequence at 1/T would be
        // interpolated by a windowed sinc nobody would build.
        let mut previous = 0.0;
        let ideal: Vec<f64> = (0..600)
            .map(|_| {
                previous = 0.7 * previous + 0.3 * next();
                previous
            })
            .collect();
        let (delay, level, tap) = (0.37, 812.0, 0.18);
        let placed = shift_centred(&ideal, delay);
        let mut observed = Vec::new();
        let mut previous = 0.0;
        for &x in &placed {
            observed.push(level * (x + tap * previous));
            previous = x;
        }
        let e = estimate(&ideal, &observed, 0..8);
        assert!((e.delay - delay).abs() < 1.0 / COARSE as f64, "delay {}", e.delay);
        assert!((e.level - level).abs() < 25.0, "level {}", e.level);
        assert!((e.tap - tap).abs() < 0.05, "tap {}", e.tap);
        // And the correction it asks for cancels the delay it measured.
        let params = e.correction(level, 900.0);
        assert!((params.delay + delay).abs() < 1.0 / COARSE as f64);
    }

    /// `docs/255` S1.2: the far end's fractional instant and the CO's sampling
    /// instant are two *positions on one continuous line*, so they compose to a
    /// single kernel at their difference (`shift_centred(x, d_fe + d_co)`).
    /// Applying a kernel at each -- the convention `docs/250`-`docs/254`'s loop
    /// numbers were taken in -- composes them to `H(f)^2` and leaves an ISI the
    /// single kernel does not: the truncation error, squared and applied to a
    /// symbol-rate stream that is not bandlimited below the interpolator's
    /// passband. This is the size of the loop's old residual, and it is the
    /// model's, not the far end's.
    #[test]
    fn two_kernels_in_series_leave_isi_that_one_kernel_at_the_residual_does_not() {
        // A deterministic, full-band T-spaced stream: the upstream's own symbol
        // rate is one sample per symbol, so nothing here is oversampled.
        let mut state = 0x2545_F491_4F6C_DD1Du64;
        let mut symbols = Vec::with_capacity(4096);
        for _ in 0..4096 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let point = (state >> 40) % 16;
            symbols.push(point as f64 - 7.5);
        }
        let rms = |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt();
        let power = rms(&symbols);
        // The error each chain leaves, as a fraction of the symbol stream's rms,
        // measured over the interior so the zero-padded edges do not count.
        let error = |v: &[f64]| {
            let from = 2 * FE_HALF;
            let to = v.len() - 2 * FE_HALF;
            let e: Vec<f64> = (from..to).map(|i| v[i] - symbols[i]).collect();
            rms(&e) / power
        };
        // The far end asks for -0.3 and the CO samples at +0.3: two kernels.
        let twice = shift_centred(&shift_centred(&symbols, -0.3), 0.3);
        // The same chain, sampled once: one kernel at the residual. 1e-4 is
        // `docs/250`'s estimate error, which the loop cannot do better than.
        let once = shift_centred(&symbols, -0.3 + 0.3 + 1e-4);
        let (two, one) = (error(&twice), error(&once));
        println!("kernel test: two={two:e} one={one:e} ratio={:e}", two / one);
        // The single kernel at a 1e-4 residual is the interpolation error at
        // that residual, i.e. nothing a slicer can see; the composed pair is
        // three orders of magnitude larger and is what `docs/255` S1 removes.
        assert!(one < 2e-3, "one kernel at the residual: {one:e} of the stream");
        assert!(two > 1e-2, "two kernels in series: {two:e} of the stream");
        assert!(two > 20.0 * one, "{two:e} against {one:e}");
        // And a zero residual through one kernel is the identity to within one
        // ulp -- not bit for bit, because `taps(0)` is normalised by a sum that
        // is 1 +- 1e-16. That is why `FarEndParams::apply` short-circuits
        // `delay == 0.0` instead of convolving with it, and it is the only
        // cost the single kernel adds at a zero residual.
        let exact = shift_centred(&symbols, 0.0);
        let mut worst = 0.0f64;
        for (a, b) in exact.iter().zip(&symbols) {
            worst = worst.max((a - b).abs());
        }
        assert!(worst < 1e-13, "the zero-residual kernel moved {worst:e}");
    }

    /// `docs/255` S1.4: the loop's ceiling above `drn 10` is the *three-tap*
    /// inverse's own residual, `a^3`, which the ideal case does not have. The
    /// product of the pre-emphasis with the channel is `1 + a^3 z^-3`, and at
    /// LC 64's 4-8 LSB half-gaps that is several whole code steps at the
    /// stream's peak. The exact inverse is a stable causal recursion (the
    /// channel's zero is at `z = -a`, inside the unit circle), so the depth is
    /// a choice about the far end's filter, not a limit of the arithmetic.
    #[test]
    fn the_three_tap_inverse_leaves_a_cubed_and_that_is_the_high_rung_ceiling() {
        let a = 0.24;
        let eq = EqTaps::inverse_of(a);
        // The pre-emphasis convolves with the channel `1 + a z^-1`.
        let impulse = {
            let mut v = vec![0.0; 8];
            v[0] = 1.0;
            v
        };
        let product = {
            let shaped = eq.filter(&impulse);
            let mut out = vec![0.0; 8];
            for n in 0..8 {
                out[n] = shaped[n] + if n > 0 { a * shaped[n - 1] } else { 0.0 };
            }
            out
        };
        assert!((product[0] - 1.0).abs() < 1e-12);
        assert!(product[1].abs() < 1e-12, "a^1 must be cancelled: {}", product[1]);
        assert!(product[2].abs() < 1e-12, "a^2 must be cancelled: {}", product[2]);
        assert!((product[3] - a * a * a).abs() < 1e-12, "the residual is a^3");
        // What a^3 costs against the rung it has to fit: LC 64's finest gap is
        // 8 LSB (`rung::gap_lsb`), its half is 4, and the stream's peak at
        // 48 000 is 3772 LSB (`docs/254` S5). The depth that would clear it:
        let peak = 3772.0;
        let half_gap = 4.0;
        assert!(a.powi(3) * peak > 4.0 * half_gap, "a^3 must be over the gap");
        let mut depth = 1;
        let mut residual = a;
        while residual * peak > half_gap && depth < 8 {
            residual *= a;
            depth += 1;
        }
        assert_eq!(depth, 5, "five taps clear LC 64's half-gap, not three");
    }
}
