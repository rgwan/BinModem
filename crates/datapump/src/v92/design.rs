//! The digital modem's precoder and prefilter design (6.4.2, 8.8.3 and its
//! NOTE): an estimate of the upstream channel, turned into the coefficients
//! and the gain that CPd downloads to the analogue modem, inside the limits
//! the analogue modem announced in INFO1a bits 12:17.
//!
//! V92-21. The shape of the answer, and why it works:
//!
//! ```text
//! analogue:  u(n) -> precoder x(n) -> prefilter v(n) -> line
//! line:      v(n) -> channel c -> codec's A/D -> digital modem
//! ```
//!
//! 8.8.3's NOTE says to "design the precoder coefficients assuming the
//! analogue modem minimizes the power at the precoder output symbol by
//! symbol", which is what [`Chain`](super::precoder::Chain) does when it picks
//! the constellation member nearest the precoder's tail. That gives the design
//! its freedom: the prefilter's job is to shorten the *channel*, so that the
//! precoder's feedback can cancel what is left of it.
//!
//! So there are two steps. The prefilter `z2` is the feed-forward equaliser
//! for the channel, found as the least-squares inverse of the estimated
//! impulse response within the length the far end offered. The precoder's
//! feedback `p1` is then the negative of the residual response's tail, which is
//! what makes the cascade `u -> precoder -> prefilter -> channel` come back to
//! `u` at the codec. What neither can reach -- a leading tap, or a tail longer
//! than `p1` may be -- is reported as [`Design::residual`], so that the rate
//! choice can add it to the measured noise before it picks moduli (V92-62).
//!
//! Nothing here is on the wire: `Cpd` is `v92::sequences`' business, and this
//! package deals in [`Filters`], as AD-3 asks.

use dsp::Complex;

use super::{FilterLimits, Filters};
use crate::v34::probe::{Reading, Tone};

/// How many taps of the channel are estimated when a caller has no better
/// number. The upstream channel is the analogue modem's transmit filter, the
/// local loop, the central office's band limiting and the A/D; TRN1u is 2040
/// symbols, so a least-squares fit of this many taps has many times as many
/// equations as unknowns.
pub const CHANNEL_TAPS: usize = 64;

/// The floor under the inverse's ridge, as a fraction of the autocorrelation's
/// trace: the smallest the arithmetic will take.
///
/// A least-squares inverse of a channel with a null in its band -- every real
/// loop has one at DC -- puts an enormous coefficient on that null, and the
/// quantiser then either saturates it or loses it. `RIDGE` alone was the whole
/// story here once, and it is far too small to stop that: on the harness the
/// section came out needing twelve and a half times the dynamic range Q0.15 has
/// (`Design::scale` 12.586), which is a prefilter no far end can run at the
/// amplitude `G` names.
///
/// What the ridge should be is the reciprocal of the signal to noise the design
/// is being made into. Minimising ISI alone is the wrong problem -- it is
/// solvable and it is what the constant below did -- and minimising ISI subject
/// to a transmit power the line can carry is the right one. That trade is
/// exactly what a ridge is, so it is [`Channel::noise_ratio`], floored here and
/// capped by [`ALPHA_MOST`].
const RIDGE: f64 = 1e-6;

/// The largest ridge a design will take.
///
/// The ridge trades residual ISI for transmit power, and past about a tenth the
/// inverse stops being an inverse at all -- it is a matched filter, which puts
/// the whole of the channel's memory back as ISI for the precoder to cancel.
/// A design that asks for more than this is a design for a channel that should
/// not be carrying this rung, not a design with a better ridge.
const ALPHA_MOST: f64 = 0.1;

/// The upstream channel as the digital modem sees it: T-spaced taps and the
/// noise the estimate was made in.
#[derive(Debug, Clone, PartialEq)]
pub struct Channel {
    pub taps: Vec<f64>,
    /// The power of what the taps do not explain, as a mean square on the
    /// same scale as the taps' input.
    pub noise: f64,
    /// The mean square of the line the estimate was made on, on that same
    /// scale: what `noise` is a noise *against*.
    ///
    /// A telephone loop's loss is not fixed -- it depends on the loop, on the
    /// far end's transmit level and on the network -- so the one number the
    /// design must not take from a constant is the signal to noise it is
    /// designing into. This is the measurement, taken on every call from the
    /// same window the taps came from.
    pub signal: f64,
}

impl Channel {
    /// The least-squares fit of `received` against `known`, T-spaced.
    ///
    /// `known` is what the analogue modem was told to send -- TRN1u's GPA
    /// sequence, which this side can generate exactly -- and `received` what
    /// the codec made of it. The fit is over `taps` taps and every symbol whose
    /// whole support is inside the two runs.
    pub fn estimate(known: &[f64], received: &[f64], taps: usize) -> Option<Self> {
        let length = known.len().min(received.len());
        let taps = taps.clamp(1, (length / 2).max(1));
        if length < 2 * taps {
            return None;
        }
        // The normal equations of the convolution: A is the known sequence's
        // autocorrelation, b its cross-correlation with what arrived.
        let mut a = vec![Complex::ZERO; taps * taps];
        let mut b = vec![Complex::ZERO; taps];
        for i in 0..taps {
            for j in 0..taps {
                let mut sum = 0.0;
                for n in taps - 1..length {
                    sum += known[n - i] * known[n - j];
                }
                a[i * taps + j] = Complex::new(sum, 0.0);
            }
            let mut sum = 0.0;
            for n in taps - 1..length {
                sum += known[n - i] * received[n];
            }
            b[i] = Complex::new(sum, 0.0);
        }
        // A ridge, so that a null in the channel's band cannot take the fit
        // with it.
        let trace: f64 = (0..taps).map(|i| a[i * taps + i].re).sum();
        let ridge = RIDGE * trace / taps as f64;
        for i in 0..taps {
            a[i * taps + i].re += ridge;
        }
        let solved = dsp::complex::solve_hermitian(&a, &b)?;
        let fitted: Vec<f64> = solved.iter().map(|value| value.re).collect();
        // What the fit leaves: the noise the design has to work in.
        let mut power = 0.0;
        let mut count = 0usize;
        for n in taps - 1..length {
            let mut model = 0.0;
            for (k, &tap) in fitted.iter().enumerate() {
                model += tap * known[n - k];
            }
            power += (received[n] - model) * (received[n] - model);
            count += 1;
        }
        if count == 0 {
            return None;
        }
        let mut signal = 0.0;
        for n in taps - 1..length {
            signal += received[n] * received[n];
        }
        Some(Self { taps: fitted, noise: power / count as f64, signal: signal / count as f64 })
    }

    /// The reciprocal of this channel's signal to noise, which is the ridge its
    /// prefilter's inverse wants: see [`RIDGE`].
    ///
    /// Bounded at both ends so that a measurement taken in the wrong window --
    /// silence, or a burst -- cannot take the design with it.
    #[must_use]
    pub fn noise_ratio(&self) -> f64 {
        if !(self.signal > 0.0) || !self.noise.is_finite() {
            return RIDGE;
        }
        (self.noise / self.signal).clamp(RIDGE, ALPHA_MOST)
    }

    /// The channel's response to a unit impulse over `length` symbols.
    pub fn impulse(&self, length: usize) -> Vec<f64> {
        let mut out = vec![0.0; length.max(self.taps.len())];
        for (k, &tap) in self.taps.iter().enumerate() {
            out[k] = tap;
        }
        out
    }

    /// How much response a symbol's own position carries against the rest of
    /// it: zero for a channel with no memory, and infinity for none at all.
    pub fn severity(&self) -> f64 {
        let peak = self.taps.iter().fold(0.0f64, |most, &tap| most.max(tap.abs()));
        if peak == 0.0 {
            return f64::INFINITY;
        }
        let rest: f64 = self.taps.iter().map(|tap| tap * tap).sum::<f64>().sqrt();
        rest / peak
    }
}

/// What the design settled on.
#[derive(Debug, Clone, PartialEq)]
pub struct Design {
    /// z1, p1, z2 and p2, on the scale the levels are in.
    pub filters: Filters,
    /// The residual intersymbol interference, as a fraction of the level
    /// spacing: what the precoder could not cancel.
    pub residual: f64,
    /// How far the whole design was scaled to keep the prefilter inside
    /// Q0.15, which the rate choice multiplies into G.
    pub scale: f64,
    /// Whether the far end's limits, rather than the channel, decided the
    /// sections' lengths.
    pub limited: bool,
    /// The symbol delay the design puts between what the analogue modem sends
    /// and what the codec reads: the position of the response's main tap.
    /// A receiver has to find this on TRN1u before it can frame data, and
    /// [`Decoder`](super::decoder::Decoder) is built to be started at it.
    pub delay: usize,
}

impl Design {
    /// The residual as a signal-to-interference ratio, in decibels.
    pub fn residual_db(&self) -> f64 {
        if self.residual <= 0.0 { f64::INFINITY } else { -20.0 * self.residual.log10() }
    }
}

/// Design the prefilter and the precoder for `channel` inside `limits`.
///
/// The prefilter is the least-squares inverse of the channel inside LZ2; the
/// precoder's feedback is the residual response's tail inside LP1, negated;
/// and where the far end offered z1, the response's leading taps are moved into
/// it rather than left as ISI. What could not be reached is reported as
/// [`Design::residual`] rather than failing: a design that is merely poor is
/// still a design, and the rate choice is what decides whether it is worth
/// trying.
pub fn design(channel: &Channel, limits: &FilterLimits) -> Result<Design, &'static str> {
    if channel.taps.is_empty() {
        return Err("there is no channel to design against");
    }
    let most = usize::from(limits.most).max(1);
    let total = usize::from(limits.total).max(1);
    if limits.total == 0 || limits.most == 0 {
        return Err("the far end offered no filter sections at all");
    }
    let peak = channel.taps.iter().fold(0.0f64, |a, &tap| a.max(tap.abs()));
    if peak == 0.0 {
        return Err("the channel is empty");
    }
    // z2 is the feed-forward section, and it is the one section every PCM
    // upstream modem must offer (Table 18 bit 12: p1 and z2 at least).
    // `V92_LZ2` overrides the section length, which is the one lever on how
    // hard the prefilter inverts: the normalised inverse's peak does not move
    // with the ridge (raising it scales the inverse and the response's own main
    // tap by the same factor), so if a channel's prefilter does not fit Q0.15
    // the length is what has to give, not the ridge.
    let lz2 = std::env::var("V92_LZ2").ok().and_then(|text| text.parse().ok()).unwrap_or(most.min(total));
    let inverse = inverse(channel, lz2);
    // h is c convolved with z2: what is left for the precoder to cancel.
    let mut response = convolve(&channel.taps, &inverse);
    // Where the response's own energy is: the precoder's reference tap.
    let main = response
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .map(|(at, _)| at)
        .unwrap_or(0);
    let unit = response[main];
    if unit.abs() < 1e-9 {
        return Err("the prefilter's response has no main tap to build on");
    }
    // Normalising on the main tap is what lets p1 talk about levels rather
    // than about the channel's own scale, and it is done before the
    // prefilter is scaled so that the two do not chase each other: p1 is a
    // ratio of the response to itself, and survives the scaling below
    // untouched.
    response.iter_mut().for_each(|value| *value /= unit);
    let mut z2: Vec<f64> = inverse.iter().map(|value| value / unit).collect();
    // z2 is Q0.15, [-1, 1). A channel whose inverse needs more than that is
    // answered by scaling the whole prefilter down and telling the rate
    // choice, which folds the scale into G.
    let largest = z2.iter().fold(0.0f64, |a, &tap| a.max(tap.abs()));
    let mut scale = 1.0;
    if largest >= 0.99 {
        scale = largest / 0.99;
        z2.iter_mut().for_each(|value| *value /= scale);
    }
    let mut filters = Filters { z2, ..Filters::default() };

    let mut unreached = 0.0f64;
    let mut limited = false;

    // The lead-in: taps before the main one. With z1 the precoder can cancel
    // them from u's own history; without it they are ISI.
    if main > 0 {
        if limits.sections.z1 {
            let room = most.min(total.saturating_sub(filters.z2.len()));
            let take = room.min(main);
            filters.z1 = (1..=take).map(|k| -response[main - k]).collect();
            for k in take + 1..=main {
                unreached += response[main - k] * response[main - k];
            }
            if take < main {
                limited = true;
            }
        } else {
            for k in 1..=main {
                unreached += response[main - k] * response[main - k];
            }
            limited = true;
        }
    }

    // The tail: what the precoder's feedback cancels.
    let room = most.min(total.saturating_sub(filters.z2.len() + filters.z1.len()));
    let tail = response.len().saturating_sub(main + 1);
    let take = room.min(tail);
    filters.p1 = (1..=take).map(|k| -response[main + k]).collect();
    for k in take + 1..=tail {
        unreached += response[main + k] * response[main + k];
    }
    // A channel with nothing to cancel gets no precoder at all rather than a
    // section of near-zero coefficients: a far end told to run a 320-tap
    // feedback of zeros is being asked to do work for nothing.
    while filters.p1.last().is_some_and(|value| value.abs() < 1e-9) {
        filters.p1.pop();
    }
    if take < tail {
        limited = true;
    }

    // p1 is Q1.14, [-2, 2): a coefficient outside it means the response is not
    // the shortened one the design intended.
    if let Some(&largest) = filters.p1.iter().max_by(|a, b| a.abs().total_cmp(&b.abs()))
        && largest.abs() >= 2.0
    {
        return Err("a precoder coefficient does not fit q1.14");
    }
    if filters.z2.iter().any(|value| value.abs() >= 1.0) {
        return Err("a prefilter coefficient does not fit q0.15");
    }
    filters.fits(limits)?;

    // A prefilter that is a pure delay is an identity with latency, and it is
    // not free: measured on the Agere at 24000 (`/tmp/reports/z2-path-diagnosis.md`,
    // `docs/94` P0), the design this end produced was
    // `z2 = 0.9999 * delta(n - 8)` with every other tap below 6.2e-5 -- a
    // spectrally flat no-op (Z2 = 0.0 dB at all 21 tones, `Z2H = H` at every
    // one) that still cost 750 bits of CPd and, worse, **eight symbols of
    // latency this end did not account for**, which rotates the twelve-symbol
    // data frame and the four-symbol trellis phase. Sending `z2 = [1]` instead
    // is the same line, one code shorter, and the delay goes away.
    //
    // One code is 2^-15: a tap below that is not representable in the field it
    // would be sent in, so "nothing but the main tap survives quantisation" is
    // the test, not "the other taps are small".
    // One code is 2^-15: a tap below that is not representable in the field it
    // would be sent in, so it is a pure delay and the far end need not run it.
    // Trim those off the front and the *latency* goes with them, which is what
    // the failure below is actually about:
    //
    // measured on the Agere at 24000 (`/tmp/reports/z2-path-diagnosis.md`,
    // `docs/94` P0), this end's design was `0.9999 * delta(n - 8)` with every
    // other tap below 6.2e-5 -- spectrally a no-op (`Z2 = 0.0 dB` at all 21
    // tones, `Z2H = H` everywhere) that still cost 750 bits of CPd and, worse,
    // **eight symbols of latency this end did not account for**: it rotates the
    // twelve-symbol data frame and the four-symbol trellis phase, and B1u reads
    // chance (291 of 576 against 543 for the same line with `z2 = [1]`). Moving
    // the main tap to index zero removes the latency and keeps whatever
    // filtering the design actually did.
    let one_code = 1.0 / 32768.0;
    let mut delay = main;
    if main > 0 && filters.z2[..main].iter().all(|value| value.abs() < one_code) {
        filters.z2.drain(..main);
        delay = 0;
    }
    if filters.z2.len() == 1 {
        // Nothing but the main tap survives: send the identity rather than a
        // one-code section, so the CPd is 480 bits and says what happens.
        filters = Filters { z1: Vec::new(), p1: Vec::new(), z2: vec![1.0], p2: Vec::new() };
    }
    Ok(Design { filters, residual: unreached.sqrt(), scale, limited, delay })
}

/// 8.8.3's answer when nothing the design can send is admissible: the far end
/// runs no filters at all and the line's own ISI is this receiver's to undo.
///
/// `z2 = [1]`, no `z1`, no `p1`, no `p2` is legal in every field of Table 30
/// (LZ1 = LP1 = LP2 = 0, LZ2 = 1, and `G` from `upchoice::gain_for`) and it is
/// not a lie: it says exactly what will happen to the far end's symbols on the
/// line. The alternative -- sending no CPd at all -- is what an end does when
/// it has not designed anything, and it leaves the far end's SUVu
/// acknowledgement clear for ever (docs/27 section 0.5.5), so phase 4 never
/// starts and no B1u is ever transmitted (docs/28 P2).
#[must_use]
pub fn identity(channel: &Channel) -> Design {
    let delay = channel
        .taps
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .map_or(0, |(at, _)| at);
    Design {
        filters: Filters { z1: Vec::new(), p1: Vec::new(), z2: vec![1.0], p2: Vec::new() },
        // Nothing is asked of the far end's precoder, so there is nothing for
        // it to fail to cancel. Whatever ISI the design would have shortened is
        // still on the line, and it is the *receiver's* equaliser that has to
        // take it; the rate choice cannot see it through this design, which is
        // why the caller keeps this fallback on the slowest rung.
        residual: 0.0,
        scale: 1.0,
        limited: true,
        delay,
    }
}

/// The least-squares inverse of the channel inside `length` taps.
///
/// Wiener-Hopf on the channel's own autocorrelation: the coefficients that
/// make `c * w` nearest a unit impulse at the channel's first significant tap,
/// with the ridge that keeps a null in the band from running away with them.
fn inverse(channel: &Channel, length: usize) -> Vec<f64> {
    inverse_with(channel, length, channel.noise_ratio())
}

/// [`inverse`] at a stated ridge, as a fraction of the autocorrelation's trace.
fn inverse_with(channel: &Channel, length: usize, relative_ridge: f64) -> Vec<f64> {
    let taps = &channel.taps;
    let length = length.max(1);
    let mut a = vec![Complex::ZERO; length * length];
    for i in 0..length {
        for j in 0..length {
            let mut sum = 0.0;
            for k in 0..taps.len() {
                let left = k as i64 - i as i64;
                let right = k as i64 - j as i64;
                if left >= 0 && right >= 0 {
                    sum += taps[left as usize] * taps[right as usize];
                }
            }
            a[i * length + j] = Complex::new(sum, 0.0);
        }
    }
    let trace: f64 = (0..length).map(|i| a[i * length + i].re).sum();
    let ridge = relative_ridge * trace / length as f64;
    for i in 0..length {
        a[i * length + i].re += ridge;
    }
    // The target impulse sits where the channel's own energy is, so that the
    // inverse is causal and as short as it can be.
    let at = taps
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .map_or(0, |(index, _)| index);
    // (C^T delta)[i] is the channel at the target's position minus i, so the
    // right-hand side is the channel read backwards from `at`.
    let mut b = vec![Complex::ZERO; length];
    for i in 0..length.min(at + 1) {
        b[i] = Complex::new(taps[at - i], 0.0);
    }
    let solved = dsp::complex::solve_hermitian(&a, &b).unwrap_or_else(|| {
        // A channel so degenerate that the ridge cannot make the system
        // solvable is answered with a pass-through: the design reports a poor
        // residual, and the rate choice takes that as it finds it.
        let mut fallback = vec![Complex::ZERO; length];
        fallback[0] = Complex::ONE;
        fallback
    });
    solved.iter().map(|value| value.re).collect()
}

/// The convolution of two sequences.
fn convolve(first: &[f64], second: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; first.len() + second.len() - 1];
    for (i, &a) in first.iter().enumerate() {
        for (j, &b) in second.iter().enumerate() {
            out[i + j] += a * b;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// V92-63: the prefilter designed from the L1/L2 reading
// ---------------------------------------------------------------------------
//
// The section above designs z2 as the least-squares inverse of a T-spaced fit
// of TRN1u. That is an inverse of the *whole complex* channel, unconstrained in
// level: on this harness it comes out asking for twelve and a half times the
// dynamic range Q0.15 has, `upchoice` refuses the G it would need, and the far
// end is told nothing (`no upstream design, CPd would say nothing` until the
// identity fallback landed at 449e47f).
//
// The L1 that phase 2 already sends is a *measurement* of the same channel,
// taken by this receiver off the analogue modem's own transmitter, and it
// carries the two things a power-constrained shaper needs: the magnitude at
// every tone and the signal to noise at every tone. So the design here is a
// magnitude problem with a power constraint, in this order:
//
// 1. **Level.** `sum z^2` is the upstream power and the far end does not
//    renormalise, so it is the received level too -- and this receiver's cliff
//    is *one decibel of flat attenuation*, with no channel shape involved at
//    all (measured this round: 0 dB of flat gain reads 7128 ones, -1 dB
//    reads 19). A design may not
//    spend level for shape. The shape is normalised to a mean gain of one and
//    `sum z^2` lands at one plus its own variance, which is level-preserving by
//    construction.
// 2. **Tilt.** Inside that, the shape is the reciprocal of the line clipped
//    into the actuator's window: the quiet half of the band is lifted, the loud
//    half trimmed, and what a section this long cannot reach is left alone.
// 3. **Noise.** `snr_db` decides where nothing may be asked: a tone at the
//    floor is not inverted, not lifted and not cut, so a null above the band
//    that survives stays where the line put it.
//
// Three facts about the actuator decide the constraints, all measured rather
// than assumed (`docs/33`):
//
// - HSF applies z2 exactly, at the 8 kHz symbol rate, from B1u onward, and
//   does not renormalise afterwards. So `sum z^2` *is* the upstream power, and
//   `sum z^2 = 8` already reached 81% of full scale.
// - Tap order does not matter: reversal is a conjugate phase plus a delay, so
//   the magnitude is invariant. This designs on `|H|` only, and a symmetric
//   (linear-phase) section is therefore free to be used.
// - p1 does not behave as 8.8.3 predicts (2.1 to 3.6 dB of residual, and its
//   level moves the wrong way). Nothing here designs for p1: it is sent empty,
//   and a linear-phase z2 needs no precoder anyway -- its response is a pure
//   delay, not ISI.

/// Signal to noise, in dB, at which a probing tone stops being worth designing
/// toward, and the level at which it is shaped as hard as the budget allows.
///
/// Between the two the target eases off linearly. This is what stops the
/// design asking for gain where there is no signal to carry it: a tone 10 dB
/// under the noise is a tone whose "gain" is mostly measurement error, and the
/// power that lifting it would cost is paid for by the tones that do carry
/// data. It is also what stops a null *above* the employed band being inverted
/// -- that tone's signal to noise is low, so its target is low, so the design
/// leaves it where the line put it.
pub const SNR_FLOOR: f64 = 10.0;
pub const SNR_FULL: f64 = 26.0;

/// The most gain, in dB, the design will ask of any frequency. The stop line
/// the actuator sets, kept as a stop line: a design that lands on it has spent
/// the whole of its headroom on one edge of the band.
pub const PEAK_CAP_DB: f64 = 2.0;

/// How far a tone's gain may sit from the band's mean before the shaper
/// corrects it, in decibels.
///
/// Bounded from both sides by measurement: 0.69 dB above the worst structure a
/// flat line reports (2.31 dB at 3750 Hz, a15) and 1.56 dB below the worst
/// deviation the G.712 channel reports (4.56 dB). See `fit_shape` for what the
/// band acts on and why it is on the deviation rather than on the gain.
pub const DEAD_BAND_DB: f64 = 3.0;


/// The most `sum z^2` a design will spend, which 8.8.3 makes the upstream
/// transmit power. The identity section, `z2 = [1]`, spends 1; 2 is 3 dB more
/// than the far end would send unfiltered, and a quarter of the 8 that already
/// reached 81% of full scale.
pub const POWER_BUDGET: f64 = 2.0;

/// The largest coefficient a design will put in z2, so that
/// [`Design::scale`] stays one.
///
/// Q0.15 carries up to 1 - 2^-15, and the identity section `[1]` is sent
/// saturated at that; a shaped section has to stay inside it too, or the
/// scaling that fits it takes `G` with it (`upchoice::build`) and the level
/// with `G`. The headroom is this tight because it *is* the level: the shape is
/// normalised to a mean gain of one, so this coefficient and `sum z^2` are the
/// same few hundredths of a decibel.
const TAP_HEADROOM: f64 = 0.9999;

/// The length of the prefilter designed from a reading, unless the far end
/// offers less.
///
/// Seventeen taps is eight either side and one in the middle, and the symmetric
/// section's delay is those eight symbols. Twenty-one measurements 150 Hz apart
/// support about that much and no more; the actuator does not need more either
/// -- 2 to 32 taps verify to 0.02 to 0.28 dB against the predicted `|H|^2`, and
/// halving 32 to 8 shows as 0.00 dB of level (`docs/33`).
pub const READING_TAPS: usize = 17;

/// z2 runs at the upstream symbol rate, which is the network's own rate:
/// one symbol per 125 microseconds (5/V.92).
const SYMBOL_RATE: f64 = 8000.0;

/// Above the band that survives the design asks for nothing: those frequencies
/// are held at unity.
///
/// Not rolled off, which is what the first form of this design did. The
/// receiver depends on the *presence* of content above 3.85 kHz rather than on
/// its power (docs/25, the 3.86/3.84 kHz cliff), so cutting above the band
/// would be spending this end's transmit power to remove something the far end
/// of the call can hear. Unity is the honest answer here too: the far end sends
/// what it would have sent, and if the line has a null there it is the line's.
const ABOVE_FROM: f64 = 150.0;
const ABOVE_TO: f64 = 3950.0;
const ABOVE_STEP: f64 = 100.0;
const ABOVE_WEIGHT: f64 = 0.3;

/// What a tone that carries no signal is worth: left at unity, and worth little
/// against the tones that do carry data, so that it cannot drag the shape.
const QUIET_TONE_WEIGHT: f64 = 0.2;

/// The ridge on the design's normal equations, as a fraction of their trace.
/// Small: it is there to keep an over-determined system conditioned between
/// measurement points, not to trade ISI for power the way [`RIDGE`] does.
const READING_RIDGE: f64 = 1e-3;

/// What the design does to one probing tone, for the log and for the tests.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToneResult {
    pub frequency: f64,
    /// The line, as the probing measured it.
    pub gain_db: f64,
    pub snr_db: f64,
    /// How much of that measurement the design was worth aiming at, 0 to 1.
    pub weight: f64,
    /// What the prefilter does at that frequency.
    pub z_db: f64,
    /// What the cascade does: `20 log10 |Z2 H|`.
    pub zh_db: f64,
}

/// What a reading-designed prefilter is predicted to do, before it is sent.
#[derive(Debug, Clone, PartialEq)]
pub struct Predicted {
    pub taps: usize,
    /// The band the design was aimed at, in hertz.
    pub band: (f64, f64),
    /// The upstream power relative to the identity section: `10 log10 (sum
    /// z^2)`, and therefore the change in received level this design is. This
    /// is the number that matters most, because the receiver's cliff is about
    /// one decibel of flat attenuation. It must not be negative.
    pub level_db: f64,
    /// The most gain asked of any frequency in the band.
    pub peak_db: f64,
    /// The deepest cut asked of any tone that carries signal.
    pub least_db: f64,
    /// `sum z^2`, which 8.8.3 makes the upstream power.
    pub power: f64,
    /// The worst departure of `|Z2 H|` from its own mean inside the band.
    pub flatness_db: f64,
    /// The spread of the line across the band before the design, and after it.
    pub tilt_before_db: f64,
    pub tilt_after_db: f64,
    pub tones: Vec<ToneResult>,
}

impl Predicted {
    /// One line: what was designed, what it costs and how flat it lands.
    #[must_use]
    pub fn summary(&self) -> String {
        format!(
            "{} taps, band {:.0}-{:.0} Hz, level {:+.2} dB (sum z^2 {:.2}), \
             window {:+.1}..{:+.1} dB, tilt {:.1} -> {:.1} dB, flat within {:.1} dB",
            self.taps,
            self.band.0,
            self.band.1,
            self.level_db,
            self.power,
            self.least_db,
            self.peak_db,
            self.tilt_before_db,
            self.tilt_after_db,
            self.flatness_db
        )
    }

    /// The same, a tone at a time: `frequency Hz H/snr/Z2/Z2H` in decibels.
    ///
    /// The four numbers are the line as the probing measured it, the noise it
    /// measured with it, what the prefilter is asked to do there, and what is
    /// left of the product. A bench call that reads this can say the received
    /// spectrum landed where the design intended without gdb.
    #[must_use]
    pub fn table(&self) -> String {
        self.tones
            .iter()
            .map(|tone| {
                format!(
                    "{:.0}Hz {:+.1}/{:.1}/{:+.1}/{:+.1}",
                    tone.frequency, tone.gain_db, tone.snr_db, tone.z_db, tone.zh_db
                )
            })
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// Design the far end's upstream prefilter from the L1/L2 reading.
///
/// The reading is the analogue modem's own L1, as this receiver measured it, so
/// it is the upstream channel rather than a fit through a two-point stream: no
/// timing recovery, no discriminator, no TRN1u's two thirds of its power at
/// 4000 Hz where a loop has a null.
///
/// What comes back is a symmetric FIR whose `|Z2(f)|` is the weighted inverse of
/// `|H(f)|` over the band the reading says survives, zero above it, scaled to
/// the power and peak limits above. `p1` is empty: a linear-phase section is a
/// delay, not ISI, so there is nothing for a precoder to cancel, and the
/// actuator's p1 does not behave as the Recommendation predicts anyway.
/// The symmetric section that best fits the shape the reading asks for, inside
/// a window of `window` either side of unity.
///
/// Returns the cosine coefficients of the zero-phase response, the taps they
/// are a section of, and the response's own peak.
fn fit_shape(
    tones: &[Tone],
    usable: &[(&Tone, f64, f64)],
    window: f64,
    m: usize,
    lz2: usize,
    high: f64,
) -> Result<(Vec<f64>, Vec<f64>, f64, f64), &'static str> {
    let omega = |hz: f64| std::f64::consts::TAU * hz / SYMBOL_RATE;
    // The shape to aim at: the reciprocal of the line, brought to a mean gain
    // of one and clipped into the window.
    //
    // A mean gain of one is the level. The reciprocal is a ratio, so its own
    // amplitude means nothing; what the far end will send is its `sum z^2`, and
    // a receiver whose cliff is one decibel of flat attenuation cannot afford
    // to pay for flatness with that. Normalising the mean to one leaves `sum
    // z^2` at one plus the shape's own variance, so the design is level-
    // preserving by construction and what the peak and the cut do not cancel is
    // the only level it can gain.
    //
    // The clip is the actuator's window: the peak cap on one side, and the same
    // amount on the other, because a section whose magnitude is above one
    // everywhere has a centre tap above one, which Q0.15 cannot carry -- scaled
    // back to fit, it would land at the same net level anyway. Boosting the
    // quiet half of the band is paid for by trimming the loud half, which keeps
    // the level and buys tilt.
    // The dead band, on the deviation from the band's own mean gain.
    //
    // The reading is not the thing the shaper wants to invert: it is the line
    // *plus this end's receive path*, and on a line that is straight it still
    // reports structure -- measured on the identity arm, 2.31 dB at 3750 Hz
    // against the band's mean, with only +/-0.7 dB of wobble anywhere else
    // (a15). Correcting that inverts our own receiver at the transmitter, which
    // the equaliser was already going to handle, and it tilts a line that had
    // nothing wrong with it -- on the arm with no channel at all.
    //
    // So the target is unity for any tone within `DEAD_BAND_DB` of the mean and
    // the inverse of the *excess* beyond it for any tone further out:
    //
    //     correction(f) = -(d(f) - clamp(d(f), -DEAD_BAND_DB, +DEAD_BAND_DB))
    //     d(f)          = gain_db(f) - mean(gain_db over the usable tones)
    //
    // Three decibels is bounded from both sides by measurement and not chosen:
    // it sits 0.69 dB above the worst structure a flat line reports (2.31 dB),
    // so a good line is left completely alone, and 1.56 dB below the worst
    // deviation the specification channel reports (4.56 dB on G.712's own
    // figure), so a gross tilt is still corrected.
    //
    // The band is on the *deviation*, not on the absolute gain: an absolute band
    // would leave a line that is 10 dB down everywhere untouched and correct one
    // that is 10 dB down in the middle, which is the wrong way round -- the mean
    // is the level, and the level is the receiver's to set.
    //
    // This is a gate and not a knee, and the cost is written down rather than
    // argued away: a tone just outside the band is fully inverted while one just
    // inside it is not touched, so a line measured close to the edge can land on
    // either side of it (the reading is good to 0.1-0.2 dB, the edge is a step).
    // What buys that back is that the fitted section is seventeen taps and
    // smooths a step in the target into a ramp over the band it covers, and that
    // the alternative -- correcting only the excess beyond the band -- gave up
    // the thing the band was introduced for: measured on a reading with an
    // eight decibel tilt, the knee took 8.0 to 6.3 dB where the gate takes it to
    // 4.0, i.e. the knee keeps almost none of the tilt correction the actuator
    // window exists to spend. A gate with nothing to correct inside it and a
    // real correction outside it is the version that does both jobs.
    let weighted_mean_db = {
        let (mut sum, mut weights) = (0.0, 0.0);
        for (tone, weight, _) in usable {
            sum += weight * tone.gain_db;
            weights += weight;
        }
        sum / weights
    };
    // Does the band hold any structure the shaper should act on at all?
    //
    // This is one decision for the whole band and not one per tone, and that is
    // the point: what the reading's structure is mostly made of on a good line
    // is *this end's own receive response* (2.31 dB at 3750 Hz on the identity
    // arm, a15), which is the same shape every call, so correcting "the part
    // beyond three decibels" tone by tone would keep inverting half of our own
    // path on every call and would leave a tilt only two of its four available
    // decibels. Either the band's structure is worth inverting or it is not.
    //
    // Measured on a reading with an eight decibel tilt: per-tone correction of
    // the excess takes 8.0 to 5.6 dB and a one-decibel ramp at the edge rings
    // the seventeen-tap section to a cut of -2.37 dB where the actuator allows
    // -2.00; this decision takes the same tilt to 4.0 dB inside the window, and
    // leaves the flat arm at 3.1 -> 3.1, which is what the band is for.
    let structure_db = usable
        .iter()
        .map(|(tone, _, _)| (tone.gain_db - weighted_mean_db).abs())
        .fold(0.0f64, f64::max);
    let band_holds_structure = structure_db > DEAD_BAND_DB;
    let mut target: Vec<f64> = usable
        .iter()
        .map(|(tone, _, h)| {
            if !band_holds_structure {
                // The band is doing its job: no correction, and note what is
                // *not* here -- the absolute `1/h` the target used to be.
                // Inverting the line's gain is not this section's job; the mean
                // is the level, the level is the receiver's, and the
                // normalisation below removes the mean anyway.
                let _ = h;
                1.0
            } else {
                // Mean-removed: only the *shape* of the line is inverted, the
                // rest of what the reading found is level and is not touched.
                10f64.powf(-(tone.gain_db - weighted_mean_db) / 20.0)
            }
        })
        .collect();
    for _ in 0..8 {
        let mean = target.iter().sum::<f64>() / target.len() as f64;
        if !(mean > 0.0) || !mean.is_finite() {
            return Err("the reading has no shape to it");
        }
        let norm = 1.0 / mean;
        for value in &mut target {
            *value = (*value * norm).clamp(1.0 / window, window);
        }
    }

    // The equations: the tones that carry signal, aimed at the shape; the tones
    // that do not, and the band above the one that survives, aimed at unity --
    // left where the line put them, neither lifted nor cut.
    let mut points: Vec<(f64, f64, f64)> = Vec::new();
    for ((tone, weight, _), &value) in usable.iter().zip(&target) {
        points.push((omega(tone.frequency), value, *weight));
    }
    for tone in tones {
        let carried = usable.iter().any(|(t, _, _)| t.frequency == tone.frequency);
        if !carried {
            points.push((omega(tone.frequency), 1.0, QUIET_TONE_WEIGHT));
        }
    }
    let mut hz = high + ABOVE_FROM;
    while hz <= ABOVE_TO {
        points.push((omega(hz), 1.0, ABOVE_WEIGHT));
        hz += ABOVE_STEP;
    }

    // Weighted least squares for the cosine coefficients of the zero-phase
    // response A(w) = sum a_k cos(k w), with the ridge that keeps the system
    // conditioned between the tones.
    let mut a = vec![Complex::ZERO; m * m];
    let mut b = vec![Complex::ZERO; m];
    for &(w, target, weight) in &points {
        if weight <= 0.0 {
            continue;
        }
        for k in 0..m {
            let left = (k as f64 * w).cos();
            for j in 0..m {
                a[k * m + j].re += weight * left * (j as f64 * w).cos();
            }
            b[k].re += weight * left * target;
        }
    }
    let trace: f64 = (0..m).map(|k| a[k * m + k].re).sum();
    if !(trace > 0.0) {
        return Err("the reading gives nothing to design against");
    }
    let ridge = READING_RIDGE * trace / m as f64;
    for k in 0..m {
        a[k * m + k].re += ridge;
    }
    let solved =
        dsp::complex::solve_hermitian(&a, &b).ok_or("the reading's normal equations do not solve")?;
    let coefficients: Vec<f64> = solved.iter().map(|value| value.re).collect();
    let response = |w: f64| -> f64 {
        coefficients.iter().enumerate().map(|(k, &a)| a * (k as f64 * w).cos()).sum()
    };
    let peak = points.iter().fold(0.0f64, |most, &(w, _, _)| most.max(response(w).abs()));
    // How well a section this long realises the shape it was aimed at. That is
    // the only error this design can honestly report: it is a magnitude-only
    // shaper, it shortens nothing, and the channel's own ISI is the receiver's
    // equaliser's to undo, not this number.
    let mut error = 0.0;
    let mut wanted = 0.0;
    for ((tone, weight, _), &value) in usable.iter().zip(&target) {
        let achieved = response(omega(tone.frequency)).abs();
        error += weight * (achieved - value) * (achieved - value);
        wanted += weight * value * value;
    }
    let fit = (error / wanted.max(f64::MIN_POSITIVE)).sqrt();
    // A symmetric section of this response: the middle tap is the constant
    // term and each cosine is half of a tap either side of it.
    let mut taps = vec![0.0; lz2];
    taps[m - 1] = coefficients[0];
    for k in 1..m {
        taps[m - 1 - k] = coefficients[k] / 2.0;
        taps[m - 1 + k] = coefficients[k] / 2.0;
    }
    Ok((coefficients, taps, peak, fit))
}

pub fn from_reading(reading: &Reading, limits: &FilterLimits) -> Result<(Design, Predicted), &'static str> {
    if reading.tones.len() < 6 {
        return Err("the reading has too few tones to design from");
    }
    if limits.total == 0 || limits.most == 0 {
        return Err("the far end offered no filter sections at all");
    }
    // Type I: odd length, symmetric, so the response is real up to the delay on
    // its main tap and the design can be solved for magnitude alone.
    let mut lz2 = READING_TAPS.min(usize::from(limits.most)).min(usize::from(limits.total)).max(5);
    if lz2 % 2 == 0 {
        lz2 -= 1;
    }
    let m = (lz2 + 1) / 2;
    let omega = |hz: f64| std::f64::consts::TAU * hz / SYMBOL_RATE;

    // What the probing says of each tone, and how much of it is worth aiming
    // at. A tone at or under the noise floor carries nothing, so nothing is
    // asked of it: the design leaves it where the line put it, which is what
    // stops a null above the band being inverted.
    let weight_of = |tone: &Tone| -> f64 {
        if !(tone.gain_db.is_finite() && tone.snr_db.is_finite()) || tone.gain_db < -120.0 {
            return 0.0;
        }
        ((tone.snr_db - SNR_FLOOR) / (SNR_FULL - SNR_FLOOR)).clamp(0.0, 1.0)
    };
    let usable: Vec<(&Tone, f64, f64)> = reading
        .tones
        .iter()
        .filter_map(|tone| {
            let weight = weight_of(tone);
            let h = 10f64.powf(tone.gain_db / 20.0);
            (weight > 0.0 && h > 1e-12).then_some((tone, weight, h))
        })
        .collect();
    if usable.len() < 3 {
        return Err("no probing tone carried enough signal to design from");
    }
    let low = usable.iter().map(|(tone, _, _)| tone.frequency).fold(f64::INFINITY, f64::min);
    let high = usable.iter().map(|(tone, _, _)| tone.frequency).fold(f64::NEG_INFINITY, f64::max);
    let cap = 10f64.powf(PEAK_CAP_DB / 20.0);

    // Solve, then check the window on the amplitude the section will really be
    // sent at, and ease the window and solve again if it overshot. Three or
    // four passes is all it ever takes; the fit is smooth in the window.
    let mut used = cap;
    let mut solved = None;
    for _ in 0..6 {
        let (coefficients, taps, peak, fit) = fit_shape(&reading.tones, &usable, used, m, lz2, high)?;
        let energy: f64 = taps.iter().map(|tap| tap * tap).sum();
        if !(energy > 0.0) || !energy.is_finite() {
            return Err("the reading's shape has no response to it");
        }
        // Sum z^2 of one: the power the far end would send with no filter at
        // all, which is the level the receiver's cliff is measured against.
        let scale = 1.0 / energy.sqrt();
        solved = Some((coefficients, taps, peak, scale, fit));
        if scale * peak <= cap * (1.0 + 1e-9) || used <= 1.0 {
            break;
        }
        used *= cap / (scale * peak);
    }
    let (coefficients, mut taps, peak, level_scale, residual) =
        solved.ok_or("the reading's shape does not solve")?;
    let response =
        |w: f64| -> f64 { coefficients.iter().enumerate().map(|(k, &a)| a * (k as f64 * w).cos()).sum() };

    // Q0.15 is the one thing the level cannot pay for: a coefficient it cannot
    // carry is a section the far end cannot run. Scaling down to fit costs that
    // ratio of `sum z^2`, and it is reported rather than hidden.
    let largest = taps.iter().fold(0.0f64, |most, &tap| most.max(tap.abs()));
    if !(largest > 0.0) {
        return Err("the reading's shape has no response to it");
    }
    let scale = level_scale.min(TAP_HEADROOM / largest);
    for tap in &mut taps {
        *tap *= scale;
    }
    let _ = level_scale;

    // What it does, tone by tone.
    let mut tones = Vec::new();
    for tone in &reading.tones {
        let weight = weight_of(tone);
        let amp = scale * response(omega(tone.frequency)).abs();
        let h = if tone.gain_db.is_finite() { 10f64.powf(tone.gain_db / 20.0) } else { 0.0 };
        tones.push(ToneResult {
            frequency: tone.frequency,
            gain_db: tone.gain_db,
            snr_db: tone.snr_db,
            weight,
            z_db: 20.0 * amp.max(1e-12).log10(),
            zh_db: 20.0 * (amp * h).max(1e-12).log10(),
        });
    }
    let power: f64 = taps.iter().map(|tap| tap * tap).sum();
    let mut mean = 0.0;
    let mut weights = 0.0;
    let (mut before_low, mut before_high) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut after_low, mut after_high) = (f64::INFINITY, f64::NEG_INFINITY);
    for tone in &tones {
        if tone.weight <= 0.0 {
            continue;
        }
        mean += tone.weight * 10f64.powf(tone.zh_db / 20.0);
        weights += tone.weight;
        before_low = before_low.min(tone.gain_db);
        before_high = before_high.max(tone.gain_db);
        after_low = after_low.min(tone.zh_db);
        after_high = after_high.max(tone.zh_db);
    }
    mean /= weights.max(f64::MIN_POSITIVE);
    let mut flatness: f64 = 0.0;
    let mut least = f64::INFINITY;
    for tone in &tones {
        if tone.weight <= 0.0 {
            continue;
        }
        flatness = flatness.max(20.0 * (10f64.powf(tone.zh_db / 20.0) / mean).max(1e-12).log10().abs());
        least = least.min(tone.z_db);
    }

    let filters = Filters { z1: Vec::new(), p1: Vec::new(), z2: taps, p2: Vec::new() };
    filters.fits(limits)?;
    let predicted = Predicted {
        taps: lz2,
        band: (low, high),
        level_db: 10.0 * power.max(1e-12).log10(),
        peak_db: 20.0 * (scale * peak).max(1e-12).log10(),
        least_db: least,
        power,
        flatness_db: flatness,
        tilt_before_db: before_high - before_low,
        tilt_after_db: after_high - after_low,
        tones,
    };
    let design = Design {
        filters,
        residual,
        scale: 1.0,
        limited: lz2 < READING_TAPS,
        delay: m - 1,
    };
    Ok((design, predicted))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::decoder::{Decoder, SURVIVOR_DEPTH};
    use crate::v92::upstream::UpEncoder;
    use crate::v92::{CONSTELLATION_FRAME, Parameters, Trellis, up_bits};

    fn limits(sections: u8, total: u8, most: u8) -> FilterLimits {
        FilterLimits::from_info1a(sections, total, most)
    }

    /// One tap at unit gain: nothing to equalise.
    #[test]
    fn an_ideal_channel_gets_a_pass_through_and_no_precoder() {
        let channel = Channel { taps: vec![1.0], noise: 0.0, signal: 1.0 };
        let design = design(&channel, &limits(3, 3, 3)).expect("an ideal channel is designable");
        assert!(design.filters.z2[0] > 0.98, "{:?}", design.filters.z2[0]);
        assert_eq!(design.delay, 0, "an ideal channel adds no delay");
        // The rest of the section is nothing: an ideal channel needs one tap.
        let rest = design.filters.z2[1..].iter().fold(0.0f64, |a, &tap| a.max(tap.abs()));
        assert!(rest < 1e-3, "the prefilter's tail is {rest}, not nothing");
        assert!(design.filters.p1.is_empty(), "nothing to cancel: {:?}", design.filters.p1);
        assert!(design.residual < 1e-3, "residual {}", design.residual);
    }

    /// A channel with the shape a local loop and a codec give it: a null near
    /// DC, flat to 3.4 kHz and a roll-off after. The design has to leave less
    /// ISI than a symbol's noise margin.
    #[test]
    fn a_loop_model_is_equalised_to_well_under_the_level_spacing() {
        let mut taps = vec![0.0; 40];
        taps[0] = 1.0;
        taps[1] = -0.9;
        for k in 2..18 {
            taps[k] = 0.12 / (k as f64 - 1.0);
        }
        let norm = taps.iter().map(|t| t * t).sum::<f64>().sqrt();
        taps.iter_mut().for_each(|t| *t /= norm);
        let channel = Channel { taps, noise: 1e-6, signal: 1.0 };
        let design = design(&channel, &limits(3, 3, 3)).expect("a loop is designable");
        assert!(design.residual_db() > 25.0, "residual {:.1} dB, filters {:?}", design.residual_db(), design.filters);
    }

    /// 8.8.3 and Table 18 bits 14:17: the coefficients have to fit what the
    /// analogue modem offered, whatever the channel wants.
    #[test]
    fn the_design_fits_inside_every_set_of_limits() {
        let mut taps = vec![0.0; 30];
        taps[0] = 1.0;
        for k in 1..30 {
            taps[k] = 0.3 / k as f64;
        }
        let channel = Channel { taps, noise: 1e-6, signal: 1.0 };
        for sections in 0..4u8 {
            for (total, most) in [(0u8, 0u8), (1, 1), (2, 2), (3, 3)] {
                let limit = limits(sections, total, most);
                let design = design(&channel, &limit).expect("designable");
                design.filters.fits(&limit).unwrap_or_else(|error| panic!("{sections}/{total}/{most}: {error}"));
                assert!(design.filters.total() <= usize::from(limit.total));
                if sections & 1 == 0 {
                    assert!(design.filters.z1.is_empty(), "z1 was not offered");
                }
                assert!(design.filters.p2.is_empty(), "p2 is never designed here");
            }
        }
    }

    /// A coefficient set that does not fit is refused by the check it will
    /// face when a CPd carries it.
    #[test]
    fn a_broken_set_is_caught_by_the_self_check() {
        let too_long = Filters { z2: vec![1.0; 130], ..Filters::default() };
        assert!(too_long.fits(&limits(0, 0, 0)).is_err(), "a 130-tap section is over Lmax 128");
        let too_many = Filters { z2: vec![1.0; 100], p1: vec![0.5; 100], ..Filters::default() };
        assert!(too_many.fits(&limits(0, 0, 3)).is_err(), "200 taps is over Ltot 192");
        let unoffered = Filters { z1: vec![0.1], z2: vec![1.0], ..Filters::default() };
        assert!(unoffered.fits(&limits(0, 3, 3)).is_err(), "z1 was not offered");
    }

    /// 8.8.3's NOTE, end to end: the design is what makes the precoder's
    /// symbol-by-symbol power minimisation enough on its own. The encoder's
    /// prefilter output goes down the same channel the design was made for,
    /// and the decoder gets every bit back.
    #[test]
    fn a_designed_chain_decodes_through_its_own_channel() {
        let mut taps = vec![0.0; 24];
        taps[0] = 1.0;
        for k in 1..24 {
            taps[k] = 0.25 / k as f64;
        }
        let norm = taps.iter().map(|t| t * t).sum::<f64>().sqrt();
        taps.iter_mut().for_each(|t| *t /= norm);
        let channel = Channel { taps: taps.clone(), noise: 1e-6, signal: 1.0 };
        let plan = limits(3, 3, 3);
        let design = design(&channel, &plan).expect("designable");
        assert!(design.residual_db() > 20.0, "residual {:.1} dB", design.residual_db());

        let drn = 13u8;
        let parameters = Parameters {
            drn,
            trellis: Trellis::Sixteen,
            extend_e2u: false,
            gain: 1.0,
            moduli: [1u8 << (up_bits(drn) / 12); 12],
            filters: design.filters.clone(),
            sets: vec![(0..64).map(|index| (2 * index + 1) as u16).collect()],
            indices: [0; CONSTELLATION_FRAME],
        };
        let mut encoder = UpEncoder::new(&parameters).expect("feasible");
        let mut decoder = Decoder::new(&parameters);
        let mut seed = 0x1234_5678_u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed >> 63 == 0
        };
        // The channel's own history, as the line carries it.
        let mut line = vec![0.0f64; taps.len()];
        let mut sent = Vec::new();
        let mut received = Vec::new();
        // The design's delay is what a receiver finds on TRN1u before it can
        // frame data, so the first `delay` symbols of the line are skipped.
        let mut skip = design.delay;
        for _ in 0..(design.delay + SURVIVOR_DEPTH * 6 * 12) {
            let symbol = encoder.next_symbol(|| {
                let bit = next();
                sent.push(bit);
                bit
            });
            line.rotate_right(1);
            line[0] = symbol.v;
            let mut seen = 0.0;
            for (k, &tap) in taps.iter().enumerate() {
                seen += tap * line[k];
            }
            if skip > 0 {
                skip -= 1;
                continue;
            }
            // The design scales the prefilter down to fit Q0.15, so what
            // arrives is that much quieter than the levels are. A receiver
            // normalises it, which is what `Design::scale` is for -- the rate
            // choice folds the same number into G.
            decoder.feed(seen * design.scale);
            received.extend(decoder.take_bits());
        }
        assert!(received.len() > sent.len() / 2, "{} bits decoded of {}", received.len(), sent.len());
        let shortest = sent.len().min(received.len());
        let at = (0..shortest.min(400))
            .find(|&offset| {
                let length = shortest - offset;
                length > 100
                    && sent[offset..offset + length].iter().zip(&received[..length]).all(|(a, b)| a == b)
            })
            .unwrap_or_else(|| {
                let errors = sent.iter().zip(&received).filter(|(a, b)| a != b).count();
                panic!("the streams do not line up: {errors} of {shortest} bits wrong")
            });
        let length = shortest - at;
        let errors = sent[at..].iter().zip(&received[..length]).filter(|(a, b)| a != b).count();
        assert_eq!(errors, 0, "{errors} of {length} bits wrong through the designed channel");
    }

    /// The whole upstream chain as a bench sees it: the analogue modem's
    /// precoder and prefilter, the gain, a quantiser, and this end's receiver at
    /// the scale the design tells it to use.
    ///
    /// Returns (bits wrong, bits compared) after lining the two streams up.
    fn through_the_line(quantise: impl Fn(f64) -> f64) -> (usize, usize) {
        let mut taps = vec![0.0; 20];
        taps[0] = 1.0;
        for k in 1..20 {
            taps[k] = 0.35 / k as f64;
        }
        let norm = taps.iter().map(|t| t * t).sum::<f64>().sqrt();
        taps.iter_mut().for_each(|t| *t /= norm);
        let channel = Channel { taps: taps.clone(), noise: 1e-6, signal: 1.0 };
        let design = design(&channel, &limits(3, 3, 3)).expect("designable");
        let drn = 1u8;
        let choice = crate::v92::upchoice::choose(&design, 1e-6, 1 << (drn - 1), Trellis::Sixteen)
            .expect("the slowest rung carries this line");
        let parameters = choice.parameters;
        let mut encoder = UpEncoder::new(&parameters).expect("feasible");
        let mut decoder = Decoder::new(&parameters);
        let mut seed = 0x5eed_1234_u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed >> 63 == 0
        };
        let mut line = vec![0.0f64; taps.len()];
        let (mut sent, mut received) = (Vec::new(), Vec::new());
        for _ in 0..(SURVIVOR_DEPTH * 6 * 12 + 2400) {
            let symbol = encoder.next_symbol(|| {
                let bit = next();
                sent.push(bit);
                bit
            });
            line.rotate_right(1);
            line[0] = symbol.out;
            let seen: f64 = taps.iter().enumerate().map(|(k, &tap)| tap * line[k]).sum();
            // The analogue modem transmits at its own power, well below the
            // codec's full scale: G x v having mean square one is the *design's*
            // unit, not a level to put on a 16-bit codec at.
            let level = 0.25;
            decoder.feed(quantise(seen * level) / level * design.scale);
            received.extend(decoder.take_bits());
        }
        let shortest = sent.len().min(received.len());
        assert!(shortest > 400, "{} bits decoded of {}", received.len(), sent.len());
        let at = (0..shortest.min(400))
            .find(|&offset| {
                let length = shortest - offset;
                length > 200
                    && sent[offset..offset + length].iter().zip(&received[..length]).all(|(a, b)| a == b)
            })
            .unwrap_or(0);
        let length = shortest - at;
        let errors = sent[at..].iter().zip(&received[..length]).filter(|(a, b)| a != b).count();
        (errors, length)
    }

    /// A quantiser fine enough that the design's own residual is all that is
    /// left: the chain is what it should be.
    #[test]
    fn a_designed_chain_survives_a_fine_quantiser() {
        let (errors, length) = through_the_line(|seen| (seen * 32768.0).round() / 32768.0);
        assert_eq!(errors, 0, "{errors} of {length} bits wrong through a 16 bit line");
    }

    /// Through the codec a PCM upstream line actually ends in: the same chain,
    /// to the bit, once the levels are put on the line at a power a codec has
    /// room for.
    ///
    /// That last clause is the whole of it. `G x v` having mean square one is
    /// the design's own unit (Table 30's NOTE), not a level to hand a 16-bit
    /// codec at: sent at full scale the top constellation points sit on
    /// mu-law's 32124 ceiling, clip, and cost six per cent of the bits. The
    /// analogue modem transmits at its own power instead, and the receiver's
    /// `Design::scale` is what folds that back out (which is why it is measured
    /// from the channel estimate and not assumed).
    #[test]
    fn a_mu_law_line_reads_back_through_the_codec() {
        let law = crate::v90::ucode::Law::Mu;
        let (errors, length) = through_the_line(|seen| {
            let (u, negative) =
                crate::v90::ucode::nearest(law, (seen * 32768.0).round().clamp(-32768.0, 32767.0) as i32);
            crate::v90::ucode::amplitude(law, u, negative) as f64 / 32768.0
        });
        assert_eq!(errors, 0, "{errors} of {length} bits wrong through the codec");
    }

    /// [`Channel::estimate`] against a channel it was given: TRN1u's own
    /// levels in, the same channel's output with a little noise, and the taps
    /// come back.
    #[test]
    fn the_estimate_finds_the_channel_it_was_shown() {
        let truth = [0.6, -0.25, 0.1, -0.05];
        let mut seed = 0xfeed_face_u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed >> 63 == 0
        };
        // A two-level GPA-like run, as TRN1u is.
        let known: Vec<f64> = (0..600).map(|_| if next() { 1.0 } else { -1.0 }).collect();
        let mut received = vec![0.0; known.len()];
        for n in truth.len()..known.len() {
            let mut sum = 0.0;
            for (k, &tap) in truth.iter().enumerate() {
                sum += tap * known[n - k];
            }
            received[n] = sum;
        }
        let channel = Channel::estimate(&known, &received, 8).expect("a two-level run is enough");
        for (k, &tap) in truth.iter().enumerate() {
            assert!((channel.taps[k] - tap).abs() < 0.05, "tap {k}: {} against {tap}", channel.taps[k]);
        }
        assert!(channel.noise < 1e-6, "noise {}", channel.noise);
        assert!(channel.severity() > 0.0);
        assert_eq!(channel.impulse(16).len(), 16);
    }


    // -----------------------------------------------------------------
    // V92-63: the prefilter designed from the L1/L2 reading
    // -----------------------------------------------------------------

    /// How far outside its window the realized shape may land: the fit's own
    /// rounding of a corner, in decibels.
    const RIPPLE_DB: f64 = 0.1;

    /// A reading of the twenty-one probing tones with the line and the noise
    /// given as functions of frequency.
    fn reading_of(gain: impl Fn(f64) -> f64, snr: impl Fn(f64) -> f64) -> Reading {
        Reading {
            tones: crate::v34::probe::TONES
                .iter()
                .map(|&(frequency, _)| Tone {
                    frequency,
                    gain_db: gain(frequency),
                    snr_db: snr(frequency),
                })
                .collect(),
            frequency_offset: Some(0.0),
        }
    }

    /// A line 20 dB down and falling 8 dB across the band, with 40 dB of signal
    /// to noise at every tone: what a G.712 loop looks like to L1.
    fn a_loop() -> Reading {
        reading_of(|f| -20.0 - 8.0 * (f - 150.0) / 3600.0, |_| 40.0)
    }

    /// The design is legal, it does not spend level for flatness, and it takes
    /// the tilt out of the band as far as the window goes.
    ///
    /// Level first, because the receiver's cliff is one decibel of flat
    /// attenuation: a design that came out at -26 dB in return for a flat
    /// spectrum would be worse than no design at all, however flat it looked.
    #[test]
    fn a_tilted_reading_keeps_its_level_and_takes_the_tilt_down() {
        let reading = a_loop();
        let plan = limits(3, 3, 3);
        let (design, predicted) = from_reading(&reading, &plan).expect("a loop is designable");
        design.filters.fits(&plan).unwrap_or_else(|error| panic!("{error}"));
        assert_eq!(design.filters.z2.len(), READING_TAPS, "{} taps", design.filters.z2.len());
        assert!(design.filters.p1.is_empty(), "p1 is not designed for: {:?}", design.filters.p1);
        assert!(design.filters.z1.is_empty() && design.filters.p2.is_empty());
        assert_eq!(design.scale, 1.0, "the section fits Q0.15 without a scale");
        assert_eq!(design.delay, (READING_TAPS - 1) / 2, "a symmetric section's own delay");
        let power: f64 = design.filters.z2.iter().map(|tap| tap * tap).sum();
        assert!((power - predicted.power).abs() < 1e-12, "{} against {}", predicted.power, power);
        // The first rule: no level spent. `sum z^2` above one means the far end
        // sends at least what it would have sent unfiltered.
        assert!(predicted.power >= 1.0 - 1e-6, "sum z^2 {:.3}: the design attenuates", predicted.power);
        assert!(predicted.level_db >= -1e-4, "level {:.2} dB", predicted.level_db);
        // The second: inside the actuator's window.
        // The window is aimed at, not clipped onto: a section this long rounds
        // the corner of a shape that is flat and then not, and the rounding
        // shows as a tenth of a decibel either side of the window.
        assert!(predicted.peak_db <= PEAK_CAP_DB + RIPPLE_DB, "peak {:+.2} dB", predicted.peak_db);
        assert!(predicted.least_db >= -PEAK_CAP_DB - RIPPLE_DB, "cut {:+.2} dB", predicted.least_db);
        assert!(predicted.power <= POWER_BUDGET, "sum z^2 {:.2}", predicted.power);
        assert!(design.filters.z2.iter().all(|tap| tap.abs() <= TAP_HEADROOM + 1e-9), "{:?}", design.filters.z2);
        // The third: it buys something. An eight decibel tilt loses at least
        // three of them -- the window is four decibels end to end.
        assert!(
            predicted.tilt_after_db <= predicted.tilt_before_db - 3.0,
            "tilt {:.1} -> {:.1} dB",
            predicted.tilt_before_db,
            predicted.tilt_after_db
        );
        assert!(predicted.flatness_db < 3.0, "flat to {:.2} dB", predicted.flatness_db);
    }

    /// The point of weighting by the signal to noise: a null above the band
    /// that survives is not inverted. The tones past it carry no signal, so
    /// nothing is asked of them and they are left where the line put them.
    #[test]
    fn a_null_above_the_band_is_left_where_the_line_put_it() {
        // The same loop, falling into the noise above 2.7 kHz: those tones read
        // 6 dB of signal to noise, which is under the floor.
        let reading = reading_of(
            |f| -20.0 - 8.0 * (f - 150.0) / 3600.0 - 40.0 * ((f - 2700.0) / 1050.0).max(0.0),
            |f| if f >= 2700.0 { 6.0 } else { 40.0 },
        );
        let (_, predicted) = from_reading(&reading, &limits(3, 3, 3)).expect("designable");
        assert_eq!(predicted.band.1, 2550.0, "the band that survives ends at 2550: {:?}", predicted.band);
        let at = |hz: f64| *predicted.tones.iter().find(|t| t.frequency == hz).expect("a tone");
        let mid = at(1500.0);
        for hz in [2700.0, 3000.0, 3450.0, 3750.0] {
            let tone = at(hz);
            assert_eq!(tone.weight, 0.0, "{hz} Hz was designed toward: {tone:?}");
            assert!(tone.z_db < 1.0, "{hz} Hz was lifted: {tone:?} against {mid:?}");
            assert!(tone.z_db > -3.0, "{hz} Hz was cut: {tone:?}");
        }
        println!("{}", predicted.summary());
        println!("{}", predicted.table());
    }

    /// A clean flat line needs almost nothing: the section is a pass-through
    /// with a delay, and it costs the level nothing.
    #[test]
    fn a_flat_reading_costs_next_to_nothing() {
        let reading = reading_of(|_| 0.0, |_| 45.0);
        let (design, predicted) = from_reading(&reading, &limits(3, 3, 3)).expect("designable");
        let main = design.filters.z2[design.delay];
        assert!(main > 0.9, "the main tap is {main}");
        assert!(predicted.peak_db.abs() < 0.5, "peak {:+.2} dB", predicted.peak_db);
        assert!(predicted.level_db.abs() < 0.5, "level {:+.2} dB", predicted.level_db);
        assert!(predicted.tilt_after_db < 0.5, "flat to {:.2} dB", predicted.tilt_after_db);
    }

    /// A reading with no signal in it is refused, and the caller falls back to
    /// the fit: silence must not become a design.
    #[test]
    fn a_reading_with_no_signal_in_it_is_refused() {
        let dark = reading_of(|_| -40.0, |_| 3.0);
        assert!(from_reading(&dark, &limits(3, 3, 3)).is_err());
        // Three tones are not a reading of a band.
        let mut few = a_loop();
        few.tones.truncate(3);
        assert!(from_reading(&few, &limits(3, 3, 3)).is_err(), "three tones is not a reading");
        // And a far end that offered nothing gets no design from one.
        let mut no_sections = a_loop();
        no_sections.tones.truncate(6);
        let _ = from_reading(&no_sections, &limits(3, 3, 3));
    }

    /// Whatever the line, the section the far end is offered is one it can
    /// carry: inside Q0.15 without a scale, inside the peak, and no level
    /// spent.
    #[test]
    fn every_reading_inside_the_limits_keeps_its_level_and_fits_q0_15() {
        for (tilt, snr) in [(0.0, 45.0), (4.0, 30.0), (8.0, 40.0), (12.0, 40.0), (20.0, 25.0), (30.0, 40.0)] {
            let reading = reading_of(|f| -15.0 - tilt * (f - 150.0) / 3600.0, |_| snr);
            let plan = limits(3, 3, 3);
            let (design, predicted) = from_reading(&reading, &plan)
                .unwrap_or_else(|error| panic!("{tilt} dB tilt: {error}"));
            assert_eq!(design.scale, 1.0, "{tilt} dB tilt needed a scale");
            assert!(
                design.filters.z2.iter().all(|tap| tap.abs() <= TAP_HEADROOM + 1e-9),
                "{tilt} dB tilt: {:?}",
                design.filters.z2
            );
            assert!(predicted.power >= 0.9995, "{tilt} dB tilt: sum z^2 {:.4}", predicted.power);
            assert!(predicted.power <= POWER_BUDGET, "{tilt} dB tilt: sum z^2 {:.3}", predicted.power);
            assert!(
                predicted.peak_db <= PEAK_CAP_DB + RIPPLE_DB,
                "{tilt} dB tilt: peak {:+.2}",
                predicted.peak_db
            );
            assert!(
                predicted.tilt_after_db <= predicted.tilt_before_db + RIPPLE_DB,
                "{tilt} dB tilt got worse: {:.1} -> {:.1}",
                predicted.tilt_before_db,
                predicted.tilt_after_db
            );
        }
    }
}
