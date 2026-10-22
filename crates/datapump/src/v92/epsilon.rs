//! Where the central office's A/D instants fall inside the analogue modem's
//! symbol: measured by the digital modem on **Su**, and sent back in Jp bits
//! 18:33 (8.6.3, 9.5.1.1.6-9.5.1.1.8).
//!
//! The digital modem cannot move the network's sampler, so the far end moves
//! instead: S-bar-u is 24.5T rather than 24T, which slides every later upstream
//! symbol half a symbol, and whatever is left over comes back as epsilon in Jp
//! and the analogue modem applies it by lengthening the next S-bar-u to
//! (24 + epsilon)T. From then on its symbols land on the network's instants,
//! which is the whole point: a two-point stream read half a symbol out is two
//! symbols added together, and that is 19 dB rather than 160 (see
//! [`crate::v92::receiver`]'s table) -- not enough for a CPt's CRC.
//!
//! The Recommendation fixes the signal and the format and says nothing about the
//! estimator, so this one is chosen for what Su is: six symbols of a period, and
//! a fundamental at 8000/6 Hz that is both dominant and easy to phase against.
//! One bin of a six-point DFT over the received Su gives the phase, the same bin
//! over the ideal Su gives the reference, and the difference is how far the far
//! end's symbols sit from ours.

use crate::v92::up_signals::{RU_PERIOD, su};

/// A fraction of a symbol, in epsilon's own units: "16-bit unsigned integer
/// covering the range [0, 1) symbol" (Table 22 bits 18:33).
#[must_use]
pub fn to_field(fraction: f64) -> u16 {
    let wrapped = fraction.rem_euclid(1.0);
    let field = (wrapped * 65536.0).round() as u32;
    field.min(u32::from(u16::MAX)) as u16
}

/// The phase of Su's fundamental in a run of received symbols, in radians.
///
/// Su repeats every six symbols (`RU_PERIOD`), so one bin of a six-point DFT
/// is all of it.
/// Returns None when the window is too short to have a phase, or is silent.
#[must_use]
pub fn phase(received: &[f64]) -> Option<f64> {
    if received.len() < 2 * RU_PERIOD {
        return None;
    }
    let (mut re, mut im) = (0.0, 0.0);
    for (n, &x) in received.iter().enumerate() {
        let theta = -2.0 * std::f64::consts::PI * (n % RU_PERIOD) as f64 / RU_PERIOD as f64;
        re += x * theta.cos();
        im += x * theta.sin();
    }
    if re.hypot(im) < 1e-9 {
        return None;
    }
    Some(im.atan2(re))
}

/// The ideal Su's own phase, so that [`phase`] can be turned into a distance.
fn ideal_phase() -> f64 {
    let ideal: Vec<f64> = (0..RU_PERIOD).map(|n| su(n as u64, false)).collect();
    phase(&ideal).unwrap_or(0.0)
}

/// How far the far end's symbols sit from this end's sampling instants, as a
/// fraction of a symbol in `[0, 1)`.
///
/// Measured on Su: `received` is what the equaliser produced over a run of the
/// far end's Su, one value a symbol.
#[must_use]
pub fn offset(received: &[f64]) -> Option<f64> {
    let measured = phase(received)?;
    // A sixth of a turn is one symbol, so a phase difference over a sixth of a
    // turn is a distance in symbols -- signed so that a symbol arriving *late*
    // comes out positive.
    Some(((measured - ideal_phase()) / std::f64::consts::TAU * RU_PERIOD as f64).rem_euclid(1.0))
}

/// The epsilon Jp carries, from the Su measured before the far end's 24.5T
/// S-bar-u.
///
/// 9.5.1.1.6-7 measures the phase over "the Su before and after the first
/// S-bar-u", and that S-bar-u is itself a shift of half a symbol. Both readings
/// are of the same signal, so the second is the first plus half a symbol: what
/// is left to ask the far end for is the fraction the half symbol does not
/// already cover.
#[must_use]
pub fn measure(before_s_bar: &[f64]) -> Option<u16> {
    let offset = offset(before_s_bar)?;
    // The far end's next symbols will be at `offset + 0.5`; epsilon closes the
    // rest of the way.
    let residual = (offset + 0.5).rem_euclid(1.0);
    Some(to_field(1.0 - residual))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A run of Su, delayed by a known fraction of a symbol and interpolated the
    /// way a band-limited line does to a symbol edge, comes back with that
    /// fraction.
    #[test]
    fn su_says_how_far_its_symbols_are_from_ours() {
        let ideal: Vec<f64> = (0..144).map(|n| su(n as u64, false)).collect();
        for shift in [0.0, 0.25, 0.5, 0.75] {
            let mut delayed = Vec::with_capacity(ideal.len());
            let mut previous = ideal[0];
            for &x in &ideal {
                delayed.push(previous + shift * (x - previous));
                previous = x;
            }
            let found = offset(&delayed).unwrap_or_else(|| panic!("no phase at {shift}"));
            let want = shift.rem_euclid(1.0);
            let error = ((found - want + 0.5).rem_euclid(1.0) - 0.5).abs();
            assert!(error < 0.06, "shift {shift}: offset {found:.3} (error {error:.3})");
        }
    }

    /// And the field is the fraction it stands for, in the units Table 22 uses.
    #[test]
    fn epsilon_is_the_fraction_it_stands_for() {
        for fraction in [0.0, 0.25, 0.5, 0.75, 0.999] {
            let field = to_field(fraction);
            let back = f64::from(field) / f64::from(u16::MAX);
            assert!((back - fraction).abs() < 1.0 / 1024.0, "{fraction} -> {field:#06x} -> {back}");
        }
        assert_eq!(to_field(0.5), 0x8000);
        assert_eq!(to_field(0.25), 0x4000);
        assert_eq!(to_field(0.0), 0x0000);
    }
}
