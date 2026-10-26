//! The numeric half of the upstream design: the constellation sets, the twelve
//! moduli, the rung of the 6.1 rate ladder and the gain, chosen against
//! 2^K <= M (6.4.1) and the class-feasibility rules of 6.4.2.
//!
//! V92-62. [`design`](super::design::design) has said what the channel costs:
//! an ISI residual the precoder could not cancel, a scale the prefilter was
//! shrunk by, and a delay. This module turns that into the numbers CPd
//! carries, from the top of the ladder down, because the answer wanted is the
//! fastest rung that still works.
//!
//! The judgement is 6.4.2's level spacing against the noise. A set of LC
//! positive points on a spacing of two, used uniformly, has a mean square of
//! about (2 LC)^2 / 3; G is chosen to bring that to one (8.8.3), so what a
//! receiver sees at unit gain is a spacing of 2 G. A symbol error needs the
//! noise to carry a point past the halfway mark, which at a symbol error rate
//! of 1e-5 is about 4.5 standard deviations. So a rung is usable when
//!
//! ```text
//! 2 G / 2 >= 4.5 sigma        i.e.   LC <= sqrt(3) / (9 sigma)
//! ```
//!
//! with G = sqrt(3) / (2 LC).
//!
//! **What the code below does, and how it differs from the paragraph above.**
//! [`choose`] compares `LC` against `sqrt(3) / (4.5 sigma)`, not
//! `sqrt(3) / (9 sigma)` -- one factor of two looser, i.e. it admits rungs
//! whose margin is `G >= 2.25 sigma` and whose own [`Choice::predicted`] is
//! around 1e-4 rather than the 1e-5 the paragraph asks for. On the bench of
//! `docs/72` (sigma ~ 1.4e-2) that factor is exactly the boundary between
//! "24000 only" and "32000 legal": drn 7 has `LC = 16`, which the paragraph's
//! `13.7` refuses and the code's `27.5` admits. `docs/86` §2 has the table.
//!
//! # This rule is now a sanity check, not the decision
//!
//! The deciding arithmetic is [`rung`](super::rung): a measured residual sets a
//! **ceiling** on the rung, `sigma_abs <= 2 G(LC) SIGMA_MAX` at a per-symbol
//! error probability of 1e-7, and `digital.rs` hands [`choose`] the Ja mask AND
//! that ceiling. [`MARGIN`]'s bound is the same shape with a looser constant
//! and a looser target: in `LC` terms the ceiling permits
//! `LC <= sqrt(3) * 0.0939 / (2 sigma)` and this rule `LC <= sqrt(3) / (4.5
//! sigma)`, so for every rung of the ladder `residual_max(LC) < sqrt(3) / (4.5
//! LC)` by a factor of about 2.4 -- a rung the ceiling permits is always a rung
//! this rule permits. It therefore cannot bite on a mask that has already been
//! through the ceiling, which is what
//! [`the_ceiling_is_always_tighter_than_the_spacing_rule`] pins.
//!
//! What it still does is the one thing a ceiling cannot do for a caller that
//! has none: refuse a rung on a spacing no measurement vouched for. That is
//! every caller in this crate's tests and [`choose`] used without a mask, so it
//! is kept as a sanity bound rather than deleted -- deleted, [`choose`] would
//! answer the top of the ladder at any noise at all, and it is the only number
//! standing between a caller with no measurement and a rung nothing measured.
//! It is no longer the rule the *call* is made on, and it must not become one
//! again: 4.5 sigma is a citation of 1e-5, not this crate's 1e-7.
//!
//! Anything above the bound the code uses is refused and the next rung down is
//! tried.

use super::codegrid;
use super::design::Design;
use super::{
    CONSTELLATION_FRAME, CONSTELLATION_POINTS, GAIN_LARGEST, Parameters, Trellis, UP_INTERVALS, UP_RATES,
    up_bits,
};

/// Standard deviations between a level and the decision boundary that a symbol
/// error rate of 1e-5 asks for.
///
/// Q(4.5) is 3.4e-6 and a PAM alphabet's worst case is twice that, so a design
/// landing exactly here has a symbol error rate of about 7e-6 before the
/// trellis takes anything off it.
///
/// **A sanity check, not the rule.** The rule a call is made on is
/// [`rung`](super::rung)'s measured ceiling, which is about 2.4 times tighter
/// in `LC` at every rung of the ladder; this bound can therefore only refuse a
/// rung to a caller that supplied no ceiling. See the module comment.
pub const MARGIN: f64 = 4.5;

/// CPd's bit 29, which 8.7.2 says extends E2u by one symbol.
///
/// One constant rather than a literal in `build`, because the *receiver* has
/// to count the same number: [`E2U_SYMBOLS`] is this flag plus a data frame,
/// and the boundary search in `digital.rs` slides its candidates over exactly
/// that many symbols. A CPd that started asking for the extension while the
/// receiver still counted twelve symbols would move B1u one symbol with
/// nothing failing to compile, which is the silent breakage the constant
/// exists to prevent.
///
/// False is what every call this end has made sends. The extension buys a far
/// end nothing *here*: it lengthens the descrambled zero run by one symbol,
/// and the boundary is read from the end of that run -- which the receiver
/// measures over a candidate set rather than trusting a counter for (see
/// `digital::E2U_SYMBOLS`).
pub const EXTEND_E2U: bool = false;

/// E2u's length in symbols (8.7.2): one data frame, plus whatever CPd bit 29
/// asks for.
pub const E2U_SYMBOLS: usize = UP_INTERVALS + if EXTEND_E2U { 1 } else { 0 };

/// The gain a set of `levels` positive points on a spacing of two wants, with
/// its mean square brought to one (8.8.3).
///
/// A uniform alphabet at 1, 3, ..., 2 LC - 1 has a mean square of
/// ((2 LC)^2 - 1) / 3, and G is its reciprocal square root.
///
/// This is the *odd lattice's* gain, and the gain of the default level set
/// ([`LevelSet::Uniform`]). A design on the far end's code grid takes its gain
/// from the code ladder's own mean square ([`codegrid::gain_for`]), which is a
/// different number for the same transmit power: at 24 000 the odd lattice asks
/// for G = 0.1085 and the code ladder for G = 6.17e-4. Both are kept here
/// because the selector between them is [`LEVELSET_VAR`], and a reader comparing
/// the two arms needs both numbers side by side.
pub fn gain_for(levels: usize) -> f64 {
    let span = 2.0 * levels as f64;
    (3.0 / (span * span - 1.0).max(1.0)).sqrt()
}

/// Which constellation level set [`build`] puts out: `V92_LEVELSET`.
///
/// `uniform` -- the odd lattice `a(eta) = 2 eta + 1` with [`gain_for`]'s gain --
/// is the default, and is the set every call made before docs/114 R1 asked the
/// far end for, field for field. `code` is docs/114 R1's far end G.711 code grid
/// ([`codegrid::grid`]) with the set's own gain ([`codegrid::gain_for`]).
///
/// # Why it is a switch and not a default
///
/// The code grid's benefit is a *snapping* offset, and it is unproven: on the
/// replay corpus there is no per-level systematic offset to remove (per-level
/// means within 1.6 LSB of zero, 19-23 sigma away from the offsets the grid
/// predicts), so "the code grid is better" is a hypothesis only a live A/B can
/// settle. What the grid certainly costs is minimum gap, because a maximin code
/// set inside the power ceiling is coarser than the odd lattice at the same
/// power: 349 -> 256 LSB at 24 000 (-2.7 dB) and 43.6 -> 16 LSB at 48 000
/// (-8.7 dB). So the default is the set the line has been shown to carry, and
/// the bench compares the two arms out of one binary.
///
/// A variable of its own, and deliberately **not** `V92_CODE_GRID`: that one
/// arms the one-per-call field dump (docs/114 R6), and a dump switch that moved
/// the wire would make every observation of a call an intervention in it.
pub const LEVELSET_VAR: &str = "V92_LEVELSET";

/// The two level sets [`build`] can put out. See [`LEVELSET_VAR`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LevelSet {
    /// `a(eta) = 2 eta + 1`: the odd lattice. The default.
    Uniform,
    /// The far end's own G.711 code values (docs/114 R1).
    Code,
}

impl LevelSet {
    /// The name the log carries, which is also the value `V92_LEVELSET` takes.
    pub fn name(self) -> &'static str {
        match self {
            LevelSet::Uniform => "uniform",
            LevelSet::Code => "code",
        }
    }

    /// What a `V92_LEVELSET` value names, or `None` for a value this build does
    /// not have. Unset and empty are `uniform`: the default is the default, not
    /// a value the caller had to write down.
    pub fn named(raw: Option<&str>) -> Option<Self> {
        match raw.map(str::trim) {
            None | Some("") | Some("uniform") => Some(LevelSet::Uniform),
            Some("code") => Some(LevelSet::Code),
            Some(_) => None,
        }
    }

    /// One constellation set's `levels` positive points.
    ///
    /// `uniform` cannot fail for any `levels` [`build`] admits; `code` can, and
    /// does at 48 000, where the grid has no 64-point set with a usable gap
    /// (docs/103 §3). That is a rung-specific refusal and not a clamp: [`build`]
    /// returns `None` and the ladder steps down.
    pub fn points(self, levels: usize) -> Option<Vec<u16>> {
        match self {
            LevelSet::Uniform => Some((0..levels).map(|index| (2 * index + 1) as u16).collect()),
            LevelSet::Code => codegrid::grid(levels),
        }
    }

    /// 8.8.3's gain for `points`: `ms(v G) = 1`.
    ///
    /// The odd lattice has a closed form -- a mean square of
    /// `((2 LC)^2 - 1) / 3` -- and the code grid does not, so it is the
    /// reciprocal square root of the set's own mean square. G is a function of
    /// the set and not a knob: the clause that fixes it for one set fixes it for
    /// the other (docs/114 R8).
    pub fn gain(self, points: &[u16]) -> f64 {
        match self {
            LevelSet::Uniform => gain_for(points.len()),
            LevelSet::Code => codegrid::gain_for(points),
        }
    }
}

/// `V92_LEVELSET`'s value for this call.
pub fn levelset() -> LevelSet {
    resolve(std::env::var(LEVELSET_VAR).ok().as_deref())
}

/// [`levelset`] for one value. An unrecognised value sends `uniform` -- a typo
/// does not get to move the wire -- and the value it ignored is said out loud
/// once per process, so a batch's log can tell a typo from an environment that
/// never set the variable at all.
pub fn resolve(raw: Option<&str>) -> LevelSet {
    match LevelSet::named(raw) {
        Some(set) => set,
        None => {
            report_ignored(raw.unwrap_or_default());
            LevelSet::Uniform
        }
    }
}

/// Whether this call is the one that reports an unrecognised `V92_LEVELSET`.
///
/// Once per process: the value cannot differ between the rungs of one call, and
/// a line per rung would bury the call's own notes. The note line carries the
/// level set that actually went out ([`LevelSet::name`]), which is how a batch
/// is classified; this is the explanation for a name that looks wrong.
fn report_ignored(raw: &str) -> bool {
    use std::sync::atomic::{AtomicBool, Ordering};
    static SAID: AtomicBool = AtomicBool::new(false);
    if SAID.swap(true, Ordering::Relaxed) {
        return false;
    }
    eprintln!("{LEVELSET_VAR}: {raw:?} is not uniform or code; sending the odd lattice (uniform)");
    true
}

/// What the choice settled on.
#[derive(Debug, Clone, PartialEq)]
pub struct Choice {
    pub parameters: Parameters,
    /// The standard deviation the choice was made against, in the units the
    /// levels are in: the channel's noise and what the precoder could not
    /// cancel, after the prefilter's scaling.
    pub sigma: f64,
    /// The symbol error rate the spacing predicts, before the trellis.
    pub predicted: f64,
}

/// The fastest rung of 6.1's ladder these conditions carry, or `None` when
/// even the slowest does not.
///
/// `ja` is the mask of rates the analogue modem's Ja allowed (bit k for drn
/// k + 1); a zero mask means "every rung".
pub fn choose(design: &Design, noise: f64, ja: u32, trellis: Trellis) -> Option<Choice> {
    // What the receiver sees, on the scale the levels are in: the design's own
    // residual ISI, and the channel's noise carried through the prefilter's
    // scaling.
    // Both terms are on the levels' own scale: the residual is already
    // relative to the response's main tap, and the noise is carried through
    // the prefilter's scaling -- variance by the square of it.
    let carried = design.scale * design.scale * noise.max(0.0);
    let sigma = (design.residual * design.residual + carried).sqrt();
    let room = if sigma > 0.0 { 3f64.sqrt() / (MARGIN * sigma) } else { f64::INFINITY };
    let mask = if ja == 0 { (1u32 << UP_RATES) - 1 } else { ja };
    for drn in (1..=UP_RATES as u8).rev() {
        if mask & (1 << (drn - 1)) == 0 {
            continue;
        }
        let Some(parameters) = build(design, drn, trellis) else { continue };
        // 6.4.2's class feasibility and Table 30's fields, before the spacing
        // is looked at: a rung whose moduli the sets cannot carry is not a
        // rung at all.
        if parameters.fits().is_err() {
            continue;
        }
        let levels = parameters.sets[0].len();
        if (levels as f64) > room {
            continue;
        }
        let spacing = parameters.step();
        let predicted = 2.0 * normal_tail(spacing / (2.0 * sigma.max(f64::MIN_POSITIVE)));
        return Some(Choice { parameters, sigma, predicted });
    }
    None
}

/// A rung the caller has pinned instead of searched for.
///
/// [`choose`] refuses a rung on 6.4.2's spacing judgement and on the noise
/// estimate behind it; pinning one builds it without asking that judgement.
/// Table 30 and the class rules are still asked -- a parameter set that cannot
/// be encoded, or whose set cannot carry its classes, returns `None` -- and the
/// spacing is reported through `sigma` and `predicted` rather than applied.
///
/// Two callers, both of them a decision already made before this function is
/// reached:
///
/// * the bench's stress arm, which reads `V92_UP_RATE` and pins the receive
///   path at a spacing the design would refuse, and
/// * the floor of [`rung`](super::rung)'s ceiling: a measurement that permits
///   no rung of the ladder leaves the slowest one, because the alternative is
///   no call at all. The sanity rule below that floor would refuse even 24 000
///   at the fit's own noise, so the floor cannot go through [`choose`].
///
/// It cannot become a default by accident: neither caller searches at another
/// rung on the strength of a better line assumed rather than measured.
pub fn pin(design: &Design, drn: u8, trellis: Trellis, noise: f64) -> Option<Choice> {
    pin_with(design, drn, trellis, noise, levelset())
}

/// [`pin`] with the level set named rather than read from the environment.
///
/// The two entry points differ in one argument and nothing else, which is the
/// whole of the switch: `digital.rs` calls [`pin`], and a test that has to hold
/// one arm still calls this.
pub fn pin_with(
    design: &Design,
    drn: u8,
    trellis: Trellis,
    noise: f64,
    levelset: LevelSet,
) -> Option<Choice> {
    let parameters = build_with(design, drn, trellis, levelset)?;
    parameters.fits().ok()?;
    let carried = design.scale * design.scale * noise.max(0.0);
    let sigma = (design.residual * design.residual + carried).sqrt();
    let spacing = parameters.step();
    let predicted = 2.0 * normal_tail(spacing / (2.0 * sigma.max(f64::MIN_POSITIVE)));
    Some(Choice { parameters, sigma, predicted })
}

/// The parameters for one rung, with the moduli spread evenly over the twelve
/// intervals and one constellation set shared by all six constellation frames.
///
/// The set comes from `V92_LEVELSET` ([`LEVELSET_VAR`]); everything else -- the
/// moduli, the class arithmetic, the gain's clause -- is the same either way.
fn build(design: &Design, drn: u8, trellis: Trellis) -> Option<Parameters> {
    build_with(design, drn, trellis, levelset())
}

/// [`build`] with the level set named rather than read from the environment.
fn build_with(design: &Design, drn: u8, trellis: Trellis, levelset: LevelSet) -> Option<Parameters> {
    let bits = up_bits(drn);
    // 6.4.1: K bits over twelve intervals, as evenly as they go. The spare bits
    // go to the intervals that come first, which is where the mixed radix
    // spends them.
    let each = bits / UP_INTERVALS as u32;
    let spare = bits % UP_INTERVALS as u32;
    let mut moduli = [0u8; UP_INTERVALS];
    for (i, modulus) in moduli.iter_mut().enumerate() {
        let width = each + u32::from((i as u32) < spare);
        let levels = 1u32 << width;
        if levels > 255 {
            return None;
        }
        *modulus = levels as u8;
    }
    // 6.4.2: a class steps by Mi, and at k = 3 by 2 x Mi, so the set needs
    // N >= 2 Mi there, which is LC >= Mi. Everywhere else LC >= Mi / 2 is
    // enough.
    let mut levels = 1usize;
    for (i, &modulus) in moduli.iter().enumerate() {
        let need = if i % 4 == 3 { usize::from(modulus) } else { usize::from(modulus.div_ceil(2)) };
        levels = levels.max(need);
    }
    if levels > CONSTELLATION_POINTS {
        return None;
    }
    // The points, from the level set this call is running. `levels` is 6.4.2's
    // LC, unchanged, so the class arithmetic and the moduli above are the same
    // for both sets: what differs is where the points sit.
    //
    // `uniform` is `a(eta) = 2 eta + 1`, the set every call before docs/114 R1
    // asked for. `code` is docs/114 R1: the far end's own G.711 code values, on
    // the ground that a codec can only emit codes, so a request at a code value
    // makes its rounding a no-op and leaves a static per-point offset instead.
    // That offset is what a live A/B has to settle, and it is why this is a
    // switch with the old set as its default rather than a new truth.
    let set = levelset.points(levels)?;
    // 8.8.3: "The digital modem shall design the modulation parameters assuming
    // that, when the prefilter output multiplied by G has a mean-square value
    // of 1, the analogue modem will transmit at the desired power." Read as an
    // equation, that is `E[(G v)^2] = 1`, and `v` is 6.4.2's prefilter output --
    // *the section this CPd declares*, not the unit section it would be
    // convenient to have declared. For a data frame on the level set,
    //
    //     ms(v) = ms(a) * sum z2(k)^2
    //
    // so the clause's gain is `1/sqrt(ms(a) sum z2^2)`: the set's own mean
    // square, from `LevelSet::gain`, divided by the L2 norm of the `z2` that
    // goes on the wire. Both halves are named quantities, and the second is
    // [`crate::v92::sent_prefilter_energy`], which squares the Q0.15 *codes*
    // rather than the design's floats -- the far end runs the codes.
    //
    // This replaces a `G` that was `1/sqrt(ms(a))` unconditionally -- the gain
    // for a unit prefilter -- beside a `z2` that was not unit. That pairing
    // satisfied `E[(G v)^2] = 1` only for a section of unit energy: a design
    // scaled down to fit Q0.15 came out `scale^2` low in power, i.e. the far
    // end transmitted `scale^2` quiet, and `docs/249` §3.2 is that finding.
    // `fold` used to be an environment variable because the folded gain could
    // not fit `4G <= 0xFFFF` at the old `scale`; that `scale` was the defect, so
    // the fold is now unconditional and the field's own limit is the refusal.
    let unit_gain = levelset.gain(&set);
    let energy = crate::v92::sent_prefilter_energy(&design.filters.z2);
    if !(energy > 0.0) || !energy.is_finite() {
        // A section of no energy has no `v` to put a gain on: nothing about
        // its power can be named, and a CPd full of zeros is not a prefilter.
        return None;
    }
    let gain = unit_gain / energy.sqrt();
    if gain > GAIN_LARGEST {
        // The gain this section needs does not fit bits 35:50. Refused rather
        // than clamped: a clamped gain puts a quieter analogue modem on the
        // wire than `E[(G v)^2] = 1` describes, which is the very mismatch this
        // arithmetic exists to remove.
        return None;
    }
    if !(gain > 0.0) || gain * crate::v92::GAIN_SCALE < 1.0 {
        // A gain the field would round to nothing is not a gain.
        return None;
    }
    Some(Parameters {
        drn,
        trellis,
        extend_e2u: EXTEND_E2U,
        gain,
        moduli,
        filters: design.filters.clone(),
        // One set of `levels` positive points, shared by every constellation
        // frame: the simple case of CPd's six. The spacing is two on the odd
        // lattice and the codec's own on the grid, so nothing here may assume
        // it -- `Parameters::step` reads the set.
        sets: vec![set],
        indices: [0; CONSTELLATION_FRAME],
        // The set this call was built on, carried so that the receiver can see
        // it: the K machinery is a property of the code arm alone, and asking
        // the environment again at the receiver is what let it stay armed in
        // the uniform arm (see `Parameters::levelset`).
        levelset,
    })
}

/// The upper tail of the standard normal, Q(x), from the complement of the
/// error function's rational approximation (Abramowitz and Stegun 26.2.17).
fn normal_tail(x: f64) -> f64 {
    if !x.is_finite() {
        return 0.0;
    }
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.231_641_9 * z);
    let poly = t
        * (0.319_381_530
            + t * (-0.356_563_782 + t * (1.781_477_937 + t * (-1.821_255_978 + t * 1.330_274_429))));
    let tail = 0.5 * (-0.5 * z * z).exp() * poly;
    if x >= 0.0 { tail } else { 1.0 - tail }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::design::{Channel, design};
    use crate::v92::{FilterLimits, Filters, up_rate};

    fn plan() -> FilterLimits {
        FilterLimits::from_info1a(3, 3, 3)
    }

    fn design_for(noise: f64) -> Design {
        let mut taps = vec![0.0; 16];
        taps[0] = 1.0;
        for k in 1..16 {
            taps[k] = 0.2 / k as f64;
        }
        let norm = taps.iter().map(|t| t * t).sum::<f64>().sqrt();
        taps.iter_mut().for_each(|t| *t /= norm);
        design(&Channel { taps, noise, signal: 1.0 }, &plan()).expect("designable")
    }

    /// The measured ceiling is strictly tighter than [`MARGIN`]'s spacing rule,
    /// at every rung of the ladder -- so the rule reduced to a sanity check
    /// cannot be what moves a rate.
    ///
    /// The two bounds are the same shape with different constants. The ceiling
    /// permits an absolute residual of `residual_max(LC) = 2 G(LC) SIGMA_MAX`;
    /// the sanity rule permits `sqrt(3) / (4.5 LC)`. Every rung the ceiling
    /// permits is a rung this rule permits, by about 2.4 times, which is why a
    /// mask that has been through `rung::apply_ceiling` cannot be narrowed
    /// further here -- and why `MARGIN` is not the rule the call is made on.
    #[test]
    fn the_ceiling_is_always_tighter_than_the_spacing_rule() {
        for drn in 1..=UP_RATES as u8 {
            let lc = crate::v92::rung::lc_for(drn);
            let ceiling = crate::v92::rung::residual_max(lc);
            let sanity = 3f64.sqrt() / (MARGIN * lc as f64);
            assert!(
                ceiling < sanity,
                "drn {drn} (LC {lc}): the ceiling {ceiling:.5} is not tighter than the sanity rule {sanity:.5}"
            );
            assert!(
                sanity / ceiling > 2.0,
                "drn {drn}: the sanity rule is only {:.2} times the ceiling",
                sanity / ceiling
            );
        }
    }

    /// `gain_for` is 8.8.3's: a uniform set on a spacing of two, brought to a
    /// mean square of one.
    #[test]
    fn the_gain_brings_the_set_s_power_to_one() {
        for levels in [4usize, 8, 32, 64, 128] {
            let gain = gain_for(levels);
            let power: f64 =
                (0..levels).map(|index| (2 * index + 1) as f64).map(|v| v * v).sum::<f64>() / levels as f64;
            assert!((gain * gain * power - 1.0).abs() < 1e-9, "{levels}: {} against 1", gain * gain * power);
            assert!(gain > 0.0 && gain <= GAIN_LARGEST, "{levels}: gain {gain}");
        }
    }

    /// The default level set is the odd lattice, by value: the eight points at
    /// 24 000 are 1, 3, ..., 15, the gain is `gain_for(8)` exactly, and
    /// `Parameters::step` is `2 G` as it was before the code grid existed.
    ///
    /// This is the half of the switch that has to be a *restoration* rather
    /// than a new behaviour: a call made with `V92_LEVELSET` unset is the call
    /// every earlier build made, and the numbers below are the ones this crate
    /// sent then.
    #[test]
    fn the_default_level_set_is_the_odd_lattice_it_replaced() {
        let design = design_for(1e-9);
        let parameters = build_with(&design, 1, Trellis::Sixteen, LevelSet::Uniform).expect("drn 1 builds");
        // LC = 8 at 24 000, and the points are the odd lattice itself.
        assert_eq!(parameters.sets.len(), 1);
        assert_eq!(parameters.sets[0], vec![1, 3, 5, 7, 9, 11, 13, 15]);
        // 8.8.3's closed form, sqrt(3 / ((2 x 8)^2 - 1)), over the energy of the
        // section that is sent. The design normalises `z2` to unit energy, so
        // the divisor is one to the last code (`sent_prefilter_energy` squares
        // the Q0.15 words, not the floats) -- the gain is the closed form to
        // 1e-5 relative, and that residue is the field's rounding, not a design
        // difference.
        let energy = crate::v92::sent_prefilter_energy(&parameters.filters.z2);
        assert!((energy - 1.0).abs() < 1e-4, "the sent section's energy is {energy}, not 1");
        assert_eq!(parameters.gain, gain_for(8) / energy.sqrt());
        assert!((parameters.gain - 0.108_465_228_909_328_08).abs() < 2e-5, "gain {}", parameters.gain);
        // Every other rung of the ladder, by value: LC = 16, 32 and 64 are the
        // same lattice one octave up, and the gain follows the set.
        for (drn, lc) in [(1u8, 8usize), (7, 16), (13, 32), (19, 64)] {
            let parameters = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform)
                .unwrap_or_else(|| panic!("drn {drn} is buildable on the odd lattice"));
            let wanted: Vec<u16> = (0..lc).map(|index| (2 * index + 1) as u16).collect();
            assert_eq!(parameters.sets[0], wanted, "drn {drn}: not the odd lattice");
            // The set's gain over the sent section's energy, which is one here.
            let energy = crate::v92::sent_prefilter_energy(&parameters.filters.z2);
            assert_eq!(parameters.gain, gain_for(lc) / energy.sqrt(), "drn {drn}: gain");
            // The spacing on the line: the lattice's gap is two, so `step` is
            // `2 G` -- the number every pre-code-grid design was scored on.
            assert!(
                (parameters.step() - 2.0 * parameters.gain).abs() < 1e-15,
                "drn {drn}: step"
            );
            assert!((parameters.step() - 2.0 * gain_for(lc)).abs() < 1e-4, "drn {drn}: step");
        }
        // And the two public entry points, unset environment and all, agree
        // with it: `levelset()` is `uniform` when nothing says otherwise.
        assert_eq!(levelset(), LevelSet::Uniform);
        let pinned = pin_with(&design, 1, Trellis::Sixteen, 1e-9, LevelSet::Uniform).expect("pinnable");
        assert_eq!(pinned.parameters.sets[0], vec![1, 3, 5, 7, 9, 11, 13, 15]);
    }

    /// `code` is the far end's G.711 grid by value, and its gain is the set's
    /// own mean square rather than the odd lattice's.
    ///
    /// The points are checked against `codegrid::value` of the code they claim
    /// to be -- i.e. against G.711's own table, `_DEC` -- and not against a
    /// list typed here. The minimum gap is checked against `codegrid::min_gap`,
    /// because that is the quantity the grid costs (256 LSB at 24 000 against
    /// the lattice's 349).
    #[test]
    fn the_code_level_set_is_the_far_end_s_g711_grid() {
        let design = design_for(1e-9);
        let parameters = build_with(&design, 1, Trellis::Sixteen, LevelSet::Code).expect("drn 1 builds");
        let grid = codegrid::grid(8).expect("the grid holds eight points");
        assert_eq!(parameters.sets[0], grid, "not the grid the module builds");
        // Every point is a code value: the µ-law decode of the code it is
        // nearest to. That is what "emits `_DEC[code]` values" means.
        for &point in &grid {
            let code = codegrid::nearest_code(f64::from(point));
            assert_eq!(codegrid::value(code), point, "code {code} decodes to {point}");
        }
        // 8.8.3 on the set's own mean square: 1/sqrt(ms), which at LC = 8 is
        // the codec's scale and not the lattice's. Divided by the sent
        // section's energy, which is one to the last Q0.15 code here.
        assert_eq!(parameters.gain, codegrid::gain_for(&grid) / crate::v92::sent_prefilter_energy(&parameters.filters.z2).sqrt());
        assert!(
            (parameters.gain - codegrid::gain_for(&grid) / crate::v92::sent_prefilter_energy(&parameters.filters.z2).sqrt())
                .abs()
                < 1e-12,
            "gain {}",
            parameters.gain
        );
        assert!((parameters.gain - 6.172_48e-4).abs() < 1e-8, "gain {}", parameters.gain);
        assert!((parameters.gain - gain_for(8)).abs() > 0.1, "the grid's gain is not the lattice's");
        // The gap it costs: 256 LSB at 24 000, against the lattice's 349.
        let gap = codegrid::min_gap(&grid);
        assert!((gap - 256.0).abs() < 1.0, "gap {gap}");
        assert!((parameters.step() - parameters.gain * gap).abs() < 1e-15, "step is not G x the set's gap");
        // And what the gap costs on the line, against the lattice's `2 G`:
        // 0.1580 against 0.2169, i.e. the -2.7 dB docs/114 quotes for 24 000.
        // The lattice's own 349 LSB is not this arithmetic -- it is on the same
        // set scaled to the codec's power, and it is the number the grid gives
        // up (349 -> 256) rather than a step either end sends.
        let loss = 20.0 * (parameters.step() / (2.0 * gain_for(8))).log10();
        assert!((loss + 2.7).abs() < 0.1, "the grid costs {loss:.2} dB, not 2.7");
    }

    /// An unrecognised `V92_LEVELSET` sends the odd lattice and says so once.
    ///
    /// The typo does not get a vote on the wire; what it gets is one line, so
    /// that a batch whose log says `levelset uniform` under `V92_LEVELSET=kode`
    /// can be told from one that never set the variable.
    #[test]
    fn an_unrecognised_levelset_is_uniform_and_is_said_once() {
        assert_eq!(LevelSet::named(None), Some(LevelSet::Uniform), "unset is the default");
        assert_eq!(LevelSet::named(Some("")), Some(LevelSet::Uniform), "empty is the default");
        assert_eq!(LevelSet::named(Some("uniform")), Some(LevelSet::Uniform));
        assert_eq!(LevelSet::named(Some("code")), Some(LevelSet::Code));
        assert_eq!(LevelSet::named(Some("uniform ")), Some(LevelSet::Uniform), "trailing space");
        assert_eq!(LevelSet::named(Some("kode")), None, "not a level set this build has");
        // The report is once per process, and the first value is the one it
        // names: a rung-by-rung repeat would bury the call's own notes.
        assert!(report_ignored("kode"), "the first unrecognised value is reported");
        assert!(!report_ignored("kode"), "and the second is not");
        assert!(!report_ignored("LEVELSET"), "nor a third");
        // The value itself still resolves to the old set, by value.
        assert_eq!(resolve(Some("kode")), LevelSet::Uniform);
        let design = design_for(1e-9);
        let parameters = build_with(&design, 1, Trellis::Sixteen, resolve(Some("kode"))).expect("drn 1");
        assert_eq!(parameters.sets[0], vec![1, 3, 5, 7, 9, 11, 13, 15], "a typo moved the wire");
    }

    /// 8.8.3 on the wire: `E[(G v)^2] = 1` for the filters this CPd declares.
    ///
    /// The measurement is not the formula read back. It plays a white data
    /// frame through 6.4.2's `v(n) = sum x(n-kappa) z2(kappa)` -- the Q0.15
    /// codes, since those are what the far end runs -- multiplies by the `G` the
    /// far end reads out of bits 35:50 after its own rounding, and takes the
    /// mean square. One for both design paths: [`design`], the least-squares
    /// inverse, and [`from_reading`], which is what a call actually designs from.
    ///
    /// The failure it is aimed at is `docs/249` §3.2's: `G = 1/sqrt(ms(a))` for
    /// *every* section, so a section the design had to scale down came out
    /// `scale^2` quiet and the far end transmitted `scale^2` low. The emulation
    /// at the end of this test is that pairing, and it must not measure 1.
    #[test]
    fn the_gain_sent_belongs_to_the_filter_sent() {
        let set_gain = |parameters: &Parameters| parameters.levelset.gain(&parameters.sets[0]);
        for (label, design) in [
            ("least squares", design_for(0.0)),
            ("the reading", reading_design()),
        ] {
            for drn in [1u8, 5, 11, 15, UP_RATES as u8] {
                let Some(parameters) = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform) else {
                    continue;
                };
                // What the far end runs and what it multiplies by: the codes and
                // the field, not the floats and the parameter.
                let gain = crate::v92::gain_from_4g(crate::v92::four_g_from_gain(parameters.gain));
                let measured = mean_square_out(gain, &parameters.sets[0], &parameters.filters.z2);
                assert!(
                    (measured - 1.0).abs() < 0.05,
                    "{label}, drn {drn}: ms(G v) is {measured:.4}, not 1 (G {gain:.6}, sum z2^2 {:.4})",
                    crate::v92::sent_prefilter_energy(&parameters.filters.z2),
                );
                // And `G` is that set's gain over that section's energy, which is
                // the provenance the doc names.
                let energy = crate::v92::sent_prefilter_energy(&parameters.filters.z2).sqrt();
                // Within the field's own step: bits 35:50 carry 4G in Q0.16, so
                // `G` is the computed value to half of 2^-18.
                assert!(
                    (gain - set_gain(&parameters) / energy).abs() <= 0.5 / crate::v92::GAIN_SCALE + 1e-12,
                    "{label}, drn {drn}: G is {} not {} -- the set's gain over the sent section's energy",
                    gain,
                    set_gain(&parameters) / energy,
                );
            }
        }

        // The pairing that shipped on f8ac825: the unit prefilter's gain beside a
        // section the design had scaled down. Measured with the same instrument,
        // it is `1/scale^2` -- at the design's own `scale` of 12.586 that is
        // 1/158, i.e. 22 dB of transmit power missing from the far end.
        let design = design_for(0.0);
        let shipped = build_with(&design, 5, Trellis::Sixteen, LevelSet::Uniform).expect("a rung");
        let mut shrunk = shipped.clone();
        for tap in &mut shrunk.filters.z2 {
            *tap /= 2.5;
        }
        let old_gain = crate::v92::gain_from_4g(crate::v92::four_g_from_gain(set_gain(&shipped)));
        let old = mean_square_out(old_gain, &shipped.sets[0], &shrunk.filters.z2);
        assert!(
            (old - 1.0 / 6.25).abs() < 0.05,
            "the old pairing measured {old:.4}, not 1/scale^2 = 0.16",
        );
        // The fix is the gain, not the filter: the same shrunk section with the
        // gain this module now computes for it -- the set's gain over the energy
        // actually sent, i.e. 2.5 times larger, which this example needs past
        // `4G <= 0xFFFF` and so past what `build_with` would emit -- is exactly
        // 1 again.
        let fixed = mean_square_out(set_gain(&shipped) * 2.5, &shipped.sets[0], &shrunk.filters.z2);
        assert!((fixed - 1.0).abs() < 0.05, "the corrected gain measured {fixed:.4}");
    }

    /// `p2` is absent by decision, and the decision has a clause and a bound.
    ///
    /// 6.4.2 (`v92.txt:463-466`) prints `v(n) = sum x(n-kappa) z2(kappa) +
    /// sum v(n-kappa) p2(kappa)`, so `LP2 > 0` is a recursive prefilter and
    /// `LP2 = 0` an FIR one. Both designers here produce an FIR section, so
    /// every CPd this end emits declares `LP2 = 0` -- Table 30's "p2(1) :
    /// p2(LP2) (if LP2 > 0)" makes that a legal length rather than a gap -- and
    /// nothing may put a `p2` on the wire without a designer behind it. The
    /// bound on the field, should one ever exist, is INFO1a bits 12:17: the
    /// section must have been offered (bits 12:13, codes 2 and 3 name `p2`) and
    /// `LP2` counts in `Ltot <= 192/256/320/384` and `Lmax <= 128/192/256/320`.
    #[test]
    fn a_prefilter_without_a_feedback_section_is_declared_not_omitted() {
        let design = design_for(0.0);
        for drn in 1..=UP_RATES as u8 {
            let Some(parameters) = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform) else {
                continue;
            };
            assert!(parameters.filters.p2.is_empty(), "drn {drn}: an undesigned p2 reached the wire");
        }
        let reading = reading_design();
        assert!(reading.filters.p2.is_empty(), "the reading's shaper is FIR");
        // The bound the field would be held to: a p2 the far end did not offer
        // is refused before the wire, and one that fits is only refused by the
        // lengths INFO1a announced.
        let offered = FilterLimits::from_info1a(0, 0, 0); // p1 and z2 only
        let mut with_p2 = Filters { z2: vec![1.0], p2: vec![0.25], ..Filters::default() };
        assert!(with_p2.fits(&offered).is_err(), "a p2 nobody offered was accepted");
        with_p2.p2.truncate(1);
        assert!(with_p2.fits(&plan()).is_ok(), "a single p2 is inside Ltot and Lmax");
        // The section would also change `v`: a pole on the prefilter's own
        // output is a different `v`, hence a different 8.8.3 gain. That is why
        // the absence is a decision and not a default -- see the module docs of
        // `Filters::p2`.
        let mut state = 0.0f64;
        for _ in 0..32 {
            state = 1.0 + 0.25 * state;
        }
        assert!(state > 1.3, "a p2 of 0.25 raises the section's steady-state output to {state}");
    }

    /// A design from the L1/L2 probing of a G.712-like loop: the path a call
    /// actually takes (`Prefilter::Reading` is the default).
    fn reading_design() -> Design {
        use crate::v34::probe::{Reading, Tone};
        let reading = Reading {
            tones: crate::v34::probe::TONES
                .iter()
                .map(|&(frequency, _)| Tone {
                    frequency,
                    gain_db: -20.0 - 8.0 * (frequency - 150.0) / 3600.0,
                    snr_db: 40.0,
                })
                .collect(),
            frequency_offset: Some(0.0),
        };
        crate::v92::design::from_reading(&reading, &plan()).expect("a loop is designable").0
    }

    /// `E[(G v)^2]` for a white data frame on `set` through a section `z2`, with
    /// `z2` and `G` taken as the sixteen-bit fields carry them.
    ///
    /// White in the sense 6.4.2's design assumption uses: every point of the set
    /// equally often, in an order no modulator would produce but whose mean
    /// square converges on the set's own. The frame is long enough (4096
    /// symbols) that the sampling error is under 2 per cent, which is the
    /// tolerance the callers use.
    fn mean_square_out(gain: f64, set: &[u16], z2: &[f64]) -> f64 {
        use crate::v92::{PREFILTER_FRACTION, PREFILTER_WIDTH, signed_q, to_signed_q};
        let codes: Vec<f64> = z2
            .iter()
            .map(|&tap| signed_q(to_signed_q(tap, PREFILTER_WIDTH, PREFILTER_FRACTION), PREFILTER_WIDTH, PREFILTER_FRACTION))
            .collect();
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        let mut v = Vec::with_capacity(4096);
        for _ in 0..4096 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            // Both signs: a CPd lists positive points on a spacing of two and
            // 6.4.2's `a(eta)` is the whole two-sided set, so a frame that only
            // ever sent +1, +3, ... would carry a DC of 8 that no `z2` with
            // `sum z2 != 0` would leave alone.
            let sign = if (state >> 40) & 1 == 0 { 1.0 } else { -1.0 };
            v.push(sign * f64::from(set[(state >> 33) as usize % set.len()]));
        }
        let mut out = vec![0.0; v.len()];
        for n in 0..v.len() {
            for (k, &code) in codes.iter().enumerate() {
                if n >= k {
                    out[n] += code * v[n - k];
                }
            }
        }
        let tail = &out[codes.len().max(1)..];
        tail.iter().map(|value| (gain * value) * (gain * value)).sum::<f64>() / tail.len() as f64
    }

    /// 8.7.2's extension bit and the receiver's count of E2u are one number.
    ///
    /// CPd bit 29 lengthens E2u by a symbol, and E2u is what the receiver's
    /// boundary candidates are counted over (`digital::E2U_SYMBOLS`). Both come
    /// from [`EXTEND_E2U`], so a future change to the bit cannot leave the
    /// receiver searching a window that no longer contains the boundary.
    #[test]
    fn the_e2u_length_and_the_bit_this_end_sends_agree() {
        let design = design_for(1e-9);
        let parameters = pin(&design, 1, Trellis::Sixteen, 1e-9).expect("drn 1 is pinnable").parameters;
        assert_eq!(parameters.extend_e2u, EXTEND_E2U, "CPd bit 29 is the constant the receiver counts from");
        assert_eq!(E2U_SYMBOLS, UP_INTERVALS + usize::from(EXTEND_E2U));
        assert_eq!(
            crate::v92::digital::E2U_SYMBOLS,
            E2U_SYMBOLS,
            "the receiver's window and the sequence's length are the same number"
        );
    }

    /// A channel with almost no noise carries the top of the ladder, a noisy
    /// one carries less, and the choice never goes back up as the noise does.
    #[test]
    fn the_rate_falls_as_the_noise_rises() {
        let mut previous = u8::MAX;
        let mut seen = Vec::new();
        for noise in [1e-9, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2] {
            let design = design_for(noise);
            let Some(choice) = choose(&design, noise, 0, Trellis::ThirtyTwo) else {
                seen.push((noise, 0));
                continue;
            };
            let drn = choice.parameters.drn;
            assert_eq!(choice.parameters.rate(), up_rate(drn));
            assert!(drn <= previous, "the rate went up with the noise: {drn} after {previous} at {noise}");
            previous = drn;
            choice.parameters.fits().unwrap_or_else(|error| panic!("{noise}: {error}"));
            seen.push((noise, drn));
        }
        println!("noise -> drn: {seen:?}");
        assert!(seen[0].1 >= 15, "a quiet channel only reached drn {:?}", seen[0]);
        assert!(seen.last().expect("a trend").1 <= 8, "a noisy channel still reached drn {:?}", seen.last());
    }

    /// Every rung the choice can return obeys 6.4.1 and 6.4.2: the moduli
    /// carry K bits and no interval's set is too small for its class step.
    #[test]
    fn every_rung_it_can_return_is_feasible() {
        for drn in 1..=UP_RATES as u8 {
            let design = design_for(0.0);
            let Some(parameters) = build(&design, drn, Trellis::Sixteen) else { continue };
            parameters.fits().unwrap_or_else(|error| panic!("drn {drn}: {error}"));
            assert_eq!(parameters.bits(), up_bits(drn));
            assert!(parameters.product() >= 1u128 << up_bits(drn), "drn {drn}: the moduli cannot hold K bits");
            for i in 0..UP_INTERVALS {
                let n = 2 * parameters.sets[0].len();
                let step = usize::from(parameters.moduli[i]) * if i % 4 == 3 { 2 } else { 1 };
                assert!(n >= step, "drn {drn}, interval {i}: N {n} is smaller than its class step {step}");
            }
        }
    }

    /// Pinning the slowest rung hands back exactly what the search would have.
    ///
    /// This is the offline half of "the 24 000 CPd does not move": `pin` is the
    /// only new entry point, and at drn 1 it agrees with the path a default
    /// call takes, field for field.
    #[test]
    fn pinning_the_slowest_rung_changes_nothing() {
        let design = design_for(1e-9);
        let chosen = choose(&design, 1e-9, 1, Trellis::Sixteen).expect("drn 1 is always offered");
        let pinned = pin(&design, 1, Trellis::Sixteen, 1e-9).expect("drn 1 is pinnable");
        assert_eq!(chosen.parameters, pinned.parameters);
        assert_eq!(chosen.sigma, pinned.sigma);
    }

    /// The rungs this bench can offer and the set each one needs, derived
    /// rather than typed: 4, 5 and 6 bits a symbol ask for 16, 32 and 64
    /// positive magnitudes, and N = 2 x LC saturates 6.4.2's k = 3 rule at
    /// every one of them. 48 000 is the top of the ladder *and* the top of what
    /// Table 30 can carry: 64 positive points is `SET_POINTS_SENT`.
    #[test]
    fn the_rates_this_bench_offers_and_the_sets_they_need() {
        let design = design_for(0.0);
        let mut table = Vec::new();
        for (rate, drn, modulus, lc) in
            [(24_000u32, 1u8, 8usize, 8usize), (32_000, 7, 16, 16), (40_000, 13, 32, 32), (48_000, 19, 64, 64)]
        {
            let choice = pin(&design, drn, Trellis::Sixteen, 1e-9).expect("a buildable rung");
            let parameters = &choice.parameters;
            assert_eq!(parameters.rate(), rate);
            assert_eq!(up_bits(drn), 12 * modulus.ilog2(), "{rate}: K against the modulus");
            assert_eq!(parameters.moduli, [modulus as u8; UP_INTERVALS]);
            assert_eq!(parameters.sets[0].len(), lc, "{rate}: LC");
            assert_eq!(2 * lc, 2 * modulus, "{rate}: N against k = 3's 2 x Mi");
            parameters.fits().unwrap_or_else(|error| panic!("{rate}: {error}"));
            // Table 30's gain field: "4 x G > 0" in unsigned Q0.16.
            //
            // The tolerance is 1 %, not 0.1 %: on the far end's code grid `G`
            // is the codec's own scale (6.17e-4 at 24 000, `codegrid::gain_for`)
            // rather than the odd lattice's 0.1085, so the field's quantum is
            // 1/262144 of that, i.e. 0.6 % -- exactly the resolution `docs/114`
            // R8 warns about, and it is why `G` is not used as a grid knob. At
            // 0.6 % the gain is 0.05 dB, five times finer than the 0.25 dB
            // `Table 27` bits 27:31 report in, so nothing this end can measure
            // would see it.
            let four_g = crate::v92::four_g_from_gain(parameters.gain);
            assert!(four_g > 0);
            assert!((crate::v92::gain_from_4g(four_g) - parameters.gain).abs() / parameters.gain < 1e-2);
            table.push((rate, lc, 2 * lc, four_g));
        }
        println!("rate, LC, N, 4G: {table:?}");
        assert_eq!(table.iter().map(|row| row.1).collect::<Vec<_>>(), vec![8, 16, 32, 64]);
        // The last rung's N is the most this end will put in a set, so the
        // ladder's 48 000 is the ceiling CPd itself sets.
        assert_eq!(2 * 64, CONSTELLATION_POINTS);
        assert_eq!(64, crate::v92::sequences::SET_POINTS_SENT);
    }

    /// A rate the analogue modem's Ja did not offer is never chosen.
    #[test]
    fn the_ja_mask_is_obeyed() {
        let design = design_for(1e-9);
        let only = 3u8;
        let choice = choose(&design, 1e-9, 1u32 << (only - 1), Trellis::Sixteen).expect("one rung was offered");
        assert_eq!(choice.parameters.drn, only);
        assert!(choose(&design, 1e-9, 1u32 << 0, Trellis::Sixteen).is_some(), "drn 1 is always offered");
    }

    /// The residual the precoder could not cancel is noise as far as the choice
    /// is concerned: a channel it cannot shorten carries less.
    #[test]
    fn residual_isi_counts_as_noise() {
        let clean = Design {
            filters: Filters { z2: vec![1.0], ..Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        // Three per cent of a spacing: worse than a good design, better than
        // a broken one, which is the case that should lose a rung or two
        // rather than everything.
        let poor = Design { residual: 0.03, ..clean.clone() };
        let a = choose(&clean, 1e-7, 0, Trellis::Sixteen).expect("a clean design");
        let b = choose(&poor, 1e-7, 0, Trellis::Sixteen).expect("a poor design still carries something");
        assert!(b.parameters.drn < a.parameters.drn, "ISI cost nothing: {} against {}", b.parameters.drn, a.parameters.drn);
        assert!(b.sigma > a.sigma);
    }
}
