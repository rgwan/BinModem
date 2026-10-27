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
use crate::v90::ucode::Law;
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
/// This is the *odd lattice's* gain: `LevelSet::Uniform`'s, which is reachable
/// by name and is not what a build ships. A design on the far end's code grid
/// takes its gain from the code ladder's own mean square
/// ([`codegrid::gain_for`]), which is a different number for the same transmit
/// power: at 24 000 the odd lattice asks for G = 0.1085 and the code ladder for
/// G = 6.17e-4 ([`declared_gain`] is the one to call for the set on the wire).
/// Both are kept here because the selector between them is [`LEVELSET_VAR`], and
/// a reader comparing the two arms needs both numbers side by side.
///
/// It is also the unit of [`rung`](super::rung)'s measurement: one index step of
/// the LC = 8 odd lattice is `2 G` at 349 LSB, and every residual in that module
/// is quoted in those steps whatever the declaration is.
pub fn gain_for(levels: usize) -> f64 {
    let span = 2.0 * levels as f64;
    (3.0 / (span * span - 1.0).max(1.0)).sqrt()
}

/// Which constellation level set [`build`] puts out: `V92_LEVELSET`.
///
/// `code` -- docs/114 R1's far end G.711 code grid ([`codegrid::grid`]) with the
/// set's own gain ([`codegrid::gain_for`]) -- is **the default**. It is the only
/// one of the two that is a set the far end can emit: the far end's transmitter
/// is a codec, so a level that is not a code value is not a level. `uniform` --
/// the odd lattice `a(eta) = 2 eta + 1` with [`gain_for`]'s gain -- is kept
/// reachable by name for the arms that reproduce a call made before this change.
///
/// # Why the default moved, and what it costs
///
/// The old default was the odd lattice, on the ground that "the code grid is
/// better" was an unproven A/B. That question was the wrong one. A uniform
/// declaration is not a *worse* set, it is **not a set at all** above LC 16:
/// its points are index units, the far end rounds each to a code, and from LC 32
/// up two of them round to the same code -- 3 of 32 at LC 32 and 20 of 64 at LC
/// 64 [verified, `the_set_the_cpd_declares_is_a_set_the_far_end_can_emit`]. Below
/// that it is a cost, and `docs/310` sec 1.2 prices it: 19.2 LSB of residual at
/// LC 8, 50.0 at 16, 93.9 at 32, 184.9 at 64, all of it the declaration's own and
/// none of it the line's.
///
/// What the code grid certainly costs is minimum gap, because a maximin code set
/// inside `[8.8.3]`'s power ceiling is coarser than the odd lattice at the same
/// power: 349 -> 256 LSB at 24 000 (-2.7 dB) and 43.6 -> 16 LSB at 48 000
/// (-8.7 dB). That is what the licence has to pay, and the criterion now prices
/// it (`rung::priced_gap_lsb`).
///
/// A variable of its own, and deliberately **not** `V92_CODE_GRID`: that one
/// arms the one-per-call field dump (docs/114 R6), and a dump switch that moved
/// the wire would make every observation of a call an intervention in it.
pub const LEVELSET_VAR: &str = "V92_LEVELSET";

/// The two level sets [`build`] can put out. See [`LEVELSET_VAR`].
///
/// **`Uniform` is the default again** ([`shipped`]), and `Code` stays reachable
/// by name (`V92_LEVELSET=code`), fully implemented, so the closed-loop harness
/// and the suite exercise both out of one binary.
///
/// # `Code` is the arm to reach, and one number holds it off the default
///
/// A uniform declaration is a set the far end **cannot emit**. Its points are
/// `2 eta + 1` in *index units*; the far end's transmitter is a G.711 codec, so
/// every one of them is rounded by its quantiser to a code value. Up to LC 16
/// that is only a cost: each level still lands in a cell of its own, and the
/// rounding is the residual `docs/310` sec 1.2 priced at 19.2 LSB (LC 8). From
/// **LC 32 up it is a failure**: two declared levels land in the same code cell,
/// so one of the pair is never transmitted at all and the receiver is asked to
/// decide between two points that arrived as one.
///
/// [verified] `the_set_the_cpd_declares_is_a_set_the_far_end_can_emit` prints
/// the table: LC 8/16 hold 0 collisions, **LC 32 collides 3 of 32** (`drn 9` to
/// `drn 14`) and **LC 64 collides 20 of 64** (`drn 15` to `drn 19`). That is why
/// `docs/307` sec 3.1's "from LC = 16 up" is corrected to **from LC 32**: the
/// boundary is `drn 9`, not `drn 3`. It is also why the declaration is a
/// modelling defect and not a tuning choice -- no loop, scalar or tracker can
/// make a far end emit a level its codec does not have.
///
/// # Why it is not the default *yet* (`docs/312`)
///
/// S1 made `Code` the shipped default and lost `drn 1`, the one rung the bench
/// carries payload at: the call fell back to V.90 with `B1u` at chance (308 of
/// 576) while the same minute's `abb3b72` + `Uniform` control reached
/// `CONNECT 24000`. The rule on this project is that a change which breaks a
/// working call goes back **first**, and the re-argument has to include a call
/// that passes the class gate *and* carries payload. So the default is
/// `Uniform` until that call exists; the code declaration itself is untouched
/// and stays exercised by the two tests below and by `rung`'s pricing tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LevelSet {
    /// `a(eta) = 2 eta + 1`: the odd lattice. **Undeclarable from LC 32 up**,
    /// which is a reason to fix the code arm and not a reason to spend the
    /// working call -- so this is what ships until the code arm reads the far
    /// end's scale (`docs/312` sec 1, sec 3).
    #[default]
    Uniform,
    /// The far end's own G.711 code values (docs/114 R1): the only one of the
    /// two the far end *can* emit, and held off the default by one measured
    /// number -- `K 2.48168 (mask)` on this arm against `K 1.00000` on the
    /// lattice arm, a level table 2.48x the arriving levels (`docs/312` sec 2).
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
    /// not have. Unset and empty are [`LevelSet::default`] -- the odd lattice:
    /// the default is the default, not a value the caller had to write down.
    pub fn named(raw: Option<&str>) -> Option<Self> {
        match raw.map(str::trim) {
            None | Some("") => Some(LevelSet::default()),
            Some("uniform") => Some(LevelSet::Uniform),
            Some("code") => Some(LevelSet::Code),
            Some(_) => None,
        }
    }

    /// One constellation set's `levels` positive points, on the alphabet the
    /// far end declared.
    ///
    /// `law` is `[Table 23]` bit 35 through [`codegrid::law_if`]: `None` means
    /// no `CPu` has named a law yet, and on the code arm that is a refusal and
    /// not a µ-law declaration (docs/313 sec 2) -- the points of a code set are
    /// code values *of a law*, and there is no law-free code value.
    ///
    /// `uniform` cannot fail for any `levels` [`build`] admits. `code` fails only
    /// for a count the grid cannot hold at all (`levels + 1 > UCODES`, or none
    /// inside the power ceiling), and in particular it does **not** fail at
    /// 48 000: `codegrid::grid(64)` returns 64 points with a 16 LSB gap and an
    /// outermost point of 3 772 LSB, which is the best a 64-point set inside
    /// `[8.8.3]`'s power ceiling can do and not a refusal. A comment here used
    /// to say `build` "returns `None` and the ladder steps down" at 48 000; that
    /// was false, and it is the trap `docs/310` sec 0.3 recorded.
    pub fn points(self, law: Option<Law>, levels: usize) -> Option<Vec<u16>> {
        match self {
            // The odd lattice is `2 eta + 1` in *index units*: a level set and
            // not an alphabet, so it builds under either law and neither is
            // read. The law still enters every consumer of this set that turns
            // an index into a level (`receiver::on_lattice`), which is why the
            // parameter is here rather than dropped.
            LevelSet::Uniform => Some((0..levels).map(|index| (2 * index + 1) as u16).collect()),
            // A code set is code values of one law. No law declared, no set.
            LevelSet::Code => codegrid::grid(law?, levels),
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

/// The level set this build declares **when nothing says otherwise**: what a
/// call with [`LEVELSET_VAR`] unset puts on the wire, and therefore the set the
/// criterion prices ([`rung::priced_gap_lsb`](super::rung::priced_gap_lsb)).
///
/// One name for it, so that a test, a log line and the `CPd` cannot disagree
/// about which set was shipped: the default is a property of the build and not
/// a value each call site re-reads out of the environment.
///
/// **`Uniform`, since `docs/312` sec 1.** S1 had it answer `Code`; the call that
/// argued for it did not connect (`docs/311` sec 5.1) and the control that did
/// connect ran this set (`docs/311` sec 5.2). `Code` is unreachable **by
/// accident** from here -- it is `V92_LEVELSET=code` and nothing else, so
/// flipping the default back is one line and one commit, which is what
/// `docs/312` sec 5 asks for once a call carries payload on it.
pub fn shipped() -> LevelSet {
    LevelSet::named(None).expect("this build has a default level set")
}

/// The gain of the set this call declares at `levels` (8.8.3): the number the
/// receiver builds its expectation from, and the one `CPd` bits 35:50 carry up
/// to the sent section's energy.
///
/// This is the *declared set's* gain and not the odd lattice's closed form, so a
/// printed level cannot disagree with the declaration: at 24 000 on the code grid
/// `1/G` is 1 620 LSB, the level the bench actually reads, where the lattice's
/// `gain_for(8)` names 9.22 index units. `None` when the declared set cannot be
/// built at that count, which [`LevelSet::Code`] answers with `None` only when the
/// grid cannot hold it.
#[must_use]
pub fn declared_gain(law: Option<Law>, levels: usize) -> Option<f64> {
    let set = levelset();
    Some(set.gain(&set.points(law, levels)?))
}

/// The smallest adjacent gap of the set this call declares at `levels`, in LSB.
///
/// The same object [`declared_gain`] is taken from, for the criterion: what the
/// far end can actually emit, in the currency a licence is spent in
/// ([`rung::priced_gap_lsb`](super::rung::priced_gap_lsb) is the caller).
#[must_use]
pub fn declared_gap_lsb(law: Option<Law>, levels: usize) -> Option<f64> {
    declared_gap_of(levelset(), law, levels)
}

/// [`declared_gap_lsb`] with the level set named, so that a caller pricing one
/// arm while another ships (the licence table, docs/313 sec 4) does not have to
/// move the environment to ask its question.
#[must_use]
pub fn declared_gap_of(levelset: LevelSet, law: Option<Law>, levels: usize) -> Option<f64> {
    let points = levelset.points(law, levels)?;
    Some(codegrid::min_gap(&points))
}

/// [`declared_gain`] with the level set named, for the same reason.
#[must_use]
pub fn declared_gain_of(levelset: LevelSet, law: Option<Law>, levels: usize) -> Option<f64> {
    let points = levelset.points(law, levels)?;
    Some(levelset.gain(&points))
}

/// `V92_LEVELSET`'s value for this call.
pub fn levelset() -> LevelSet {
    resolve(std::env::var(LEVELSET_VAR).ok().as_deref())
}

/// [`levelset`] for one value. An unrecognised value sends [`shipped`] -- a typo
/// does not get to move the wire -- and the value it ignored is said out loud
/// once per process, so a batch's log can tell a typo from an environment that
/// never set the variable at all.
pub fn resolve(raw: Option<&str>) -> LevelSet {
    match LevelSet::named(raw) {
        Some(set) => set,
        None => {
            report_ignored(raw.unwrap_or_default());
            shipped()
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
    eprintln!(
        "{LEVELSET_VAR}: {raw:?} is not uniform or code; sending the default, the odd lattice ({})",
        shipped().name()
    );
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
pub fn choose(design: &Design, noise: f64, ja: u32, trellis: Trellis, law: Option<Law>) -> Option<Choice> {
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
        let Some(parameters) = build(design, drn, trellis, law) else { continue };
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
pub fn pin(design: &Design, drn: u8, trellis: Trellis, noise: f64, law: Option<Law>) -> Option<Choice> {
    pin_with(design, drn, trellis, noise, levelset(), law)
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
    law: Option<Law>,
) -> Option<Choice> {
    let parameters = build_with(design, drn, trellis, levelset, law)?;
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
fn build(design: &Design, drn: u8, trellis: Trellis, law: Option<Law>) -> Option<Parameters> {
    build_with(design, drn, trellis, levelset(), law)
}

/// [`build`] with the level set named rather than read from the environment.
fn build_with(
    design: &Design,
    drn: u8,
    trellis: Trellis,
    levelset: LevelSet,
    law: Option<Law>,
) -> Option<Parameters> {
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
    // `code` is docs/114 R1: the far end's own G.711 code values, and the
    // default, because a codec can only emit codes -- a request at a code value
    // makes its quantiser a no-op instead of a half-cell error on every symbol.
    // `uniform` is `a(eta) = 2 eta + 1`, the set every call before this one
    // asked for; it is still buildable by name, and above LC 16 it is a set the
    // far end cannot emit (`LevelSet`'s own comment has the collision counts).
    let set = levelset.points(law, levels)?;
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
        law,
    })
}

/// The same declaration, on a law that has now been declared.
///
/// `CPd` is *designed* in Phase 3 from the second TRN1u and only *released*
/// after the far end's `CPu` has been decoded (9.6.1.1.2's bit 33 acknowledges
/// it, and `CPD_WAITS_FOR_PARKED_TX` is the same ordering one step later), so
/// the phase that builds the set can run before bit 35 arrives (docs/313 sec
/// 2). This is the minimum wiring that closes that gap: the same `levels`, the
/// same `moduli`, the same filters, the gain re-derived from the set the law
/// actually names, and `None` when the new law cannot build the set at all.
///
/// The uniform arm is bit-identical under either law -- its points are index
/// units -- so a call that ships it cannot be moved by this.
#[must_use]
pub fn relaw(parameters: &Parameters, law: Law) -> Option<Parameters> {
    if parameters.law == Some(law) {
        return Some(parameters.clone());
    }
    let levels = parameters.sets.first().map_or(0, Vec::len);
    if levels == 0 {
        return None;
    }
    let set = parameters.levelset.points(Some(law), levels)?;
    let energy = crate::v92::sent_prefilter_energy(&parameters.filters.z2);
    if !(energy > 0.0) || !energy.is_finite() {
        return None;
    }
    let gain = parameters.levelset.gain(&set) / energy.sqrt();
    if !(gain > 0.0) || gain > GAIN_LARGEST || gain * crate::v92::GAIN_SCALE < 1.0 {
        return None;
    }
    let mut out = parameters.clone();
    out.sets = vec![set];
    out.gain = gain;
    out.law = Some(law);
    Some(out)
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

    /// How many LSB one point of the declared set is worth -- the set's own unit,
    /// and the whole of the difference between the two arms.
    ///
    /// * [`LevelSet::Code`]: **one.** The points are G.711 decode values, which
    ///   are the codec's int16 domain; that is what "the far end emits
    ///   `codegrid::value(code)`" means, and it is why `--levelset code --lu 0`
    ///   reads `0.0000` (`docs/276`:147). Rescaling those points to
    ///   [`TARGET_RMS`] by `1609 / rms` would move every one of them off the codes
    ///   it was chosen to be -- that is a *scale*, and the tree measures scales
    ///   (`codegrid::estimate`, `k`) rather than assuming one.
    /// * [`LevelSet::Uniform`]: **half an index step**, `ladder_step(lc) / 2` --
    ///   174.5 LSB at LC 8. Its points are `2 eta + 1` in index units and the
    ///   lattice's step is two of them, so this is exactly the scale that puts the
    ///   set's mean square at `TARGET_RMS`: 9.22 index units x 174.5 LSB = 1609.
    ///
    /// Both are the same statement -- *one point of this set, in LSB* -- and
    /// `[8.8.3]`'s power identity is what makes the second a number rather than a
    /// knob. The code set needs nothing from it: `grid` chose its points under
    /// the power ceiling, so its rms is `TARGET_RMS` to within [`POWER_SLACK`]
    /// and its unit is 1 LSB by construction.
    /// **The set is named here and not taken from [`shipped`].** Since
    /// `docs/312` sec 1 the shipped default is the odd lattice again, and both
    /// tests below are about the *code* declaration: they ask for it by name so
    /// that which arm a build ships cannot decide whether the code grid is a set
    /// the far end can emit. That is the point of keeping them -- `Code` is
    /// implemented and exercised whatever the default is.
    const DECLARED: LevelSet = LevelSet::Code;

    fn lsb_per_point(levels: usize) -> f64 {
        match DECLARED {
            LevelSet::Code => 1.0,
            LevelSet::Uniform => codegrid::ladder_step(levels) / 2.0,
        }
    }

    /// The declared set with its points carried onto the line in LSB.
    fn on_the_line(levels: usize) -> Vec<f64> {
        let scale = lsb_per_point(levels);
        DECLARED.points(Some(Law::Mu), levels).expect("the declared set").iter().map(|&p| scale * f64::from(p)).collect()
    }

    /// The rms of levels in LSB, which is the power the set puts on the line.
    fn levels_rms(levels: &[f64]) -> f64 {
        (levels.iter().map(|level| level * level).sum::<f64>() / levels.len() as f64).sqrt()
    }

    /// **The set the CPd declares has to be a set the far end can emit.**
    ///
    /// The far end's transmitter is a G.711 codec, so the only levels it can put
    /// on the line are code values: a declared level between two codes is
    /// rounded by the far end's quantiser and arrives as a code we did not ask
    /// for. A declared set whose levels share a code is worse than that -- one of
    /// the two levels is **never transmitted at all**, and the receiver is asked
    /// to decide between two points that arrived as one (`docs/310` sec 1.2).
    ///
    /// So: for every rung, take the declared set in LSB, map each level through
    /// the far end's own quantiser ([`codegrid::nearest_code`]), and require `lc`
    /// **distinct** codes back.
    #[test]
    fn the_set_the_cpd_declares_is_a_set_the_far_end_can_emit() {
        // Collected over the whole ladder and asserted at the end, so that one
        // run prints the *table* rather than stopping at the first bad rung.
        let mut bad: Vec<String> = Vec::new();
        for drn in 1..=UP_RATES as u8 {
            let lc = crate::v92::rung::lc_for(drn);
            let levels = on_the_line(lc);
            let mut distinct: Vec<u8> =
                levels.iter().map(|&level| codegrid::nearest_code(Law::Mu, level)).collect();
            distinct.sort_unstable();
            distinct.dedup();
            let shared = lc - distinct.len();
            println!(
                "drn {drn:2} LC {lc:3} N {:3}: {} distinct codes of {lc}{}",
                2 * lc,
                distinct.len(),
                if shared > 0 { format!(", {shared} levels never transmitted") } else { String::new() }
            );
            if shared > 0 {
                bad.push(format!(
                    "drn {drn} (LC {lc}): {shared} of {lc} declared levels share a code with another, \
                     so at most {} of them can ever be transmitted",
                    distinct.len()
                ));
            }
        }
        assert!(bad.is_empty(), "the declaration is not a set the far end can emit:\n{}", bad.join("\n"));
    }

    /// **The declaration's own residual floor, in LSB, for every rung.**
    ///
    /// The companion to the test above, in the currency the licence is spent in:
    /// how far the declared level sits from the code value the far end will
    /// actually emit. `0` means the far end's quantiser is a no-op on the
    /// declaration -- nothing left for the line to explain. Anything else is a
    /// deterministic, zero-mean, per-symbol error the declaration alone created,
    /// and it is not noise: `docs/274`:57 measured its bias at `-0.0007` index
    /// steps, and `--levelset code --lu 0` reads `0.0000` where the declaration
    /// is on the grid (`docs/276`:147).
    ///
    /// What is left after the change is a **scale** and not a floor, and the line
    /// printed per rung says so: the declared set's rms against `TARGET_RMS`.
    /// The code grid's rms is inside [`POWER_SLACK`] of it because `grid` chose
    /// its points under the power ceiling; a scale is one number the far end
    /// obeys and `codegrid::estimate` measures, which is a different object from
    /// a per-symbol offset and is what `tolerance` (`rung.rs:977`) is about.
    #[test]
    fn the_declarations_own_residual_floor_is_zero() {
        let step = crate::v92::rung::index_step_lsb();
        let mut bad: Vec<String> = Vec::new();
        for drn in 1..=UP_RATES as u8 {
            let lc = crate::v92::rung::lc_for(drn);
            let levels = on_the_line(lc);
            let floor_lsb = levels
                .iter()
                .map(|&level| (level - f64::from(codegrid::value_in(Law::Mu, codegrid::nearest_code(Law::Mu, level)))).abs())
                .fold(0.0f64, f64::max);
            let floor = floor_lsb / step;
            println!(
                "drn {drn:2} LC {lc:3} N {:3}: unit {:6.1} LSB, rms {:7.1} ({:+.1}% of TARGET_RMS), \
                 floor {floor_lsb:7.2} LSB = {floor:.4} index steps",
                2 * lc,
                lsb_per_point(lc),
                levels_rms(&levels),
                100.0 * (levels_rms(&levels) / codegrid::TARGET_RMS - 1.0),
            );
            assert!(
                (levels_rms(&levels) / codegrid::TARGET_RMS - 1.0).abs() <= codegrid::POWER_SLACK + 0.02,
                "drn {drn}: the declared set is not at the power the line carries"
            );
            if floor != 0.0 {
                bad.push(format!(
                    "drn {drn} (LC {lc}): {floor_lsb:.2} LSB ({floor:.4} index steps) off the code \
                     grid, against a licence of {:.4}",
                    crate::v92::rung::threshold(Law::Mu, drn)
                ));
            }
        }
        assert!(bad.is_empty(), "the declaration has a residual floor of its own:\n{}", bad.join("\n"));
    }

    /// **The uniform arm is what a build ships again, and it is still the lattice
    /// it always was** (`docs/312` sec 1).
    ///
    /// It has to build, field for field, the set every call before S1 sent: 1,
    /// 3, ..., 15 at 24 000, the gain `gain_for(8)` exactly, and
    /// `Parameters::step` equal to `2 G`. The numbers below are unchanged from
    /// every build that made this the default; what S1 changed and this restores
    /// is **who gets them without asking**. Both arms are reachable by name, and
    /// the test asserts both directions: `uniform` is the default, `code` is one
    /// word away and builds its own set (the test below).
    #[test]
    fn the_uniform_arm_is_still_the_lattice_it_was_by_name() {
        let design = design_for(1e-9);
        let parameters = build_with(&design, 1, Trellis::Sixteen, LevelSet::Uniform, Some(Law::Mu)).expect("drn 1 builds");
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
            let parameters = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform, Some(Law::Mu))
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
        // **Both directions of the revert.** Unset is this set again, and the
        // code grid is one word away -- `V92_LEVELSET=code` -- still built, still
        // the arm the closed-loop harness and the two tests above exercise.
        assert_eq!(LevelSet::named(Some("uniform")), Some(LevelSet::Uniform));
        assert_eq!(LevelSet::named(None), Some(LevelSet::Uniform), "unset has to be the lattice");
        assert_eq!(shipped(), LevelSet::Uniform, "the shipped set is the odd lattice again");
        assert_eq!(LevelSet::named(Some("code")), Some(LevelSet::Code), "the grid has to stay reachable");
        assert_eq!(resolve(Some("code")), LevelSet::Code, "and resolved, not only named");
        let pinned = pin_with(&design, 1, Trellis::Sixteen, 1e-9, LevelSet::Uniform, Some(crate::v90::ucode::Law::Mu)).expect("pinnable");
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
        let parameters = build_with(&design, 1, Trellis::Sixteen, LevelSet::Code, Some(Law::Mu)).expect("drn 1 builds");
        let grid = codegrid::grid(Law::Mu, 8).expect("the grid holds eight points");
        assert_eq!(parameters.sets[0], grid, "not the grid the module builds");
        // Every point is a code value: the µ-law decode of the code it is
        // nearest to. That is what "emits `_DEC[code]` values" means.
        for &point in &grid {
            let code = codegrid::nearest_code(Law::Mu, f64::from(point));
            assert_eq!(codegrid::value_in(Law::Mu, code), point, "code {code} decodes to {point}");
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

    /// An unrecognised `V92_LEVELSET` sends the **default** -- the odd lattice,
    /// since `docs/312` sec 1 -- and says so once.
    ///
    /// The typo does not get a vote on the wire; what it gets is one line, so
    /// that a batch whose log says `levelset uniform` under `V92_LEVELSET=kode`
    /// can be told from one that never set the variable.
    #[test]
    fn an_unrecognised_levelset_sends_the_default_and_is_said_once() {
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
        // The value itself still resolves to the default, by value -- a typo
        // sends what an unset variable sends, and not something of its own.
        assert_eq!(resolve(Some("kode")), LevelSet::Uniform);
        assert_eq!(resolve(None), LevelSet::Uniform);
        let design = design_for(1e-9);
        let parameters = build_with(&design, 1, Trellis::Sixteen, resolve(Some("kode")), Some(Law::Mu)).expect("drn 1");
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
    fn the_gain_sent_belongs_to_the_filter_sent_and_names_a181s_operating_point() {
        let set_gain = |parameters: &Parameters| parameters.levelset.gain(&parameters.sets[0]);
        for (label, design) in [
            ("least squares", design_for(0.0)),
            ("the reading", reading_design()),
        ] {
            for drn in [1u8, 5, 11, 15, UP_RATES as u8] {
                let Some(parameters) = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform, Some(Law::Mu)) else {
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

        // `docs/254` §2: the level a far end must transmit at is `1/G`, and the `G`
        // this API emits names a number **0.03 ... 0.08 LSB below** `docs/253`
        // §1.2's computed `1620.1 / 1640.3 / 1607.1 / 1627.0`. The whole of the
        // difference is one Q0.15 code: `1.0` is not representable in a signed
        // Q0.15 word (its largest value is `32767/32768`), so the sent section's
        // energy comes out just under one -- 0.999976 for this test's 16-tap
        // channel, 0.999939 for the harness's identity section -- and the emitted
        // gain is that much larger than `1/sqrt(ms(a))`. a181's numbers are the
        // *design's* -- the clause's gain for a section of exactly unit energy --
        // and these are the *wire's*. Both are right about different objects, and
        // the difference is at most 0.08 LSB, 1 % of the half gap at LC = 64,
        // which is why it cannot move a decision.
        let design = design_for(0.0);
        for (drn, a181) in [(1u8, 1620.1), (7, 1640.3), (13, 1607.1), (19, 1627.0)] {
            let parameters = pin_with(&design, drn, Trellis::Sixteen, 0.0, LevelSet::Code, Some(Law::Mu))
                .expect("the code grid builds every rung")
                .parameters;
            let energy = crate::v92::sent_prefilter_energy(&parameters.filters.z2);
            assert!(
                energy <= 1.0 && energy > 1.0 - 1e-4,
                "drn {drn}: the sent section's energy is {energy}, not unit to a Q0.15 code",
            );
            // The design's number, recomputed from the set a181 used.
            let computed = 1.0 / codegrid::gain_for(&parameters.sets[0]);
            assert!((computed - a181).abs() < 0.05, "drn {drn}: {computed} against a181's {a181}");
            // The wire's number: what the emitted `G` names.
            let measured = 1.0 / parameters.gain;
            assert!(
                (measured - computed).abs() < 0.1,
                "drn {drn}: the emitted 1/G is {measured}, a181 computed {computed}",
            );
            assert!(measured < computed, "drn {drn}: the sent section is one code short of unit");
        }

        // The pairing that shipped on f8ac825: the unit prefilter's gain beside a
        // section the design had scaled down. Measured with the same instrument,
        // it is `1/scale^2` -- at the design's own `scale` of 12.586 that is
        // 1/158, i.e. 22 dB of transmit power missing from the far end.
        let design = design_for(0.0);
        let shipped = build_with(&design, 5, Trellis::Sixteen, LevelSet::Uniform, Some(Law::Mu)).expect("a rung");
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

    /// The two bits this end leaves at their default are named, with the clause
    /// that makes the default legal and the consequence of it.
    ///
    /// * `CPd` bit 29 (`v92.txt:1818-1821`): "Extend the length of the E2u
    ///   sequence: 0 = don't extend; 1 = extend by 1 symbol." 8.7.2
    ///   (`v92.txt:1470-1471`) makes E2u "a data frame of scrambled,
    ///   differentially encoded zeroes ... extended by a single symbol if bit 29
    ///   of CPd is set". The extension is a lever for the *digital* modem's
    ///   E2u-to-B1u boundary search: it lengthens the descrambled zero run by
    ///   one symbol. This end's search is a candidate-set measurement over the
    ///   run rather than a count (`digital::E2U_SYMBOLS`), so the extra symbol
    ///   buys nothing here; leaving it 0 is compliant, and the same bit "shall
    ///   be set to zero during rate renegotiation and fast parameter exchange"
    ///   anyway. Consequence of the default: E2u is 12 symbols, not 13, and the
    ///   receiver's window must stay 12 -- one constant, `EXTEND_E2U`, so the
    ///   bit and the count cannot disagree.
    /// * `Jp` bits 48 and 49 (Table 22, `v92.txt:1420-1424`; 8.7.6,
    ///   `v92.txt:1717`): 0 = 4-point upstream training constellation, 1 =
    ///   8-point. Consequence of the default: CPu, E2u, SUVu and TRN2u are
    ///   4-point at every rung and in both procedures. That is what this end's
    ///   receiver is built to read (Tables 28/29), so it is correct and not
    ///   merely permitted.
    #[test]
    fn the_e2u_extension_and_the_training_constellation_are_left_at_the_default_with_a_reason() {
        // CPd bit 29: 0, and the receiver's window agrees with it.
        assert!(!EXTEND_E2U, "CPd bit 29 asks the far end to lengthen E2u by a symbol");
        assert_eq!(E2U_SYMBOLS, UP_INTERVALS, "E2u is one data frame: 8.7.2's twelve symbols");
        // Jp bits 48 and 49: 0, in both procedures.
        let jp = crate::v92::sequences::Jp { epsilon: 0x8000, eight_in_training: false, eight_in_renegotiation: false };
        let bits = jp.to_bits();
        assert!(!bits[48], "Jp bit 48 asks for 8-point training constellations");
        assert!(!bits[49], "Jp bit 49 asks for 8-point constellations in a renegotiation");
        // And the parser agrees, so the pin survives a round trip.
        let back = crate::v92::sequences::Jp::from_bits(&bits).expect("a Jp comes back");
        assert_eq!(back, jp);
        // The rungs never move either bit: a CPd built from any rung carries 0.
        let design = design_for(0.0);
        for drn in 1..=UP_RATES as u8 {
            let Some(parameters) = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform, Some(Law::Mu)) else {
                continue;
            };
            assert!(!parameters.extend_e2u, "drn {drn}: CPd bit 29 was set");
        }
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
            let Some(parameters) = build_with(&design, drn, Trellis::Sixteen, LevelSet::Uniform, Some(Law::Mu)) else {
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
        let parameters = pin(&design, 1, Trellis::Sixteen, 1e-9, Some(Law::Mu)).expect("drn 1 is pinnable").parameters;
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
            let Some(choice) = choose(&design, noise, 0, Trellis::ThirtyTwo, Some(Law::Mu)) else {
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
            let Some(parameters) = build(&design, drn, Trellis::Sixteen, Some(Law::Mu)) else { continue };
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
        let chosen = choose(&design, 1e-9, 1, Trellis::Sixteen, Some(Law::Mu)).expect("drn 1 is always offered");
        let pinned = pin(&design, 1, Trellis::Sixteen, 1e-9, Some(Law::Mu)).expect("drn 1 is pinnable");
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
            let choice = pin(&design, drn, Trellis::Sixteen, 1e-9, Some(Law::Mu)).expect("a buildable rung");
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
        let choice = choose(&design, 1e-9, 1u32 << (only - 1), Trellis::Sixteen, Some(Law::Mu)).expect("one rung was offered");
        assert_eq!(choice.parameters.drn, only);
        assert!(choose(&design, 1e-9, 1u32 << 0, Trellis::Sixteen, Some(Law::Mu)).is_some(), "drn 1 is always offered");
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
        let a = choose(&clean, 1e-7, 0, Trellis::Sixteen, Some(Law::Mu)).expect("a clean design");
        let b = choose(&poor, 1e-7, 0, Trellis::Sixteen, Some(Law::Mu)).expect("a poor design still carries something");
        assert!(b.parameters.drn < a.parameters.drn, "ISI cost nothing: {} against {}", b.parameters.drn, a.parameters.drn);
        assert!(b.sigma > a.sigma);
    }
    /// The two companding laws `[Table 23]` bit 35 can name, both ways
    /// (docs/313 sec 1).
    const LAWS: [Law; 2] = [Law::Mu, Law::A];

    /// **Both laws build a legal declaration** (docs/313 sec 1).
    ///
    /// For every rung of the 6.1 ladder and for each G.711 law: the declared set
    /// has `lc` **distinct** codes, its own residual floor is `0.0000` -- the far
    /// end's quantiser is a no-op on every point -- and no point is the zero
    /// point. **[quoted]** `[8.8.3]` (`/tmp/v92.txt:1785-1786`): "Constellations
    /// shall not contain the zero point." That clause names no law, so it holds
    /// for both: under µ-law code 0 decodes to 0 and is refused by
    /// `nearest_code`'s search starting at 1; under A-law code 0 decodes to 8 and
    /// is refused anyway, because it is the silence codeword and not a level a
    /// constellation may hold.
    #[test]
    fn both_laws_build_a_legal_declaration() {
        let mut bad: Vec<String> = Vec::new();
        for law in LAWS {
            for drn in 1..=UP_RATES as u8 {
                let lc = crate::v92::rung::lc_for(drn);
                let Some(set) = LevelSet::Code.points(Some(law), lc) else {
                    bad.push(format!("{law:?} drn {drn} (LC {lc}): the grid built no set"));
                    continue;
                };
                if set.len() != lc {
                    bad.push(format!("{law:?} drn {drn}: LC {lc} but {} points", set.len()));
                }
                if set.windows(2).any(|pair| pair[0] >= pair[1]) {
                    bad.push(format!("{law:?} drn {drn}: the set is not ascending"));
                }
                // The zero point, and the floor.
                let mut floor = 0.0f64;
                let mut distinct: Vec<u8> = Vec::with_capacity(set.len());
                for &point in &set {
                    let code = codegrid::nearest_code(law, f64::from(point));
                    let decoded = codegrid::value_in(law, code);
                    if point == 0 || code == 0 || decoded == 0 {
                        bad.push(format!(
                            "{law:?} drn {drn}: point {point} is the zero point (code {code} decodes to {decoded})"
                        ));
                    }
                    floor = floor.max((f64::from(point) - f64::from(decoded)).abs());
                    distinct.push(code);
                }
                distinct.sort_unstable();
                distinct.dedup();
                if distinct.len() != lc {
                    bad.push(format!(
                        "{law:?} drn {drn}: {} distinct codes for LC {lc}", distinct.len()
                    ));
                }
                if floor != 0.0 {
                    bad.push(format!("{law:?} drn {drn}: residual floor {floor:.4} LSB"));
                }
                // `[8.8.3]`'s power ceiling is the search's constraint, and it is
                // the one number the law does *not* change: both laws decode
                // into the same int16 domain.
                let rms = codegrid::mean_square(&set).sqrt();
                println!(
                    "drn {drn:2} LC {lc:3} N {:3} {law:?}: {} codes, gap {:3.0} LSB, rms {rms:7.1} \
                     ({:+.1}% of TARGET_RMS), floor {floor:.4}",
                    2 * lc,
                    distinct.len(),
                    codegrid::min_gap(&set),
                    100.0 * (rms / codegrid::TARGET_RMS - 1.0),
                );
                if rms > codegrid::TARGET_RMS * (1.0 + codegrid::POWER_SLACK) {
                    bad.push(format!("{law:?} drn {drn}: rms {rms:.1} is over the power ceiling"));
                }
            }
        }
        assert!(bad.is_empty(), "the declaration is not legal under both laws:\n{}", bad.join("\n"));
    }

    /// **The two laws are not the same declaration**, and each one is on its own
    /// alphabet (docs/313 sec 1).
    ///
    /// The assertion that catches a half-wired parameterisation is the second
    /// half of each rung: every point of a set built under `law` must round-trip
    /// through **that** law's code table (`value_in(law, nearest_code(law, p)) ==
    /// p`) and must **not** be a code value of the other law. A `grid` that
    /// ignored its `law` argument would produce one set twice and fail both.
    ///
    /// **The minimum gap is the figure that coincides**, and that is a
    /// measurement rather than a wiring failure: at LC 8, 16, 32 and 64 the two
    /// G.711 ladders hold the same smallest step, so `min_gap` and the licence
    /// built from it are equal while the *sets* are not -- A-law puts its first
    /// point at 328 and its last at 2 496 where µ-law puts 324 and 2 492, and the
    /// declared `G` differs with the mean square. Off the ladder the two gaps
    /// separate, which is what the last block shows: the law does reach the
    /// search, and the coincidence at six counts is a property of the two
    /// ladders (docs/313 sec 4).
    #[test]
    fn the_two_laws_are_not_the_same_alphabet() {
        let mut same_gap: Vec<u8> = Vec::new();
        for drn in 1..=UP_RATES as u8 {
            let lc = crate::v92::rung::lc_for(drn);
            let mu = LevelSet::Code.points(Some(Law::Mu), lc).expect("a mu set");
            let a = LevelSet::Code.points(Some(Law::A), lc).expect("an A set");
            assert_ne!(mu, a, "drn {drn} (LC {lc}): the same points under both laws -- one side is not going through the law");
            assert_ne!(
                LevelSet::Code.gain(&mu),
                LevelSet::Code.gain(&a),
                "drn {drn}: the declared G is the same number under both laws"
            );
            for (law, other, set) in [(Law::Mu, Law::A, &mu), (Law::A, Law::Mu, &a)] {
                let mut foreign = 0;
                for &point in set.iter() {
                    let code = codegrid::nearest_code(law, f64::from(point));
                    assert_eq!(
                        codegrid::value_in(law, code),
                        point,
                        "drn {drn}: {point} is not a {law:?} code value"
                    );
                    let across = codegrid::nearest_code(other, f64::from(point));
                    if codegrid::value_in(other, across) != point {
                        foreign += 1;
                    }
                }
                assert!(
                    foreign > 0,
                    "drn {drn}: every point of the {law:?} set is also a {other:?} code value, so the two alphabets were not told apart"
                );
            }
            if codegrid::min_gap(&mu) == codegrid::min_gap(&a) {
                same_gap.push(drn);
            }
        }
        println!("the two laws' minimum gaps coincide at drn {same_gap:?}");
        // Off the ladder the gaps separate: the law is reaching `grid`.
        let mut separated = Vec::new();
        for lc in 2..=72usize {
            if let (Some(mu), Some(a)) = (codegrid::grid(Law::Mu, lc), codegrid::grid(Law::A, lc))
                && codegrid::min_gap(&mu) != codegrid::min_gap(&a)
            {
                separated.push((lc, codegrid::min_gap(&mu), codegrid::min_gap(&a)));
            }
        }
        println!("off the ladder the gaps separate at {separated:?}");
        assert!(
            !separated.is_empty(),
            "no set size distinguishes the two laws: `grid` is not reading its argument"
        );
    }

    /// **A path that cannot name the law refuses** rather than declaring µ-law
    /// (docs/313 sec 1, sec 2).
    #[test]
    fn a_path_that_cannot_name_the_law_refuses() {
        let design = design_for(1e-9);
        assert!(LevelSet::Code.points(None, 8).is_none(), "a code set with no law was built anyway");
        assert!(
            build_with(&design, 1, Trellis::Sixteen, LevelSet::Code, None).is_none(),
            "an undeclared law produced a code declaration"
        );
        assert!(codegrid::law_if(None).is_none() && codegrid::law_if(Some(false)) == Some(Law::Mu));
        for law in LAWS {
            assert!(
                build_with(&design, 1, Trellis::Sixteen, LevelSet::Code, Some(law)).is_some(),
                "{law:?} cannot build drn 1"
            );
        }
        // The uniform arm is a level set in index units and needs no law; naming
        // one still does not change it.
        let no_law = build_with(&design, 1, Trellis::Sixteen, LevelSet::Uniform, None).expect("the lattice");
        for law in LAWS {
            let named = build_with(&design, 1, Trellis::Sixteen, LevelSet::Uniform, Some(law)).expect("the lattice");
            assert_eq!(named.sets, no_law.sets, "{law:?} moved the odd lattice");
            assert_eq!(named.gain, no_law.gain, "{law:?} moved the odd lattice's G");
        }
        // A declaration can be re-derived on a law that arrives later, and the
        // re-derivation is a different declaration rather than the old set.
        let mu = build_with(&design, 1, Trellis::Sixteen, LevelSet::Code, Some(Law::Mu)).expect("mu drn 1");
        let a = crate::v92::upchoice::relaw(&mu, Law::A).expect("A drn 1");
        assert_eq!(a.law, Some(Law::A));
        assert_ne!(a.sets, mu.sets, "the re-derived declaration kept the mu set");
        assert_ne!(a.gain, mu.gain, "the re-derived declaration kept the mu G");
        assert_eq!(a.moduli, mu.moduli, "the re-derivation moved the modulus encoder");
        assert_eq!(crate::v92::upchoice::relaw(&mu, Law::Mu).map(|p| p.sets), Some(mu.sets));
    }

    /// **The licence numbers, per law** (docs/313 sec 4).
    ///
    /// The criterion is `threshold = SIGMA_MAX * gap / index_step` (docs/277
    /// sec 1.3), and the licence in LSB is `SIGMA_MAX * gap`. `docs/311`'s
    /// figures -- 256 LSB, 0.0688, 24.0 LSB at `drn 1` -- are the µ-law column;
    /// this test prints the A-law column beside it.
    #[test]
    fn the_licence_numbers_per_law() {
        println!("law  drn  LC  min_gap  priced_gap_lsb  threshold  licence_LSB  declared_G");
        let mut rows: Vec<(Law, u8, f64, f64, f64, f64)> = Vec::new();
        for law in LAWS {
            for drn in [1u8, 2, 8, 13, 14, 19] {
                let lc = crate::v92::rung::lc_for(drn);
                let set = LevelSet::Code.points(Some(law), lc).expect("a declared set");
                let gap = codegrid::min_gap(&set);
                let priced = crate::v92::rung::priced_gap(LevelSet::Code, law, drn);
                let threshold = crate::v92::rung::threshold_of(LevelSet::Code, law, drn);
                let licence = threshold * crate::v92::rung::index_step_lsb();
                let gain = LevelSet::Code.gain(&set);
                println!(
                    "{law:?} {drn:2} {lc:3} {gap:8.0} {priced:14.0} {threshold:8.4} {licence:10.1} {gain:10.6}"
                );
                assert!((priced - gap).abs() < 1e-9, "drn {drn}: the priced gap is not the set's");
                assert!(
                    (licence - crate::v92::rung::SIGMA_MAX * gap).abs() < 1e-9,
                    "drn {drn}: the licence is not SIGMA_MAX x the gap"
                );
                rows.push((law, drn, gap, threshold, licence, gain));
            }
        }
        // The published figure, reproduced from the declaration: 256 LSB /
        // 0.0688 / 24.0 LSB at drn 1, under either law's alphabet.
        for law in LAWS {
            let row = rows.iter().find(|row| row.0 == law && row.1 == 1).expect("drn 1");
            assert!((row.2 - 256.0).abs() < 1e-9, "{law:?} drn 1 gap {}", row.2);
            assert!((row.3 - 0.0688).abs() < 5e-4, "{law:?} drn 1 threshold {}", row.3);
            assert!((row.4 - 24.0).abs() < 0.1, "{law:?} drn 1 licence {}", row.4);
        }
        // The gain is where the two laws part at every rung, so a table of
        // licences alone would not show which alphabet a call declared.
        for drn in [1u8, 2, 8, 13, 14, 19] {
            let mu = rows.iter().find(|row| row.0 == Law::Mu && row.1 == drn).expect("mu");
            let a = rows.iter().find(|row| row.0 == Law::A && row.1 == drn).expect("A");
            assert_ne!(mu.5, a.5, "drn {drn}: the declared G is the same under both laws");
        }
    }

}
