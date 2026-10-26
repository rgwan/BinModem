//! The rung ceiling: what a *measurement* says a rung carries, as opposed to
//! what the schedule is willing to offer.
//!
//! V.92's upstream rate has two inputs and this module is the second one. The
//! analogue modem's Ja names the rungs it will offer at all (Table 20); the
//! digital modem's own reading of the line names the rungs the line will carry.
//! The rung chosen is the fastest one in
//!
//! ```text
//! candidates = (Ja mask) AND (the rungs this measurement permits)   -- never OR
//! ```
//!
//! and the second factor is a **ceiling**: evidence can take a rung away and can
//! never put one back. Nothing here raises a rung by assuming a better line than
//! the one that was measured.
//!
//! That is the shape the Dialfire 2960 host firmware names, with a
//! `STATUS__MSE_REPORT` and a *separate* `STATUS__MSE_REPORT_MAXRATE`
//! (`/tmp/reports/dialfire-v92.md` §5, `docs/96-v92-data-pump-model.md` §19.1
//! item 1): a measured MSE reports the rate it would allow, alongside the Ja
//! mask rather than instead of it. Our own rate path had the wrong shape in
//! both directions -- `digital.rs` ANDed the mask with the slowest rung, and
//! `upchoice::choose` walked the ladder at a sigma that came from
//! `rx.snr_db()`, divided the noise by four and retried when a rung was
//! refused, up to nine times. A retry that assumes a better SNR is the opposite
//! of a ceiling: it turns the measurement into a lower bound on the rung.
//!
//! # The arithmetic
//!
//! An upstream set's levels sit on one spacing, and `G` (8.8.3) is chosen so
//! that spacing is one **index step** in `Design`'s units: half a step is `G`,
//! two half-cells are one index step, and the levels are the odd multiples of
//! `G`. Write everything the equaliser's output does that the grid does not --
//! uncancelled ISI, line noise carried through the prefilter's scaling -- as a
//! Gaussian of `sigma` index steps. A symbol is decided wrongly when that
//! residual carries it past the halfway mark, so, one dimension:
//!
//! ```text
//! p_symbol = 2 Q(1 / (2 sigma))            Q(x) = the normal's upper tail
//! ```
//!
//! A uniform alphabet's two outermost points have a single neighbour rather
//! than two, which makes the average over the alphabet `2 Q (1 - 1/(2 LC))`
//! rather than `2 Q`; the difference is under 7 % of the figure at LC = 8 and
//! under 1 % at LC = 64, and it is on the safe side of the comparison, so the
//! worst case above is what is carried.
//!
//! Inverting it at the target gives the one number this module is built on: a
//! rung is permitted only while the residual **on that rung's own grid** is at
//! or below
//!
//! ```text
//! SIGMA_MAX = 1 / (2 Q^-1(TARGET / 2)) = 0.0939 index steps
//! ```
//!
//! ## The target, and why 1e-7 per symbol
//!
//! [`TARGET`] is `1e-7`, a symbol error probability *before* the trellis. Three
//! reasons, in the order they bind:
//!
//! 1. **The bench brackets it.** The two measured points below reproduce what
//!    happened only for `sigma_max` in `[0.0800, 0.1027)`, which is `TARGET` in
//!    `[4.1e-10, 1.14e-6)`. `1e-7` sits 17 % above the floor of that bracket
//!    (below which 24 000 is refused although it worked) and 9 % below its
//!    ceiling (above which 32 000 is permitted although it failed). `1e-6` is
//!    inside it too, by 0.5 % -- inside the noise of a 361-symbol measurement --
//!    so the tighter of the two is the defensible one. A target looser than
//!    about `1.1e-6` would have *chosen* 32 000 on the 32 000 measurement,
//!    i.e. would have predicted the call the bench watched fail.
//! 2. **It is a rate, not a tier.** At the slowest rung (8 000 symbols/s) one
//!    pre-trellis symbol error every `1/(8000 * 1e-7)` = 1 250 s is a link that
//!    the trellis makes comfortable; at `1e-6` it is one every 125 s before the
//!    trellis does anything, which is a rung that is being asked to survive on
//!    coding gain.
//! 3. **It claims nothing that is not in this file.** See the trellis note
//!    below. The target is the only free parameter here, and it is pinned by an
//!    observation rather than by a specification.
//!
//! ## Trellis coding: not claimed
//!
//! Both ends run a convolutional code over the 4D symbol -- 6.4.4 for V.92's
//! own encoder ("the convolutional encoders from ITU-T V.34 shall be used ...
//! except that the 2T delays are replaced by 4T delays", v92.txt:478-482),
//! 6.4.3 for the `2 y(k) + 1` index domain it runs in (v92.txt:470-474), and
//! Table 30 bits 27:28 for the digital receiver's requirement on the far end
//! (v92.txt:1810-1814). **This note used to cite "(6.5)", and V.92 has no
//! clause 6.5**: the text goes from 6.4.4 straight to "7 Interchange circuits"
//! (v92.txt:487), so the code's citation is 6.4.3-6.4.4 (docs/246 sec 3.2).
//! So the *real*
//! symbol error rate at a given spacing is better than `2 Q(1/(2 sigma))` --
//! typically by a dB figure that is quoted second-hand in every modem design
//! note. That figure is not used here and must not be: a threshold that moves
//! on a coding gain nobody on this bench has measured is a guessed threshold
//! with a citation stapled to it, which is the shape this work exists to
//! remove. If a coding gain is ever to move [`TARGET`], it has to arrive as a
//! measured curve of the trellis's own error rate against sigma, on this
//! receiver, with its source named. Until then the ceiling is deliberately
//! uncoded, and therefore conservative.
//!
//! # One residual, four grids
//!
//! A half-cell is `G`, and `G = sqrt(3 / (4 LC^2 - 1))` (8.8.3,
//! [`gain_for`](super::upchoice::gain_for)) nearly halves for each doubling of
//! the set. The *absolute* residual is a property of the line and is the same
//! on every rung; the number of **index steps** it is worth is not, because the
//! grid it is measured against shrinks with LC. A measurement is therefore
//! carried as the absolute residual
//!
//! ```text
//! sigma_abs = 2 G(lc_measured) sigma_measured          (one index step = 2 G)
//! sigma(lc) = sigma_abs / (2 G(lc))                    (on a candidate's grid)
//! ```
//!
//! and it is re-expressed on each candidate rung's grid before it is compared
//! with [`SIGMA_MAX`]. That single rescaling is the whole LC dependence of the
//! ceiling: at a fixed absolute residual, a rung whose set is twice as big sees
//! twice the sigma in index steps and is refused first. [`ceiling_rung`] takes
//! `lc_of` for that reason, and [`lc_for`] is this crate's own answer, derived
//! from K the way 6.4.1 and 6.4.2 derive the builder's sets.
//!
//! Internally a measurement is normalised to **index steps on the LC = 8 grid**,
//! which is the 24 000 arm's and the grid the bench's level streams are read
//! on ([`REFERENCE_LC`]). The two normalisations are the same number in
//! different units; [`Sigma::at`] converts between them and
//! [`Sigma::absolute`] takes the `Design`'s units directly.
//!
//! # The ceiling, rung by rung
//!
//! `sigma_max(LC)` is the largest measurement, in index steps of the LC = 8
//! grid, that a rung whose set is `LC` can carry. `residual_max(LC)` is the same
//! budget in `Design`'s absolute units, and is the column that a rung actually
//! changes:
//!
//! ```text
//! drn  rate   K   LC   sigma_max(LC)    residual_max(LC)
//!   1  24000  36    8   0.0939           0.02036
//!   7  32000  48   16   0.0469           0.01017
//!  13  40000  60   32   0.0234           0.00508
//!  19  48000  72   64   0.0117           0.00254
//! ```
//!
//! Each doubling of LC costs 6 dB of residual budget, and the ladder spends
//! three rungs to reach one: 6.4.1 gives K = 2 (drn + 17) over twelve intervals,
//! so three rungs is one more bit per interval and one more doubling of the
//! set. The 8 000 bit/s step is therefore 2 dB of residual budget, which is the
//! exchange rate this whole module is denominated in.
//!
//! # What it predicts, against what the bench saw
//!
//! Two level streams were recorded on the bench, one per configuration, and the
//! equaliser's output divided by `G` and compared with the nearest odd multiple
//! gives the residual (`tests/data/*.score.levels`, the `score` run of the
//! stream the decoder is fed through B1u):
//!
//! ```text
//! recorded at        sigma on its own grid   on the LC = 8 grid   ceiling
//! 24 000, LC = 8     0.0513                  0.0513               drn 2, so 24 000
//! 32 000, LC = 16    0.1602                  0.0800               drn 2, so 32 000 refused
//! ```
//!
//! (`drn 2` rather than `drn 1` because drn 2's set is also 8 points: the
//! ceiling steps in LC, not in rungs. Nothing above drn 2 survives either
//! measurement.)
//!
//! Both measurements refuse 32 000, and the 32 000 run is exactly the one the
//! bench watched fail and fall back to 24 000. The 24 000 measurement refuses
//! 32 000 as well, which is the soundness test this rule was built against: a
//! criterion that would have *chosen* 32 000 on a 0.16-index-step residual
//! predicts a call that did not happen, and is wrong however well it does on
//! the other point. Note that neither measurement reaches 40 000 or 48 000 --
//! that is not a bug in the ceiling, it is this bench, and the moment a stream
//! with `sigma < 0.0117` at LC = 64 exists the ceiling will offer 48 000.
//!
//! # Which way each error goes
//!
//! The ceiling falls as sigma rises ([`ceiling_rung`] is monotone, tested), so
//! the direction of an error in the measurement decides the direction of the
//! mistake in the rate:
//!
//! * **`Bound::Upper`** -- the true residual is *no larger* than the number
//!   given. The ceiling computed from it is *no higher* than the true ceiling:
//!   the rule under-offers and never over-offers. Safe, and it is what a real
//!   measurement gives.
//! * **`Bound::Lower`** -- all that is known is that the residual is *no
//!   smaller*. The ceiling computed from it can be *higher* than the truth, so
//!   using it as if it were a reading is optimistic. [`ceiling_rung`] refuses to
//!   do that: a lower bound is spent at the slowest rung, which is the only
//!   ceiling a lower bound justifies.
//! * **`Bound::Reading`** -- the number is the residual, as far as the receiver
//!   can tell.
//!
//! The quantity available before CPd goes out today is `rx.snr_db()`, and the
//! code beside it says what it is: a *floor* on the SNR, because the fit is
//! against TRN1u, a two-point stream with two thirds of its power at 4 000 Hz
//! where a loop has a null. A floor on the SNR is a **ceiling on the noise**,
//! so `sigma = sqrt(residual^2 + scale^2 10^(-snr/10))` is an **upper bound on
//! sigma** -- `Bound::Upper`, the safe direction, and the reason
//! [`Sigma::from_design`] says so. On a real loop it will therefore still choose
//! 24 000 most of the time, but now because the number says so rather than
//! because a constant does, and the day the noise figure is a reading rather
//! than a floor the rung follows it up.
//!
use super::design::Design;
use super::upchoice::gain_for;
use super::{JA_MASK_ALL, UP_INTERVALS, UP_RATES, up_bits, up_rate};

/// The grid a measurement is normalised to: index steps of the LC = 8 set, the
/// 24 000 rung's (`drn 1`) and the grid the bench's level streams are read on.
pub const REFERENCE_LC: usize = 8;

/// The per-symbol error probability a rung has to reach, before the trellis.
///
/// Inverted into a spacing this is [`SIGMA_MAX`]; the doc comment above gives
/// the three reasons it is this number, and the first of them is that the two
/// measured bench points bracket it.
pub const TARGET: f64 = 1e-7;

/// The largest residual, in index steps **of the rung's own grid**, that
/// [`TARGET`] allows: `1 / (2 Q^-1(TARGET / 2))`.
///
/// Carried as a constant rather than computed because the inverse normal is not
/// in `std`; [`the_constant_is_the_target_inverted`] checks this number against
/// the forward arithmetic, so the two cannot drift apart.
pub const SIGMA_MAX: f64 = 0.093_866_3;

// ---------------------------------------------------------------------------
// Rungs: drn, rate, K and LC
// ---------------------------------------------------------------------------

/// One rung of 6.1's ladder, with the four numbers that name it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rung {
    /// The rung of the 6.1 ladder, CPd bits 22:26. 1 to 19.
    pub drn: u8,
    /// 6.1's rate, rounded down as [`up_rate`] rounds it.
    pub rate: u32,
    /// K, the data bits of one twelve-symbol frame (6.4.1).
    pub k: u32,
    /// LC, the positive points of the constellation set the rung needs: the
    /// largest any of its twelve intervals asks for, which is the modulus at
    /// the interval that steps by 2 Mi (6.4.2).
    pub lc: usize,
}

impl Rung {
    /// The rung `drn` names, or `None` when `drn` is not on the ladder.
    ///
    /// drn 0 is not a rate: CPd bits 22:26 and CPu bits 21:25 use it for
    /// cleardown (9.11).
    #[must_use]
    pub fn of(drn: u8) -> Option<Rung> {
        (1..=UP_RATES as u8).contains(&drn).then(|| Rung {
            drn,
            rate: up_rate(drn),
            k: up_bits(drn),
            lc: lc_for(drn),
        })
    }

    /// The rung whose rate is `rate`, rounded the way 6.1 rounds it.
    ///
    /// The ladder's step is 8 000/6, so 25 333 and 26 666 are rungs and 25 000
    /// is not; the search is from the top so that a rate two rungs could claim
    /// after rounding goes to the faster one, which is the one that was asked
    /// for.
    #[must_use]
    pub fn of_rate(rate: u32) -> Option<Rung> {
        (1..=UP_RATES as u8).rev().find(|&drn| up_rate(drn) == rate).and_then(Rung::of)
    }
}

/// LC, the positive points of the set a rung needs, from K (6.4.1, 6.4.2).
///
/// 6.4.1 spreads K over the twelve intervals as evenly as it goes, so an
/// interval's modulus is `2^(K / 12)` or one bit more for the `K % 12`
/// intervals that come first; 6.4.2 then says a set carries an interval at the
/// trellis step only if `N >= 2 Mi`, i.e. `LC >= Mi`, and elsewhere if
/// `N >= Mi`. LC is the largest any interval asks for.
///
/// The four rungs this bench runs are the ones where K/12 is exact --
/// drn 1/7/13/19 give 36/48/60/72 bits over twelve intervals, 3/4/5/6 bits each,
/// and LC = 8/16/32/64 -- and [`lc_for_agrees_with_the_builder`] checks the
/// other fifteen against `upchoice::pin`'s own sets.
#[must_use]
pub fn lc_for(drn: u8) -> usize {
    if !(1..=UP_RATES as u8).contains(&drn) {
        return 0;
    }
    let bits = up_bits(drn);
    let each = bits / UP_INTERVALS as u32;
    let spare = bits % UP_INTERVALS as u32;
    let mut lc = 1usize;
    for i in 0..UP_INTERVALS {
        let width = each + u32::from((i as u32) < spare);
        let modulus = 1usize << width;
        let need = if i % 4 == 3 { modulus } else { modulus.div_ceil(2) };
        lc = lc.max(need);
    }
    lc
}

// ---------------------------------------------------------------------------
// The ceiling
// ---------------------------------------------------------------------------

/// The largest interval modulus a rung's twelve intervals ask for: how many
/// positive points the *worst* interval of 6.4.1's even-ish spread needs.
///
/// `lc_for` answers "how many points is the set", which is the same number only
/// when `K/12` is exact -- the four rungs this bench has run (36/48/60/72 bits,
/// three to six a interval, LC 8/16/32/64). One rung further up the ladder, 6.4.1
/// gives the first `K % 12` intervals one bit more, and a 32-point interval
/// needs a code-grid gap of 64 LSB where an 8-point one has 320. Pricing
/// 25 333 at LC = 8 is how a model that says 0.0939 licenses a rung the bench
/// watched fail ([`threshold`], `docs/240` sec 3c).
#[must_use]
pub fn points(drn: u8) -> usize {
    if !(1..=UP_RATES as u8).contains(&drn) {
        return 0;
    }
    let bits = up_bits(drn);
    let each = bits / UP_INTERVALS as u32;
    let spare = bits % UP_INTERVALS as u32;
    let mut widest = 1usize;
    for i in 0..UP_INTERVALS {
        let width = each + u32::from((i as u32) < spare);
        widest = widest.max(1usize << width);
    }
    widest
}

/// One index step of the LC = 8 grid in the codec's LSB: `2 G` at
/// [`REFERENCE_LC`], times [`codegrid::TARGET_RMS`] (`docs/241` sec 0: 349 LSB).
///
/// This is the denominator `docs/240` sec 3c's gap correction is taken against,
/// and it is the clause's own number: `[8.8.3]` fixes `G`, and the far end's
/// power identity fixes what one unit of it is in LSB.
#[must_use]
pub fn index_step_lsb() -> f64 {
    2.0 * super::upchoice::gain_for(REFERENCE_LC) * crate::v92::codegrid::TARGET_RMS
}

/// The residual, in index steps of the LC = 8 grid, that a rung actually needs:
/// [`SIGMA_MAX`] scaled by the smallest gap a maximin set of [`points`] codes
/// can hold, over one index step.
///
/// ```text
/// drn   rate   points  gap(LSB)  threshold
///   1  24000      8      320      0.0861
///   2  25333     16      128      0.0344
///   7  32000     16      128      0.0344
///  13  40000     32       64      0.0172
///  19  48000     64       16      0.0043
/// ```
///
/// `sigma_max(lc)` answers a different question -- what the *mean square* of a
/// `lc`-point set buys -- and for a rung whose set is carried by an interval one
/// bit wider than `K/12` it is nearly three times too generous. A decision taken
/// against a gap has to clear half of it, so the gap is the number the criterion
/// is denominated in, and this is the only threshold [`ceiling_rung`] compares
/// with. It is *tighter* than `sigma_max` everywhere except the LC = 8 rung the
/// bench has measured, which is the conservative direction (`docs/240` sec 3c).
#[must_use]
pub fn threshold(drn: u8) -> f64 {
    let points = points(drn);
    if points < 2 {
        return 0.0;
    }
    gap_lsb(points).map_or(0.0, |gap| SIGMA_MAX * gap / index_step_lsb())
}

/// The smallest gap a *maximin* set of `points` positive µ-law codes holds, in
/// the codec's LSB, at the power `[8.8.3]` sets (`docs/116` sec 7, re-derived in
/// `docs/240` sec 3c's gap column).
///
/// Taken from that column rather than searched here, because the search this
/// crate has ([`crate::v92::codegrid::grid`]) is greedy from the bottom and
/// finds 256 LSB at eight points where a maximin set holds 320 -- a 20 % tighter
/// reading of the same rung, which is a modelling difference and not a
/// measurement. The four counts the ladder's twelve intervals actually ask for
/// (8, 16, 32, 64 -- 6.4.1's `2^(K/12)` with the spare bits on the first
/// intervals) are the ones tabulated; anything else is searched.
#[must_use]
pub fn gap_lsb(points: usize) -> Option<f64> {
    let tabulated = match points {
        8 => Some(320.0),
        16 => Some(128.0),
        32 => Some(64.0),
        64 => Some(16.0),
        _ => None,
    };
    tabulated.or_else(|| crate::v92::codegrid::grid(points).map(|set| crate::v92::codegrid::min_gap(&set)))
}

/// The largest measurement, in index steps of the **LC = 8 grid**, that a rung
/// whose set is `lc` points can carry.
///
/// A measurement is worth more index steps on a coarser grid and fewer on a
/// finer one, because a half-cell is `G` and `G` falls with LC: so this falls as
/// `lc` grows and a bigger set needs a better line. The value at
/// `lc == REFERENCE_LC` is [`SIGMA_MAX`] itself, and [`sigma_max_table`] prints
/// the four rungs.
#[must_use]
pub fn sigma_max(lc: usize) -> f64 {
    SIGMA_MAX * gain_for(lc) / gain_for(REFERENCE_LC)
}

/// The same budget in `Design`'s absolute units: the residual a rung's grid can
/// absorb, one half-cell being `G`.
#[must_use]
pub fn residual_max(lc: usize) -> f64 {
    2.0 * gain_for(lc) * SIGMA_MAX
}

/// `sigma_max` for the four rungs this bench runs, as (drn, rate, K, LC,
/// `sigma_max`, `residual_max`).
#[must_use]
pub fn sigma_max_table() -> [(u8, u32, u32, usize, f64, f64); 4] {
    let mut table = [(0u8, 0u32, 0u32, 0usize, 0.0f64, 0.0f64); 4];
    for (slot, drn) in table.iter_mut().zip([1u8, 7, 13, 19]) {
        let rung = Rung::of(drn).expect("a rung of the ladder");
        *slot = (drn, rung.rate, rung.k, rung.lc, sigma_max(rung.lc), residual_max(rung.lc));
    }
    table
}

/// The per-symbol error probability a residual of `sigma` index steps predicts,
/// before the trellis (see the trellis note in the module comment).
#[must_use]
pub fn symbol_error(sigma: f64) -> f64 {
    if !(sigma > 0.0) {
        return 0.0;
    }
    2.0 * normal_tail(1.0 / (2.0 * sigma))
}

// ---------------------------------------------------------------------------
// The measurement
// ---------------------------------------------------------------------------

/// What the number beside a measurement is a bound on.
///
/// The ceiling falls as sigma rises, so an upper bound on sigma gives a ceiling
/// that is *never too high* and a lower bound gives one that can be. See "Which
/// way each error goes" in the module comment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bound {
    /// The residual, as far as the receiver can tell.
    Reading,
    /// The residual is no larger than this: the safe kind, and what a noise
    /// figure that is a floor on the SNR gives.
    Upper,
    /// The residual is no smaller than this: optimistic, and spent at the
    /// slowest rung rather than trusted as a ceiling.
    Lower,
}

/// A residual measurement, normalised to index steps of the [`REFERENCE_LC`]
/// grid, with the direction of its own error carried beside it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sigma {
    steps: f64,
    bound: Bound,
}

impl Sigma {
    /// A measurement in index steps on the grid of `lc`: what the bench's level
    /// streams give, `|level| / G` against the nearest odd multiple.
    #[must_use]
    pub fn reading(steps: f64, lc: usize) -> Sigma {
        Sigma { steps: steps * gain_for(lc) / gain_for(REFERENCE_LC), bound: Bound::Reading }
    }

    /// The same, known to be no smaller than `steps`.
    #[must_use]
    pub fn lower(steps: f64, lc: usize) -> Sigma {
        Sigma { bound: Bound::Lower, ..Sigma::reading(steps, lc) }
    }

    /// A measurement in `Design`'s absolute units -- the residual itself, where
    /// one index step of a set of `lc` points is `2 G(lc)`.
    #[must_use]
    pub fn absolute(sigma: f64, bound: Bound) -> Sigma {
        Sigma { steps: sigma / (2.0 * gain_for(REFERENCE_LC)), bound }
    }

    /// The residual a `Design` and a noise figure imply, which is what the rate
    /// path already computes: the ISI the precoder could not cancel, and the
    /// channel's noise carried through the prefilter's scaling.
    ///
    /// The bound is [`Bound::Upper`] because the noise figure available before
    /// CPd is `rx.snr_db()`, and that is a floor on the SNR -- see the module
    /// comment. A floor on the SNR is a ceiling on the noise, so this is a
    /// ceiling on sigma, which is the direction that cannot invent a rung.
    #[must_use]
    pub fn from_design(design: &Design, noise: f64) -> Sigma {
        let carried = design.scale * design.scale * noise.max(0.0);
        Sigma::absolute((design.residual * design.residual + carried).sqrt(), Bound::Upper)
    }

    /// The measurement in index steps of the [`REFERENCE_LC`] grid, which is
    /// what [`sigma_max`] is quoted in.
    #[must_use]
    pub fn steps(&self) -> f64 {
        self.steps
    }

    /// The same residual in index steps of a set of `lc` points: one index step
    /// is `G` smaller here, so the number is `G` larger.
    #[must_use]
    pub fn at(&self, lc: usize) -> f64 {
        self.steps * gain_for(REFERENCE_LC) / gain_for(lc)
    }

    /// What the number beside this measurement is a bound on.
    #[must_use]
    pub fn bound(&self) -> Bound {
        self.bound
    }

    /// The per-symbol error probability this residual predicts on a set of `lc`
    /// points, before the trellis.
    #[must_use]
    pub fn error(&self, lc: usize) -> f64 {
        symbol_error(self.at(lc))
    }
}

/// The highest rung a measurement permits, or `None` when even the slowest rung
/// is more than the line carries.
///
/// `lc_of` gives each candidate rung's set size; [`lc_for`] is this crate's own.
/// The measurement is re-expressed on each candidate's grid, so a rung with a
/// larger set has to clear a tighter bound on the same absolute residual.
///
/// **The property this function exists for:** the answer is a ceiling. It never
/// exceeds what the same measurement gave at a smaller sigma, and it never looks
/// at the Ja mask -- [`apply_ceiling`] is the AND. A [`Bound::Lower`] measurement
/// is spent at the slowest rung, because a lower bound cannot justify anything
/// faster than that; a measurement that already refuses the slowest rung refuses
/// everything.
#[must_use]
pub fn ceiling_rung(measured: Sigma) -> Option<u8> {
    // **Scan and decide, for every bound.** The scan used to be skipped for a
    // `Bound::Lower`: the number was spent at drn 1 without asking any rung its
    // own question, so a measurement that carried 25 333 by 2.5x could not say
    // so, and the log carried a ceiling that was a constant rather than a
    // reading. What makes the scan safe is not the bound but the threshold: it
    // is the *gap*-corrected requirement (`docs/240` sec 3c), which for every
    // rung above drn 1 is two to twenty times tighter than `sigma_max` would be,
    // and it is refused the moment the measurement does not clear it. So this is
    // not "allow more": at today's 0.0690 every rung above drn 1 is still
    // refused, and the refusal is now a number in the log rather than a
    // short-circuit (see `digital::decide_upstream_rung`).
    (1..=UP_RATES as u8).rev().find(|&drn| measured.steps <= threshold(drn))
}

/// The mask of rungs at or below `ceiling`, in the layout of Table 20's Ja mask:
/// bit k for drn k + 1. A `None` ceiling permits nothing.
#[must_use]
pub fn ceiling_mask(ceiling: Option<u8>) -> u32 {
    match ceiling {
        Some(drn) if drn >= 1 => (1u32 << drn.min(UP_RATES as u8)) - 1,
        _ => 0,
    }
}

/// The Ja mask a measurement leaves: the mask to hand `upchoice::choose`.
///
/// `ja` is the analogue modem's mask, bit k for drn k + 1, and a zero mask
/// means "every rung" there; that zero is turned into [`JA_MASK_ALL`] first so
/// that the ceiling cannot be lost by ANDing against it. The result is an
/// explicit mask -- never a second zero-means-everything -- and it is a subset
/// of what `ja` allowed, whether `ja` said it explicitly or by being zero.
///
/// ```text
/// apply_ceiling(ja, m) & ja == apply_ceiling(ja, m)
/// ```
#[must_use]
pub fn apply_ceiling(ja: u32, measured: Sigma) -> u32 {
    let allowed = if ja == 0 { JA_MASK_ALL } else { ja };
    allowed & ceiling_mask(ceiling_rung(measured))
}

// ---------------------------------------------------------------------------
// Reading a level stream
// ---------------------------------------------------------------------------

/// The residual a level stream implies, in index steps of a set of `lc` points.
///
/// The stream is the bench's: one `now <level> <mode>` line per upstream
/// symbol, and only the `score` lines -- the window B1u is scored over, after
/// the equaliser has trained -- are a measurement of the grid. The values are
/// the equaliser's output; divided by `G` they are the odd multiples, and the
/// distance from the nearest odd multiple is the residual, one half-cell being
/// `G` and one index step two of them. So the number is the rms of
/// `||level| / G - nearest odd`, halved.
///
/// `None` when the stream has no `score` lines or no parsable levels in them.
#[must_use]
pub fn sigma_from_stream(text: &str, lc: usize) -> Option<Sigma> {
    let gain = gain_for(lc);
    let (mut sum, mut count) = (0.0f64, 0usize);
    let mut previous: Option<usize> = None;
    for (index, line) in text.lines().enumerate() {
        let mut fields = line.split_whitespace();
        let (Some(_now), Some(level), Some(mode)) = (fields.next(), fields.next(), fields.next()) else {
            continue;
        };
        if mode != "score" {
            continue;
        }
        // The first contiguous run of `score` lines, which is the one window.
        // Symbols of the same call can follow after a gap, and they are a
        // different window at a different time.
        if let Some(seen) = previous
            && index != seen + 1
        {
            break;
        }
        previous = Some(index);
        let level: f64 = level.parse().ok()?;
        let points = level.abs() / gain;
        // The nearest odd multiple, which is a point of the set. Clamped at one
        // because a level below `G` is closer to the innermost point than to
        // the origin, and the origin is not a level.
        let nearest = (2.0 * ((points - 1.0) / 2.0).round() + 1.0).max(1.0);
        let residual = points - nearest;
        sum += residual * residual;
        count += 1;
    }
    (count > 0).then(|| Sigma::reading((sum / count as f64).sqrt() / 2.0, lc))
}

/// The upper tail of the standard normal, Q(x).
///
/// Two approximations, because this module lives in the far tail: below two
/// standard deviations Abramowitz and Stegun 26.2.17 is accurate to `7.5e-8`
/// absolute, which is `3e-6` of the answer there, and past it that same
/// absolute error is larger than the answer itself -- at x = 5.3, where the
/// answer is `5e-8`, the error is about twice it. So the Laplace continued
/// fraction
///
/// ```text
/// Q(x) = phi(x) / (x + 1/(x + 2/(x + 3/(x + ...))))
/// ```
///
/// is used instead, backwards, where the answer is `1e-7` and its own error is
/// 1e-15 relative. `SIGMA_MAX` is `1/(2 Q^-1(5e-8))`, i.e. x ~ 5.3: exactly
/// where the closed form cannot be trusted.
///
/// Note for a reader comparing this with `upchoice`'s private copy: that one
/// carries `0.5 exp(-x^2/2)` where 8.8.3's tail wants `exp(-x^2/2)/sqrt(2 pi)`,
/// so every figure it prints is 1.2533 times the true tail. It only feeds a log
/// line there; nothing here shares the arithmetic.
fn normal_tail(x: f64) -> f64 {
    if !x.is_finite() {
        return 0.0;
    }
    let z = x.abs();
    let tail = if z < 2.0 {
        let t = 1.0 / (1.0 + 0.231_641_9 * z);
        let poly = t
            * (0.319_381_530
                + t * (-0.356_563_782 + t * (1.781_477_937 + t * (-1.821_255_978 + t * 1.330_274_429))));
        (-0.5 * z * z).exp() / (2.0 * std::f64::consts::PI).sqrt() * poly
    } else {
        let mut fraction = 0.0;
        for k in (1..=48).rev() {
            fraction = f64::from(k) / (z + fraction);
        }
        (-0.5 * z * z).exp() / (2.0 * std::f64::consts::PI).sqrt() / (z + fraction)
    };
    if x >= 0.0 { tail } else { 1.0 - tail }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::Trellis;
    use crate::v92::design::{Channel, design};
    use crate::v92::upchoice::pin;
    use crate::v92::{FilterLimits, UP_SLOWEST};

    /// The two streams the ceiling was built against, and the rung each was
    /// recorded at. The `score` run, verbatim (361 symbols each):
    ///
    /// * 24 000, LC = 8 -- `~/v92up-runs/c24-4.agr-tx.raw`, replayed as
    ///   `/tmp/fe/ber/c24-4-off3-none.levels`;
    /// * 32 000, LC = 16 -- the live call `/dev/shm/q/b32-mi16-2.levels`.
    const TWENTY_FOUR: (&str, usize, u8) =
        (include_str!("../../tests/data/v92-c24-4-off3-none.score.levels"), 8, 1);
    const THIRTY_TWO: (&str, usize, u8) = (include_str!("../../tests/data/v92-b32-mi16-2.score.levels"), 16, 7);

    /// The measured sigmas, to three figures, for the arithmetic the tests are
    /// not allowed to re-derive from the files: they are the numbers in the
    /// module comment and in the report.
    const MEASURED_24: f64 = 0.0513;
    const MEASURED_32: f64 = 0.1602;

    fn design_for(noise: f64) -> Design {
        let mut taps = vec![0.0; 16];
        taps[0] = 1.0;
        for k in 1..16 {
            taps[k] = 0.2 / k as f64;
        }
        let norm = taps.iter().map(|t| t * t).sum::<f64>().sqrt();
        taps.iter_mut().for_each(|t| *t /= norm);
        design(&Channel { taps, noise, signal: 1.0 }, &FilterLimits::from_info1a(3, 3, 3)).expect("designable")
    }

    /// The constant is `TARGET` inverted, not a number someone liked.
    #[test]
    fn the_constant_is_the_target_inverted() {
        let implied = symbol_error(SIGMA_MAX);
        assert!(
            (implied / TARGET - 1.0).abs() < 0.01,
            "SIGMA_MAX {SIGMA_MAX} implies {implied:e} against a target of {TARGET:e}"
        );
        // And the forward direction at a couple of points, so that a sign or a
        // factor of two in `symbol_error` cannot pass: more residual is more
        // error, no residual is none, and the alphabet's factor of two is
        // outside the call rather than inside it.
        assert!(symbol_error(0.75) > symbol_error(0.5), "a bigger residual is a worse error rate");
        assert!(symbol_error(0.0).abs() < 1e-300);
        assert!((symbol_error(0.5) - 2.0 * normal_tail(1.0)).abs() < 1e-12);
    }

    /// The four rungs, and the budget each one has: 6 dB per doubling of the
    /// set, because 6.4.1's three rungs is one more bit over twelve intervals.
    #[test]
    fn the_ceiling_falls_by_6_db_per_doubling_of_the_set() {
        let table = sigma_max_table();
        let wanted = [(1u8, 24_000u32, 36u32, 8usize), (7, 32_000, 48, 16), (13, 40_000, 60, 32), (19, 48_000, 72, 64)];
        for (row, want) in table.iter().zip(wanted) {
            assert_eq!((row.0, row.1, row.2, row.3), want);
        }
        assert!((table[0].4 - SIGMA_MAX).abs() < 1e-9, "the LC = 8 rung is the reference grid");
        for pair in table.windows(2) {
            let ratio = pair[0].4 / pair[1].4;
            assert!((ratio - 2.0).abs() < 0.01, "{} -> {}: {ratio} rather than 2", pair[0].3, pair[1].3);
            let decibels = 20.0 * ratio.log10();
            assert!((decibels - 6.0).abs() < 0.05, "{} -> {}: {decibels} dB", pair[0].3, pair[1].3);
        }
        // The table in the module comment, digit for digit.
        for (row, (steps, absolute)) in table.iter().zip([
            (0.0939, 0.02036),
            (0.0469, 0.01017),
            (0.0234, 0.00508),
            (0.0117, 0.00254),
        ]) {
            assert!((row.4 - steps).abs() < 5e-5, "drn {}: {} against {steps}", row.0, row.4);
            assert!((row.5 - absolute).abs() < 5e-6, "drn {}: {} against {absolute}", row.0, row.5);
        }
        println!("{:?}", sigma_max_table());
    }

    /// drn, rate, K and LC convert into each other without loss, and LC is the
    /// one `upchoice`'s own builder makes for all nineteen rungs.
    #[test]
    fn the_rung_conversions_are_exact() {
        let mut previous = 0;
        for drn in 1..=UP_RATES as u8 {
            let rung = Rung::of(drn).expect("a rung of the ladder");
            assert_eq!(rung.drn, drn);
            assert_eq!(rung.rate, up_rate(drn));
            assert_eq!(rung.k, 2 * (u32::from(drn) + 17));
            assert_eq!(Rung::of_rate(rung.rate), Some(rung), "drn {drn} does not come back from its rate");
            assert!(rung.lc >= 8 && rung.lc <= 64 && rung.lc.is_power_of_two(), "drn {drn}: LC {}", rung.lc);
            assert!(rung.lc >= previous, "LC falls at drn {drn}: {} after {previous}", rung.lc);
            previous = rung.lc;
        }
        assert_eq!(Rung::of(0), None, "drn 0 is cleardown, not a rate");
        assert_eq!(Rung::of(20), None);
        assert_eq!(Rung::of(1).expect("drn 1").rate, UP_SLOWEST);
        assert_eq!(Rung::of_rate(25_001), None);
        assert_eq!(Rung::of_rate(24_000).expect("the slowest rung").drn, 1);
        assert_eq!(Rung::of_rate(48_000).expect("the fastest rung").drn, 19);
        for (drn, lc) in [(1u8, 8usize), (7, 16), (13, 32), (19, 64)] {
            assert_eq!(lc_for(drn), lc, "drn {drn}");
        }
    }

    /// [`lc_for`] is 6.4.1 and 6.4.2 read off K, and the builder is the thing it
    /// claims to model: the sets `upchoice::pin` makes have exactly these sizes.
    #[test]
    fn lc_for_agrees_with_the_builder() {
        let design = design_for(0.0);
        for drn in 1..=UP_RATES as u8 {
            let Some(choice) = pin(&design, drn, Trellis::Sixteen, 1e-9) else { continue };
            assert_eq!(choice.parameters.drn, drn);
            assert_eq!(
                choice.parameters.sets[0].len(),
                lc_for(drn),
                "drn {}: the builder's set is {} points and lc_for says {}",
                drn,
                choice.parameters.sets[0].len(),
                lc_for(drn)
            );
        }
    }

    /// The ceiling only ever falls as the residual grows: a larger sigma never
    /// permits a higher rung. This is the property the whole shape rests on --
    /// it is what makes an upper bound on sigma safe and a lower bound not.
    #[test]
    fn a_larger_sigma_never_permits_a_higher_rung() {
        let mut previous = u8::MAX;
        let mut seen = Vec::new();
        for step in 0..=240 {
            let steps = 10f64.powf(-4.0 + 4.0 * f64::from(step) / 120.0);
            let measured = Sigma::absolute(2.0 * gain_for(REFERENCE_LC) * steps, Bound::Reading);
            let ceiling = ceiling_rung(measured).unwrap_or(0);
            assert!(
                ceiling <= previous,
                "the ceiling rose from {previous} to {ceiling} at sigma {steps:e}"
            );
            previous = ceiling;
            seen.push(ceiling);
        }
        assert_eq!(seen.first(), Some(&(UP_RATES as u8)), "a clean enough channel must reach the top");
        assert_eq!(seen.last(), Some(&0), "a dirty enough one must permit nothing");
    }

    /// Each rung's boundary is where its own **gap-corrected** threshold puts
    /// it: a measurement just inside carries the rung, one just outside does
    /// not. The number is `SIGMA_MAX` scaled by the smallest gap of a maximin
    /// code set of [`points`] points (`docs/240` sec 3c), not `sigma_max(lc)`:
    /// 25 333 is carried by a 16-point interval and 0.0939 would be a model of a
    /// set the rung does not have.
    #[test]
    fn each_rung_s_boundary_is_where_its_own_threshold_puts_it() {
        for (drn, lc) in [(1u8, 8usize), (2, 16), (7, 16), (13, 32), (19, 64)] {
            let bound = threshold(drn);
            assert!(bound > 0.0, "drn {drn} has no threshold");
            let inside = Sigma::reading(bound * 0.999, REFERENCE_LC);
            let outside = Sigma::reading(bound * 1.001, REFERENCE_LC);
            let permitted = ceiling_rung(inside).expect("a rung");
            assert!(permitted >= drn, "drn {drn} was refused at 0.1 % inside its bound: {permitted}");
            let refused = ceiling_rung(outside);
            assert!(
                refused.is_none_or(|ceiling| ceiling < drn),
                "drn {drn} was still permitted at 0.1 % outside its bound: {refused:?}"
            );
            // The predicted error at the boundary is at or under the target on
            // the rung's own worst interval, and the gap correction only ever
            // tightens the mean-square model -- which is the direction the
            // threshold exists to be wrong in.
            let at_boundary = Sigma::reading(bound, REFERENCE_LC).error(points(drn));
            assert!(
                at_boundary <= TARGET * 1.001,
                "drn {drn}: {at_boundary:e} at its boundary against a target of {TARGET:e}"
            );
            assert!(
                bound <= sigma_max(points(drn)),
                "drn {drn}: threshold {bound:.4} is looser than the mean-square model"
            );
        }
    }

    /// The 24 000 measurement: 24 000 is carried and 32 000 is not. That is the
    /// soundness test -- a rule that would choose 32 000 here predicts a call
    /// that did not happen.
    #[test]
    fn the_24_000_measurement_carries_24_000_and_refuses_32_000() {
        let measured = Sigma::reading(MEASURED_24, 8);
        let ceiling = ceiling_rung(measured).expect("the measured arm carries a rung");
        assert!(ceiling >= 1, "24 000 was refused");
        assert!(ceiling < 7, "32 000 was permitted on a {MEASURED_24} index-step residual");
        // The two rungs, explicitly, through the mask: drn 1 survives, drn 7
        // does not, and no rung above 7 does either.
        let mask = apply_ceiling(JA_MASK_ALL, measured);
        assert_eq!(mask & (1 << 0), 1 << 0, "24 000 was masked out");
        assert_eq!(mask & (1 << 6), 0, "32 000 survived the ceiling");
        assert_eq!(mask & !((1 << 7) - 1), 0, "a rung above 32 000 survived");
        println!("24 000 arm: sigma {MEASURED_24} -> ceiling {ceiling}, mask {mask:#b}");
    }

    /// The 32 000 measurement refuses the rung it was recorded at, which is what
    /// the bench watched: 32 000 failed and the call fell back to 24 000.
    #[test]
    fn the_32_000_measurement_refuses_32_000() {
        let measured = Sigma::reading(MEASURED_32, 16);
        let ceiling = ceiling_rung(measured).expect("the fallback is still a rung");
        assert!(ceiling < 7, "32 000 was permitted on a {MEASURED_32} index-step residual");
        assert!(ceiling >= 1, "the measurement refuses even 24 000, which the call went on to use");
        // On the reference grid the same absolute residual is half as many
        // index steps, because LC = 16's G is half LC = 8's.
        assert!((measured.steps() - MEASURED_32 / 2.0).abs() < 1e-3, "{} against {}", measured.steps(), MEASURED_32 / 2.0);
        println!("32 000 arm: sigma {MEASURED_32} -> sigma(LC 8) {} -> ceiling {ceiling}", measured.steps());
    }

    /// Every rung the ceiling permits has a predicted symbol error at or under
    /// the target, and every rung above it does not. Checked on the mask rather
    /// than the ceiling so that the two cannot disagree.
    #[test]
    fn the_mask_is_the_error_probability_it_promises() {
        for step in 0..40 {
            let steps = 0.005 * f64::from(step);
            let measured = Sigma::reading(steps, REFERENCE_LC);
            let mask = apply_ceiling(JA_MASK_ALL, measured);
            for drn in 1..=UP_RATES as u8 {
                let permitted = mask & (1 << (drn - 1)) != 0;
                // The rung's own worst interval, which is what the scan prices.
                let error = measured.error(points(drn));
                let budget = threshold(drn);
                // The permission *is* the scan's comparison, and it is at least
                // as strict as the error probability -- the gap correction only
                // tightens -- so a permitted rung never misses the target and a
                // refused one is refused on a number that is not looser than
                // the model.
                assert_eq!(
                    permitted,
                    steps <= budget,
                    "drn {drn} at sigma {steps}: permitted {permitted} against {budget:.4}"
                );
                if permitted {
                    assert!(
                        error <= TARGET * 1.001,
                        "drn {drn} at sigma {steps} was permitted with a predicted {error:e}"
                    );
                }
            }
        }
    }

    /// A ceiling can never raise a rung above what the Ja mask allowed: whatever
    /// the measurement, the result is a subset of Ja, and it is never more than
    /// the ceiling on its own would allow.
    #[test]
    fn a_ceiling_can_never_raise_a_rung_above_the_ja_mask() {
        let masks = [0u32, JA_MASK_ALL, 1, 1 << 6, 0b1010_1010_1010_1010_101, 1 << 18, (1 << 7) - 1];
        let mut rose = 0;
        for mask in masks {
            for step in 0..=60 {
                let steps = 0.004 * f64::from(step);
                for bound in [Bound::Reading, Bound::Upper, Bound::Lower] {
                    let measured = Sigma { bound, ..Sigma::reading(steps, REFERENCE_LC) };
                    let allowed = if mask == 0 { JA_MASK_ALL } else { mask };
                    let out = apply_ceiling(mask, measured);
                    assert_eq!(out & !allowed, 0, "mask {mask:#b} at sigma {steps} grew to {out:#b}");
                    assert!(out & !ceiling_mask(ceiling_rung(measured)) == 0);
                    if out & !mask != 0 {
                        // Only possible through the zero-means-every-rung rule.
                        assert_eq!(mask, 0);
                        rose += 1;
                    }
                }
            }
        }
        // A zero mask means "every rung" to `upchoice::choose`, so the one way
        // the output can hold bits the input did not is that convention; it must
        // never happen for a real mask.
        assert!(rose > 0, "the zero-mask case was not exercised");
    }

    /// **The rule the scan exists for, both ways.**
    ///
    /// Above a rung's threshold that rung is *refused* and the granted `drn` is
    /// what it was before the scan existed -- today's 0.0690 measures drn 1 and
    /// nothing higher, and the ladder must not move on it. Below the threshold
    /// the rung rises to the highest one whose threshold the measurement meets,
    /// which is the scan doing what the short-circuit could not do at all.
    #[test]
    fn the_scan_refuses_above_a_threshold_and_grants_below_it() {
        // The ticket's two-sided rule, named after it: `drn 2` = 25 333 has a
        // threshold of 0.0344 (`docs/240` sec 3c), and 0.0690 does not clear it.
        let above = Sigma::lower(0.0690, REFERENCE_LC);
        assert_eq!(ceiling_rung(above), Some(1), "0.0690 was allowed past drn 1");
        assert!(0.0690 > threshold(2), "the threshold the refutation rests on moved");
        println!("drn 2 threshold {} at 0.0690 -> {:?}", threshold(2), ceiling_rung(above));

        // Just inside it, drn 2 is carried -- and so is every rung at or under
        // the same threshold, which is the highest rung the measurement meets.
        let below = Sigma::lower(0.0344 * 0.999, REFERENCE_LC);
        let granted = ceiling_rung(below).expect("0.0339 must carry a rung");
        assert!(granted >= 2, "drn 2 was refused at 0.0339: {granted}");
        let expected = (1..=UP_RATES as u8).rev().find(|&drn| 0.0344 * 0.999 <= threshold(drn));
        assert_eq!(Some(granted), expected, "the scan did not take the highest rung met");
        println!("0.0339 -> drn {granted} ({})", up_rate(granted));

        // A bound does not change the scan any more: the same number is the same
        // rung whether it is called a reading or a lower bound. What the bound
        // still does is travel in the log (`docs/242` sec 4).
        for bound in [Bound::Reading, Bound::Upper, Bound::Lower] {
            let measured = Sigma { bound, ..Sigma::reading(0.0344 * 0.999, REFERENCE_LC) };
            assert_eq!(ceiling_rung(measured), Some(granted), "{bound:?} read differently");
        }
        let dirty = Sigma::lower(1.0, REFERENCE_LC);
        assert_eq!(ceiling_rung(dirty), None, "a measurement that refuses 24 000 must refuse all");
    }

    /// The thresholds reproduce `docs/240` sec 3c's verified column, which is
    /// what stops a later reader from rounding the rung above 24 000 back to the
    /// mean-square model.
    #[test]
    fn the_thresholds_are_the_gap_corrected_ones() {
        for (drn, want) in [(1u8, 0.0861), (2, 0.0344), (7, 0.0344), (13, 0.0172), (19, 0.0043)] {
            let got = threshold(drn);
            assert!((got - want).abs() < 5e-4, "drn {drn}: {got:.4} against {want}");
        }
        // 349 LSB is one index step at LC 8 (`docs/241` sec 0).
        assert!((index_step_lsb() - 349.0).abs() < 1.0, "{}", index_step_lsb());
        // 25 333's worst interval is 16 points, and an 8-point reading of the
        // same rung would be 2.7x too generous.
        assert_eq!(points(2), 16, "25 333's widest interval");
        assert_eq!(points(1), 8);
        assert!(sigma_max(lc_for(2)) > 2.5 * threshold(2), "the mean-square model is not tighter");
    }

    /// The wiring's own input: a design and a noise figure give an *upper* bound
    /// on sigma, because the noise figure available before CPd is a floor on the
    /// SNR -- and an upper bound on sigma is the direction that cannot invent a
    /// rung.
    #[test]
    fn a_design_gives_an_upper_bound() {
        let design = design_for(1e-6);
        let measured = Sigma::from_design(&design, 1e-6);
        assert_eq!(measured.bound(), Bound::Upper, "the fit's noise is a floor on the SNR, so this is a ceiling");
        // The same residual carried by hand: one index step of the LC = 8 grid
        // is 2 G(8) in absolute units.
        let absolute = (design.residual * design.residual + design.scale * design.scale * 1e-6).sqrt();
        assert!((measured.steps() * 2.0 * gain_for(REFERENCE_LC) - absolute).abs() < 1e-12);
        // More noise is never a faster rung: walk the noise *up*, so that the
        // ceiling may only fall.
        let mut previous = u8::MAX;
        for exp in (0..12).rev() {
            let noise = 10f64.powi(-exp);
            let ceiling = ceiling_rung(Sigma::from_design(&design, noise)).unwrap_or(0);
            assert!(ceiling <= previous, "noise {noise:e} gave {ceiling} after {previous}");
            previous = ceiling;
        }
        assert_eq!(previous, 0, "a 1e-12 noise floor still carries nothing, so the sweep measured nothing");
    }

    /// The offline reproduction of the bench's outcome, from the streams
    /// themselves rather than from the two numbers above: read each level
    /// stream, measure the residual on the grid it was recorded on, and ask the
    /// ceiling what it permits.
    ///
    /// 24 000's own stream carries 24 000 and refuses 32 000; 32 000's stream
    /// refuses 32 000. That second line is the failure the bench saw, predicted
    /// from the recording of the failing call.
    #[test]
    fn the_recorded_streams_predict_what_the_bench_did() {
        for (text, lc, recorded) in [TWENTY_FOUR, THIRTY_TWO] {
            let measured = sigma_from_stream(text, lc).expect("a score run");
            assert_eq!(measured.bound(), Bound::Reading);
            let ceiling = ceiling_rung(measured).expect("the slowest rung survives both");
            let mask = apply_ceiling(JA_MASK_ALL, measured);
            println!(
                "recorded at drn {recorded} (LC {lc}): sigma {:.4} index steps, {:.4} on the LC = 8 grid, \
                 ceiling drn {ceiling}, mask {mask:#b}",
                measured.at(lc),
                measured.steps()
            );
            if recorded == 1 {
                assert!(mask & 1 != 0, "the 24 000 stream does not even carry 24 000");
            } else {
                assert!(
                    mask & (1 << (recorded - 1)) == 0,
                    "the stream recorded at drn {recorded} permits drn {recorded}, which is not what happened"
                );
            }
            assert!(ceiling < 7, "the stream recorded at drn {recorded} permits 32 000");
        }
        // The numbers in the module comment, from the files.
        let a = sigma_from_stream(TWENTY_FOUR.0, TWENTY_FOUR.1).expect("a score run");
        let b = sigma_from_stream(THIRTY_TWO.0, THIRTY_TWO.1).expect("a score run");
        assert!((a.at(8) - MEASURED_24).abs() < 5e-4, "{} against {MEASURED_24}", a.at(8));
        assert!((b.at(16) - MEASURED_32).abs() < 5e-4, "{} against {MEASURED_32}", b.at(16));
        // 32 000's residual is about three times 24 000's in index steps, which
        // is why it fails: two of the three are the set being half the size.
        let ratio = b.at(16) / a.at(8);
        println!("32 000's residual against 24 000's, in index steps: {ratio:.3}");
        assert!((3.0..3.3).contains(&ratio), "{ratio}");
    }

    /// A stream with no `score` lines, or one that is not a level stream at all,
    /// measures nothing rather than measuring zero.
    #[test]
    fn a_stream_without_a_score_run_measures_nothing() {
        assert!(sigma_from_stream("", 8).is_none());
        assert!(sigma_from_stream("111711 0.5 data\n111712 -1.1 data\n", 8).is_none());
        assert!(sigma_from_stream("not a level stream\n", 8).is_none());
        // One clean run measures zero, and a zero residual carries the top rung.
        let g = gain_for(REFERENCE_LC);
        let clean = format!("1 {g:.17} score\n2 {:.17} score\n3 {:.17} score\n", -3.0 * g, 5.0 * g);
        let clean = sigma_from_stream(&clean, REFERENCE_LC).expect("a score run");
        assert!(clean.steps() < 1e-12, "{}", clean.steps());
        assert_eq!(ceiling_rung(clean), Some(UP_RATES as u8));
        // And the mode is what selects the window: the same levels as `data` are
        // not a measurement.
        assert!(sigma_from_stream("1 0.5 data\n", 8).is_none());
    }
}
