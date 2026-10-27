//! The upstream constellation on the **far end's code grid** (docs/114 R1, R4).
//!
//! CPd's constellation points are "the linear value of the (v + 1)-th
//! constellation point" (Table 30, bits 307 + alpha onward) and the
//! Recommendation attaches no unit to them: `docs/114` §1.1 searched the whole
//! text for `full scale`, `proportional`, `normalized`, `refered`, `in units`
//! and `quantis*` and found nothing. The only anchor is a *power* convention,
//! `[8.8.3]`: "the digital modem shall design the modulation parameters
//! assuming that, when the prefilter output multiplied by G has a mean-square
//! value of 1, the analogue modem will transmit at the desired power".
//!
//! What the Recommendation does say about the far end is that its transmitter
//! is a G.711 codec, and that it declares so: `Table 23` bit 35 is "Codec type:
//! 0 = mu-law; 1 = A-law", bit 128 says whether "the transmit constellations
//! differ from the codec's D/A output constellations", and the constellation it
//! reports back is a *Ucode mask* -- "a bit set to 1 indicates that the
//! constellation includes the PCM code represented by the corresponding Ucode"
//! (`[8.7.3]`, `/tmp/v92.txt:1579`). A far end with bit 128 clear is therefore
//! code-transparent: the levels it puts on the line **are** its codec's D/A
//! levels (docs/114 §1.3, and one CRC-verified Agere CPu says exactly that).
//!
//! Everything in this module follows from taking that literally. The levels
//! this end asks for are the G.711 decode values of the codes it means
//! ([`grid`]), `K` is the far code to this end's LSB scale ([`estimate`]), and
//! the receiver's decision table is those codes times `K` ([`scaled`]) so that
//! a `K` away from one costs spacing rather than adding noise (docs/114 R4).
//! `G` is still exactly `[8.8.3]`'s: the reciprocal square root of the set's
//! mean square, so `ms(v G) = 1` holds for the code ladder as it did for the
//! odd lattice it replaces. `G` is **not** a grid knob (R8).
//!
//! # Why a uniform ladder was wrong, in one number
//!
//! `a(eta) = 2 eta + 1` is a uniform ladder. Its power is right -- `G` makes it
//! right by construction -- but its *positions* are not the far end's: a codec
//! can only emit code values, so every level lands up to half a local codec
//! step away from where the design put it. `docs/103` §2 measured that term at
//! 17.6 LSB rms at 24 000 on the bench and `docs/108` §1 re-derived the total
//! as 21.3 LSB of channel noise plus a 7.6 LSB per-node quantisation
//! constant. Asking for code values instead makes the far end's rounding a
//! no-op, and what is left is a static, learnable per-point offset.

use crate::v90::ucode::{self, Law, UCODES};

/// The upstream stream's rms in the codec's own int16 domain, from the bench
/// (`docs/25` §2's measurement of a passing 24 000 B1u).
///
/// This is a **power** identity and not a placement rule: `[8.8.3]` fixes the
/// transmit power, `[3.8]` sets LU so that TRN1U reaches it, and a set whose
/// mean square is this number is therefore the set the line can carry. It is
/// used here for one thing -- how far up the code grid a `levels`-point set
/// reaches -- and never to move a point off a code (docs/114 §2: using the rms
/// to *place* the points is what buys the snapping residual).
pub const TARGET_RMS: f64 = 1609.0;

/// How far a set's rms may sit above [`TARGET_RMS`] -- 2 %, or 0.17 dB.
///
/// 1609 LSB is itself one measurement of one call (`docs/25` §2), and `G`
/// normalises whatever the set's power turns out to be (8.8.3), so the band
/// costs nothing on the line. What it buys is the grid's coarseness: on the top
/// half of the µ-law grid the next usable code is 256 LSB away, so a set that
/// had to sit *at* 1609 would give up a whole 128 LSB of minimum gap at 32 000
/// (96 against 128, `docs/103` §3). The residue is the far end's scale, which
/// is what `K` measures rather than what this constant guesses.
pub const POWER_SLACK: f64 = 0.02;

/// The law `[Table 23]` bit 35 names: "0 = µ-law; 1 = A-law".
///
/// **The one place a bit becomes a law** (docs/313 sec 2). Bit 35 of `CPu` is
/// the far end's "Codec type" (`v90::sequences::Cp::a_law`, read from
/// information bit 16 at `v90/sequences.rs:682`, absolute bit 35), and no
/// function below this line may invent a law of its own.
pub fn law(a_law: bool) -> Law {
    if a_law { Law::A } else { Law::Mu }
}

/// [`law`] for a bit this end does not have yet: no bit, no law.
///
/// The declaration path carries `Option<Law>` and refuses on `None`
/// ([`crate::v92::upchoice::build_with`]), because *the far end has not
/// declared* and *the far end declared µ-law* are different states: a
/// declaration built on the second when the truth is the first is a declaration
/// on the wrong alphabet, and the alphabet it would pick is the one this
/// network happens to use (docs/313 sec 2).
pub fn law_if(a_law: Option<bool>) -> Option<Law> {
    a_law.map(law)
}

/// The linear value of a positive code under `law`, in the codec's int16
/// domain: the single code → level entry point.
///
/// `code` is the Ucode of `[8.7.3]`'s mask, 0 ... 127; under µ-law that is the
/// magnitude index `m` of `docs/103` §1 (`0xFF - m` is the byte). Every value
/// comes from G.711 through [`ucode::linear`], whose own test reproduces all
/// 512 published Table 1/V.90 values -- no table is copied here.
///
/// **The law is an argument and never a default** (docs/313 sec 2): the
/// µ-law-only `value(code)` this replaced is gone rather than kept as a
/// convenience, so every remaining µ assumption in the tree names `Law::Mu` at
/// its own call site and shows up in a diff.
pub fn value_in(law: Law, code: u8) -> u16 {
    ucode::linear(law, code & 0x7f).clamp(0, i32::from(u16::MAX)) as u16
}

/// The code of alphabet `law` whose linear value is nearest `target`, 1 ... 127.
///
/// Code 0 is silence and `[8.8.3]` forbids it in a constellation ("the
/// constellation shall not include the zero point", `/tmp/v92.txt:1793`), so
/// the search starts at 1 and the answer is never the zero point. Under A-law
/// code 0 decodes to 8 and not to 0 -- A-law has no exact zero -- but it is
/// still the silence codeword and still excluded, so the zero point stays out
/// of the set under both laws (docs/313 sec 1).
pub fn nearest_code(law: Law, target: f64) -> u8 {
    let (mut low, mut high) = (1u8, (UCODES - 1) as u8);
    while low < high {
        let middle = low + (high - low) / 2;
        if f64::from(value_in(law, middle)) < target { low = middle + 1 } else { high = middle }
    }
    // `low` is the first code at or above the target; its predecessor may be
    // nearer.
    if low > 1 {
        let below = f64::from(value_in(law, low - 1));
        let above = f64::from(value_in(law, low));
        if (target - below).abs() <= (above - target).abs() {
            return low - 1;
        }
    }
    low
}

/// The step a `levels`-point uniform ladder has when its mean square is
/// [`TARGET_RMS`]: a uniform alphabet at `+-1, +-3, ...` has a mean square of
/// `((2 LC)^2 - 1) / 3`.
///
/// **No law argument, and that is a measurement**: `+-1, +-3, ...` is the odd
/// lattice in *index units*, the same points under either companding law, and
/// [`TARGET_RMS`] is `[8.8.3]`'s power in the codec's int16 domain -- the
/// domain both G.711 laws decode into (A-law's loudest code is 32 256, µ-law's
/// 32 124, `v90::ucode`'s own table test). So this is a number about the set's
/// power and not about its alphabet, and it takes the law nowhere (docs/313
/// sec 2, "what does not take the law").
pub fn ladder_step(levels: usize) -> f64 {
    let span = 2.0 * levels as f64;
    2.0 * TARGET_RMS / ((span * span - 1.0) / 3.0).max(1.0).sqrt()
}

/// The `levels` positive points of this end's constellation, as **code values
/// of the far end's `law`** (docs/114 R1, docs/313 sec 2).
///
/// The criterion is `docs/103` §3's, unchanged: maximise the smallest adjacent
/// gap over a set of distinct codes **of the law the far end declared** with no
/// zero point, subject to the set's mean square being no more than the power
/// the line carries. What
/// changes is that the criterion's *search space* is the far end's, because the
/// far end can only emit codes -- so the answer is a code set, and the far
/// end's quantiser is a no-op on it instead of a half-cell error on it.
///
/// The construction is one greedy pass per candidate gap: walk up the grid
/// taking the smallest code at least `gap` above the last, pushing on to the
/// next free code where the grid is coarser than the gap. Greedy from the
/// bottom is the least-power set with that gap, so "the largest gap whose set
/// stays inside the power budget" is a monotone search, which is what the loop
/// below is. The budget is a band and not just a ceiling, because the set's rms
/// is what the far end divides its own operating point by: asking in its own
/// units only works if the scale we ask at is the scale it transmits at, and
/// [`POWER_SLACK`] is how far off that estimate is allowed to be before
/// `K` (docs/114 R2) has to absorb the rest.
///
/// `None` when the code grid cannot hold `levels` points inside the budget.
/// That is the 48 000 rung's problem and not this function's: `docs/103` §3
/// shows no 64-point set on the grid has a usable gap, and what comes back here
/// (gap 16 LSB) is the best the grid can do rather than a design to certify.
pub fn grid(law: Law, levels: usize) -> Option<Vec<u16>> {
    if levels == 0 || levels + 1 > UCODES {
        return None;
    }
    let mut inside: Option<Vec<u16>> = None;
    let (mut low, mut high) = (1.0f64, 2.0 * ladder_step(levels));
    for _ in 0..64 {
        let gap = 0.5 * (low + high);
        match snapped(law, levels, gap) {
            Some(points) if mean_square(&points).sqrt() <= TARGET_RMS * (1.0 + POWER_SLACK) => {
                inside = Some(points);
                low = gap;
            }
            _ => high = gap,
        }
    }
    inside
}

/// One greedy pass of [`grid`]: `levels` codes of `law`, each at least `gap` LSB
/// above the one below where that law's grid allows it.
fn snapped(law: Law, levels: usize, gap: f64) -> Option<Vec<u16>> {
    let mut points = Vec::with_capacity(levels);
    let mut floor = 0u16;
    for _ in 0..levels {
        let want = f64::from(floor) + gap.max(1.0);
        let mut code = nearest_code(law, want);
        while value_in(law, code) <= floor {
            if code >= (UCODES - 1) as u8 {
                return None;
            }
            code += 1;
        }
        floor = value_in(law, code);
        points.push(floor);
    }
    Some(points)
}

/// The smallest gap between adjacent points, in LSB.
///
/// This is the number `[6.4.2]`'s classes are scored against: a symbol error
/// needs the noise to carry a point past the halfway mark, so the half-gap is
/// what `upchoice` compares with the design's sigma. It replaces `2 G`, which
/// was the same quantity for a set whose step was two.
pub fn min_gap(points: &[u16]) -> f64 {
    points.windows(2).map(|pair| f64::from(pair[1]) - f64::from(pair[0])).fold(f64::INFINITY, f64::min)
}

/// The mean square of the *symmetric* set these positive points describe.
pub fn mean_square(points: &[u16]) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    let sum: f64 = points.iter().map(|&point| f64::from(point) * f64::from(point)).sum();
    sum / points.len() as f64
}

/// `G` from `[8.8.3]`: the gain that makes `ms(v G) = 1` for these points.
///
/// No law argument: `G` is the reciprocal square root of the mean square of the
/// **points it is handed**, so a set built on the A-law grid takes the A-law
/// `G` and a set built on the µ-law grid the µ-law one, without this function
/// knowing either name (docs/313 sec 4's gain column is that difference).
///
/// For a unit prefilter `v` *is* `a(eta)`, so the reciprocal square root of the
/// set's mean square is the only value the clause allows. It is not a free
/// parameter and must not be used to move the grid (docs/114 R8).
pub fn gain_for(points: &[u16]) -> f64 {
    let square = mean_square(points);
    if square <= 0.0 { 0.0 } else { 1.0 / square.sqrt() }
}

/// The receiver's decision table for these codes at far-code-to-this-end
/// scale `k`: `round(k a(eta))` per point (docs/114 R4).
///
/// No law argument: `k` carries the alphabet's scale and the points carry the
/// alphabet, so multiplying and rounding is law-free; what *is* law-dependent,
/// the points, is the caller's (`Parameters::sets`, built by [`grid`]).
///
/// A level that arrives as `k` times a code value is sliced against the same
/// `k`, so a `k` away from one costs spacing and not noise: the thresholds move
/// with the levels. Rounding to whole LSB is part of the mechanism and not
/// tidiness -- the far end's codec emits integers -- and where a `k` small
/// enough pulls two codes onto one integer the table keeps them apart by one
/// LSB rather than collapsing them, because a duplicated level is a point the
/// slicer could never decide.
pub fn scaled(points: &[u16], k: f64) -> Vec<u16> {
    let mut out: Vec<u16> = Vec::with_capacity(points.len());
    let mut floor = 0u16;
    for &point in points {
        let raw = (k * f64::from(point)).round();
        let level = raw.clamp(1.0, f64::from(u16::MAX)) as u16;
        let level = level.max(floor.saturating_add(1));
        floor = level;
        out.push(level);
    }
    out
}

/// `k` from a set of received levels and the codes they are known to be, by
/// least squares through the origin: `k = sum(y c) / sum(c^2)` (docs/114 R2).
///
/// No law argument: both arguments are already numbers, and the law entered
/// them where they were decoded ([`value_in`] with the law the mask was read
/// under).
///
/// `None` when there is nothing to fit, or when the codes carry no power.
pub fn estimate(levels: &[f64], codes: &[f64]) -> Option<f64> {
    if levels.len() != codes.len() || levels.is_empty() {
        return None;
    }
    let cross: f64 = levels.iter().zip(codes).map(|(y, c)| y * c).sum();
    let square: f64 = codes.iter().map(|c| c * c).sum();
    if !(square > 0.0) || !cross.is_finite() {
        return None;
    }
    let k = cross / square;
    (k.is_finite() && k > 0.0).then_some(k)
}

/// `k` from one CPd/CPu pair: this end's design against the codes the far end
/// says it will put on the line (docs/114 R2a).
///
/// The two lists need not be the same length. A far end that snaps a uniform
/// ladder onto its codes reports *fewer* codes than the design has points -- the
/// Agere's mask for one 40 000 call is 26 positive Ucodes against a design of 32
/// (docs/114 §1.6) -- and even a design that is already on the grid loses the
/// pairs whose two points fell in one code cell. So the pairing is by *position*
/// along the two ascending lists and not by nearest neighbour: the i-th of `L`
/// points belongs at the `i (M - 1) / (L - 1)`-th of `M` reports, which is exact
/// when the report is one-for-one and degrades gracefully when it is not, and
/// the fit is the least squares slope through the origin over the pairs.
///
/// The interpolated values are then snapped back onto the reported codes before
/// the fit is taken, because the far end reports codes and not a curve: what it
/// promised at that position is one of the `M` words, and a fit to a value
/// between two of them is a fit to a level it never said.
pub fn estimate_from_mask(points: &[u16], codes: &[u16]) -> Option<f64> {
    if points.is_empty() || codes.is_empty() {
        return None;
    }
    if points.len() < 2 || codes.len() < 2 {
        let k = f64::from(codes[0]) / f64::from(points[0]);
        return (k.is_finite() && k > 0.0).then_some(k);
    }
    let (span, count) = ((points.len() - 1) as f64, (codes.len() - 1) as f64);
    let mut wanted = Vec::with_capacity(points.len());
    let mut between = Vec::with_capacity(points.len());
    for (index, &point) in points.iter().enumerate() {
        let position = index as f64 * count / span;
        let below = position.floor();
        let low = below as usize;
        let high = (low + 1).min(codes.len() - 1);
        let share = position - below;
        between.push(f64::from(codes[low]) * (1.0 - share) + f64::from(codes[high]) * share);
        wanted.push(f64::from(point));
    }
    let rough = estimate(&between, &wanted)?;
    let snapped: Vec<f64> = between
        .iter()
        .map(|&level| {
            let nearest = codes
                .iter()
                .copied()
                .min_by(|a, b| {
                    (f64::from(*a) - level).abs().partial_cmp(&(f64::from(*b) - level).abs()).unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(f64::from)
                .unwrap_or(level);
            nearest
        })
        .collect();
    estimate(&snapped, &wanted).filter(|k| *k > 0.0).or(Some(rough))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first thing `docs/114` §1.6 established about the far end: the levels
    /// it puts on the line are its codec's, and a codec's values are the G.711
    /// table. So the numbers this module emits are checked against the table's
    /// own independent implementation -- `interop/el_bridge.py`'s `_DEC` -- and
    /// not against anything this crate derives.
    ///
    /// The sixteen spot values below are that file's `_DEC` for the codes it
    /// documents (`interop/el_bridge.py`, `_DEC`/`_ENC`, µ-law, positive
    /// polarity), and the whole grid is checked against the published
    /// Table 1/V.90 by `v90::ucode`'s own test.
    ///
    /// `v90::ucode::linear` and that `_DEC` are the same function on all 128
    /// µ-law codes, not only these: the check was run once over the whole grid
    /// (`_ulaw2linear(0xFF - m)` against `4*(((2*(m&15)+33) << (m>>4)) - 33)`,
    /// **0 mismatches in 128**, `interop/el_bridge.py` lines 84-89 in this
    /// tree).
    #[test]
    fn the_values_are_the_g_711_table() {
        let published: [(u8, u16); 16] = [
            (0, 0),
            (1, 8),
            (2, 16),
            (16, 132),
            (17, 148),
            (32, 396),
            (40, 652),
            (48, 924),
            (52, 1180),
            (56, 1436),
            (60, 1692),
            (62, 1820),
            (64, 1980),
            (72, 3004),
            (96, 8316),
            (127, 32124),
        ];
        for (code, expected) in published {
            assert_eq!(value_in(Law::Mu, code), expected, "code {code}");
        }
    }

    /// `_ENC(_DEC(c)) == c` for every code, which is why a level on a code
    /// centre is a fixed point of the far end's quantiser (docs/103 §1).
    #[test]
    fn every_code_is_a_fixed_point_of_the_quantiser() {
        for code in 1..UCODES as u8 {
            let decoded = f64::from(value_in(Law::Mu, code));
            assert_eq!(nearest_code(Law::Mu, decoded), code, "code {code} does not survive a round trip");
        }
    }

    /// The sets this end emits are code values, strictly ascending, with no
    /// zero point -- at every LC the 6.1 ladder can ask for.
    #[test]
    fn every_rung_s_set_is_a_set_of_codes() {
        for levels in [1usize, 4, 5, 8, 10, 16, 20, 32, 39, 48, 64] {
            let points = grid(Law::Mu, levels).unwrap_or_else(|| panic!("{levels}: the grid ran out"));
            assert_eq!(points.len(), levels, "{levels}: LC");
            assert!(points[0] > 0, "{levels}: the zero point");
            assert!(points.windows(2).all(|pair| pair[0] < pair[1]), "{levels}: not ascending");
            for &point in &points {
                let code = nearest_code(Law::Mu, f64::from(point));
                assert_eq!(value_in(Law::Mu, code), point, "{levels}: {point} is not a code value");
            }
        }
    }

    /// 8.8.3's power convention survives the change of shape: `ms(v G) = 1` for
    /// the code ladder exactly as it did for the odd lattice.
    #[test]
    fn the_gain_brings_the_code_ladder_to_unit_power() {
        for levels in [4usize, 8, 16, 32, 64] {
            let points = grid(Law::Mu, levels).expect("a set");
            let gain = gain_for(&points);
            assert!((gain * gain * mean_square(&points) - 1.0).abs() < 1e-12, "{levels}");
            // Table 30 bits 35:50 carry 4 x G in unsigned Q0.16, and both ends
            // of the range matter: above GAIN_LARGEST there is no field, and at
            // the bottom four_g_from_gain would round the gain away.
            assert!(gain <= crate::v92::GAIN_LARGEST, "{levels}: {gain} has no field");
            assert!(gain * crate::v92::GAIN_SCALE >= 1.0, "{levels}: {gain} rounds to nothing");
        }
    }

    /// The grid is the far end's, so the power identity puts the set's rms
    /// where the line's is: within a code step of `TARGET_RMS`, without the rms
    /// ever having moved a point off a code.
    #[test]
    fn the_set_reaches_the_power_the_line_carries() {
        for levels in [8usize, 16, 32] {
            let points = grid(Law::Mu, levels).expect("a set");
            let rms = mean_square(&points).sqrt();
            let step = ladder_step(levels);
            assert!((rms - TARGET_RMS).abs() < step * 1.5, "{levels}: rms {rms} against {TARGET_RMS}");
        }
    }

    /// R4: the decision table is `round(k a(eta))`, so a `k` away from one moves
    /// the thresholds with the levels instead of shifting the levels out of the
    /// thresholds.
    #[test]
    fn the_table_follows_k() {
        let points = grid(Law::Mu, 8).expect("a set");
        assert_eq!(scaled(&points, 1.0), points, "k = 1 must be the design itself");
        for k in [0.98, 0.5, 1.03] {
            let table = scaled(&points, k);
            assert_eq!(table.len(), points.len());
            for (index, (&point, &level)) in points.iter().zip(&table).enumerate() {
                let expected = (k * f64::from(point)).round();
                assert!(
                    (f64::from(level) - expected).abs() <= 1.0,
                    "k {k}, point {index}: {level} against {expected}"
                );
            }
            assert!(table.windows(2).all(|pair| pair[0] < pair[1]), "k {k}: the table collapsed");
        }
    }

    /// A stream at scale `k` slices to the codes it was made from when the table
    /// is built at the same `k`, and does not when it is not -- which is the
    /// whole of R4 in one assertion.
    ///
    /// The two rungs are the point of it. At 24 000 the lattice is coarse enough
    /// to swallow 2 %: the smallest gap is 256 LSB between points of up to
    /// 2 492, so a 2 % scale error at the top is 50 LSB of a 128 LSB half-gap
    /// and every symbol still slices -- a fact about the rung, not a defence of
    /// an unscaled table. One rung up the gaps halve against the same
    /// magnitudes, and the error that costs a symbol halves with them, which is
    /// why the table has to follow `k` rather than hope the rung is wide.
    #[test]
    fn a_stream_at_k_slices_right_only_with_a_table_at_k() {
        let slice = |points: &[u16], table: &[u16], k: f64| {
            let mut wrong = 0;
            for index in 0..4 * points.len() {
                let eta = index % points.len();
                let level = k * f64::from(points[eta]);
                let mut best = (f64::INFINITY, 0usize);
                for (index, &point) in table.iter().enumerate() {
                    let distance = (level - f64::from(point)).abs();
                    if distance < best.0 {
                        best = (distance, index);
                    }
                }
                if best.1 != eta {
                    wrong += 1;
                }
            }
            wrong
        };
        // 24 000: 2 % is invisible to the slicer and 10 % is not.
        let coarse = grid(Law::Mu, 8).expect("a set");
        assert_eq!(slice(&coarse, &scaled(&coarse, 0.90), 0.90), 0, "a table at k must slice its own stream");
        assert_eq!(slice(&coarse, &coarse, 0.98), 0, "24 000 swallows 2 %");
        assert!(slice(&coarse, &coarse, 0.90) > 0, "24 000 does not swallow 10 %");
        // 48 000: 2 % is already enough to move a symbol to its neighbour.
        let fine = grid(Law::Mu, 64).expect("a set");
        assert_eq!(slice(&fine, &scaled(&fine, 0.98), 0.98), 0, "a table at k must slice its own stream");
        assert!(slice(&fine, &fine, 0.98) > 0, "48 000 has to be sliced at k, not at 1");
    }

    /// R2: the scale estimate is the least squares fit through the origin, and
    /// it recovers a known scale exactly.
    #[test]
    fn the_estimate_recovers_a_known_scale() {
        let points = grid(Law::Mu, 16).expect("a set");
        let codes: Vec<f64> = points.iter().map(|&p| f64::from(p)).collect();
        for k in [1.0, 0.98, 0.41, 2.5] {
            let levels: Vec<f64> = codes.iter().map(|c| k * c).collect();
            let fit = estimate(&levels, &codes).expect("a fit");
            assert!((fit - k).abs() < 1e-12, "{k} came back as {fit}");
        }
    }

    /// R2a: the same fit from a CPd/CPu pair, where what the far end reports is
    /// the codes it snapped the design onto -- fewer of them where two points
    /// fell in one code cell.
    #[test]
    fn the_mask_estimate_recovers_the_far_end_s_scale() {
        let points = grid(Law::Mu, 32).expect("a set");
        for k in [1.0f64, 0.98, 1.03] {
            let mut report: Vec<u16> = points
                .iter()
                .map(|&point| value_in(Law::Mu, nearest_code(Law::Mu, k * f64::from(point))))
                .filter(|&code| code > 0)
                .collect();
            report.dedup();
            let fit = estimate_from_mask(&points, &report).expect("a fit");
            // A report that has lost points to snapping cannot fix the scale to
            // better than a code step; what it must do is not be wrong by a
            // factor.
            assert!((fit - k).abs() < 0.05 * k, "k {k} came back as {fit} from {} codes", report.len());
        }
        assert!(estimate_from_mask(&[], &[1]).is_none());
        assert!(estimate_from_mask(&[1], &[]).is_none());
    }

    /// A-law is a different table and the mask's law is bit 35's, so a far end
    /// that declares A-law gets A-law values (docs/114 R7's detection half).
    #[test]
    fn the_law_is_bit_35_s() {
        assert_eq!(value_in(law(false), 16), 132, "mu-law");
        assert_eq!(value_in(law(true), 16), 264, "A-law's code 16");
        assert_ne!(value_in(law(false), 40), value_in(law(true), 40));
    }
}
