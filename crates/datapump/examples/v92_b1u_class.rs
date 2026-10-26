//! B1u's *class* sequence against a capture, which is the one thing about 6.4.2
//! a forward model can predict.
//!
//! 6.4.2 gives the precoder a class E(Ki) and lets it pick a member; it never
//! says which member, and with Mi = 8 and N = 16 the class at k = 0, 1, 2 has
//! exactly two: eta = Ki and eta = Ki - 8. That free bit is why a forward model
//! of the *levels* reproduces 16.4 % of a capture and cannot do better
//! (`v92_b1u_search`). The class itself -- Ki, which is what
//! `index_to_ki` reads back out of a decided index -- is invariant to it, so it
//! *is* computable, and US6414989B1 col.14 says the receiver is entitled to
//! treat it that way: "the receiver need only be concerned with the coset
//! received and may ignore which coset member was transmitted".
//!
//! Everything the class sequence depends on is the bit layer and nothing else:
//!
//! ```text
//! 48 data frames of ones -> GPA (6.3) -> 6.4.1's modulus encoder -> Ki[12]
//! ```
//!
//! with every memory zeroed before the first symbol (8.7.1). The precoder, the
//! prefilter, the trellis and G are *not* inputs: they choose the member of the
//! class, not the class. So this builds the 576 Ki from the same production
//! objects the encoder uses -- no second implementation of the scrambler or the
//! mixed radix -- and compares them, position by position, against the classes
//! the equaliser's levels fall into.
//!
//! ```text
//! cargo run --release --example v92_b1u_class -- LEVELS [--gain G] [--start N]
//!   [--offset 2,0,2,0] [--bits K] [--moduli m,...] [--decode]
//! ```
//!
//! `LEVELS` is `V92_B1U_LEVELS`: the stream the *decoder* is fed through B1u,
//! one `now level score` line per symbol of the scoring window. The `score`
//! lines are written before the polarity multiply and carry the receiver's own
//! `now`, which is the only symbol numbering that agrees with the decoder's
//! frame grid -- `V92DFE_TRACE` labels a level with `now` when it belongs to
//! `now + 1` and cannot be used for this.
//!
//! What it prints: the model start that lines the two sequences up, the
//! per-position histogram of `observed - predicted` mod 8 for both polarity
//! readings, and -- with `--offset` -- the agreement a *fixed* offset table
//! achieves, which is how a far end's convention is compared against the one
//! the decoder applies rather than fitted. `--decode` runs the observed classes
//! through 6.4.1 and GPA and counts the ones, reproducing the decoder's own B1u
//! count with no Viterbi or equaliser in the way.

use std::fmt::Write as _;

use datapump::v32::{Mode, Scrambler};
use datapump::v92::UP_INTERVALS;
use datapump::v92::TRELLIS_FRAME;
use datapump::v92::modulus::{self, FrameBits, Moduli12};

/// The number of data frames in B1u (8.7.1), and so of predicted symbols.
const B1U_FRAMES: usize = 48;

/// The most classes a rung this benched can carry: 6.4.2's class map has period
/// Mi, and 16 is the 32 000 rung's Mi. The histograms are cut to this width; a
/// rung with fewer classes prints fewer columns.
const CLASSES_MAX: usize = 16;

use std::hint::black_box;

fn main() {
    let mut path = None;
    let mut gain = 0.108_465_229_f64;
    let mut bits = 36_u32;
    let mut moduli = [8_u8; UP_INTERVALS];
    // The declared set's LC: 6.4.2 runs the indices `-N/2 <= eta < N/2` with
    // `N = 2 LC`, so the range is `LC` and not any modulus. 8 is the odd
    // lattice of 16 points, which is every rung up to `drn 2`.
    let mut lc = 8_usize;
    // `--legacy`: score every interval in `max(moduli)` and the range at
    // `max(moduli)` too, which is what this tool did before `docs/272` sec 1.2
    // found the defect. Kept only to re-read an old table against the new one;
    // it is the wrong question, not a second convention.
    let mut legacy = false;
    let mut start: Option<usize> = None;
    let mut offset: Option<[i64; TRELLIS_FRAME]> = None;
    let mut decode = false;
    let mut dump = false;
    let mut sweep = false;
    let mut per_frame = false;
    let mut fit = false;
    let mut level_table = [0.0f64; TRELLIS_FRAME];
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--gain" => gain = args.next().and_then(|v| v.parse().ok()).expect("--gain"),
            "--bits" => bits = args.next().and_then(|v| v.parse().ok()).expect("--bits"),
            "--moduli" => {
                let text = args.next().expect("--moduli");
                let values: Vec<u8> = text.split(',').map(|v| v.trim().parse().expect("modulus")).collect();
                moduli.copy_from_slice(&values);
            }
            "--start" => start = Some(args.next().and_then(|v| v.parse().ok()).expect("--start")),
            "--offset" => {
                let text = args.next().expect("--offset");
                let values: Vec<i64> = text.split(',').map(|v| v.trim().parse().expect("offset")).collect();
                let mut table = [0i64; TRELLIS_FRAME];
                table.copy_from_slice(&values);
                offset = Some(table);
            }
            "--decode" => decode = true,
            "--dump" => dump = true,
            "--sweep" => sweep = true,
            "--per-frame" => per_frame = true,
            "--fit" => fit = true,
            "--cost" => {
                // What the switch adds per symbol, timed on the operations
                // themselves rather than on a whole replay: a replay of a
                // 294k-symbol capture is a couple of seconds and the difference
                // between two runs of it is larger than the thing measured.
                let fixes: Option<([f64; TRELLIS_FRAME], [i64; TRELLIS_FRAME])> =
                    Some(([0.2, 0.1, 0.1, 0.9], [6, 6, 6, 7]));
                let rounds = 20_000_000u32;
                let mut sink = 0.0f64;
                let start = std::time::Instant::now();
                for n in 0..rounds {
                    let level = f64::from(black_box(n) % 97) - 48.0;
                    let table = std::hint::black_box(fixes.unwrap().0);
                    sink += level - table[n as usize % TRELLIS_FRAME];
                }
                let level_time = start.elapsed();
                let start = std::time::Instant::now();
                let mut ki = 0u8;
                for n in 0..rounds {
                    let k = n as usize % TRELLIS_FRAME;
                    let table = std::hint::black_box(fixes.unwrap().1);
                    ki = ki.wrapping_add(
                        (i64::from(black_box((n % 8) as u8)) + 0 - table[k]).rem_euclid(8) as u8,
                    );
                }
                let rotation_time = std::hint::black_box(start.elapsed());
                println!(
                    "cost per symbol, {} rounds: level table {:.2} ns, rotation {:.2} ns (sink {sink:.0} {ki})",
                    rounds,
                    level_time.as_secs_f64() * 1e9 / f64::from(rounds),
                    rotation_time.as_secs_f64() * 1e9 / f64::from(rounds),
                );
                return;
            }
            "--lc" => lc = args.next().expect("--lc").parse().expect("lc"),
            "--legacy" => legacy = true,
            "--levels" => {
                let text = args.next().expect("--levels");
                let values: Vec<f64> = text.split(',').map(|v| v.trim().parse().expect("level")).collect();
                level_table.copy_from_slice(&values);
            }
            other => path = Some(other.to_string()),
        }
    }
    let Some(path) = path else {
        eprintln!(
            "usage: v92_b1u_class LEVELS [--gain G] [--bits K] [--moduli m,...] [--start N] \
             [--offset a,b,c,d] [--levels a,b,c,d] [--decode] [--fit]"
        );
        std::process::exit(2);
    };
    // **6.4.2's class map has period Mi, per interval, and the index range is
    // the declared set's `LC`.** One radix for all twelve intervals
    // (`max(moduli)`) is what `docs/272` sec 1.2 found: at `drn 2` an `Mi = 16`
    // interval and an `Mi = 8` one sit at the same trellis position, so a
    // one-step rotation reads 15 in radix 16 and 7 in radix 8, `Mi/2` apart,
    // and the `Mi = 8` class's free second member prints as a second `+-8`.
    // Neither exists in each interval's own radix. `classes` is now the set's
    // `LC` -- the index range and the histogram width -- and every symbol
    // carries its own `Mi` for the class itself.
    let classes = lc;
    let widest = usize::from(*moduli.iter().max().expect("twelve moduli"));
    println!("LC {lc} (index range {} .. {}), Mi per interval {moduli:?} (widest {widest}, what every table below `docs/272` was scored in)", -(lc as i32), lc as i32 - 1);
    let levels = match levels(&path) {
        Ok(levels) if !levels.is_empty() => levels,
        Ok(_) => {
            eprintln!("{path}: no `score` lines -- V92_B1U_LEVELS has to be on during the run");
            std::process::exit(1);
        }
        Err(error) => {
            eprintln!("{path}: {error}");
            std::process::exit(1);
        }
    };

    // The model: the production scrambler and the production modulus encoder,
    // from the memories 8.7.1 zeroes.
    let predicted = predicted_classes(bits, &moduli);
    // `V92_PREDICTED=FILE`: the model's five hundred and seventy-six classes,
    // one digit a symbol, so that a decision trace taken off the receiver
    // (`V92_ETA_TRACE`) can be lined up against them by a reader that knows
    // where in B1u it is. A start search cannot answer that here: the scoring
    // window starts one symbol after the boundary symbol and the model counts
    // from the boundary symbol itself, so the right alignment is not a multiple
    // of the trellis frame and every `k` below is then off by one.
    if let Some(path) = std::env::var_os("V92_PREDICTED") {
        let text: String = predicted.iter().map(|class| char::from(b'0' + class)).collect();
        std::fs::write(path, format!("{text}\n")).expect("V92_PREDICTED is writable");
    }
    // What the capture did: the class each level falls into, under the decoder's
    // own convention for turning an index into one.
    let radices = if legacy { [u8::try_from(widest.max(1)).unwrap_or(16); UP_INTERVALS] } else { moduli };
    let classes = if legacy { widest } else { lc };
    if legacy {
        println!("--legacy: one radix for all twelve intervals, the defect `docs/272` sec 1.2 names");
    }
    let observed = observed_classes(&levels, gain, &level_table, &radices, classes);

    println!("{} scored symbols from {path}: gain {gain:.9}, K {bits}, Mi {classes}", observed.len());

    // The model's symbol 0 is the first symbol of B1u; the capture's first
    // `score` symbol is one after the boundary symbol, and neither file says
    // so, so the start is measured rather than assumed. It is measured before
    // anything is printed per trellis position, because it moves them: the
    // model's index of a captured symbol is `start + n`, and the trellis
    // position is `that mod four` and not `n mod four`. The scoring window
    // begins one symbol after B1u's first, so the two differ, and a tool that
    // keyed the histograms on `n` would print four positions of noise on a
    // capture whose classes are in fact perfect.
    let (start, right) = match start {
        Some(start) => (start, agreement(&observed, &predicted, start, classes)),
        None => best_start(&observed, &predicted, classes).into(),
    };
    for k in 0..TRELLIS_FRAME {
        let residual: f64 = observed.iter().filter(|s| s.k(start) == k).map(|s| s.residual).sum::<f64>()
            / observed.iter().filter(|s| s.k(start) == k).count().max(1) as f64;
        println!("  k {k}: mean |level / gain - a(eta)| = {residual:.3} (one level is 2)");
    }
    println!("\nmodel symbol {start} lines up with the capture's first scored symbol ({right} classes)");

    for (name, complement) in [("model as 8.7.1 reads it", false), ("model complemented", true)] {
        let histograms = offset_histograms(&observed, &predicted, start, complement, classes);
        println!("\noffset table, {name}: (observed - predicted) mod {classes}");
        println!("  k   mode  histogram [0..{}]                    agree", classes - 1);
        for (k, histogram) in histograms.iter().enumerate() {
            let histogram = &histogram[..classes];
            let mode = histogram.iter().enumerate().max_by_key(|&(_, &n)| n).map_or(0, |(offset, _)| offset);
            let total: usize = histogram.iter().sum();
            let mut row = String::new();
            for count in histogram {
                let _ = write!(row, "{count:5}");
            }
            println!("  {k}  {mode:5}  [{row} ]  {}/{total}", histogram[mode]);
        }
    }

    if dump {
        // Every symbol, the two classes side by side: the table the offset
        // histograms above only summarise.
        println!("\n  n  frame  i  k   observed  predicted  diff  level");
        for symbol in &observed {
            let Some(&wanted) = predicted.get(start + symbol.n) else { break };
            let diff = (symbol.class(start) as i64 - i64::from(wanted)).rem_euclid(classes as i64);
            println!(
                "  {:3}  {:5}  {:2}  {}  {:8}  {:9}  {:4}  {:+.3}",
                symbol.n,
                (start + symbol.n) / UP_INTERVALS,
                (start + symbol.n) % UP_INTERVALS,
                symbol.k(start),
                symbol.class(start),
                wanted,
                diff,
                symbol.eta
            );
        }
    }

    if let Some(offset) = offset {
        let histograms = offset_histograms(&observed, &predicted, start, false, classes);
        let total: usize = histograms.iter().flatten().sum();
        let right: usize = histograms.iter().enumerate().map(|(k, h)| h[offset[k] as usize]).sum();
        println!(
            "\nwith the fixed offsets {offset:?}: {right}/{total} = {:.1} % of classes",
            100.0 * right as f64 / total as f64
        );
        for (k, histogram) in histograms.iter().enumerate() {
            let total: usize = histogram.iter().sum();
            println!("  k {k}: {}/{total}", histogram[offset[k] as usize]);
        }
    }

    if fit {
        // `docs/47`'s fit: per position, the sub-step level correction that
        // maximises class agreement with the model, the class rotation allowed
        // to move with it. The rotation is the far end's convention and is not
        // what is being measured (`docs/46 §4`), so every combination of the
        // two is reported: offset alone, rotation alone, and both.
        //
        // The gain is one number for all four positions and is searched only
        // because a per-position gain would be the same free parameter as a
        // per-position offset, and the offset is what the data path can carry.
        let mut best = None;
        for complement in [false, true] {
            for step in 0..=40 {
                let gamma = 0.9 + 0.005 * f64::from(step);
                let (offsets, rotations, total) =
                    fit_table(&levels, gain, &predicted, start, gamma, complement, classes);
                // A tie is not evidence for a gain: the offsets can absorb any
                // gamma, so equal is only equal, and the answer closest to the
                // gain CPd named is the one that says nothing happened.
                if best
                    .as_ref()
                    .is_none_or(|b: &Fit| (total, -((gamma - 1.0).abs())) > (b.total, -((b.gamma - 1.0).abs())))
                {
                    best = Some(Fit { gamma, complement, offsets, rotations, total });
                }
            }
        }
        let fit = best.expect("the search is not empty");
        let today = pair_offset();
        let none = [0.0f64; TRELLIS_FRAME];
        println!(
            "\nfitted on {} symbols: gamma {:.3}, model {} complemented, {} classes",
            observed.len(),
            fit.gamma,
            if fit.complement { "" } else { "not" },
            fit.total
        );
        println!("  k   today  offset only  rotation only  both   offset  rotation  (distance)");
        for k in 0..TRELLIS_FRAME {
            let mut only_offset = [0.0f64; TRELLIS_FRAME];
            only_offset[k] = fit.offsets[k];
            let mut only_rotation = [0i64; TRELLIS_FRAME];
            only_rotation[k] = fit.rotations[k];
            let count = |offsets: &[f64; TRELLIS_FRAME], rotations: &[i64; TRELLIS_FRAME]| {
                count_agree(
                    &levels, gain, &predicted, start, fit.gamma, offsets, rotations, fit.complement, Some(k), classes,
                )
            };
            println!(
                "  {k}  {:5}  {:11}  {:13}  {:5}  {:+7.3}  {:8}  (lattice {:.3})",
                count(&none, &today),
                count(&only_offset, &today),
                count(&none, &only_rotation),
                count(&only_offset, &only_rotation),
                fit.offsets[k],
                fit.rotations[k],
                lattice_distance(&levels, gain, fit.gamma, fit.offsets[k], k, start, classes)
            );
        }
        let mut signed_best = None;
        for complement in [false, true] {
            for step in 0..=40 {
                let gamma = 0.9 + 0.005 * f64::from(step);
                let (positive, negative, rotations, total) =
                    fit_signed(&levels, gain, &predicted, start, gamma, complement, classes);
                if signed_best.as_ref().is_none_or(|b: &(f64, bool, [f64; TRELLIS_FRAME], [f64; TRELLIS_FRAME], [i64; TRELLIS_FRAME], usize)| total > b.5) {
                    signed_best = Some((gamma, complement, positive, negative, rotations, total));
                }
            }
        }
        if let Some((gamma, complement, positive, negative, rotations, total)) = signed_best {
            println!(
                "\nfitted with one offset a sign (gamma {gamma:.3}, model {} complemented): {total} classes",
                if complement { "" } else { "not" }
            );
            println!("  k   offset on positives  offset on negatives  rotation  classes");
            for k in 0..TRELLIS_FRAME {
                let right = count_agree_signed(
                    &levels,
                    gain,
                    &predicted,
                    start,
                    gamma,
                    &positive,
                    &negative,
                    &rotations,
                    complement,
                    Some(k),
                    classes,
                );
                let total_at = levels
                    .iter()
                    .enumerate()
                    .filter(|&(n, _)| (start + n) % TRELLIS_FRAME == k && start + n < predicted.len())
                    .count();
                println!(
                    "  {k}  {:+19.3}  {:+18.3}  {:8}  {right}/{total_at}",
                    positive[k], negative[k], rotations[k]
                );
            }
        }
        let mut offsets = none;
        let mut rotations = today;
        println!(
            "  with today's rotation (2,0,2,0) and the fitted offsets: {} classes",
            count_agree(&levels, gain, &predicted, start, fit.gamma, &offsets, &rotations, fit.complement, None, classes)
        );
        offsets = fit.offsets;
        rotations = fit.rotations;
        println!(
            "  with the fitted rotation and the fitted offsets: {} classes of {}",
            count_agree(
                &levels, gain, &predicted, start, fit.gamma, &offsets, &rotations, fit.complement, None, classes,
            ),
            observed.len()
        );
    }

    if sweep {
        // Every rotation table, scored by how many frames read as all ones --
        // which is what B1u is. A rotation that is right for the class map but
        // wrong by one class shows up as no table at all, not as a best table.
        let mut best: Vec<(usize, usize, [i64; TRELLIS_FRAME])> = Vec::new();
        for a in 0..16i64 {
            for b in 0..16i64 {
                for c in 0..16i64 {
                    for d in 0..16i64 {
                        let table = [a, b, c, d];
                        let frames = frame_bits(&observed, bits, &moduli, table, start);
                        let clean = frames.iter().filter(|f| f.2 == bits as usize).count();
                        let ones: usize = frames.iter().map(|f| f.2).sum();
                        best.push((clean, ones, table));
                    }
                }
            }
        }
        best.sort_by(|x, y| (y.0, y.1).cmp(&(x.0, x.1)));
        println!("\nrotation sweep: the best tables by frames that decode to all ones");
        for (clean, ones, table) in best.iter().take(5) {
            println!("  {table:?}: {clean} clean frames, {ones} ones");
        }
    }

    if std::env::var_os("V92_MODEL_FRAMES").is_some() {
        // The payload each model frame carries, and the two bits 6.4.1 derives
        // from it: s(f) = [R > (M-1)/2] and d(f) = s(f) xor d(f-1). Printed so
        // that a far end's reflection chain can be compared with ours rather
        // than only its effect on the classes.
        let moduli = Moduli12::new(moduli);
        let mut scrambler = Scrambler::new(Mode::Answer);
        let mut d_prev = false;
        let m = moduli.product();
        for frame in 0..B1U_FRAMES {
            let bits: Vec<bool> = (0..bits).map(|_| scrambler.scramble(true)).collect();
            let r: u128 = FrameBits::from_bits(&bits).value();
            let s = 2 * r > m - 1;
            let d = s ^ d_prev;
            println!("  frame {frame:2}  R {r:012x}  s {}  d_prev {}  d {}", s as u8, d_prev as u8, d as u8);
            d_prev = d;
        }
    }

    if std::env::var_os("V92_ZERO_PAYLOAD").is_some() {
        // The two sources of a whole-frame complement, side by side.
        //
        // A far end whose B1u data are zero sends `scramble(0)` where an
        // all-ones B1u sends `scramble(1)`. With an additive scrambler that is
        // the bitwise complement of the ones stream; the production scrambler
        // is multiplicative, so the two streams are built here by flipping the
        // bits rather than by feeding `false`, which a zeroed multiplicative
        // scrambler turns into an all-zero stream.
        //
        // The interesting part is that the *chain* comes along: a complemented
        // payload has s(f) complemented, so d(f) = s(f) xor d(f-1) is
        // complemented too, and 6.4.1's step 4 reflects with d(f-1). Reflect a
        // complemented payload with a complemented chain and the two cancel,
        // which is why this model has a different prediction from "the ones
        // model complemented": it should agree with the capture on *every*
        // frame, not on the ones the capture reads as ours.
        let moduli12 = Moduli12::new(moduli);
        let mut scrambler = Scrambler::new(Mode::Answer);
        let mut encoder = modulus::Encoder::new();
        let mut zero = Vec::with_capacity(B1U_FRAMES * UP_INTERVALS);
        for _ in 0..B1U_FRAMES {
            let bits_of: Vec<bool> = (0..bits).map(|_| !scrambler.scramble(true)).collect();
            let frame = encoder
                .encode(FrameBits::from_bits(&bits_of), &moduli12)
                .expect("the moduli carry K bits");
            zero.extend_from_slice(&frame);
        }
        for (name, model) in [("all-ones payload", &predicted), ("zero payload", &zero)] {
            let mut same = 0usize;
            let mut mirrored = 0usize;
            for symbol in &observed {
                let Some(&wanted) = model.get(start + symbol.n) else { break };
                let class = (symbol.class(start) as i64 - offset.as_ref().map_or(0, |o| o[symbol.k(start)]))
                    .rem_euclid(classes as i64) as usize;
                same += usize::from(class == usize::from(wanted));
                mirrored += usize::from(class == classes - 1 - usize::from(wanted));
            }
            println!("  {name}: {same} as-is, {mirrored} complemented, of {}", observed.len());
        }
    }

    if per_frame {
        let table = offset.unwrap_or([0; TRELLIS_FRAME]);
        println!("\nper frame: rotation {table:?}, 1 = a one, . = a zero");
        for (frame, bits_of, ones) in frame_bits(&observed, bits, &moduli, table, start) {
            let text: String = bits_of.iter().map(|&bit| if bit { '1' } else { '.' }).collect();
            println!("  {frame:3}  {ones:2}/{bits}  {text}");
        }
    }

    if decode {
        // The classes read back into bits: 6.4.1 backwards, then GPA. No
        // Viterbi, no equaliser: a class is what a trellis frame carries, and
        // 6.4.1's differential step is what makes an inverted line decode the
        // same.
        let table = offset.unwrap_or([0; TRELLIS_FRAME]);
        let (ones, total) = decode_classes(&observed, bits, &moduli, table, start);
        println!("\nclasses alone decode to {ones}/{total} ones ({:.1} %)", 100.0 * ones as f64 / total as f64);
    }
}

/// One fitted table, and how many classes it agrees on.
struct Fit {
    gamma: f64,
    complement: bool,
    offsets: [f64; TRELLIS_FRAME],
    rotations: [i64; TRELLIS_FRAME],
    total: usize,
}

/// The class rotation the decoder applies today, in this file's convention:
/// `index_to_ki`'s pair offset at k = 0 and k = 2 and nothing elsewhere.
fn pair_offset() -> [i64; TRELLIS_FRAME] {
    let first = i64::from(datapump::v92::precoder::PAIR_FIRST_INDEX_OFFSET);
    [first, 0, first, 0]
}

/// The class one level falls into at one position, once `gamma` and `offset`
/// have been taken out of it: [`Observed::class`] applied to a moved level.
fn class_at(level: f64, gain: f64, gamma: f64, offset: f64, k: usize, classes: usize) -> usize {
    class_at_signed(level, gain, gamma, offset, offset, k, classes)
}

/// The same, with one offset for the positive levels and another for the
/// negative ones -- US6002713B1's "the positive and the negative transitions
/// are not symmetric", which `docs/47 §2` measures on this line.
fn class_at_signed(
    level: f64,
    gain: f64,
    gamma: f64,
    positive: f64,
    negative: f64,
    k: usize,
    classes: usize,
) -> usize {
    let offset = if level >= 0.0 { positive } else { negative };
    let target = level / gamma / gain - offset;
    // 6.4.2 runs the indices -N/2 <= eta < N/2, and the rungs this bench pins
    // have N = 2 Mi, so the range is -Mi .. Mi - 1 at every one of them.
    let (lowest, highest) = (-(classes as i32), classes as i32 - 1);
    let eta = (lowest..=highest)
        .min_by(|&a, &b| (target - level_of(a)).abs().total_cmp(&(target - level_of(b)).abs()))
        .unwrap_or(0);
    if k == TRELLIS_FRAME - 1 {
        (((eta - eta.rem_euclid(2)) / 2).rem_euclid(classes as i32)) as usize
    } else {
        (eta.rem_euclid(classes as i32)) as usize
    }
}

/// How far, on average, one position's levels sit from the lattice once
/// `offset` is taken off them: `|target - a(eta)|` against the nearest a(eta),
/// which is 1 at a class boundary and 0 on a point.
///
/// The class agreement alone cannot break a tie between two offsets, and
/// `docs/47 §3` shows why: the grid is two half-steps wide, so every offset in a
/// whole half-step leaves the *class* alone and only changes how sure the slicer
/// is. A Viterbi reads the distance and not just the class, so the fit has to
/// come down on one side of that plateau, and this is the number that says which.
fn lattice_distance(
    levels: &[f64],
    gain: f64,
    gamma: f64,
    offset: f64,
    k: usize,
    start: usize,
    classes: usize,
) -> f64 {
    let (mut sum, mut count) = (0.0f64, 0usize);
    for (n, &level) in levels.iter().enumerate() {
        if (start + n) % TRELLIS_FRAME != k {
            continue;
        }
        let target = level / gamma / gain - offset;
        let (lowest, highest) = (-(classes as i32), classes as i32 - 1);
        let near = (lowest..=highest)
            .map(|eta| (target - level_of(eta)).abs())
            .fold(f64::INFINITY, f64::min);
        sum += near;
        count += 1;
    }
    sum / count.max(1) as f64
}

/// How many classes the model and the levels agree on, at one position or all
/// four, under a level table and a rotation table.
fn count_agree(
    levels: &[f64],
    gain: f64,
    predicted: &[u8],
    start: usize,
    gamma: f64,
    offsets: &[f64; TRELLIS_FRAME],
    rotations: &[i64; TRELLIS_FRAME],
    complement: bool,
    only: Option<usize>,
    classes: usize,
) -> usize {
    let mut right = 0usize;
    for (n, &level) in levels.iter().enumerate() {
        let k = (start + n) % TRELLIS_FRAME;
        if only.is_some_and(|wanted| wanted != k) {
            continue;
        }
        let Some(&wanted) = predicted.get(start + n) else { break };
        let wanted = if complement { classes - 1 - usize::from(wanted) } else { usize::from(wanted) };
        // The tool's convention throughout: the model's class is the observed
        // one *minus* the rotation, which is `offset_histograms` read backwards.
        let class = (class_at(level, gain, gamma, offsets[k], k, classes) as i64 - rotations[k])
            .rem_euclid(classes as i64) as usize;
        if class == wanted {
            right += 1;
        }
    }
    right
}

/// How many classes agree when each position carries two offsets, one for the
/// positive levels and one for the negative ones.
fn count_agree_signed(
    levels: &[f64],
    gain: f64,
    predicted: &[u8],
    start: usize,
    gamma: f64,
    positive: &[f64; TRELLIS_FRAME],
    negative: &[f64; TRELLIS_FRAME],
    rotations: &[i64; TRELLIS_FRAME],
    complement: bool,
    only: Option<usize>,
    classes: usize,
) -> usize {
    let mut right = 0usize;
    for (n, &level) in levels.iter().enumerate() {
        let k = (start + n) % TRELLIS_FRAME;
        if only.is_some_and(|wanted| wanted != k) {
            continue;
        }
        let Some(&wanted) = predicted.get(start + n) else { break };
        let wanted = if complement { classes - 1 - usize::from(wanted) } else { usize::from(wanted) };
        let class = (class_at_signed(level, gain, gamma, positive[k], negative[k], k, classes) as i64
            - rotations[k])
            .rem_euclid(classes as i64) as usize;
        if class == wanted {
            right += 1;
        }
    }
    right
}

/// The same search with two offsets a position, which is `docs/47 §2`'s
/// sign-asymmetric displacement given a table to live in.
fn fit_signed(
    levels: &[f64],
    gain: f64,
    predicted: &[u8],
    start: usize,
    gamma: f64,
    complement: bool,
    classes: usize,
) -> ([f64; TRELLIS_FRAME], [f64; TRELLIS_FRAME], [i64; TRELLIS_FRAME], usize) {
    let mut positive = [0.0f64; TRELLIS_FRAME];
    let mut negative = [0.0f64; TRELLIS_FRAME];
    let mut rotations = [0i64; TRELLIS_FRAME];
    let mut total = 0usize;
    for k in 0..TRELLIS_FRAME {
        let mut best = (0usize, 0.0f64, 0.0f64, 0i64);
        for step in 0..=60 {
            let offset = -1.5 + 0.05 * f64::from(step);
            for step2 in 0..=60 {
                let offset2 = -1.5 + 0.05 * f64::from(step2);
                for rotation in 0..classes as i64 {
                    let mut p = [0.0f64; TRELLIS_FRAME];
                    p[k] = offset;
                    let mut m = [0.0f64; TRELLIS_FRAME];
                    m[k] = offset2;
                    let mut turn = [0i64; TRELLIS_FRAME];
                    turn[k] = rotation;
                    let count = count_agree_signed(
                        levels, gain, predicted, start, gamma, &p, &m, &turn, complement, Some(k), classes,
                    );
                    if count > best.0
                        || (count == best.0 && offset.abs() + offset2.abs() < best.1.abs() + best.2.abs())
                    {
                        best = (count, offset, offset2, rotation);
                    }
                }
            }
        }
        positive[k] = best.1;
        negative[k] = best.2;
        rotations[k] = best.3;
        total += best.0;
    }
    (positive, negative, rotations, total)
}

/// The best offset and rotation at each position, on a grid fine enough for a
/// quarter of a half-step and coarse enough to be a search and not a fit.
fn fit_table(
    levels: &[f64],
    gain: f64,
    predicted: &[u8],
    start: usize,
    gamma: f64,
    complement: bool,
    classes: usize,
) -> ([f64; TRELLIS_FRAME], [i64; TRELLIS_FRAME], usize) {
    let mut offsets = [0.0f64; TRELLIS_FRAME];
    let mut rotations = [0i64; TRELLIS_FRAME];
    let mut total = 0usize;
    for k in 0..TRELLIS_FRAME {
        let mut best = (0usize, 0.0f64, 0i64);
        let mut best_distance = (0usize, f64::INFINITY, 0.0f64, 0i64);
        for step in 0..=60 {
            let offset = -1.5 + 0.05 * f64::from(step);
            for rotation in 0..classes as i64 {
                let mut table = [0.0f64; TRELLIS_FRAME];
                table[k] = offset;
                let mut turn = [0i64; TRELLIS_FRAME];
                turn[k] = rotation;
                let count = count_agree(
                    levels, gain, predicted, start, gamma, &table, &turn, complement, Some(k), classes,
                );
                if count > best.0 || (count == best.0 && offset.abs() < best.1.abs()) {
                    best = (count, offset, rotation);
                }
                if count == best.0 {
                    // Inside the plateau the class cannot choose; the distance
                    // can. Recorded separately so both are reported.
                    let distance = lattice_distance(levels, gain, gamma, offset, k, start, classes);
                    if count > best_distance.0 || (count == best_distance.0 && distance < best_distance.1) {
                        best_distance = (count, distance, offset, rotation);
                    }
                }
            }
        }
        // The plateau's answer wins: same classes, but the levels as close to
        // the lattice as one offset can put them.
        let (count, _, offset, rotation) = if best_distance.0 == best.0 {
            (best.0, best_distance.1, best_distance.2, best_distance.3)
        } else {
            (best.0, 0.0, best.1, best.2)
        };
        let _ = count;
        offsets[k] = offset;
        rotations[k] = rotation;
        total += best.0;
    }
    (offsets, rotations, total)
}

/// B1u's class sequence, as 6.4.1 and 8.7.1 make it.
///
/// 48 data frames of ones through GPA, then the mixed radix, from memories that
/// start zero. The trellis is not consulted: 6.4.2's class at k = 3 takes its
/// parity from the three indices before it, but the class is Ki either way,
/// because the parity is the low bit of the index and comes back out with it.
fn predicted_classes(bits: u32, moduli: &[u8; UP_INTERVALS]) -> Vec<u8> {
    let moduli = Moduli12::new(*moduli);
    let mut scrambler = Scrambler::new(Mode::Answer);
    let mut encoder = modulus::Encoder::new();
    let mut out = Vec::with_capacity(B1U_FRAMES * UP_INTERVALS);
    for _ in 0..B1U_FRAMES {
        let scrambled: Vec<bool> = (0..bits).map(|_| scrambler.scramble(true)).collect();
        let frame = encoder
            .encode(FrameBits::from_bits(&scrambled), &moduli)
            .expect("the moduli carry K bits, or there would be no encoder");
        out.extend_from_slice(&frame);
    }
    out
}

/// The capture's levels, from the `score` lines of `V92_B1U_LEVELS`.
fn levels(path: &str) -> Result<Vec<f64>, Box<dyn std::error::Error>> {
    let text = std::fs::read_to_string(path)?;
    let mut levels = Vec::new();
    for line in text.lines() {
        let mut words = line.split_whitespace();
        let (Some(_now), Some(level), Some(tag)) = (words.next(), words.next(), words.next()) else { continue };
        if tag == "score" {
            levels.push(level.parse::<f64>()?);
        }
    }
    Ok(levels)
}

/// a(eta), 6.4.2's level for one index: "a(eta) = P[eta] for eta >= 0 and
/// a(eta) = -P[-eta - 1] for eta < 0", so that a(-1) is the smallest negative
/// level and the levels rise with eta across the whole range.
fn level_of(eta: i32) -> f64 {
    if eta >= 0 {
        f64::from(2 * eta + 1)
    } else {
        -f64::from(2 * (-eta - 1) + 1)
    }
}

/// What the classes are measured against: the CPd's own set, read the way 6.4.2
/// prints it -- "the indices are in the same order as the levels".
fn observed_classes(
    levels: &[f64],
    gain: f64,
    table: &[f64; TRELLIS_FRAME],
    moduli: &[u8; UP_INTERVALS],
    classes: usize,
) -> Vec<Observed> {
    // 6.4.2 runs the indices -N/2 <= eta < N/2 with N = 2 LC, which is
    // -LC .. LC - 1: -8 .. 7 on a 16-point set. It is the set's LC and not
    // `max(Mi)` -- at `drn 2` those are 8 and 16.
    let (lowest, highest) = (-(classes as i32), classes as i32 - 1);
    levels
        .iter()
        .enumerate()
        .map(|(n, &level)| {
            // `docs/47`'s sub-step correction, keyed on the position in the
            // twelve: one scalar per position, taken off before the grid is
            // asked which point this is.
            let k = n % UP_INTERVALS % TRELLIS_FRAME;
            let target = level / gain - table[k];
            let eta = (lowest..=highest)
                .min_by(|&a, &b| (target - level_of(a)).abs().total_cmp(&(target - level_of(b)).abs()))
                .unwrap_or(0);
            Observed {
                n,
                eta,
                residual: (target - level_of(eta)).abs(),
                classes,
                modulus: usize::from(moduli[n % UP_INTERVALS]),
            }
        })
        .collect()
}

/// One observed symbol: which index its level falls on, and how far off it is.
struct Observed {
    n: usize,
    eta: i32,
    residual: f64,
    /// The set's LC: the width of the index range and of every histogram.
    classes: usize,
    /// **This symbol's own interval's Mi** -- the period of the class map at
    /// the position it is in, which is the only radix its class can be read in.
    modulus: usize,
}

impl Observed {
    /// Which trellis position this symbol is in: the model's, so that a start
    /// which is not a multiple of four is handled. `n` is the position in the
    /// scoring window and the window's first symbol is model symbol `start`.
    fn k(&self, start: usize) -> usize {
        (start + self.n) % TRELLIS_FRAME
    }

    /// The class of the index: 6.4.2 backwards at offset zero, which is
    /// `index_to_ki` with `PAIR_FIRST_INDEX_OFFSET` taken out.
    ///
    /// k = 0, 1, 2 are eta = Ki + z Mi, so the class is eta mod Mi. k = 3 is
    /// eta = 2 Ki + 2 z Mi + p with p the parity of eta itself, so the parity
    /// comes off the bottom first. Either way the answer is invariant to z,
    /// which is the point: it is invariant to the free bit.
    fn class(&self, start: usize) -> usize {
        let radix = self.modulus.max(1) as i32;
        if self.k(start) == TRELLIS_FRAME - 1 {
            (((self.eta - self.eta.rem_euclid(2)) / 2).rem_euclid(radix)) as usize
        } else {
            (self.eta.rem_euclid(radix)) as usize
        }
    }
}

/// `observed - predicted` mod 8, per trellis position, at one model start.
///
/// `complement` takes the model's class sequence as (7 - Ki) instead. That is
/// not a fit: the sequence is defined up to one global sign, because the line's
/// polarity is not known a priori and 6.4.1's differential step exists precisely
/// so that it does not have to be -- a receiver holding the polarity backwards
/// sees every Ki as Mi - 1 - Ki, which is the whole frame as M - 1 - R0, and the
/// reflection undoes a reflection. One bit, not five hundred and seventy-six.
fn offset_histograms(
    observed: &[Observed],
    predicted: &[u8],
    start: usize,
    complement: bool,
    classes: usize,
) -> [[usize; CLASSES_MAX]; TRELLIS_FRAME] {
    let mut histograms = [[0usize; CLASSES_MAX]; TRELLIS_FRAME];
    for symbol in observed {
        let Some(&predicted) = predicted.get(start + symbol.n) else { break };
        let predicted = if complement { classes - 1 - usize::from(predicted) } else { usize::from(predicted) };
        let offset = (symbol.class(start) + classes - predicted) % classes;
        histograms[symbol.k(start)][offset] += 1;
    }
    histograms
}

/// The agreement a start achieves, counted as the largest bin of each position:
/// the best any single offset table can do there.
fn agreement(observed: &[Observed], predicted: &[u8], start: usize, classes: usize) -> usize {
    offset_histograms(observed, predicted, start, false, classes)
        .iter()
        .map(|histogram| *histogram.iter().max().unwrap_or(&0))
        .sum()
}

/// The start that makes the two sequences agree best, over every start the
/// capture can support, so that nothing has to be assumed about where in the
/// file B1u begins.
fn best_start(observed: &[Observed], predicted: &[u8], classes: usize) -> (usize, usize) {
    let mut best = (0usize, 0usize);
    for start in 0..predicted.len() {
        if observed.iter().filter(|symbol| start + symbol.n < predicted.len()).count() < observed.len() / 2 {
            continue;
        }
        let right = agreement(observed, predicted, start, classes);
        if right > best.1 {
            best = (start, right);
        }
    }
    best
}

/// The 48 bits of every frame, as the class sequence decodes to them: 6.4.1
/// backwards, then GPA. One line a frame, so that the frames that read as B1u
/// and the frames that do not are distinguishable from their contents rather
/// than from a count.
fn frame_bits(
    observed: &[Observed],
    bits: u32,
    moduli: &[u8; UP_INTERVALS],
    offset: [i64; TRELLIS_FRAME],
    start: usize,
) -> Vec<(usize, Vec<bool>, usize)> {
    let moduli = Moduli12::new(*moduli);
    let mut decoder = modulus::Decoder::new();
    let mut descrambler = Scrambler::new(Mode::Answer);
    let mut out = Vec::new();
    let mut labels = [0u8; UP_INTERVALS];
    for symbol in observed {
        // The model's frame grid, not the window's: a data frame is twelve
        // symbols from B1u's first, and the scoring window does not start on
        // one, so the boundary is `(start + n) mod twelve`.
        let index = start + symbol.n;
        let k = symbol.k(start);
        labels[index % UP_INTERVALS] = (symbol.class(start) as i64 - offset[k]).rem_euclid(symbol.modulus.max(1) as i64) as u8;
        if index % UP_INTERVALS == UP_INTERVALS - 1
            && let Ok(frame) = decoder.decode(&labels, &moduli, bits)
        {
            let frame_bits: Vec<bool> = frame.bits().collect();
            let ones = frame_bits.iter().filter(|&&bit| descrambler.descramble(bit)).count();
            out.push((index / UP_INTERVALS, frame_bits, ones));
        }
    }
    out
}

/// The classes through 6.4.1 backwards and GPA: the bits a B1u carrying this
/// class sequence decodes to, under the offsets given.
///
/// The decoder's own sign convention, so that an inverted line decodes the same
/// -- 6.4.1's differential step exists for exactly that.
fn decode_classes(
    observed: &[Observed],
    bits: u32,
    moduli: &[u8; UP_INTERVALS],
    offset: [i64; TRELLIS_FRAME],
    start: usize,
) -> (usize, usize) {
    let moduli = Moduli12::new(*moduli);
    let mut decoder = modulus::Decoder::new();
    let mut descrambler = Scrambler::new(Mode::Answer);
    let mut ones = 0usize;
    let mut total = 0usize;
    let mut labels = [0u8; UP_INTERVALS];
    for symbol in observed {
        // The decoder's class map: the class rotated back by the offset it
        // applies at this position, which is `index_to_ki`'s pair offset at
        // k = 0 and 2 and nothing elsewhere. The grid is the model's, so the
        // frame closes on `(start + n) mod twelve`.
        let index = start + symbol.n;
        let k = symbol.k(start);
        labels[index % UP_INTERVALS] = (symbol.class(start) as i64 - offset[k]).rem_euclid(symbol.modulus.max(1) as i64) as u8;
        if index % UP_INTERVALS == UP_INTERVALS - 1
            && let Ok(frame) = decoder.decode(&labels, &moduli, bits)
        {
            for bit in frame.bits() {
                ones += usize::from(descrambler.descramble(bit));
            }
            total += bits as usize;
        }
    }
    (ones, total)
}
