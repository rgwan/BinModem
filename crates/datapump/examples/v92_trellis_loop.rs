//! Does our own trellis model close on our own transmitter, with both of the
//! decoder's decisions?
//!
//! The question this answers is *not* "does the far end send V.92's 4D code".
//! It is the narrower one a loopback can answer: with the transmitter and the
//! receiver both ours, does 6.4.4's chain -- labels to `[Y4..Y1]` to `Y0`,
//! `Y0` read off the state before the frame's own inputs, parity of the four
//! indices at `k = 3`, and 6.4.3's pairs read with the **second** index as the
//! real part (`docs/110`) -- carry the same number at both ends?
//!
//! Strictest form: [`Chain`](datapump::v92::precoder::Chain)'s own levels (via
//! [`UpEncoder`], so the modulus encoder, GPA and 6.4.1's differential sign are
//! in the path too) into two [`Decoder`]s, one taking each [`TrellisMode`], on a
//! channel that is the identity (the prefilter is `z2 = [1]`, the precoder is
//! empty, and the gain is `G`, so a level on the wire is `G x a(eta)` exactly).
//! Three numbers come out per state count:
//!
//! * (a) the receiver's `Y0`, from the label chain and the code, against the
//!   transmitter's `Y0` for every trellis frame;
//! * (b) the bits the two decisions read out, against each other;
//! * (c) the bits either of them read out, against the ones the encoder was
//!   given.
//!
//! **What it cannot prove.** Both ends run the same tables, so a convention
//! error made once in `v34::trellis` or in the delay 6.4.4 reads into 9.6.3.2
//! passes here and still disagrees with a far end that reads the clause
//! differently. This is a test of *our chain against itself*, which is exactly
//! what "no internal inconsistency" means and no more. The far end is the
//! experiment the CPd trellis field sweep and `docs/110` are for.
//!
//! Usage: `cargo run --release -p datapump --example v92_trellis_loop`

use datapump::v92::decoder::{Decoder, TrellisMode};
use datapump::v92::precoder::{Constellation, inverse_map};
use datapump::v92::upstream::UpEncoder;
use datapump::v92::{
    CONSTELLATION_FRAME, Filters, Parameters, TRELLIS_FRAME, Trellis, UP_INTERVALS, up_bits,
};

/// Parameters for a flat channel: one set of `levels` positive points, every
/// interval on modulus `2^(K/12)`, a prefilter of unity gain and no precoder,
/// so `Chain`'s `out` is `G a(eta)` with nothing else in the way. `G` is the
/// largest the chain's own output limit allows, which is what keeps the
/// transmission out of the clamp -- a clamped level is not on the lattice and
/// nothing downstream can recover it.
fn parameters(drn: u8, levels: usize, trellis: Trellis) -> Parameters {
    #[allow(clippy::cast_possible_truncation)]
    let modulus = 1u8 << (up_bits(drn) / UP_INTERVALS as u32);
    Parameters {
        drn,
        trellis,
        extend_e2u: false,
        gain: 8.0 / (2.0 * levels as f64 - 1.0),
        moduli: [modulus; UP_INTERVALS],
        filters: Filters { z2: vec![1.0], ..Filters::default() },
        sets: vec![(0..levels).map(|index| (2 * index + 1) as u16).collect()],
        indices: [0; CONSTELLATION_FRAME],
        levelset: datapump::v92::LevelSet::Uniform,
    }
}

/// A deterministic bit source: no crate, and the same bits on every run.
struct Random(u64);

impl Random {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn next(&mut self) -> bool {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 63 == 0
    }
}

/// The nearest index overall, which is what the slicing decoder's `decide`
/// takes: each residue class keeps its cheapest member and the nearest is the
/// cheapest of the four, so the cheapest overall is the same number.
fn slice(set: &Constellation, level: f64) -> i32 {
    let mut best: Option<(f64, i32)> = None;
    for eta in set.lowest()..=set.highest() {
        let Some(point) = set.level(eta) else { continue };
        let cost = (level - point) * (level - point);
        if best.is_none_or(|(cost_of, _)| cost < cost_of) {
            best = Some((cost, eta));
        }
    }
    best.expect("a set with a member").1
}

fn main() {
    let drn = 1; // 24 000 bit/s, the bench's own rung: Mi = 8, LC = 8, K = 36
    let mut failed = false;
    for trellis in [Trellis::Sixteen, Trellis::ThirtyTwo, Trellis::SixtyFour] {
        let parameters = parameters(drn, 8, trellis);
        let code = datapump::v92::precoder::code_for(trellis);
        let mut encoder = UpEncoder::new(&parameters).expect("the parameters are feasible");

        // B1u, then data frames of pseudo-random bits: the transmitted bits are
        // recorded in the order the encoder pulled them, which is the order
        // 6.4.1 keeps them in (b0 first) and the order the decoder emits.
        let mut sent: Vec<bool> = Vec::new();
        let mut symbols = encoder.b1u();
        sent.extend(std::iter::repeat_n(true, 48 * up_bits(drn) as usize));
        let mut random = Random::new(0x5EED_1234);
        let frames = 200;
        for _ in 0..frames * UP_INTERVALS {
            symbols.push(encoder.next_symbol(|| {
                let bit = random.next();
                sent.push(bit);
                bit
            }));
        }

        // The two receivers, from B1u's first symbol: every memory zero (8.7.1).
        let mut sliced = Decoder::with_trellis(&parameters, TrellisMode::Slice);
        let mut viterbi = Decoder::with_trellis(&parameters, TrellisMode::Viterbi);
        for symbol in &symbols {
            sliced.feed(symbol.out);
            viterbi.feed(symbol.out);
        }
        let from_slice = sliced.take_bits();
        let from_viterbi = viterbi.take_bits();

        // (a) Y0 from the label chain, against the transmitter's own Y0. The
        // indices are the ones the slicing receiver read, so this is the
        // transmitter's Y0 sequence checked against the frames it sent.
        let mut state = 0u8;
        let mut from_code = 0usize;
        let mut from_parity = 0usize;
        let mut frames_seen = 0usize;
        let mut sliced_indices: Vec<i32> = Vec::with_capacity(symbols.len());
        for symbol in &symbols {
            let set = Constellation::new(
                parameters.set_for(datapump::v92::interval(symbol.n as u64)).expect("a set"),
            );
            sliced_indices.push(slice(&set, symbol.out / parameters.gain));
        }
        for frame in 0..symbols.len() / TRELLIS_FRAME {
            let at = frame * TRELLIS_FRAME;
            let etas: [i32; TRELLIS_FRAME] = sliced_indices[at..at + TRELLIS_FRAME].try_into().expect("four");
            assert_eq!(etas, [symbols[at].eta, symbols[at + 1].eta, symbols[at + 2].eta, symbols[at + 3].eta],
                "a flat channel did not carry the levels it was given");
            let wanted = symbols[at].y0;
            // The receiver's reading of Y0: the state's output, before this
            // frame's own inputs are clocked in -- 6.4.4 reading 9.6.3.2.
            if code.output(state) != wanted {
                from_code += 1;
            }
            // And the parity the frame's four indices carry.
            if (etas.iter().sum::<i32>().rem_euclid(2) == 1) != wanted {
                from_parity += 1;
            }
            state = code.next(state, inverse_map(etas) & code.inputs());
            frames_seen += 1;
        }

        // (b) and (c), the two decisions against each other and against what
        // was sent. The window holds the last forty trellis frames back, so
        // only the prefix exists to compare.
        let agree = from_slice == from_viterbi;
        let n = from_viterbi.len().min(sent.len());
        let wrong = (0..n).filter(|&i| from_viterbi[i] != sent[i]).count();
        let first_wrong = (0..n).find(|&i| from_viterbi[i] != sent[i]);
        let slice_wrong = (0..n).filter(|&i| from_slice.get(i).copied() != Some(sent[i])).count();

        println!("trellis {} states, {frames_seen} trellis frames, {} symbols", trellis.states(), symbols.len());
        println!(
            "  (a) Y0: code said {}, parity said {}, of {frames_seen} frames wrong",
            from_code, from_parity
        );
        println!(
            "  (b) viterbi vs slice: {} of {} bits agree, {} decided frames each",
            if agree { "all" } else { "NOT all" },
            from_slice.len(),
            sliced.decided()
        );
        println!(
            "  (c) bits: {n} compared, {wrong} wrong through the viterbi, {slice_wrong} through the slice{}",
            first_wrong.map(|i| format!(" (first at bit {i})")).unwrap_or_default()
        );
        failed |= from_code != 0 || from_parity != 0 || !agree || wrong != 0 || slice_wrong != 0;
    }
    assert!(!failed, "our own chain does not close on our own transmitter");
}
