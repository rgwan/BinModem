//! The V.92 digital modem from Phase 3 on (9.5.1, 9.6.1): the V.90 downstream
//! encoder unchanged (clause 5), with a PCM receiver facing up the line.
//!
//! ```text
//! digital  (silent) ......................  Sd S'd TRN1d Jd ... Jd Jp ... Jp' DIL ... Ri R'i TRN2d
//! analogue  sil Ru R'u [MD] Ru R'u TRN1u Ja      (silent)         Su S'u Su S'u TRN1u CPt E1u
//! ```
//!
//! Phase 3 is one long wait on the analogue modem's training sequence followed
//! by one long answer, and every arrow in the figure is an event this modem
//! either hears on the line or decides on its own:
//!
//! - **Hear Ru turn over**, wait out the MD that INFO1a asked for if any, and
//!   train the receiver on TRN1u (9.5.1.1.1-9.5.1.1.2).
//! - **Read Ja** -- the DIL descriptor and the nineteen-bit upstream rate mask
//!   of Table 20 (9.5.1.1.3), which is what says which segments of DIL to send
//!   back.
//! - **Send Sd, S-bar-d, TRN1d and Jd** (9.5.1.1.3-9.5.1.1.5). These are
//!   V.90's signals unchanged (8.6.7 and 8.6.8 point at V.90's 8.4.4 and
//!   8.4.5); only Jd's bit 47 has changed meaning, and [`crate::v92::sequences`]
//!   owns that.
//! - **Hear Su**, and answer with **Jp** (9.5.1.1.6-9.5.1.1.8). Jp's epsilon
//!   is the whole point of Phase 3 for the upstream: it is the only thing the
//!   digital modem can do about where the network's A/D samples. It is
//!   measured on the Su that follows the first S-bar-u, once enough of that run
//!   is in, which is what 9.5.1.1.7-9.5.1.1.8 ask for.
//! - **Hear the second S-bar-u**, end Jp, raise circuit 107, and send **Jp'**
//!   and then the **DIL** the analogue modem asked for (9.5.1.1.9-9.5.1.1.11).
//! - **Read CPt** behind the DIL, answer **Ri**, and on **E1u** answer
//!   **R-bar-i** and go to Phase 4 with TRN2d (9.5.1.1.12).
//!
//! The readings that are the Recommendation's but not its letter are named
//! constants rather than buried: [`ZERO_DIL_USES_SCR`] for the branch with no
//! DIL asked for (9.5.1.1.13), and Jp's epsilon, which comes from
//! [`crate::v92::epsilon`]'s phase measurement of Su **in the line's own
//! samples** and on the run that follows the 24.5T S-bar-u (9.5.1.1.7). It is
//! the only number in Phase 3 that says where the far end's transmit clock is,
//! and everything a CPt needs -- thirty-odd decibels of a two-point stream --
//! depends on it being right: see [`crate::v92::receiver`]'s table.

use std::collections::VecDeque;

use crate::v32::{Mode, Scrambler};
use crate::v34::info::Info1aPcmUp;
use crate::v90::encoder::Encoder;
use crate::v90::encoder::Mapping as CodeMapping;
use crate::v90::sequences::{Cp, CpFinder, Descriptor, DescriptorFinder, Jd, JA_UNIT_BITS, Layout};
use crate::v90::ucode::{self, Law};
use crate::v92::UP_INTERVALS;
use crate::v92::decoder::Decoder;
use crate::v92::receiver::reference;
use crate::v92::design::{Channel, Design, design};
use crate::v92::epsilon;
use crate::v92::exchange::{Context, Exchange};
use crate::v92::receiver::{ALIGN_WINDOW, Heard, Receiver, Slicer, Symbol, TRAIN_WINDOW};
use crate::v92::sequences::{
    CpFamily, Cpd, J, JP_PRIME_BITS, Jp, Suvd, UpFinder, trn2u_unit,
};
use crate::v34::probe::Reading;
use crate::v92::up_signals::{Trn2uReader, Trn2uSize, TwoPointReader};
use crate::v92::upchoice::{choose, pin};
use crate::v92::{FilterLimits, Parameters, PeerCp, PeerSuv, SequenceKind, Trellis};

/// The network's rate: one codeword per 125 microseconds (5/V.92).
pub const FS: f64 = 8000.0;

/// The downstream data frame interval count, which V.92 does not change.
const INTERVALS: usize = 6;

/// Sd and S-bar-d, in frames: 384T and 48T (V.90 8.4.4).
const SD_FRAMES: usize = 64;
const SD_BAR_FRAMES: usize = 8;

/// R-bar-i: "4 repetitions of the 6-symbol sequence" (V.90 8.6.4).
const R_BAR_FRAMES: usize = 4;

/// TRN2d: "at least 2040T" (9.6.1.1), and this end sends more than that.
///
/// The minimum is what the Recommendation requires; what the far end needs is
/// time to notice the change from R-bar-i to a scrambled two-point stream, and
/// until it has, its descrambler is not where this end's is and the first SUVd
/// that follows comes out of it as noise. An Agere SV92PP on the bench read that
/// noise as **Jd'** -- the digital modem's retrain request -- and put the call
/// back through V.34 Phase 2 to its death, three times running, at a TRN2d of
/// exactly the 2040T the clause asks for.
const TRN2D_LEAST: usize = 4800;

/// Symbols of TRN2u the Phase 4 fit waits for before it is taken.
const TRN2U_FIT_AFTER: u64 = 2400;

/// Symbols of TRN2u dropped after the fit is in, before the finder sees any
/// bits.
const TRN2U_DROP_AFTER_FIT: u64 = 300;

/// Symbols between the CPds of a repeat group, with SUVd in all the gaps.
///
/// 9.6.2.1.2 has the analogue modem send its CPu on receiving an *SUVd*, and an
/// Agere SV92PP needs a couple of seconds of SUVd before its detector acquires
/// at all; a CPd is 3732 bits, near half a second of the line. A group repeated
/// every fifteen hundredths of a second is three quarters CPd and starves it --
/// measured: with `1200` the far end never acquired in nine seconds, while the
/// same call with the repeats never firing at all (its acknowledge bit came
/// back) had the line SUVd for the two seconds it needed and detected at once.
/// Three seconds is long enough for several acquisitions and still well inside
/// 9.6.1.1.3's "repeated CPd sequences".
const REPEAT_CP_GAP: u64 = 1200;

/// Symbols of TRN1u that are allowed to go by before the fit is taken. It is
/// not a comfort margin: the receiver's history is a window of 4096 samples,
/// so the stretch that is still in it when the fit is taken is
/// `history - (this - 4096)` long. Waiting 700 left seven hundred symbols,
/// fourteen equations per tap, and the fit came out at 17 dB with the
/// reference reaching past TRN1u's end. Waiting 1800 leaves eighteen hundred
/// and stays inside TRN1u's 2040T minimum.
const TRAIN_AFTER: u64 = 1300;

/// Symbols either side of the hunt's reversal estimate that the TRN1u fit is
/// tried at: the hunt places the reversal to within a period, and TRN1u's
/// first symbol is 24T after it.
const TRAIN_SEARCH: u64 = 64;

/// TRN1u starts 24T after the Ru-to-R-bar-u reversal (9.5.1.1.2).
const TRN_START: u64 = 24;

/// One step of MD: 276T per unit of INFO1a's length (8.5.3).
const MD_STEP: u64 = 276;

/// 9.5.1.2.1: Ja within 4500 ms plus a round trip of the end of INFO1a.
const JA_WITHIN: f64 = 4.5;

/// 9.5.1.2.2: Su within 5100 ms plus a round trip of **the start of TRN1d** --
/// not of Ja, and not of the end of TRN1d.
const SU_WITHIN: f64 = 5.1;

/// Where the Su that Jp's epsilon is measured on begins, counted from the
/// Su-to-S-bar-u reversal that opens it.
///
/// 9.5.1.1.7: on that reversal the receiver "shall condition its receiver to
/// continue receiving signal Su" -- the run *after* the 24.5T S-bar-u -- and
/// keep measuring. That run is the one whose phase already carries the half
/// symbol, so what is left over is exactly the epsilon Jp carries.
///
/// The S-bar-u is 24.5T long and the hunt reports a reversal up to a period
/// after the event, so 36T in is at least twenty symbols inside the Su that
/// follows it, with its half-symbol step behind us.
const SU_AFTER_S_BAR: u64 = 36;

/// And how much of it is measured: sixteen periods of the six-symbol shape,
/// 12 ms of Su. Long enough that the phase is steady to a few thousandths of a
/// symbol, short enough that Jp is on the line well before the far end has any
/// reason to stop sending Su.
const SU_MEASURE_SYMBOLS: usize = 96;

/// Symbols into the second TRN1u at which the upstream is designed (6.4), and
/// so the length of the stretch it is estimated on.
///
/// At least this much of it has to be in the receiver's history when the design
/// is taken, and the history is a second of samples, so this is a compromise:
/// long enough that a forty-eight tap estimate has fifty equations a tap, short
/// enough that it fits whatever the analogue modem sent before Ri.
const DESIGN_AFTER: u64 = 2400;

/// Symbols of the **second** TRN1u the refit is taken over.
///
/// It only has to carry the timing step the 24.5T and (24 + epsilon)T S-bar-u
/// left, not to reach the figure a CPt needs on its own: TRN1u is at least
/// 2040T and CPt starts after it (9.5.2.1.11), so waiting out a long window
/// would put the refit after the frames it is for. Six hundred symbols is
/// seventy-five equations a tap.
const REFIT_WINDOW: usize = 600;

/// How well the line has to correlate with TRN1u, at its best alignment within
/// [`REFIT_SEARCH`], before that alignment is believed: 1 is a line carrying
/// exactly the sequence, 0 nothing like it, and a window of anything else lands
/// at about `1/sqrt(`[`ALIGN_WINDOW`]`)` = 0.04. Half is far past that.
const REFIT_QUALITY: f64 = 0.5;

/// How much worse than the fit already in place a refit may be, in decibels,
/// and still be taken for the alignment's sake. The two-point stream's figure
/// is set by what the line has at 4000 Hz rather than by the timing, so a
/// refit that lands right can still read a little lower.
const REFIT_SLACK_DB: f64 = 3.0;

/// [`REFIT_SLACK_DB`] for this run, when `V92_REFIT_SLACK` names another.
///
/// Diagnosis only, and off unless the variable is set: `docs/80` needs to know
/// whether Phase 3's exit is decided by this threshold or by the far end, and
/// the only way to ask is to move the threshold and watch the CPt read. A
/// large negative value takes every refit; a large positive one takes none.
/// `V92_REFIT_OFFSET`: whole symbols added to the second TRN1u's alignment, for
/// diagnosis only.
///
/// `docs/80` finds the refit's own fit reading 6 dB below where it reads inside
/// the window, at phases where the correlation's alignment is the same index it
/// is inside. This moves that alignment and nothing else, which is the only way
/// to ask from outside whether the fit is being taken in the wrong place.
fn refit_offset() -> i64 {
    use std::sync::OnceLock;
    static OFFSET: OnceLock<i64> = OnceLock::new();
    *OFFSET.get_or_init(|| {
        std::env::var("V92_REFIT_OFFSET").ok().and_then(|t| t.parse().ok()).unwrap_or(0)
    })
}

fn refit_slack() -> f64 {
    use std::sync::OnceLock;
    static SLACK: OnceLock<f64> = OnceLock::new();
    *SLACK.get_or_init(|| {
        std::env::var("V92_REFIT_SLACK").ok().and_then(|t| t.parse().ok()).unwrap_or(REFIT_SLACK_DB)
    })
}

/// How far either side of where 9.5.2.1.9 puts the second TRN1u its alignment
/// is searched. The hunt's reversal is up to a period late and epsilon moves
/// the far end's whole transmit grid, so the estimate is good to tens of
/// symbols and not to one.
const REFIT_SEARCH: u64 = 192;

/// E1u: "a data frame of scrambled, differentially encoded zeroes" (8.5.2),
/// which is twelve decoded zero bits where a CPt's synchronisation would be.
const E1U_ZEROES: usize = 12;

/// Whether the zero-DIL branch of 9.5.1.1.13 is taken when the descriptor asks
/// for no DIL. It is: that is what the branch is for, and V.90's DIL code
/// already reports the empty descriptor correctly.
pub const ZERO_DIL_USES_SCR: bool = true;

/// How long after Ed this end waits for an E2u it can *confirm* before the
/// phase 4 exchange is declared failed, in seconds of line time.
///
/// It used to be the moment the receiver was conditioned for B1u "regardless",
/// which is the one thing a timer cannot know: B1u's frame interval 0 is a
/// symbol of the far end's, not of this end's clock, and a receiver that
/// conditions on its own timer reads a data frame from a boundary that never
/// existed. Measured on `/tmp/p0corr/g712.raw`, that path delivered 14 kB of
/// wrong bytes and dropped a V.42 that never opened; a call that cannot find
/// B1u now fails in the open instead, where the retrain and the fallback the
/// start-up already has can act on it.
pub const B1U_WAIT: f64 = 2.0;

/// What a candidate boundary's B1u has to read like to be one (8.7.1: "B1u
/// consists of 48 data frames of scrambled binary ones").
///
/// The window is the data frames B1u spends letting a receiver settle, which
/// this end drops anyway -- the acquisition skip in `begin_b1u` -- so the
/// criterion is free: no symbol is read twice and nothing is added to the
/// per-symbol path. Two conditions, because one alone is not enough:
///
/// * three quarters of the window has to read as ones in the better of the two
///   polarities, and
/// * that polarity has to lead the other by an eighth of the window.
///
/// The first alone does not separate a real B1u from a zero run: the decoder
/// run over an E2u this end caught 720 symbols early reads 456 of 576 ones one
/// way up and 469 the other, and a zero run's scrambling reads high in both
/// directions for the same reason white data does not. The lead is what a real
/// B1u has and a wrong window cannot: 8.7.1's content is *all* ones, so the
/// right polarity reads them and the wrong one reads a wrong pattern in their
/// place.
pub const B1U_CONFIRM_EIGHTHS: u64 = 6;

/// The lead the better polarity has to have over the other, as a fraction of
/// the window: an eighth, the margin the polarity choice has always been made
/// on. Above a zero run it is what a real B1u has and a wrong window cannot.
pub const B1U_POLARITY_LEAD: u64 = 8;

/// "B1d consists of 48 data frames of scrambled ones" (V.90 8.6.1), the signal
/// that comes between Ed and the data.
pub const B1D_FRAMES: usize = 48;

/// "B1u consists of 48 data frames of scrambled binary ones" (8.7.1), the same
/// count on the upstream side, and the window the data mode's polarity is read
/// over (see `Modem::score_b1u`).
pub const B1U_FRAMES: usize = 48;

/// Data frames one downstream fill encodes at a time.
///
/// It is the unit the codeword queue is topped up in, and so the unit in which
/// DTE bits are taken from the stack above: a caller has to have that many
/// frames' worth queued or the rest of the fill goes out as the idle pattern
/// (see [`Modem::fill_bits`]).
const B1D_FILL_FRAMES: usize = 8;

/// `V92_E2U_FROM`, the symbol a zero run has to complete at or after before it
/// is taken for E2u. Zero when the knob is unset, which is every call this end
/// makes for a subscriber: a diagnostic, not a behaviour.
///
/// It exists because the two questions in phase 4 -- "is the boundary where it
/// should be" and "do the data mode's decisions come out right once it is" --
/// cannot be separated by a run that gets the first one wrong, and the first
/// one is measurable: on the arms narrowed by a channel the zero-run detector
/// fires 720 symbols before the symbol B1u really begins, and everything after
/// it is then read from the wrong place.
fn e2u_from() -> u64 {
    use std::sync::OnceLock;
    static FROM: OnceLock<u64> = OnceLock::new();
    *FROM.get_or_init(|| {
        std::env::var("V92_E2U_FROM").ok().and_then(|t| t.parse().ok()).unwrap_or(0)
    })
}

/// `V92_PHASE3_DIAG`: why Phase 3's exit did or did not happen.
///
/// Phase 3 ends on E1u, and E1u is only looked for once a CPt repetition has
/// been found **by its CRC** -- so a CPt whose CRC never checks leaves the
/// chain silent: `cp_len` stays zero, the E1u test is never reached, and no
/// line of the transcript says so. `docs/77`-`docs/79` measured a window of
/// sampling phases in which B1u is reached at all, and could not say whether
/// that window is this receiver's property or the far end's.
///
/// With this set, [`Phase3Diag`] mirrors the CPt bit stream and every
/// `DIAG_EVERY` bits prints one line: whether any CRC hit at all, the state
/// tuple E1u is decided on, and -- when nothing hit -- the closest thing in the
/// stream to a CPt frame, which is the fewest bit errors that make some window
/// check ([`cpt_diag::closest`]). A near miss and a stream that is simply not a
/// CPt read very differently in that number, and that is the whole question.
///
/// `V92_CPT_BITS` names a file to write the same stream to, one `<symbol> <bit>`
/// line each, for a reading that has to be lined up against another arm's.
/// Both are off unless asked for, and nothing on the working path depends on
/// either: the mirror is `None`, not empty, when they are not.
fn phase3_diag() -> bool {
    use std::sync::OnceLock;
    static DIAG: OnceLock<bool> = OnceLock::new();
    *DIAG.get_or_init(|| std::env::var_os("V92_PHASE3_DIAG").is_some())
}

/// Where to write the CPt bit stream, when `V92_CPT_BITS` names a file.
fn cpt_bits_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_CPT_BITS")?);
        Some(Arc::new(Mutex::new(std::fs::File::create(path).ok()?)))
    })
    .clone()
}

/// How many CPt bits go by between two Phase 3 diagnostic lines, and how many
/// bits of the stream the closest-frame scan keeps and looks at.
const DIAG_EVERY: u64 = 2000;
const DIAG_SCAN: usize = 16384;

/// The nearest a bit stream comes to a CPt frame, for [`phase3_diag`].
///
/// A CPt is a frame sync, then a start bit and sixteen information bits per
/// block, then one more start bit and the CRC ([`crate::v90::sequences::frame`],
/// which is private to that module: the shapes are spelled out here). Three
/// things can be wrong with a window that is one: its sync is not seventeen
/// ones, a start bit is set, or its CRC field is not the CRC of the information
/// it carries. Each of those is a bit that would have to be flipped for the
/// window to check, so the sum is a distance -- "this many bits off" -- and the
/// three terms say which bits. The information field is held, so the CRC term
/// is the distance to the nearest frame carrying *that* information, which is
/// the question a stream of repeated CPts asks.
///
/// Only V.92's CPt shape is tried (Table 23: seven header blocks, eight blocks
/// per constellation, up to six constellations, with or without a codec set),
/// because it is the only one a V.92 upstream's Phase 3 can carry.
mod cpt_diag {
    use crate::v34::info::crc;
    use crate::v90::sequences::{BLOCK, SYNC_ONES};

    /// Blocks before the constellations (bits 18:135), and blocks for each
    /// constellation: `v90::sequences`' own `CP_HEADER_BLOCKS` and
    /// `MASK_BLOCKS`, which are private there.
    const HEADER_BLOCKS: usize = 7;
    const MASK_BLOCKS: usize = 8;

    /// How many sync errors a window may have and still be looked at. A CPt's
    /// sync is seventeen ones and a stream that is not a CPt averages 8.5 of
    /// them wrong, so this keeps the windows that could be a near miss and
    /// drops the rest -- which is also what keeps the scan to one pass.
    pub const NEAR: usize = 4;

    /// The closest CPt frame in a bit stream: see the module comment.
    #[derive(Debug, Clone, Copy)]
    pub struct Closest {
        /// Sync bits that are not ones, start bits that are not zeroes, and CRC
        /// bits that differ from the CRC of the information carried.
        pub errors: usize,
        pub sync: usize,
        pub starts: usize,
        pub crc: usize,
        /// Where the window starts, in CPt bits from the first one Phase 3 read.
        pub at: u64,
        /// How many blocks the frame it matched has, so that its length is
        /// `SYNC_ONES + (blocks + 1) * (BLOCK + 1)` bits.
        pub blocks: usize,
    }

    impl Closest {
        /// How long the frame it matched is, through its CRC: the length
        /// `v90::sequences::cp_length` reports before the fill pads it.
        pub fn length(&self) -> usize {
            SYNC_ONES + (self.blocks + 1) * (BLOCK + 1)
        }
    }

    /// What one scan of one bit stream found.
    #[derive(Debug, Clone, Copy, Default)]
    pub struct Scan {
        /// The window with the fewest errors, if any window was close enough to
        /// be looked at.
        pub closest: Option<Closest>,
        /// The fewest sync errors any window had, whether or not it was close
        /// enough to look at. A stream that is not a CPt at all averages 8.5 of
        /// these; a stream that is one has 0 at every repetition, so this says
        /// "the shape is not even there" on its own.
        pub min_sync: usize,
        /// How many windows were within `NEAR` sync errors, and how many bits
        /// were scanned.
        pub near: usize,
        pub bits: usize,
    }

    /// Scan every window of `bits` for the one nearest to a CPt frame.
    ///
    /// `base` is how many bits were dropped from the front of the stream before
    /// `bits` began, so that the position reported is absolute.
    pub fn closest(bits: &[bool], base: u64) -> Scan {
        let mut scan = Scan { min_sync: SYNC_ONES, bits: bits.len(), ..Scan::default() };
        let mut information: Vec<bool> = Vec::with_capacity(HEADER_BLOCKS * BLOCK);
        for at in 0..bits.len().saturating_sub(SYNC_ONES) {
            let ones = bits[at..at + SYNC_ONES].iter().filter(|b| **b).count();
            let sync = SYNC_ONES - ones;
            scan.min_sync = scan.min_sync.min(sync);
            if sync > NEAR {
                continue;
            }
            scan.near += 1;
            // One to six constellations, each with or without a codec set: the
            // shapes `Cp::from_bits_in` can read (Table 23).
            for hypothesis in 0..12 {
                let masks = (hypothesis % 6 + 1) * if hypothesis < 6 { 1 } else { 2 };
                let blocks = HEADER_BLOCKS + MASK_BLOCKS * masks;
                // The frame through its CRC -- `through_crc` in
                // `v90::sequences::cp_length` -- without the fill that pads it
                // out to a whole number of symbols.
                let length = SYNC_ONES + (blocks + 1) * (BLOCK + 1);
                if at + length > bits.len() {
                    continue;
                }
                let mut starts = 0;
                information.clear();
                for block in 0..blocks {
                    let start = at + SYNC_ONES + block * (BLOCK + 1);
                    if bits[start] {
                        starts += 1;
                    }
                    information.extend_from_slice(&bits[start + 1..start + 1 + BLOCK]);
                }
                let crc_at = at + SYNC_ONES + blocks * (BLOCK + 1) + 1;
                let mut received = 0u32;
                for i in 0..16 {
                    received |= u32::from(bits[crc_at + i]) << i;
                }
                let crc = (u32::from(crc(&information)) ^ received).count_ones() as usize;
                let errors = sync + starts + crc;
                let better = match scan.closest {
                    None => true,
                    Some(best) => (errors, blocks) < (best.errors, best.blocks),
                };
                if better {
                    scan.closest =
                        Some(Closest { errors, sync, starts, crc, at: base + at as u64, blocks });
                }
            }
        }
        scan
    }
}

/// What `V92_PHASE3_DIAG` keeps of Phase 3's CPt read.
///
/// `None` unless the variable is set, so that the working path carries a `None`
/// test and not a stream: `docs/77`-`docs/79`'s lesson is that this had to be
/// measurable, and the 24000 replay had to stay byte-identical.
#[derive(Debug, Clone, Default)]
struct Phase3Diag {
    /// The last [`DIAG_SCAN`] CPt bits, oldest first.
    bits: Vec<bool>,
    /// How many bits were dropped from the front of `bits`, so that a position
    /// in it can be named from the start of Phase 3's CPt.
    dropped: u64,
    /// The symbol the CPt reader's first bit arrived on.
    first_symbol: Option<u64>,
    /// CRC hits: how many CPt repetitions were found at all. This is the number
    /// that is zero exactly when E1u was never looked for.
    hits: u64,
    /// Which bit of the stream the first and the last hit came on.
    first_hit: Option<u64>,
    last_hit: Option<u64>,
    /// The CPt one hit read, which is the far end's own frame and is what a
    /// failing arm's stream can be compared against.
    frame: Option<Vec<bool>>,
    /// What Jp asked the far end for: the lag it was measured from, and the
    /// epsilon that went out.
    jp: Option<(Option<f64>, u16)>,
    /// Whether Phase 3's exit has already been reported.
    exited: bool,
}

impl Phase3Diag {
    /// One more bit of the CPt stream, bounded to the scan's own window.
    fn push(&mut self, bit: bool) {
        self.bits.push(bit);
        if self.bits.len() > 2 * DIAG_SCAN {
            self.bits.drain(..DIAG_SCAN);
            self.dropped += DIAG_SCAN as u64;
        }
    }

    /// Where the next bit goes in the whole stream.
    fn count(&self) -> u64 {
        self.dropped + self.bits.len() as u64
    }
}

/// Whether V92_TRACE in the environment is set, so that a start-up that will
/// not find its place is legible.
fn tracing() -> bool {
    use std::sync::OnceLock;
    static TRACE: OnceLock<bool> = OnceLock::new();
    *TRACE.get_or_init(|| std::env::var_os("V92_TRACE").is_some())
}

/// Where to write the upstream bit stream, when `V92_UP_TRACE` names a file.
///
/// Phase 4's upstream is the one direction this end has never had a readout
/// for: a sequence that does not parse leaves nothing behind, so what the far
/// end actually sent has to be written down to be looked at offline.
/// `V92_B1U_BITS`: the bits the decoders read out of B1u, as they read them.
///
/// The gate counts them (`b1u_ones`/`b1u_scored`) and says nothing about where
/// they are, and neither does any other trace: `V92_B1U_TRACE` starts after the
/// scoring window and `V92_ETA_TRACE` gives the decided ideal indices, which is
/// one layer short of the bits. Without the bits themselves "reads 66.7 % of
/// B1u as ones" cannot be told from "reads two data frames in three", so the
/// pattern has to be on disk. One line per symbol: the symbol, the polarity it
/// was read at, and the bits that came out of it.
fn b1u_bits_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_B1U_BITS")?);
        Some(Arc::new(Mutex::new(std::fs::File::create(path).ok()?)))
    })
    .clone()
}

/// `V92_B1U_LEVELS`: the level stream the *decoder* is fed through B1u.
///
/// `V92DFE_TRACE` prints every symbol the receiver processes, from the symbol
/// `begin_b1u` ran on, whether or not the decoder sees it. That is not the same
/// stream: `symbol_upstream` drops `b1u_skip` symbols and feeds the decoder from
/// the next. Which of the two is one symbol out relative to a given dump cannot
/// be settled by either alone, so this records the decoder's own stream and its
/// own first symbol -- one line per fed symbol: the symbol, and the level.
fn b1u_levels_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_B1U_LEVELS")?);
        Some(Arc::new(Mutex::new(std::fs::File::create(path).ok()?)))
    })
    .clone()
}

/// `V92_B1U_LINE`: the symbol `V92_B1U_LEVELS` writes, with the two halves
/// `docs/47` needs to tell the line from this receiver's own doing.
///
/// `docs/22`'s lesson is that a per-position table is only as good as its
/// phase, so this is written at the same call site, with the same `self.now`,
/// as the level it belongs to -- not from `V92DFE_TRACE`, whose labels are one
/// symbol out (`docs/39 §2`). One line per symbol: `now raw line forward value
/// mode`, where `line` is the sample the equaliser was given and `forward` is
/// the feed-forward section's share of `value`.
fn b1u_line_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_B1U_LINE")?);
        Some(Arc::new(Mutex::new(std::fs::File::create(path).ok()?)))
    })
    .clone()
}

/// One line of [`b1u_line_file`], if it was asked for.
fn write_b1u_line(symbol: &Symbol, now: u64, mode: &str) {
    let Some(file) = b1u_line_file() else { return };
    let Ok(mut file) = file.lock() else { return };
    use std::io::Write;
    let _ = writeln!(
        file,
        "{} {} {:.9} {:.9} {:.9} {mode}",
        now, symbol.raw, symbol.line, symbol.forward, symbol.value
    );
}

fn up_trace_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::env::var_os("V92_UP_TRACE")?;
        let path = std::path::PathBuf::from(path);
        let file = std::fs::File::create(&path).ok()?;
        write_anchor(&path);
        Some(Arc::new(Mutex::new(file)))
    })
    .clone()
}

/// The wall clock, as seconds since the epoch, for a trace file's anchor.
///
/// Two processes on one machine share this clock, so writing it beside a trace
/// is what lets the received stream and the far end's own capture be lined up
/// without guessing an offset from the content.
fn epoch_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// Record `<path>.t0` beside a trace, once, at the moment it is opened.
fn write_anchor(path: &std::path::Path) {
    use std::io::Write;
    let mut side = path.to_path_buf().into_os_string();
    side.push(".t0");
    if let Ok(mut file) = std::fs::File::create(std::path::Path::new(&side)) {
        let _ = writeln!(file, "{:.6}", epoch_now());
    }
}

/// Where to write what B1u decodes to, when `V92_B1U_TRACE` names a file: one
/// `symbol level bits` line for every upstream symbol in data mode.
///
/// 8.7.1 makes B1u "48 data frames of scrambled binary ones" before any data,
/// which is the one stretch of the upstream whose contents are known without
/// the far end's help: a decoder that is right reads all ones there, and one
/// that is a symbol out reads a hole and then a run of ones.
/// Where to write the bits handed to the downstream scrambler, when
/// `V92_TX_TRACE` names a file: one character per bit, in transmission order.
///
/// This is the downstream stream V.42 owns -- before scrambling, before the
/// constellation -- so it is what a far end that says it hears nothing can be
/// checked against.
fn tx_trace_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::path::PathBuf::from(std::env::var_os("V92_TX_TRACE")?);
        let file = std::fs::File::create(&path).ok()?;
        Some(Arc::new(Mutex::new(file)))
    })
    .clone()
}

fn b1u_trace_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::env::var_os("V92_B1U_TRACE")?;
        let path = std::path::PathBuf::from(path);
        let file = std::fs::File::create(&path).ok()?;
        write_anchor(&path);
        Some(Arc::new(Mutex::new(file)))
    })
    .clone()
}

/// Where the bits the production decoder hands up are written, one line per
/// upstream symbol: `<time> <count> <bits>`.
///
/// `V92_B1U_TRACE` records the equaliser's decisions, which is what the *bit
/// layer* is judged on by `tools/v92_lapm_compare.py` -- and that comparison
/// re-implements the bit layer in Python. This records what the Rust one
/// actually produced, so the two can be told apart: a stack that never sees a
/// V.42 detection pattern on a stream that a Python model finds one in says
/// the decoder, not the line, is where to look.
fn rx_bits_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::env::var_os("V92_RX_BITS")?;
        let file = std::fs::File::create(std::path::PathBuf::from(path)).ok()?;
        Some(Arc::new(Mutex::new(file)))
    })
    .clone()
}

/// How many sample instants the far end's symbol timing is searched for
/// between this end's own, and how far either way.
const FRACTIONAL_STEPS: usize = 8;

/// The sinc kernel's half-width, in symbols.
const FRACTIONAL_HALF: i32 = 4;

fn sinc(t: f64) -> f64 {
    if t.abs() < 1e-9 { 1.0 } else { (std::f64::consts::PI * t).sin() / (std::f64::consts::PI * t) }
}

/// `received` read `delay` symbols later than it was sampled, through a
/// windowed sinc. A negative delay reads it earlier.
fn shift_by(received: &[f64], delay: f64) -> Vec<f64> {
    let last = received.len().saturating_sub(1) as i64;
    let weights: Vec<(i64, f64)> = (-FRACTIONAL_HALF..=FRACTIONAL_HALF)
        .map(|k| {
            let t = delay + f64::from(k);
            // Blackman over the kernel: the tails of a truncated sinc ring, and
            // what this is resampled for is precisely the band where ringing
            // would be mistaken for the line.
            let w = if t.abs() <= f64::from(FRACTIONAL_HALF) {
                let x = std::f64::consts::PI * t / f64::from(FRACTIONAL_HALF);
                0.42 + 0.5 * x.cos() + 0.08 * (2.0 * x).cos()
            } else {
                0.0
            };
            (i64::from(k), sinc(t) * w)
        })
        .collect();
    (0..received.len())
        .map(|n| {
            weights
                .iter()
                .map(|&(k, w)| {
                    let at = (n as i64 - k).clamp(0, last) as usize;
                    w * received[at]
                })
                .sum()
        })
        .collect()
}

/// TRN1u as it should have arrived, rather than as this end's sample instants
/// happened to catch it.
///
/// The delay is the one whose least-squares fit of the sequence leaves the
/// least behind; a whole symbol of it is already taken out by the alignment
/// that found TRN1u, so what is searched for here is the part between.
fn align_fractional(known: &[f64], received: &[f64]) -> Vec<f64> {
    if received.len() < 2 * FRACTIONAL_HALF as usize + 8 {
        return received.to_vec();
    }
    // What decides the delay is not the fit's residual -- sixteen taps can
    // absorb a fractional delay almost as well as resampling can, so the
    // residual barely moves -- but how much of the estimate that absorption
    // costs. A line's own response is a main tap and a short tail; a fractional
    // delay is a spread of taps either side of it, and it is that spread, not
    // the residual, that the far end would be asked to invert.
    let spread = |taps: &[f64]| {
        let total: f64 = taps.iter().map(|tap| tap.abs()).sum();
        if total <= 0.0 {
            return f64::INFINITY;
        }
        let main = taps
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .map_or(0, |(at, _)| at);
        taps.iter()
            .enumerate()
            .map(|(at, tap)| tap.abs() * (at as f64 - main as f64).abs())
            .sum::<f64>()
            / total
    };
    let mut best: Option<(f64, Vec<f64>)> = None;
    for step in 0..FRACTIONAL_STEPS {
        let delay = step as f64 / FRACTIONAL_STEPS as f64;
        let shifted = if step == 0 { received.to_vec() } else { shift_by(received, delay) };
        let Some(channel) = Channel::estimate(known, &shifted, 16) else { continue };
        let cost = spread(&channel.taps);
        if tracing() {
            eprintln!(
                "V92TRACE: TRN1u delay {delay:.3}: noise {:.2e}, spread {cost:.2}",
                channel.noise
            );
        }
        if best.as_ref().is_none_or(|(least, _)| cost < *least) {
            best = Some((cost, shifted));
        }
    }
    best.map_or_else(|| received.to_vec(), |(_, shifted)| shifted)
}

/// Where to write the line samples this end receives, when `V92_RX_TRACE` names
/// a file: one `symbol_index level` line each. Offline that is the whole of the
/// upstream problem -- a receiver can be fitted against it without a bench run
/// per hypothesis.
fn rx_trace_file() -> Option<std::sync::Arc<std::sync::Mutex<std::fs::File>>> {
    use std::sync::{Arc, Mutex, OnceLock};
    static FILE: OnceLock<Option<Arc<Mutex<std::fs::File>>>> = OnceLock::new();
    FILE.get_or_init(|| {
        let path = std::env::var_os("V92_RX_TRACE")?;
        let path = std::path::PathBuf::from(path);
        let file = std::fs::File::create(&path).ok()?;
        write_anchor(&path);
        Some(Arc::new(Mutex::new(file)))
    })
    .clone()
}

/// How phases 3 and 4 are going.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Running,
    /// In Phase 4 from the end of Phase 3: TRN2d is on the line and CPd is to
    /// be designed and sent.
    Phase4,
    /// In data mode: downstream and upstream rates in bit/s.
    Connected { downstream: u32, upstream: u32 },
    ClearedDown,
    Failed(&'static str),
}

/// How this digital modem goes about the parts the Recommendation leaves open.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Habits {
    /// TRN1d, in seconds: "at least 2040T", and Jd "within 4000 ms of
    /// starting TRN1d" (9.5.1.1.4).
    pub trn1d: f64,
    /// Jd's bits 49:50, "the maximum spectral shaping look-ahead".
    pub lookahead: u8,
}

impl Habits {
    /// Short TRN1d, for the tests and for the second end of our own call.
    pub const PROMPT: Self = Self { trn1d: 0.3, lookahead: 1 };

    /// What a live server sends: the analogue modem's equaliser wants the
    /// time, and the clause's own ceiling is four seconds.
    pub const LIVE: Self = Self { trn1d: 3.6, lookahead: 1 };
}

impl Default for Habits {
    fn default() -> Self {
        Self::PROMPT
    }
}

/// What Phase 2 settled, as this modem needs it.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    pub law: Law,
    /// The codeword the analogue modem trains its A/D on, from INFO1a's UINFO.
    pub uinfo: u8,
    /// The analogue modem's MD, in 276T steps (Table 18).
    pub far_md: u8,
    pub round_trip: f64,
    /// Whether both ends have the 1664-point constellation the downstream's
    /// top rates need.
    pub wide: bool,
    /// What the analogue modem said it can build, from INFO1a bits 12:17
    /// (Table 18). CPd's filters have to be inside these or the far end cannot
    /// use the parameters and will not acknowledge them (9.6.2.1.3).
    pub limits: FilterLimits,
    pub habits: Habits,
    /// V92-63. What phase 2's L1/L2 probing measured of the upstream channel,
    /// handed over by whoever ran phase 2 (`v90::startup`).
    ///
    /// L1 is transmitted by the analogue modem and received here (V.90
    /// 9.2.1.1.5, 9.2.2.1.5), so it is a measurement of the *upstream line* and
    /// not of a fit through it. It carries per-tone magnitude and per-tone
    /// signal to noise, which is what [`design::from_reading`] turns into the
    /// far end's prefilter. It lives on the settings rather than on the modem
    /// because it is decided before this modem exists: phase 2 and phase 3 are
    /// not this object's.
    pub probing: Option<Reading>,
}

impl Settings {
    /// From INFO1a as the analogue modem sent it (Table 18).
    #[must_use]
    pub fn new(law: Law, asked: &Info1aPcmUp, round_trip: f64, wide: bool) -> Self {
        Self {
            law,
            uinfo: asked.uinfo,
            far_md: asked.md_length,
            round_trip,
            wide,
            limits: FilterLimits::from_info1a(asked.sections, asked.ltot_code, asked.lmax_code),
            habits: Habits::default(),
            probing: None,
        }
    }

    /// The same settings, with phase 2's probing reading attached.
    #[must_use]
    pub fn with_probing(mut self, reading: Option<Reading>) -> Self {
        self.probing = reading;
        self
    }

    /// This end's Jd (Table 21): every downstream rate, and the look-ahead.
    #[must_use]
    pub fn jd(&self) -> Jd {
        Jd { rates: Jd::ALL_RATES, lookahead: self.habits.lookahead, ..Jd::default() }
    }

    /// Jd's seventy-two bits, as they go out.
    #[must_use]
    pub fn jd_bits(&self) -> Vec<bool> {
        J::Jd(self.jd()).to_bits()
    }
}

/// One signal of the downstream timetable, as the source is asked for the next
/// codeword.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Out {
    /// Nothing: the line before Sd, and wherever a signal must not overlap.
    Silence,
    /// Sd, 384T (V.90 8.4.4).
    Sd,
    /// S-bar-d, 48T.
    SdBar,
    /// TRN1d, at least 2040T (V.90 8.4.5). SCR is the same signal (8.6.6).
    Trn1d,
    /// Jd, repeated (8.6.2).
    Jd,
    /// Jp, repeated (8.6.3).
    Jp,
    /// Jp', twelve symbols that end Jp (8.6.4).
    JpPrime,
    /// The DIL the analogue modem's descriptor asked for (8.6.1).
    Dil,
    /// Ri (V.90 8.6.4).
    Ri,
    /// R-bar-i, 24T.
    RiBar,
    /// TRN2d, from the start of Phase 4 (9.6.1.1).
    Trn2d,
    /// SUVd and SUVd-bar: the short information sequence, repeated, while the
    /// analogue modem's SUVu and CPu are read (9.6.1.1.2, Table 31).
    Suvd,
    /// CPd: the one sequence that tells the analogue modem how to encode the
    /// upstream (9.6.1.1.2, Table 30).
    Cpd,
    /// Ed: one data frame of scrambled zeros, after both acknowledges have
    /// crossed (9.6.1.1.4, 8.8.2).
    Ed,
    /// B1d: the downstream in data mode (9.6.1.1.5, V.90 8.6.1).
    B1d,
    /// SCR, "the same signal as TRN1d" (V.90 8.6.6), sent when the analogue
    /// modem asked for no DIL (9.5.1.1.13).
    Scr,
}

/// The Phase 4 downstream coder: V.90 5.4's encoder, built from the analogue
/// modem's CPt.
///
/// TRN2d, SUVd, CPd and Ed all go out through it. V.92 8.8.5 sends SUVd "using
/// the corresponding TRN2d modulation", 8.8.3 says the same of CPd and 8.8.2 of
/// Ed, and V.90 8.6.5 generates TRN2d "by applying scrambled binary ones to the
/// encoder of 5.4" with "the constellation set to be used ... passed in CPt" --
/// so Phase 4's whole downstream is one encoder at `drn + 8` bits per data
/// frame, six codewords a frame.
///
/// What this replaces is a two-point stream, one bit a symbol, which is what a
/// Phase 4 downstream cannot be: an Agere SV92PP slicer takes CPt's word for the
/// constellation, reads four bits a symbol from a line that carries one, and
/// never sees a frame sync in the result.
#[derive(Debug, Clone)]
struct Down4 {
    coder: Encoder,
    /// Codewords the coder has released and this has not sent: six per data
    /// frame, `ucode` and sign.
    samples: VecDeque<(u8, bool)>,
}

/// The downstream source: what goes out, a codeword at a time.
#[derive(Debug, Clone)]
pub struct Source {
    law: Law,
    uinfo: u8,
    out: Out,
    /// Codewords sent since this signal began.
    count: usize,
    /// Codewords since the first symbol of Sd, for the frame boundaries.
    symbol: u64,
    pending: Option<Out>,
    bits: VecDeque<bool>,
    jd: Vec<bool>,
    jp: Vec<bool>,
    dil: Vec<(u8, bool)>,
    dil_ends: Vec<usize>,
    /// "The scrambler is initialized to zero prior to the transmission of
    /// TRN1d" (V.90 8.4.5); it runs on through Jd, Jp and TRN2d.
    scrambler: Scrambler,
    /// The differential encoder's memory, "initialized with the final symbol of
    /// the transmitted TRN1d" (V.90 8.4.2, 8.6.3).
    sign: bool,
    trn1d_symbols: usize,
    /// The bits of the Phase 4 sequence going out now, kept so that a sequence
    /// the exchange has not followed yet can be repeated (9.6.1.1.1).
    last: Vec<bool>,
    /// B1d's codewords, already mapped: one octet a symbol (V.90 8.6.1).
    codes: VecDeque<u8>,
    /// How long TRN2d lasts: "at least 2040T" (9.6.1.1.1).
    trn2d_symbols: usize,
    /// Set when a Phase 4 sequence has run out of bits: see
    /// [`Source::take_finished`].
    finished: Option<Out>,
    /// Phase 4's own coder, from CPt; `None` until the analogue modem's CPt
    /// has been read, which is the only thing that says what constellation
    /// Phase 4 goes out on (V.90 8.6.5).
    phase4: Option<Down4>,
}

impl Source {
    fn new(settings: &Settings) -> Self {
        Self {
            law: settings.law,
            uinfo: settings.uinfo,
            out: Out::Silence,
            count: 0,
            symbol: 0,
            pending: None,
            bits: VecDeque::new(),
            jd: settings.jd_bits(),
            jp: Jp::default().to_bits(),
            dil: Vec::new(),
            dil_ends: Vec::new(),
            scrambler: Scrambler::new(Mode::Call),
            sign: false,
            trn1d_symbols: (settings.habits.trn1d * FS) as usize,
            last: Vec::new(),
            codes: VecDeque::new(),
            trn2d_symbols: TRN2D_LEAST,
            finished: None,
            phase4: None,
        }
    }

    /// The signal on the line.
    #[must_use]
    pub fn out(&self) -> Out {
        self.out
    }

    /// Whether Phase 4's TRN2d has begun.
    #[must_use]
    pub fn in_phase4(&self) -> bool {
        matches!(self.out, Out::Trn2d | Out::Suvd | Out::Cpd | Out::Ed | Out::B1d)
    }

    /// Whether TRN2d has run its least length, which is when the exchange may
    /// begin (9.6.1.1.1).
    #[must_use]
    pub fn trn2d_done(&self) -> bool {
        self.out == Out::Trn2d && self.count >= self.trn2d_symbols
    }

    /// Put a Phase 4 sequence on the line: SUVd, CPd or Ed, as bits.
    ///
    /// Only the *ask* goes here, and the sequence already on the line is left
    /// alone: `start` fills the queue from `self.last` at the boundary where
    /// that one ends. Clearing the queue here instead -- which is what this did
    /// -- emptied a sequence that was still going out, so the very next symbol
    /// found the queue empty, reported the sequence finished, and the exchange
    /// asked for the next one, which emptied it again. Each sequence got exactly
    /// one symbol onto the line, and since the first bit of both SUVd and CPd is
    /// the first bit of the frame sync, a Phase 4 that meant to repeat them put
    /// a run of scrambled **ones** there instead. An Agere SV92PP never found a
    /// frame sync in it, kept sending SUVu with the acknowledge bit clear, and
    /// after the ten seconds its wait allows raised
    /// `V34 exception: 60096212` -- its Phase 4 state waiting for an SUVd.
    pub fn send(&mut self, out: Out, bits: Vec<bool>) {
        debug_assert!(matches!(out, Out::Suvd | Out::Cpd | Out::Ed));
        self.last = self.padded(bits);
        self.change(out);
    }

    /// Install Phase 4's coder from the analogue modem's CPt.
    ///
    /// V.90 8.6.5: "TRN2d is generated by applying scrambled binary ones to the
    /// encoder of 5.4. The constellation set to be used is passed in CPt", at
    /// `(drn + 8) * 8000/6` bit/s -- and V.92 8.8.2/8.8.3/8.8.5 put CPd, Ed and
    /// SUVd on the same modulation. So every Phase 4 signal is this encoder at
    /// CPt's rate: for a CPt of drn 16, 24 bits over six codewords, four bits a
    /// symbol.
    ///
    /// False when CPt's rate does not fit its own constellations, which is the
    /// one thing 5.4.3 forbids and which leaves Phase 4 with nothing to send.
    pub fn set_cpt(&mut self, cpt: &Cp) -> bool {
        let Some(mapping) = CodeMapping::from_cp(cpt) else { return false };
        self.phase4 = Some(Down4 {
            coder: Encoder::new(mapping, self.law),
            samples: VecDeque::new(),
        });
        true
    }

    /// The bits one Phase 4 data frame carries, or zero before CPt arrives.
    #[must_use]
    pub fn phase4_frame_bits(&self) -> usize {
        self.phase4.as_ref().map_or(0, |p| p.coder.frame_bits())
    }

    /// 8.8.2: Ed is "2 data frames of scrambled binary zeroes".
    #[must_use]
    pub fn ed_bits(&self) -> Vec<bool> {
        vec![false; 2 * self.phase4_frame_bits().max(INTERVALS)]
    }

    /// Pad a sequence out to whole data frames.
    ///
    /// Table 30/31's last field is "fill bits: 0s to extend the sequence length
    /// to the next multiple of 6 symbols", and six symbols is exactly one data
    /// frame, so the unit is the encoder's `frame_bits` and not the six the
    /// two-point stream used to make it. A sequence that is not a whole number
    /// of frames shortens the last frame, and every frame after it lands four
    /// bits out of place.
    fn padded(&self, mut bits: Vec<bool>) -> Vec<bool> {
        let d = self.phase4_frame_bits();
        if d > 1 {
            while bits.len() % d != 0 {
                bits.push(false);
            }
        }
        bits
    }

    /// Whether the coder still has codewords queued for the sequence in flight.
    fn phase4_queued(&self) -> bool {
        self.phase4.as_ref().is_some_and(|p| !p.samples.is_empty())
    }

    /// The next codeword of a Phase 4 signal, mapping another data frame
    /// whenever the queue runs dry.
    ///
    /// `ones` is TRN2d, whose encoder input is "scrambled binary ones"
    /// (V.90 8.6.5); otherwise the bits come from the sequence, scrambled as
    /// 8.8.3/8.8.5 say. The scrambler and the differential encoder inside the
    /// coder carry on from one sequence to the next: only the bits change.
    ///
    /// `None` only when no CPt has been installed -- Phase 4 cannot be signalled
    /// without one, and the caller falls back to what it did before.
    fn phase4_codeword(&mut self, ones: bool) -> Option<(u8, bool)> {
        let mut phase4 = self.phase4.take()?;
        let codeword = loop {
            if let Some(sample) = phase4.samples.pop_front() {
                break sample;
            }
            let d = phase4.coder.frame_bits();
            let mut bits = Vec::with_capacity(d);
            let mut exhausted = false;
            for _ in 0..d {
                let raw = if ones {
                    true
                } else {
                    self.bits.pop_front().unwrap_or_else(|| {
                        exhausted = true;
                        false
                    })
                };
                bits.push(self.scrambler.scramble(raw));
            }
            // With look-ahead the coder holds a frame back until it has seen
            // the shaper's window past it, so push until one comes out; past the
            // end of the sequence those extra frames are fill bits, and
            // `finishing` says no more are coming.
            phase4.coder.push(&bits);
            if let Some(frame) = phase4.coder.pop(!ones && exhausted) {
                for i in 0..INTERVALS {
                    phase4.samples.push_back((frame.ucodes[i], frame.positive[i]));
                }
            }
        };
        self.phase4 = Some(phase4);
        Some(codeword)
    }

    /// And B1d's codewords, one octet a symbol.
    pub fn send_codes(&mut self, octets: impl IntoIterator<Item = u8>) {
        if self.out != Out::B1d {
            self.change(Out::B1d);
        }
        self.codes.extend(octets);
    }

    /// The Phase 4 sequence that has just run out of bits, once: what tells the
    /// exchange a sequence has ended and something else may go out.
    pub fn take_finished(&mut self) -> Option<Out> {
        self.finished.take()
    }

    /// Whether B1d wants more codewords.
    ///
    /// B1d counts while it is only *asked* for as well: `begin_data` queues the
    /// signal at the symbol that ends Ed, but B1d's first symbol is produced a
    /// tick later, and the queue has to be full *before* it is. Until this said
    /// so, `fill_downstream` was a no-op there (it asks this), so the first B1d
    /// symbol found the queue empty and the B1d arm fell back to the U_INFO
    /// codeword -- one symbol of not-B1d in front of the forty-eight data
    /// frames. Six symbols are a data frame and the far end counts them from
    /// Sd, so one extra symbol rotates every interval in the call: it slices the
    /// right codewords into the wrong six, the sign bits and the modulus bits
    /// come from the wrong places, and what it descrambles is noise.
    #[must_use]
    pub fn needs_codes(&self) -> bool {
        (self.out == Out::B1d || self.pending == Some(Out::B1d)) && self.codes.len() < 2 * INTERVALS
    }

    fn level(&self, ucode: u8, positive: bool) -> f64 {
        ucode::level(self.law, ucode) * if positive { 1.0 } else { -1.0 }
    }

    fn start(&mut self, out: Out) {
        self.out = out;
        self.count = 0;
        self.bits.clear();
        match out {
            // A Phase 4 sequence is handed in whole by `Source::send` and goes
            // out once; 9.6.1.1.1's repetition is the exchange's business.
            Out::Suvd | Out::Cpd | Out::Ed => {
                self.bits.extend(self.last.clone());
                if let Some(phase4) = self.phase4.as_mut() {
                    phase4.samples.clear();
                }
            }
            _ => self.last.clear(),
        }
        if out != Out::B1d {
            self.codes.clear();
        }
        match out {
            // "The first symbol of Sd is defined to be transmitted in data
            // frame interval 0" (V.90 8.4.4).
            Out::Sd => self.symbol = 0,
            Out::Trn1d | Out::Scr => self.scrambler.reset(),
            // 8.6.5/V.90: "the scrambler, differential encoder and spectral
            // shape filter memory shall be initialized to zero prior to
            // transmitting TRN2d". The first two are here and in the fresh
            // encoder; the third is that encoder's own shaping filter.
            Out::Trn2d => {
                self.scrambler.reset();
                if let Some(phase4) = self.phase4.as_mut() {
                    let mapping = phase4.coder.mapping().clone();
                    phase4.coder = Encoder::new(mapping, self.law);
                    phase4.samples.clear();
                }
            }
            Out::JpPrime => self.bits.extend([false; JP_PRIME_BITS]),
            _ => {}
        }
    }

    fn at_frame_boundary(&self) -> bool {
        self.symbol.is_multiple_of(INTERVALS as u64)
    }

    /// The bits of the signal in flight, for an arm that has to hold it one
    /// more repetition until a frame boundary.
    fn last_bits(&self) -> Vec<bool> {
        match self.out {
            Out::Jd => self.jd.clone(),
            Out::Jp | Out::JpPrime => self.jp.clone(),
            _ => Vec::new(),
        }
    }

    /// Ask for `out` at the next place the signal now on the line can end.
    pub fn change(&mut self, out: Out) {
        self.pending = Some(out);
    }

    /// One codeword out.
    pub fn next(&mut self) -> f64 {
        let level = self.next_level();
        // Silence occupies data frame intervals like anything else: the frame
        // count is V.90 8.4.4's "first symbol of Sd is interval 0", and every
        // later frame-structured signal -- Ri above all -- is measured against
        // it. A silence that did not advance it left the next signal starting
        // mid-frame, which rotated Ri's + + + - - - by however long the silence
        // had lasted, and an Agere SV92PP answered a rotated Ri with nothing at
        // all for two seconds and then a retrain.
        self.symbol += 1;
        level
    }

    fn next_level(&mut self) -> f64 {
        let w = 16 + self.uinfo;
        loop {
            match self.out {
                Out::Silence => {
                    if self.at_frame_boundary()
                        && let Some(next) = self.pending.take()
                    {
                        self.start(next);
                        continue;
                    }
                    return 0.0;
                }
                Out::Sd | Out::SdBar => {
                    let frames = if self.out == Out::Sd { SD_FRAMES } else { SD_BAR_FRAMES };
                    if self.count == frames * INTERVALS {
                        self.start(if self.out == Out::Sd { Out::SdBar } else { Out::Trn1d });
                        continue;
                    }
                    // {+W, +0, +W, -W, -0, -W}, turned over for S-bar-d.
                    let pattern = [(w, true), (0, true), (w, true), (w, false), (0, false), (w, false)];
                    let (u, positive) = pattern[self.count % INTERVALS];
                    self.count += 1;
                    return self.level(u, positive ^ (self.out == Out::SdBar));
                }
                Out::Trn1d => {
                    if self.count >= self.trn1d_symbols && self.at_frame_boundary() {
                        self.start(Out::Jd);
                        continue;
                    }
                    self.count += 1;
                    self.sign = self.scrambler.scramble(true);
                    return self.level(self.uinfo, self.sign);
                }
                Out::Jd | Out::Jp | Out::JpPrime => {
                    if self.bits.is_empty() {
                        match self.out {
                            // Jp' is twelve symbols, and what follows is not
                            // decided until the descriptor has been read.
                            Out::JpPrime => {
                                // 9.5.1.1.11: Jp' is followed by the DIL the
                                // analogue modem's Ja asked for; with none
                                // asked for it is SCR in its place (9.5.1.1.13
                                // with V.90 8.6.6). Until this went to `Dil`,
                                // the DIL was never put on the line at all and
                                // a far end that had asked for one waited for
                                // it while this end sat silent.
                                let next = self.pending.take().unwrap_or(if self.dil.is_empty() {
                                    Out::Scr
                                } else {
                                    Out::Dil
                                });
                                self.start(next);
                                continue;
                            }
                            // "shall finish the current Jd before sending Jp"
                            // (9.5.1.1.8) and the same for Jp': a whole
                            // repetition, not a bit count.
                            _ => match self.pending.take() {
                                Some(next) if self.at_frame_boundary() => {
                                    self.start(next);
                                    continue;
                                }
                                Some(next) => {
                                    self.pending = Some(next);
                                    self.bits.extend(self.last_bits());
                                }
                                None => {
                                    let bits = if self.out == Out::Jd { self.jd.clone() } else { self.jp.clone() };
                                    self.bits.extend(bits);
                                }
                            },
                        }
                    }
                    let bit = self.bits.pop_front().unwrap_or(false);
                    self.count += 1;
                    self.sign ^= self.scrambler.scramble(bit);
                    return self.level(self.uinfo, self.sign);
                }
                Out::Suvd | Out::Cpd | Out::Ed => {
                    // 8.8.2, 8.8.3 and 8.8.5: scrambled, and "transmitted using
                    // the corresponding TRN2d modulation" -- the same encoder
                    // TRN2d went out on, at CPt's rate, carrying on its
                    // scrambler and its differential encoder. A sequence ends at
                    // a whole data frame, so both the bits and the codewords the
                    // coder has released have to be gone before it has.
                    if self.bits.is_empty() && !self.phase4_queued() {
                        self.finished = Some(self.out);
                        match self.pending.take() {
                            Some(next) => {
                                self.start(next);
                                continue;
                            }
                            None => {
                                // The exchange has not asked for the next
                                // sequence yet: the line keeps the last one's
                                // signal by repeating it, which is what 9.6.1.1.1
                                // asks of SUVd ("repeatedly").
                                let repeat = self.last.clone();
                                self.bits.extend(repeat);
                            }
                        }
                    }
                    if let Some((ucode, positive)) = self.phase4_codeword(false) {
                        self.count += 1;
                        return self.level(ucode, positive);
                    }
                    // No CPt was installed: the two-point stream this used to
                    // be everywhere. Phase 4 cannot be signalled without CPt, so
                    // this is a caller that never set one.
                    if let Some((ucode, positive)) = self.phase4_codeword(false) {
                        self.count += 1;
                        return self.level(ucode, positive);
                    }
                    let bit = self.bits.pop_front().unwrap_or(false);
                    self.count += 1;
                    self.sign ^= self.scrambler.scramble(bit);
                    return self.level(self.uinfo, self.sign);
                }
                Out::B1d => {
                    // 9.6.1.1.5 with V.90 8.6.1: the data mode signal at the
                    // rate and constellation the analogue modem's CPu named.
                    // Its octets are handed in whole -- the mapper has already
                    // chosen them -- so they go out as they are (3.6).
                    if self.codes.is_empty()
                        && let Some(next) = self.pending.take()
                    {
                        self.start(next);
                        continue;
                    }
                    self.count += 1;
                    match self.codes.pop_front() {
                        Some(octet) => {
                            // `from_octet` gives back whether the code is
                            // *negative* -- it is `octet`'s inverse, and
                            // `octet` was asked for with `!positive`. Reading
                            // its answer as "positive" inverted every sign on
                            // the line, which is a constant the differential
                            // chain hides from all but its first symbol and
                            // nothing hides from a receiver that looks at the
                            // codeword itself.
                            let (u, negative) = ucode::from_octet(self.law, octet);
                            return self.level(u, !negative);
                        }
                        // Nothing queued yet: the last codeword again rather
                        // than a silence the far end would read as a cleardown.
                        None => return self.level(self.uinfo, self.sign),
                    }
                }
                Out::Dil => {
                    if self.dil.is_empty() {
                        let next = self.pending.take().unwrap_or(Out::Silence);
                        self.start(next);
                        continue;
                    }
                    // V.90 8.4.1, which 8.6.1/V.92 adopts whole: "The entire
                    // sequence, not just the last DIL-segment, is repeated
                    // until either the analogue modem causes it to be
                    // terminated or a timeout occurs." What ends it here is the
                    // CPt that `heard_cp` answers with Ri (9.5.1.1.12), so the
                    // descriptor runs round and round until that comes. Sending
                    // it once and then falling silent handed a far end that was
                    // still reading DIL a hole to read instead.
                    let at = self.count % self.dil.len();
                    // "The sequence shall be terminated on a DIL-segment
                    // boundary": a wrap is one, and so is every `dil_ends`.
                    if (at == 0 || self.dil_ends.contains(&at))
                        && let Some(next) = self.pending.take()
                    {
                        self.start(next);
                        continue;
                    }
                    self.count += 1;
                    let (u, positive) = self.dil[at];
                    return self.level(u, positive);
                }
                Out::Ri | Out::RiBar => {
                    let bar = self.out == Out::RiBar;
                    if bar && self.count == R_BAR_FRAMES * INTERVALS {
                        self.start(Out::Trn2d);
                        continue;
                    }
                    // Frame-aligned, because everything after depends on where
                    // this ends: R-bar-i is twenty-four symbols and the far end
                    // counts them to know where TRN2d's scrambler starts
                    // (V.90 8.6.5). Ending Ri mid-frame moves that reset by up
                    // to five symbols, after which the far end's descrambler
                    // never matches this end's -- and an Agere SV92PP read the
                    // resulting noise as Jd', its own retrain request, and put
                    // the call back through V.34 Phase 2.
                    if !bar
                        && self.at_frame_boundary()
                        && let Some(next) = self.pending.take()
                    {
                        self.start(next);
                        continue;
                    }
                    // "+ + + - - -": UINFO in every interval, and the bar the
                    // other way up (V.90 8.6.4).
                    let interval = (self.symbol % INTERVALS as u64) as usize;
                    self.count += 1;
                    return self.level(self.uinfo, (interval < 3) ^ bar);
                }
                // 9.5.1.1.13 with V.90 8.6.6: with no DIL asked for, the
                // signal in the DIL's place is SCR -- TRN1d's own -- and it
                // runs the length TRN1d would have. Then Ri, because the
                // analogue modem is waiting for it to send CPt (9.5.2.1.10).
                // And like the DIL, it can be cut short at a frame boundary by
                // whatever has been asked for since -- the analogue modem's
                // CPt arriving is what ends it when that comes first.
                Out::Scr => {
                    if self.at_frame_boundary() {
                        if let Some(next) = self.pending.take() {
                            self.start(next);
                            continue;
                        }
                        if self.count >= self.trn1d_symbols {
                            self.start(Out::Ri);
                            continue;
                        }
                    }
                    self.count += 1;
                    self.sign = self.scrambler.scramble(true);
                    return self.level(self.uinfo, self.sign);
                }
                Out::Trn2d => {
                    // "shall transmit signal TRN2d for at least 2040T"
                    // (9.6.1.1.1), and the exchange of SUVd and CPd follows it.
                    // V.90 8.4.1: a signal ends on a data frame boundary, so
                    // the far end's frame counter and this one's stay together.
                    if self.count >= self.trn2d_symbols
                        && self.at_frame_boundary()
                        && let Some(next) = self.pending.take()
                    {
                        self.start(next);
                        continue;
                    }
                    self.count += 1;
                    // 8.6.5/V.90 is the whole of it: TRN2d "is generated by
                    // applying scrambled binary ones to the encoder of 5.4.
                    // The constellation set to be used is passed in CPt. The
                    // scrambler, differential encoder and spectral shape filter
                    // memory shall be initialized to zero prior to transmitting
                    // TRN2d" -- and 8.8.2/8.8.3/8.8.5 put Ed, CPd and SUVd on
                    // "the corresponding TRN2d modulation", the same encoder
                    // carrying on. So TRN2d, SUVd, CPd and Ed are one continuous
                    // signal at CPt's rate, which is what a far end's Phase 4
                    // receiver conditions on.
                    //
                    // This end used to send TRN2d as a two-point stream at
                    // U_INFO instead -- one bit a symbol, four and a bit dB below
                    // the peak of the SUVd after it and off the constellation
                    // CPt named. A Conexant HSF reads no SUVd out of that: on
                    // the bench it sent forty-five SUVu sequences with the
                    // acknowledge bit clear, never asked for a CPd, never sent a
                    // CPu, and at 18.60 s began 9.7.2.1's retrain (70 ms of
                    // silence, then Tone A) with this end still repeating SUVd.
                    // With TRN2d through the encoder the same call has it
                    // acknowledge the CPd' seventy-eight times and both ends run
                    // on to Ed, B1d and B1u. See docs/17.
                    //
                    // A receiver with only a sign slicer is no worse off: over
                    // CPt's constellation every symbol is a legal point, and the
                    // sign stream is still the differentially encoded scrambled
                    // ones that 5.4.5 defines.
                    if let Some((ucode, positive)) = self.phase4_codeword(true) {
                        return self.level(ucode, positive);
                    }
                    self.sign ^= self.scrambler.scramble(true);
                    return self.level(self.trn2d_point(), self.sign);
                }
            }
        }
    }

    /// The codeword TRN2d's two points sit on.
    ///
    /// The loudest point that every one of CPt's six constellations has, so
    /// that one two-point signal is a legal symbol in all six data frame
    /// intervals -- 8.6.5's "the constellation set to be used is passed in
    /// CPt". U_INFO is the fallback for a Phase 4 with no CPt, which is a
    /// Phase 4 that cannot go out on the encoder either.
    fn trn2d_point(&self) -> u8 {
        let Some(phase4) = self.phase4.as_ref() else {
            return self.uinfo;
        };
        let sets = &phase4.coder.mapping().sets;
        let Some(first) = sets.first() else {
            return self.uinfo;
        };
        // `points` is loudest first, so the first one common to all six is the
        // loudest common one.
        first
            .points()
            .iter()
            .copied()
            .find(|&u| sets.iter().all(|s| s.points().contains(&u)))
            .unwrap_or(self.uinfo)
    }
}

/// Where the Phase 3 state machine has got to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// Silent, listening for Ru (D1).
    AwaitRu,
    /// Waiting out MD, then hunting Ru again (D1).
    WaitMd,
    /// Ru has turned over and TRN1u is on the line: training (D2).
    Training,
    /// Trained: reading Ja (D3).
    ReadJa,
    /// Sd, S-bar-d, TRN1d and Jd are going out; Su is what is awaited (D4-D5).
    AwaitSu,
    /// Su has turned over: Jp goes out, and the second S-bar-u ends it (D8-D9).
    AwaitSuBar,
    /// Jp' has been sent: the DIL goes out while CPt is read (D10-D12).
    Dil,
    /// Ri has gone out; CPt is still being read (D12).
    Ri,
    /// R-bar-i is on the line, and Phase 4 follows it.
    RiBar,
    /// Phase 4: TRN2d (9.6.1.1).
    Phase4,
}

/// The V.92 digital modem.
#[derive(Debug, Clone)]
pub struct Modem {
    settings: Settings,
    source: Source,
    rx: Receiver,
    stage: Stage,
    reader: TwoPointReader,
    ja: DescriptorFinder,
    cps: CpFinder,
    descriptor: Option<Descriptor>,
    cpt: Option<Cp>,
    /// Where TRN1u began, in receiver history indices.
    trn_from: Option<u64>,
    /// Where the Su that Jp's epsilon is measured on begins, in absolute
    /// symbols, once the S-bar-u reversal that opens it has been heard.
    su_from: Option<u64>,
    /// Whether Jp has gone out. It waits for the measurement (9.5.1.1.8), so
    /// the stage alone does not say whether it has.
    jp_sent: bool,
    /// And where the **second** TRN1u is expected to begin. Its fit is the one
    /// that reads CPt: the 24.5T S-bar-u between them moves the far end's
    /// symbols half a symbol, and the taps fitted before it are half a symbol
    /// out for everything after.
    refit_from: Option<u64>,
    /// The delay the first TRN1u asked for, waiting for the TRN2u that can
    /// absorb it: see `fe_install` below.
    fe_pending: Option<f64>,
    /// How many times that refit has moved its anchor on.
    refit_tries: u8,
    /// The length of one CPt repetition in bits, once one has been read, and
    /// how far through the next repetition the reader has got. E1u is twelve
    /// zeroes in the place where that repetition's synchronisation would be,
    /// so where they arrive is the whole of telling them from a CPt's own
    /// zero-filled fields (8.5.2).
    cp_len: usize,
    cp_pos: usize,
    /// Zeroes seen so far at the start of the repetition.
    e1u_zeroes: usize,
    /// Bits since the CPt reader last found a CPt by its CRC.
    ///
    /// E1u is not "twelve zeros anywhere": it is twelve zeros where the next
    /// CPt's synchronisation would be, one repetition after a CPt that was
    /// found. Without this the detector fires on silence -- and the analogue
    /// modem is silent for as long as the digital one is sending a DIL, which
    /// on this bench is nearly four seconds. Measured: this end called E1u at
    /// 20.7 s while the far end was still in Phase 3 and did not finish CPt
    /// until 22.902 s, so it left Phase 3 two seconds early and everything
    /// after it was out of step with the far end's Ri/R-bar-i window.
    since_cpt: usize,
    /// What `V92_PHASE3_DIAG` keeps of the CPt read, and nothing at all
    /// otherwise: see [`Phase3Diag`].
    diag: Option<Phase3Diag>,
    /// Bits the Ja reader has produced, and ones in the last stretch of them.
    ja_bits: u64,
    ja_ones: u64,
    /// And the same for the CPt reader, which is what says whether the far
    /// end's CPt is being decoded at all.
    cp_bits: u64,
    cp_ones: u64,
    /// Symbols since Phase 2 ended.
    now: u64,
    ja_deadline: Option<u64>,
    su_deadline: Option<u64>,
    status: Status,
    md_until: Option<u64>,
    /// Whether the MD that INFO1a asked for has been waited out. It is sent
    /// once, between the first Ru and the second (9.5.1.1.1); the second
    /// reversal is the one that starts TRN1u.
    md_done: bool,
    /// Circuit 107, raised on the Jp-to-Jp' change (9.5.1.1.9).
    circuit107: bool,
    /// When the upstream design is taken: a few thousand symbols into the second
    /// TRN1u, which is as long as the history lets it be. Taking it at the refit
    /// would leave six hundred symbols to estimate a forty-eight tap channel on;
    /// taking it at Phase 4 would leave none, because this end's own DIL is as
    /// long as the analogue modem asked for.
    design_at: Option<u64>,
    /// Where the timetable says the second TRN1u begins, in absolute symbols:
    /// the refit's correlation may not have run, and 9.6.1.1.1 needs an
    /// alignment either way.
    trn2_estimate: Option<u64>,
    /// Where the second TRN1u's first symbol is, in absolute symbols. 9.6.1.1.1
    /// designs the upstream from it, and 9.5.1.1.10 counts data frames from it.
    trn2_from: Option<u64>,
    /// When to take the Phase 4 fit on TRN2u, in symbols.
    four_at: u64,
    /// The absolute symbol where TRN2u's scrambler resets (8.7.6: the E1u that
    /// ends CPt), which is the alignment the Phase 4 fit is taken against.
    trn2u_from: u64,
    /// Up to which symbol the reader's bits are thrown away rather than given
    /// to [`Self::up_finder`] and the E2u counter.
    ///
    /// TRN2u is a known sequence: it is fitted on, not read. What it decodes to
    /// before the Phase 4 equaliser is in place is a line read at thirteen
    /// decibels, and a finder looking for a seventy-two-bit SUVu in that will
    /// eventually find one that is not there. So the first stretch of it is
    /// dropped. The receiver behind it is not gated: it keeps decoding, keeps
    /// its differential chain, and the fit it makes is what ends the drop.
    upstream_drop_until: u64,
    /// Bits the reader gave back that were dropped, for the trace.
    upstream_dropped: u64,
    /// 9.6.1.1.2-9.6.1.1.4: the SUV/CP/E exchange, which decides what goes out
    /// next and when the handshake is done.
    exchange: Option<Exchange>,
    /// 8.7.2-8.7.6: four points to a symbol, differentially decoded and GPA
    /// descrambled -- what TRN2u, SUVu and CPu all arrive as.
    upstream: Option<Trn2uReader>,
    /// Which of Tables 28 and 29 the far end is transmitting, which is Jp bit
    /// 48's to say (9.5.1.1.8). It is one field rather than two so that the bit
    /// this end sends and the reader it builds cannot disagree: an eight-point
    /// TRN2u read as four points is a quarter of the bits, and the SUVu' that
    /// CPd needs never comes out.
    upstream_size: Trn2uSize,
    /// And what picks a CPu or an SUVu out of the bits it gives back.
    up_finder: Option<UpFinder>,
    /// The CPd this end will send, and the parameters behind it, which are also
    /// what the upstream decoder needs once B1u starts (6.4).
    cpd: Option<Cpd>,
    parameters: Option<Parameters>,
    /// The analogue modem's CPu, once it has arrived: the downstream's data
    /// mode parameters (9.6.1.1.5).
    cpu: Option<Cp>,
    /// `V92_B1U_ADJ`: what to do to the data-mode level before the decoder
    /// sees it. Empty means nothing, which is the only behaviour in use.
    b1u_adjust: String,
    /// The levels `V92_B1U_ADJ=dc` averages over.
    b1u_history: std::collections::VecDeque<f64>,
    /// Zero bits in the run now on the line, for E2u: a data frame's worth of
    /// descrambled zeros is what tells it from anything else (8.5.2, 8.7.2),
    /// counted in bits because bits are what the reader gives back. See
    /// [`Self::arm_e2u_search`] for what a frame's worth is, and for why the
    /// run is measured to its end rather than to its twelfth bit.
    e2u_zeroes: u32,
    /// Whether the E2u search is listening. See [`Self::arm_e2u_search`].
    ///
    /// 9.6.2.1.4 makes this a *superset* of Ed: the analogue modem transmits
    /// E2u once it has "received a CPd or SUVd sequence with the acknowledgement
    /// bit set **or Ed**", so a far end may send E2u before this end's Ed goes
    /// out, and both far ends measured here do. Arming at Ed alone left the
    /// search deaf over exactly the window the far end uses: on the HSF capture
    /// `20261024-063632-a41c-off-d1` the far end's E2u ends at symbol 108 733
    /// and this end's Ed went out at symbol ~109 800, so the real E2u was
    /// dropped and the first false zero run *inside the far end's data mode*
    /// (symbol 110 745) was taken for it -- which is why every symbol ever
    /// scored as B1u on that call was the far end's data and not B1u at all.
    awaiting_e2u: bool,
    /// Whether the receiver is still waiting for the symbol B1u starts on.
    /// 9.6.1.1.6 conditions it for B1u *after* E2u, and E2u ends one symbol
    /// before B1u's frame interval 0 -- which is the only symbol that says how
    /// the twelve-interval data frame and the four-symbol trellis frame fall.
    b1u_pending: bool,
    /// The symbol by which B1u has to have been found, so that an E2u this end
    /// failed to read costs the call nothing rather than everything.
    b1u_deadline: Option<u64>,
    /// The downstream data encoder, once CPu has said what to encode (V.90
    /// 8.6.1).
    downstream: Option<crate::v90::encoder::Encoder>,
    /// Data frames of B1d still to go: 8.6.1/V.90 makes B1d "48 data frames of
    /// scrambled ones" and only what follows it is data.
    b1d_left: usize,
    /// The data scrambler the downstream data mode runs on (V.90 8.6.1), reset
    /// at B1d's first symbol.
    downstream_scrambler: Scrambler,
    /// The upstream data decoder, in the parameters this end's own CPd named.
    upstream_data: Option<Decoder>,
    /// Which prefilter the CPd is designed from: `V92_Z2=fit` takes the
    /// two-point fit, anything else the L1/L2 reading. Read once, at
    /// construction, so that the choice is a property of the modem and a test
    /// can make it without the process environment.
    prefilter: Prefilter,
    /// The design's prefilter scale, folded out of the receiver for B1u.
    upstream_scale: f64,
    /// The symbol delay the design puts between what the analogue modem sends
    /// and what reaches the codec: B1u's frame interval 0 is `delay` symbols
    /// into what is received, because the prefilter and the channel it was
    /// designed against are not a plain wire.
    upstream_delay: usize,
    /// Symbols of B1u still to be skipped for that delay.
    b1u_skip: usize,
    /// The same decoder on the other polarity, kept while B1u is being read
    /// both ways up: see [`Self::b1u_polarity`] and `decide_b1u_polarity`.
    b1u_probe: Option<Decoder>,
    /// Ones each decoder has read out of B1u: `[0]` is the line as it arrived,
    /// `[1]` is it negated.
    b1u_ones: [u64; 2],
    /// Bits each of them has read, for the margin the choice is made on.
    b1u_scored: [u64; 2],
    /// Symbols of B1u still to be read both ways up before the choice is made.
    b1u_scoring: usize,
    /// Which way up the data mode is, +1 or -1, applied to the equaliser's
    /// output before the decoder sees it. See [`Self::b1u_scoring`].
    b1u_polarity: f64,
    /// Upstream data bits the decoder has produced since B1u began.
    upstream_bits: u64,
    /// DTE bits fed in from above, sent as B1d's payload (9.6.1.1.5).
    outbound: VecDeque<bool>,
    /// Upstream data bits recovered since B1u began, handed up on `take_bits`.
    recovered: Vec<bool>,
    /// Largest sample seen since the last trace line.
    peak: f64,
    notes: Vec<String>,
}

/// Where the upstream prefilter is designed from (V92-63).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Prefilter {
    /// Phase 2's L1/L2 reading: per-tone magnitude and per-tone signal to
    /// noise, the two things a power-constrained shaper wants.
    Reading,
    /// The least-squares fit of TRN1u: what there was before V92-63.
    Fit,
}

/// Which prefilter the CPd is designed from, from the value of `V92_Z2`.
///
/// [`Prefilter::Reading`] is the default and `V92_Z2=fit` is the opt-out: the
/// reading is a power-constrained shaper designed from twenty-one measurements
/// of the line, the fit is a least-squares inverse of one two-point burst, and
/// the reading is the better of the two when it is a reading.
///
/// This default has been both ways, and the record of why is worth keeping
/// because the second flip was for a reason that turned out to be a different
/// bug:
///
/// * it was `Reading` when the reading was built (V92-63, `ecb3c2`);
/// * a paired Agere bench batch then showed the identity arm at **232/540** with
///   the reading and **568/576** with the fit, i.e. a call that was working
///   made broken by a design change, and the default moved to `Fit`;
/// * the cause was not the shaping. `Design::delay` became 8 with a symmetric
///   seventeen-tap section, and `begin_b1u` added that delay to B1u's frame
///   origin on top of the completion symbol it had measured the origin from.
///   Eight is not a multiple of twelve, so the data frame rotated. With the
///   delay term removed the reading arm reads B1u as B1u again -- 7128 and 2063
///   bytes on `/tmp/p0corr/ident.raw` and on both proven-V.92 Agere captures,
///   568 ones against 449 -- so the shaping is innocent and the default belongs
///   back on it.
///
/// What the reading still cannot be trusted for is *level*: measured on a flat
/// line it reports -9.8..-12.9 dB, structure the line does not have, because it
/// includes this end's own receive response. That is a separate defect in the
/// estimate, not in the idea, and the test beside this pins whichever default
/// the argument ends on so that it cannot move again without saying so.
fn prefilter_for(requested: Option<&str>) -> Prefilter {
    match requested {
        Some("fit") => Prefilter::Fit,
        _ => Prefilter::Reading,
    }
}

fn prefilter_source() -> Prefilter {
    prefilter_for(std::env::var("V92_Z2").ok().as_deref())
}

impl Modem {
    /// From the end of Phase 2, silent until the analogue modem says Ru.
    #[must_use]
    pub fn new(settings: Settings) -> Self {
        let ja_deadline = Some(((JA_WITHIN + settings.round_trip) * FS) as u64);
        let mut rx = Receiver::new();
        // The training reference is GPA, which is the receiver's default: what
        // this end fits is the *analogue* modem's TRN1u, and that is the one
        // sequence in Phase 3 that follows the sender's role rather than the
        // receiver's (6.3, 9.5.1.1.2).
        //
        // Phase 3 opens with Ru and with nothing else (9.5.2.1.1), so the hunt
        // looks for that shape alone until the reversal -- see
        // `Receiver::expect_ru`.
        rx.expect_ru();
        // Ru becomes R-bar-u with no timing step at all (9.5.2.1.1).
        rx.expect_step(0.0);
        let source = Source::new(&settings);
        let mut modem = Self {
            settings,
            source,
            rx,
            stage: Stage::AwaitRu,
            reader: TwoPointReader::differential(),
            ja: DescriptorFinder::v92(),
            cps: CpFinder::v92(JA_UNIT_BITS),
            descriptor: None,
            cpt: None,
            trn_from: None,
            su_from: None,
            jp_sent: false,
            refit_from: None,
            fe_pending: None,
            refit_tries: 0,
            ja_bits: 0,
            ja_ones: 0,
            cp_bits: 0,
            cp_ones: 0,
            cp_len: 0,
            cp_pos: 0,
            e1u_zeroes: 0,
            since_cpt: 0,
            diag: phase3_diag().then(Phase3Diag::default),
            now: 0,
            ja_deadline,
            su_deadline: None,
            status: Status::Running,
            md_until: None,
            md_done: false,
            circuit107: false,
            trn2_from: None,
            design_at: None,
            trn2_estimate: None,
            exchange: None,
            upstream: None,
            upstream_size: Trn2uSize::Four,
            four_at: 0,
            trn2u_from: 0,
            upstream_drop_until: 0,
            b1u_adjust: std::env::var("V92_B1U_ADJ").unwrap_or_default(),
            b1u_history: std::collections::VecDeque::with_capacity(1024),
            upstream_dropped: 0,
            up_finder: None,
            cpd: None,
            parameters: None,
            cpu: None,
            e2u_zeroes: 0,
            awaiting_e2u: false,
            b1u_pending: false,
            b1u_deadline: None,
            upstream_bits: 0,
            outbound: VecDeque::new(),
            recovered: Vec::new(),
            downstream: None,
            b1d_left: 0,
            downstream_scrambler: Scrambler::new(Mode::Call),
            upstream_data: None,
            prefilter: prefilter_source(),
            upstream_scale: 1.0,
            upstream_delay: 0,
            b1u_skip: 0,
            b1u_probe: None,
            b1u_ones: [0; 2],
            b1u_scored: [0; 2],
            b1u_scoring: 0,
            b1u_polarity: 1.0,
            peak: 0.0,
            notes: Vec::new(),
        };
        // V92-63: phase 2 measured the upstream channel and used to drop the
        // measurement on the floor here. Say what was kept, once, so that a
        // call's transcript says the reading arrived even before the design
        // that uses it is computed.
        if let Some(reading) = modem.settings.probing.clone() {
            modem.note(format!("V.92: L1 probing retained: {}", reading.summary()));
            modem.note(format!("V.92: L1 probing per tone: {}", reading.compact()));
        }
        modem
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    pub fn status(&self) -> Status {
        self.status
    }

    /// DTE bits for B1d, fed in by the stack above the datapump (9.6.1.1.5).
    pub fn send_bits(&mut self, bits: &[bool]) {
        {
            use std::sync::atomic::{AtomicU64, Ordering};
            static N: AtomicU64 = AtomicU64::new(0);
            let n = N.fetch_add(bits.len() as u64, Ordering::Relaxed);
            if std::env::var_os("V92_TX_TRACE").is_some() && (n == 0 || n % 4096 < bits.len() as u64) {
                eprintln!("V92TX send_bits: +{} (total {})", bits.len(), n + bits.len() as u64);
            }
        }
        self.outbound.extend(bits.iter().copied());
    }

    /// How many B1d bits are still waiting to be encoded.
    pub fn pending_bits(&self) -> usize {
        self.outbound.len()
    }

    /// How many DTE bits [`Self::fill_downstream`] can take in one go.
    ///
    /// Eight data frames, each of which is a frame's worth of bits -- 272 of
    /// them at 45333 bit/s, because a frame is six symbols of five or six.
    /// This is a floor and not a target: a caller that keeps less queued runs
    /// the queue dry in the middle of a fill, and everything after the dry
    /// point goes out as the idle pattern instead of what the stack above
    /// handed over. The stack is not told, so a detection pattern handed down
    /// as one run of characters arrives cut into pieces -- at 45333 bit/s into
    /// 64 bits of pattern and 208 bits of ones, over and over, which no
    /// originator can read as two adjacent ADPs (V.42 7.2.1.2).
    pub fn fill_bits(&self) -> usize {
        match self.downstream.as_ref() {
            Some(encoder) => B1D_FILL_FRAMES * encoder.frame_bits(),
            // No data mode yet, so there is nothing to keep fed.
            None => 0,
        }
    }

    /// Upstream data bits recovered since B1u began, for the stack above.
    pub fn take_bits(&mut self) -> Vec<bool> {
        std::mem::take(&mut self.recovered)
    }

    /// What is on the line, for the transcript.
    #[must_use]
    pub fn phase(&self) -> &'static str {
        match self.stage {
            Stage::AwaitRu | Stage::WaitMd => "V.92 phase 3: waiting for Ru",
            Stage::Training => "V.92 phase 3: training on TRN1u",
            Stage::ReadJa => "V.92 phase 3: reading Ja",
            Stage::AwaitSu => match self.source.out() {
                Out::Sd | Out::SdBar => "V.92 phase 3: Sd",
                Out::Trn1d => "V.92 phase 3: TRN1d",
                _ => "V.92 phase 3: Jd",
            },
            Stage::AwaitSuBar => "V.92 phase 3: Jp",
            Stage::Dil => match self.source.out() {
                Out::Scr => "V.92 phase 3: SCR",
                _ => "V.92 phase 3: DIL",
            },
            Stage::Ri => "V.92 phase 3: Ri",
            Stage::RiBar => "V.92 phase 3: R-bar-i",
            Stage::Phase4 => match self.source.out() {
                Out::Trn2d => "V.92 phase 4: TRN2d",
                Out::Suvd => "V.92 phase 4: SUVd",
                Out::Cpd => "V.92 phase 4: CPd",
                Out::Ed => "V.92 phase 4: Ed",
                _ => "V.92 phase 4: B1d",
            },
        }
    }

    /// The DIL descriptor the analogue modem asked for, once Ja has been read.
    pub fn descriptor(&self) -> Option<&Descriptor> {
        self.descriptor.as_ref()
    }

    /// The upstream rate mask that came with it (Table 20 bit 188).
    pub fn upstream_rates(&self) -> Option<u32> {
        self.ja.upstream_rates()
    }

    /// The analogue modem's CPt, once one has been read.
    pub fn cpt(&self) -> Option<&Cp> {
        self.cpt.as_ref()
    }

    /// Whether circuit 107 has been raised.
    pub fn circuit107(&self) -> bool {
        self.circuit107
    }

    /// How far into the start-up this modem has got, in symbols.
    #[must_use]
    pub fn elapsed(&self) -> u64 {
        self.now
    }

    /// Lines for the transcript, taken by whoever shows one.
    pub fn take_notes(&mut self) -> Vec<String> {
        std::mem::take(&mut self.notes)
    }

    /// One network sample in, one codeword out.
    pub fn step(&mut self, input: f64) -> f64 {
        self.now += 1;
        self.peak = self.peak.max(input.abs());
        if tracing() && self.now % 2000 == 0 {
            eprintln!(
                "V92TRACE: {} symbols, stage {:?}, out {:?}, peak {:.0}, rx trained {}, snr {:.1}",
                self.now,
                self.stage,
                self.source.out(),
                self.peak,
                self.rx.is_trained(),
                self.rx.snr_db()
            );
            self.peak = 0.0;
        }
        if let Some(file) = rx_trace_file() {
            use std::io::Write;
            if let Ok(mut file) = file.lock() {
                let _ = writeln!(file, "{} {input:.8}", self.now);
            }
        }
        self.rx.feed(input);
        while let Some(heard) = self.rx.heard() {
            self.heard(heard);
        }
        self.timers();
        let out = self.source.next();
        // The SCR path ends by putting Ri on the line itself, with no CPt to
        // prompt it: once Ri is going out, the DIL stage is over.
        if self.stage == Stage::Dil && matches!(self.source.out(), Out::Ri | Out::RiBar) {
            self.stage = Stage::Ri;
        }
        if self.diag.is_some() && !matches!(self.stage, Stage::Dil | Stage::Ri | Stage::RiBar) {
            // The CPt reader has stopped: this is Phase 3's exit, printed once,
            // whether it ended on E1u or never got looked for.
            self.phase3_exit();
        }
        if self.source.in_phase4() {
            // 9.6.1.1: TRN2d is Phase 4's first signal, so the moment it
            // starts is the moment Phase 3 is over.
            self.stage = Stage::Phase4;
            if self.status == Status::Running {
                self.status = Status::Phase4;
            }
        }
        out
    }

    fn timers(&mut self) {
        // Phase 4's equaliser is fitted on TRN2u itself (8.7.6 makes it a known
        // sequence), a couple of thousand symbols into it: by then the history
        // is all TRN2u, the reader has its differential chain seeded from its
        // first symbol, and the exchange that has to be read has not started.
        if self.upstream.is_some() && self.rx.symbols() >= self.four_at {
            self.four_at = u64::MAX;
            let fit_started = std::time::Instant::now();
            if let Some(delta) = self.fe_pending.take() {
                self.rx.fe_install(delta);
                self.note(format!("V.92: fractional delay {delta:.3} installed at TRN2u"));
            }
            if self.rx.train_on_trn2u(TRAIN_WINDOW, self.trn2u_from) && self.rx.install_phase_four() {
                let fit_ms = fit_started.elapsed().as_secs_f64() * 1000.0;
                self.upstream_drop_until = self.now + TRN2U_DROP_AFTER_FIT;
                self.note(format!(
                    "V.92: phase 4 receiver fitted on TRN2u in {fit_ms:.2} ms, {:.1} dB",
                    self.rx.snr_db()
                ));
            } else {
                self.upstream_drop_until = self.upstream_drop_until.min(self.now);
                self.note("V.92: no phase 4 fit on TRN2u".into());
            }
        }
        // 9.5.1.1.7-9.5.1.1.8: Jp waits for the phase the receiver has to work
        // out, and the phase is measured on the Su that follows the 24.5T
        // S-bar-u -- the run whose own phase already carries the half symbol,
        // so what is left to ask for is the whole rest of it. Once sixteen
        // periods of that run are in, epsilon is known and Jp goes out; the far
        // end sends Su "running on" until it hears it (9.5.2.1.7).
        if self.stage == Stage::AwaitSuBar
            && !self.jp_sent
            && let Some(from) = self.su_from
            && self.now >= from + SU_MEASURE_SYMBOLS as u64
        {
            let samples = self.rx.history_range(from, from + SU_MEASURE_SYMBOLS as u64);
            let lag = epsilon::lag(&samples);
            if tracing() {
                // Where the Su runs and the S-bar-u's half symbol are, seen
                // from the measurement: 0.5 of a symbol is exactly what the
                // step between them looks like.
                let mut line = format!("V92TRACE: lag profile to {from}:");
                for step in 0..20u64 {
                    let at = from.saturating_sub(500 - 24 * step);
                    let window = self.rx.history_range(at, at + 48);
                    if window.len() < 48 {
                        line.push_str(" ----");
                        continue;
                    }
                    match epsilon::lag(&window) {
                        Some(lag) => line.push_str(&format!(" {lag:.3}")),
                        None => line.push_str(" ...."),
                    }
                }
                eprintln!("{line}");
            }
            let epsilon = lag.map_or(0, epsilon::after_half);
            if tracing() {
                eprintln!(
                    "V92TRACE: Su lag {lag:?} over {} samples from {from}, epsilon {epsilon:#06x}",
                    samples.len()
                );
            }
            self.jp_sent = true;
            self.su_from = None;
            // Four points, and the far end is told so (bit 48 = 0). TRN2u is
            // "scrambled binary ones" (8.7.6): eight points a symbol would put
            // a third more bits on the line and carry nothing extra, and the
            // data mode's constellation is CPd's to choose, not this bit's.
            // What it would cost is the reader: `upstream_size` above is what
            // decodes the whole exchange, and a bit that asked for a
            // constellation this end was not reading left Phase 4 waiting for
            // an SUVu' that could not arrive.
            let jp = Jp { epsilon, eight_in_training: false, eight_in_renegotiation: false };
            self.source.jp = jp.to_bits();
            // "shall finish the current Jd and then send Jp" (9.5.1.1.8). The
            // S-bar-u that answers Jp is (24 + epsilon)T (9.5.2.1.8), so the
            // step the next reversal takes is exactly what was just asked for.
            self.rx.expect_step(epsilon::step_direction() * f64::from(epsilon) / 65536.0);
            self.source.change(Out::Jp);
            if let Some(diag) = self.diag.as_mut() {
                diag.jp = Some((lag, epsilon));
            }
            self.note(format!("V.92: Jp with epsilon {epsilon}"));
        }
        // The second TRN1u carries the equaliser's refit at the new phase:
        // "the second TRN1u also lets the digital modem refine its equalizer at
        // the new phase", which is what the 24.5T and (24 + epsilon)T S-bar-u
        // left behind. Without it every CPt is read half a symbol out -- 19 dB
        // on a two-point stream rather than 160, see `Receiver`'s table -- and
        // its CRC never passes.
        if self.stage == Stage::Dil
            && let Some(from) = self.refit_from
            && self.rx.symbols() >= from + REFIT_WINDOW as u64
        {
            let base = self.rx.history_base();
            let at = from.saturating_sub(base);
            let before = self.rx.fitted();
            let baseline = before.snr();
            // Where the second TRN1u actually begins. The hunt reports the
            // reversal up to a period late and the far end's whole grid has
            // moved by epsilon since the first fit, so the estimate is good to
            // tens of symbols, and a least-squares fit at the wrong alignment
            // is a fit against the wrong sequence -- which is what kept this
            // refit from ever happening on a real line.
            //
            // What the correlation has to say is *that* TRN1u is here; what it
            // cannot say is *where* to the symbol. `docs/80` measured two
            // alignments two symbols apart scoring 0.7206 and 0.6734 -- a gap
            // no 512-symbol correlation can call -- where the one the
            // correlation preferred fitted 7 dB worse, because the window
            // carrying the most of TRN1u's shape was not the window TRN1u began
            // at. So the correlation shortlists and the fit chooses, in
            // `align_trn1u_by_fit`: no offset, no new constant, and the figure
            // that decides is the same least-squares fit the threshold below
            // judges.
            let mut search: Option<(u64, f64)> = None;
            let found = match self
                .rx
                .align_trn1u_by_fit(at, REFIT_SEARCH, ALIGN_WINDOW, REFIT_QUALITY, TRAIN_WINDOW)
            {
                Some((aligned, fit)) => {
                    search = Some((aligned, fit));
                    // 9.5.1.1.10: "the digital modem shall keep a modulo 12 data
                    // frame interval count from the first symbol of TRN1u" --
                    // the *second* one, and this is the first the alignment of
                    // its first symbol is known to a symbol.
                    self.rx.set_frame_origin(base + aligned);
                    self.trn2_from = Some(base + aligned);
                    // 9.6.1.1.1's design is scheduled here: see `design_at`.
                    self.design_at = self.trn2_from.map(|at| at + DESIGN_AFTER);
                    let anchor = (aligned as i64 + refit_offset()).max(0) as u64;
                    let taken = self.rx.train_verified(anchor, TRAIN_WINDOW);
                    // The two numbers this test is made of, taken where the
                    // refit will be installed: the taps in place and the ones
                    // just fitted, same samples, same reference, each at its
                    // own whole-symbol anchor. `docs/81` measured the old pair
                    // as incomparable -- the incumbent's figure came from the
                    // *first* TRN1u, the refit's from the second, and the line
                    // moves between them by more than the whole tolerance.
                    let judged = taken
                        .then(|| self.rx.best_on_window(before.sections(), anchor as usize, TRAIN_WINDOW))
                        .flatten();
                    if tracing() {
                        eprintln!(
                            "V92TRACE: refit at {aligned}, fit {fit:.3} -> {taken} at {:.1} dB (was {baseline:.1})",
                            self.rx.snr_db()
                        );
                    }
                    taken.then(|| (self.rx.snr_db(), judged))
                }
                _ => None,
            };
            if self.diag.is_some()
                && let Some((aligned, fit)) = search
            {
                // The pair the accept test was made of, printed as measured:
                // the same window, the same reference, each tap set at its own
                // whole-symbol anchor. The gap between them is the slack the
                // rule would have needed to reject, so an always-accepting rule
                // is visible here rather than inferred from the outcome.
                let judged = match found {
                    Some((_, judged)) => judged,
                    None => None,
                };
                let (in_place, refit) = match judged {
                    Some((a, b)) => (a, b),
                    None => {
                        let anchor = ((aligned as i64 + refit_offset()).max(0) as u64) as usize;
                        let a = self.rx.score_of(before.sections().0, before.sections().1, anchor, 0, TRAIN_WINDOW);
                        let b = self.rx.score_of(self.rx.taps(), self.rx.feedback(), anchor, 0, TRAIN_WINDOW);
                        (a.unwrap_or(f64::NAN), b.unwrap_or(f64::NAN))
                    }
                };
                if std::env::var_os("V92_SCORE_TRACE").is_some() {
                    eprintln!("V92DIAG: same window, own anchor: in place {in_place:.3} dB, refit {refit:.3} dB");
                }
                self.diag_line(&format!(
                    "refit at {aligned}: fit {fit:.3} dB, on this window the taps in place read \
                     {in_place:.3} dB and the refit {refit:.3} dB (the fit in place read \
                     {baseline:.3} dB at the first TRN1u), so the refit leads it by {:.3} dB \
                     against a tolerance of {} dB",
                    refit - in_place,
                    refit_slack()
                ));
            }
            if self.diag.is_some() {
                let line = match search {
                    Some((aligned, fit)) => format!(
                        "refit search around {at}: aligned {aligned} (expected {from}), \
                         quality over {REFIT_QUALITY}, fit {fit:.3} dB, taken {found:?}, \
                         snr now {:.1} dB, was {baseline:.1}, slack {}",
                        self.rx.snr_db(),
                        refit_slack()
                    ),
                    None => format!(
                        "refit search around {at}: no alignment over {REFIT_QUALITY}, \
                         snr {:.1} dB, was {baseline:.1}, slack {}",
                        self.rx.snr_db(),
                        refit_slack()
                    ),
                };
                self.diag_line(&line);
            }
            match found {
                // And a fit that reads the line much worse than the one in place
                // is not a refit: the taps would be worse for the CPt than what
                // they replaced. "Much worse" is measured on one window, the one
                // about to be installed, against the sequence this end
                // generates, each tap set at its own whole-symbol anchor --
                // `docs/81`'s finding was that the two figures the old test
                // compared had been taken on two different TRN1u windows and so
                // were not the same measurement at all.
                Some((snr, Some((in_place, refit)))) if refit > in_place - refit_slack() => {
                    self.refit_from = None;
                    self.note(format!(
                        "V.92: refit at the new phase, {snr:.1} dB ({refit:.1} dB on the window, \
                         against {in_place:.1} dB for the taps it replaces)"
                    ));
                    // The reader and the CPt finder are left alone: a CPt is a
                    // whole frame with a CRC, so those that cross the refit are
                    // thrown out by it and the next repetition is read with the
                    // new taps.
                }
                _ => {
                    self.rx.restore(&before);
                    // Move the anchor on and try again, a couple of times: the
                    // far end may still be a little way from the reversal, and
                    // a trial that lands in the gap between our Jp' and its
                    // TRN1u has nothing to fit. Past that the first fit stands.
                    self.refit_tries += 1;
                    self.refit_from = (self.refit_tries < 3).then_some(from + REFIT_WINDOW as u64 / 2);
                }
            }
        }
        // 9.6.1.1.2's CPd is designed from the second TRN1u, taken far enough in
        // that the history still has a few thousand of its symbols.
        if self.cpd.is_none()
            && let Some(at) = self.design_at
            && self.now >= at
        {
            self.design_and_store();
            self.design_at = None;
        }
        // Phase 4, from TRN2d to data mode (9.6.1.1).
        if self.stage == Stage::Phase4 {
            self.phase4_out();
            self.fill_downstream();
        }
        // No E2u, or no candidate that reads as B1u, by the time the wait is
        // out. This end says so and fails the exchange; it does not condition
        // its receiver on its own clock, because B1u's frame interval 0 is the
        // far end's symbol and a boundary taken from a timer is a boundary this
        // end does not have. The start-up turns a failure into the retrain and
        // the V.90 fallback it already has (`v90::startup`), which is a visible
        // outcome; a data mode read from an invented boundary is a silent one
        // and delivered 14 kB of somebody else's bits on the arms measured.
        if self.b1u_pending
            && let Some(at) = self.b1u_deadline
            && self.now >= at
        {
            self.b1u_pending = false;
            self.b1u_deadline = None;
            self.fail("no E2u that reads as B1u within 2 s of Ed");
        }
        // D2/D3: once enough of TRN1u has gone by, the fit is made.
        if self.stage == Stage::Training
            && let Some(from) = self.trn_from
            && self.rx.symbols() >= from + TRAIN_AFTER
        {
            // The fit is indexed against the receiver's history, which is a
            // window that has long since slid past the reversal: `from` is an
            // absolute symbol count and has to be brought back to where the
            // window starts.
            let base = self.rx.symbols() - self.rx.history_len() as u64;
            let at = from.saturating_sub(base);
            // The fixed fractional delay line, if `V92_FE` asked for one, is
            // aimed here and installed here: once, on the TRN1u about to be
            // trained on, and then held. The phase it picks is `docs/78`'s eye
            // optimum measured on our own sequence, and the refit at the second
            // TRN1u re-fits the equaliser at that phase by the machinery that
            // already exists -- which is what makes the gain real rather than
            // absorbed: an equaliser is free to interpolate a fraction of a
            // symbol, and `docs/78` measured one whole symbol as worth 0.2 per
            // cent of residual, so installing a delay without re-fitting would
            // buy nothing.
            // Measured here because this is where the phase is observable, and
            // installed at TRN2u because that is where it can be *absorbed*.
            // Installing it here was tried and breaks the call: the fit window
            // [at, at + 1200] straddles the install, so the taps are fitted on
            // one phase while everything after is read at another, and Ja is
            // lost (measured: trained at 15.6 dB, then "reading Ja down - / up -"
            // and a fallback to V.90 phase 2). At TRN2u the fit that follows is
            // entirely on converted samples, which is what `docs/77` found to be
            // the only workable enable point.
            // `V92_FE_DELTA` overrides the aim, for separating "the mechanism
            // does not work" from "the criterion chooses badly" -- the two
            // questions this measurement has to keep apart.
            let forced = std::env::var("V92_FE_DELTA").ok().and_then(|t| t.parse::<f64>().ok());
            if self.rx.fe_enabled()
                && let Some((delta, residual)) = forced
                    .map(|d| (d, f64::NAN))
                    .or_else(|| self.rx.fe_aim(at as usize, TRAIN_WINDOW))
            {
                self.fe_pending = Some(delta);
                self.note(format!(
                    "V.92: fractional delay {delta:.3} symbols aimed (eye residual {residual:.4})"
                ));
            }
            if self.rx.train_near(at, TRAIN_SEARCH) {
                self.stage = Stage::ReadJa;
                self.note(format!("V.92: TRN1u trained at {:.1} dB", self.rx.snr_db()));
            } else {
                // A burst at the start of TRN1u must cost a retry, not the
                // call: move the fit on and try again (as the receiver's own
                // retries do).
                self.trn_from = Some(from + TRAIN_AFTER / 2);
            }
        }
        if let Some(until) = self.md_until
            && self.now >= until
        {
            self.md_until = None;
            self.stage = Stage::AwaitRu;
            self.rx.restart_hunt();
        }
        if let Some(at) = self.ja_deadline
            && self.now > at
            && matches!(self.stage, Stage::AwaitRu | Stage::WaitMd | Stage::Training | Stage::ReadJa)
        {
            self.ja_deadline = None;
            self.fail("no Ja within 4.5 s and a round trip");
        }
        if let Some(at) = self.su_deadline
            && self.now > at
            && self.stage == Stage::AwaitSu
            && self.source.out() == Out::Jd
        {
            self.su_deadline = None;
            self.fail("no Su within 5.1 s of TRN1d");
        }
    }

    fn fail(&mut self, why: &'static str) {
        self.note(format!("V.92: {why}"));
        self.status = Status::Failed(why);
    }

    fn heard(&mut self, heard: Heard) {
        if tracing() {
            if let Heard::Symbol(_) = heard {
            } else {
                eprintln!("V92TRACE: {heard:?} at {} stage {:?}", self.now, self.stage);
            }
        }
        match heard {
            // The first Ru is the one whose reversal opens the sequence; the
            // hunt reports it and the reversal separately (8.5.5).
            Heard::Ru { .. } => {}
            Heard::RuReversal { at } => self.ru_reversal(at),
            Heard::Su { at } => {
                if self.stage == Stage::AwaitSu {
                    self.note(format!("V.92: Su at {at}"));
                }
            }
            Heard::SuReversal { at } => self.su_reversal(at),
            Heard::Trained { snr_db } => {
                if self.stage == Stage::Training {
                    self.note(format!("V.92: TRN1u trained at {snr_db:.1} dB"));
                    self.stage = Stage::ReadJa;
                }
            }
            Heard::Symbol(symbol) => self.symbol(symbol),
            Heard::Lost | Heard::Found => {}
        }
    }

    fn ru_reversal(&mut self, at: u64) {
        match self.stage {
            Stage::AwaitRu => {
                if self.settings.far_md > 0 && !self.md_done {
                    // 9.5.1.1.1: wait MD (length x 276T, allowing for the 24T
                    // R-bar-u inside it), then hunt Ru again.
                    self.stage = Stage::WaitMd;
                    self.md_done = true;
                    self.md_until = Some(self.now + u64::from(self.settings.far_md) * MD_STEP);
                    self.rx.restart_hunt();
                    self.note("V.92: waiting out the MD INFO1a asked for".into());
                    return;
                }
                self.begin_training(at);
            }
            // Nothing else is a Ru reversal: after the first one the hunt is
            // restarted and only Su is of interest.
            _ => {}
        }
    }

    fn begin_training(&mut self, reversal: u64) {
        // TRN1u begins 24T after the reversal (9.5.1.1.2), and the hunt
        // reports where the reversal was rather than when it noticed.
        let from = reversal + TRN_START;
        self.trn_from = Some(from);
        self.stage = Stage::Training;
        // What comes back is Su: Sd is this end's own, and the analogue modem
        // has no other six-symbol sequence to send here (9.5.2.1.6).
        self.rx.expect_su();
        // Su ends in 9.5.2.1.7's 24.5T S-bar-u, which steps the far end's
        // transmitter half a symbol the way `epsilon` says it steps it.
        self.rx.expect_step(epsilon::step_direction() * 0.5);
        self.rx.restart_hunt();
        self.note(format!("V.92: TRN1u expected from symbol {from}"));
    }

    /// 9.6.1.1: Phase 4 from TRN2d to data mode.
    ///
    /// ```text
    /// 9.6.1.1.1  TRN2d >= 2040T, then SUVd repeatedly, receiver on SUVu
    /// 9.6.1.1.2  one CPd after the first SUVu; the ack bit after a CPu
    /// 9.6.1.1.4  Ed once an acknowledged sequence has been both sent and
    ///            received
    /// 9.6.1.1.5-6 B1d out; B1u in, then data mode
    /// ```
    fn phase4_out(&mut self) {
        // 9.6.1.1.1: TRN2d "for at least 2040T" and then the exchange.
        if self.exchange.is_none() {
            if !self.source.trn2d_done() {
                return;
            }
            self.begin_exchange();
        }
        let now = self.now;
        // A sequence that has run out is a sequence that has ended (9.6.1.1.2's
        // ack "changes at a boundary and nowhere else").
        if let Some(done) = self.source.take_finished() {
            let kind = match done {
                Out::Cpd => SequenceKind::Cp,
                Out::Ed => SequenceKind::E,
                _ => SequenceKind::Suv,
            };
            if let Some(exchange) = self.exchange.as_mut() {
                exchange.sequence_ended(kind, now);
            }
            if kind == SequenceKind::E && !matches!(self.status, Status::Connected { .. }) {
                self.begin_data();
            }
        }
        let Some(exchange) = self.exchange.as_mut() else { return };
        if exchange.sending().is_some() || self.source.out() == Out::B1d {
            return;
        }
        let kind = exchange.next_sequence(now);
        let ack = exchange.sequence_started(kind);
        match kind {
            SequenceKind::Suv => {
                let bits = Suvd { silence: false, ack }.to_bits(INTERVALS);
                self.source.send(Out::Suvd, bits);
                // 9.6.2.1.4: an acknowledged SUVd is one of the two things that
                // lets the far end send E2u, so the search opens here. See
                // `arm_e2u_search`.
                if ack {
                    self.arm_e2u_search();
                }
            }
            SequenceKind::Cp => {
                let Some(mut cpd) = self.cpd.clone() else {
                    // Nothing to declare yet: keep the SUVs going rather than
                    // send a CPd that says nothing (9.6.1.1.2 allows the wait).
                    let bits = Suvd { silence: false, ack }.to_bits(INTERVALS);
                    self.source.send(Out::Suvd, bits);
                    return;
                };
                cpd.ack = ack;
                let bits = cpd.to_bits(INTERVALS);
                // What the far end's detector will make of it: the sequence is
                // a 17-one sync, a start bit and sixteen bits per word, then a
                // start bit and sixteen CRC bits -- so the data bytes it counts
                // are `(bits - 17 - words) / 16 * 2`, and the length it computes
                // from the header must agree or it reads the wrong number of
                // them and the CRC cannot pass.
                let words = (bits.len().saturating_sub(35)) / 17;
                let data_bytes = words * 2;
                self.note(format!(
                    "V.92: CPd drn {}, ack {ack}, {} bits, {words} words, {data_bytes} data bytes",
                    cpd.drn,
                    bits.len()
                ));
                self.source.send(Out::Cpd, bits);
                // 9.6.2.1.4: a CPd with its acknowledgement bit set is the
                // other thing that lets the far end send E2u. See
                // `arm_e2u_search`.
                if cpd.ack {
                    self.arm_e2u_search();
                }
            }
            // 8.8.2: Ed is two data frames of scrambled zeros, at CPt's rate.
            SequenceKind::E => {
                self.source.send(Out::Ed, self.source.ed_bits());
                // Ed, and not only Ed: see `arm_e2u_search`.
                self.arm_e2u_search();
                self.note("V.92: Ed".into());
            }
            _ => {}
        }
    }

    /// The exchange's first day: TRN2u is on the line, so the upstream reader
    /// starts here, and the design behind CPd is taken from the TRN1u this end
    /// has just refitted on.
    fn begin_exchange(&mut self) {
        // Where the second TRN1u begins: exactly, if the refit's correlation
        // found it; otherwise by that same correlation over the timetable's own
        // estimate, because a least-squares design against a reference a dozen
        // symbols out is a design against noise.
        if self.trn2_from.is_none() {
            let base = self.rx.history_base();
            let estimate = self.trn2_estimate.or(self.refit_from).and_then(|at| at.checked_sub(base));
            let Some(estimate) = estimate else {
                self.note(format!(
                    "V.92: no estimate for the second TRN1u (now {}, base {base}, estimate {:?})",
                    self.now, self.trn2_estimate
                ));
                return;
            };
            let found = self.rx.align_trn1u(estimate, REFIT_SEARCH, ALIGN_WINDOW);
            match found {
                Some((aligned, quality)) if quality > REFIT_QUALITY => {
                    let at = base + aligned;
                    self.trn2_from = Some(at);
                    self.note(format!("V.92: second TRN1u at {at}, quality {quality:.2}"));
                }
                other => {
                    self.note(format!(
                        "V.92: second TRN1u not found near {estimate} (now {}, base {base}, {other:?})",
                        self.now
                    ));
                    return;
                }
            }
        }
        if self.cpd.is_none() {
            // The refit did not reach here with a design of its own (a line
            // whose second TRN1u the correlation would not take); try once more
            // from whatever the history still holds.
            self.design_and_store();
        }
        self.begin_upstream();
        let mut exchange = Exchange::new(Context::Training, self.settings.round_trip, FS);
        exchange.repeat_between_suvs(REPEAT_CP_GAP);
        self.exchange = Some(exchange);
        self.upstream_data = None;
        self.note("V.92: TRN2d done, SUVd out".into());
        self.apply_exchange();
    }

    /// Hand the source whatever the exchange has decided, once.
    fn apply_exchange(&mut self) {
        let now = self.now;
        let Some(exchange) = self.exchange.as_mut() else { return };
        let kind = exchange.next_sequence(now);
        let ack = exchange.sequence_started(kind);
        match kind {
            SequenceKind::Suv => {
                let bits = Suvd { silence: false, ack }.to_bits(INTERVALS);
                self.source.send(Out::Suvd, bits);
                if ack {
                    self.arm_e2u_search();
                }
            }
            _ => {}
        }
    }

    /// Put the upstream reader on the line from TRN2u's first symbol
    /// (8.7.6: its scrambler resets there and nowhere else until B1u).
    fn begin_upstream(&mut self) {
        if self.upstream.is_some() {
            return;
        }
        // 9.6.1.1.1 with 8.7.6: TRN2u is the first four-point signal since CPt,
        // so the fit designed on TRN1u goes in here, before a single symbol of
        // it is read. Without it this end reads TRN2u at the thirteen decibels
        // the two-point fit gives, which is the 5 to 13 per cent of wrong bits
        // the handover keeps quoting. `train_on_trn2u` refines it afterwards.
        let was = self.rx.snr_db();
        if self.rx.begin_phase_four() {
            self.note(format!("V.92: phase 4 receiver {was:.1} -> {:.1} dB", self.rx.snr_db()));
        }
        // 8.7.6's TRN2u begins at this symbol: the Phase 4 fit is designed on it
        // a little way in, and nothing is given to the finder until that fit is
        // in (see `upstream_drop_until`).
        let start = self.rx.symbols();
        self.trn2u_from = start;
        self.four_at = start + TRN2U_FIT_AFTER;
        self.upstream_drop_until = self.now + TRN2U_FIT_AFTER + TRN2U_DROP_AFTER_FIT;
        // Jp bit 48 chose four points or eight for TRN2u, and every sequence
        // between here and B1u rides the same modulation (8.7.2-8.7.6).
        let size = self.upstream_size;
        self.upstream = Some(Trn2uReader::new(size));
        self.up_finder = Some(UpFinder::new(trn2u_unit(size == Trn2uSize::Eight)));
        // The equaliser's decisions are what its feedback section subtracts, so
        // it has to know which constellation it is deciding: a two-point
        // decision held against a four-point symbol is a wrong symbol fed back
        // for the next eight, which is the one way a decision-feedback
        // equaliser makes a line worse rather than better.
        self.rx.set_slicer(if size == Trn2uSize::Eight { Slicer::EightPoint } else { Slicer::FourPoint });
        let (inner, outer, gain, seen) = self.rx.measured_levels();
        if seen > 0 {
            self.note(format!(
                "V.92: upstream four points measure {inner:.3}/{outer:.3}, gain {gain:.3} over {seen}"
            ));
        }
        self.note("V.92: upstream reader on TRN2u".into());
    }

    /// The parameters the analogue modem is to transmit with (9.6.1.1.2's CPd
    /// is "the whole of the 6.4 chain"), taken from the channel the second
    /// TRN1u measured.
    fn design_and_store(&mut self) {
        if self.cpd.is_some() {
            return;
        }
        let Some(from) = self.trn2_from else { return };
        match self.design_upstream(from) {
            Some(parameters) => {
                if tracing() {
                    let f = &parameters.filters;
                    eprintln!(
                        "V92TRACE: CPd drn {} gain {:.4} moduli {:?} sets {:?} filters z1 {} p1 {} z2 {} p2 {} limits {:?}/{:?}/{:?}",
                        parameters.drn,
                        parameters.gain,
                        parameters.moduli,
                        parameters.sets.iter().map(|s| s.len()).collect::<Vec<_>>(),
                        f.z1.len(),
                        f.p1.len(),
                        f.z2.len(),
                        f.p2.len(),
                        self.settings.limits.sections,
                        self.settings.limits.total,
                        self.settings.limits.most
                    );
                }
                if let Some(path) = std::env::var_os("V92_CPD_TRACE") {
                    // What this end actually told the analogue modem to run:
                    // the design, coefficient by coefficient, so that a
                    // captured line can be filtered with the same numbers.
                    let f = &parameters.filters;
                    let text = format!(
                        "gain {:.9}\nscale {:.9}\nmoduli {:?}\nsets {:?}\nz1 {:?}\np1 {:?}\nz2 {:?}\np2 {:?}\n",
                        parameters.gain,
                        self.upstream_scale,
                        parameters.moduli,
                        parameters.sets,
                        f.z1,
                        f.p1,
                        f.z2,
                        f.p2
                    );
                    let _ = std::fs::write(std::path::Path::new(&path), text);
                }
                self.cpd = Some(Cpd::from(&parameters));
                self.note(format!(
                    "V.92: upstream drn {}, {:.1} kbit/s",
                    parameters.drn,
                    f64::from(parameters.rate()) / 1000.0
                ));
                self.parameters = Some(parameters);
            }
            None => self.note("V.92: no upstream design, CPd would say nothing".into()),
        }
    }

    /// 8.8.3's prefilter, from whichever measurement of the upstream this call
    /// has: phase 2's L1 reading where it survived, the TRN1u fit otherwise,
    /// and the identity where neither makes a legal design.
    ///
    /// V92-63. The reading comes first because it is the only measurement here
    /// of the *line*. TRN1u is a two-point stream with two thirds of its power
    /// at 4000 Hz, where a loop has a null, so a fit through it says more about
    /// the null than about the channel; L1 is the analogue modem's own probing
    /// signal, flat across the band, and what this receiver makes of it is the
    /// channel. `V92_Z2=fit` puts the old design back in front for a bench
    /// comparison.
    ///
    /// What is logged here is deliberately loud: a bench call has to be able to
    /// audit the reading and the design that came out of it without gdb, so the
    /// reading goes in as one line of twenty-one tones and the design as its
    /// predicted level, its window, the tilt it took out and the flatness it
    /// reached, plus the same twenty-one tones with what the section does to
    /// each.
    fn upstream_design(
        &mut self,
        channel: &Channel,
        limits: &FilterLimits,
    ) -> (Design, Option<&'static str>) {
        if let Some(reading) = self.settings.probing.clone() {
            self.note(format!(
                "V.92: upstream design from the retained L1 reading: {}",
                reading.summary()
            ));
            if self.prefilter == Prefilter::Reading {
                match crate::v92::design::from_reading(&reading, limits) {
                    Ok((designed, predicted)) => {
                        self.note(format!("V.92: z2 from the L1 reading: {}", predicted.summary()));
                        self.note(format!(
                            "V.92: z2 per tone (H/snr/Z2/Z2H, dB): {}",
                            predicted.table()
                        ));
                        return (designed, None);
                    }
                    Err(why) => self.note(format!(
                        "V.92: no z2 from the L1 reading ({why}); designing from the TRN1u fit"
                    )),
                }
            }
        }
        match design(channel, limits) {
            Ok(designed) => (designed, None),
            Err(why) => {
                eprintln!("V92TRACE: no upstream design: {why}");
                (crate::v92::design::identity(channel), Some(why))
            }
        }
    }

    /// 6.4 from the channel the second TRN1u measured: the fastest rung of the
    /// upstream ladder this line carries, or the slowest it will take.
    fn design_upstream(&mut self, from: u64) -> Option<Parameters> {
        const FIT: usize = DESIGN_AFTER as usize;
        let known = reference(0, FIT, Mode::Answer)?;
        let received = self.rx.history_range(from, from + known.len() as u64);
        if received.len() < FIT / 2 {
            return None;
        }
        // The fit above is only worth what the sample instants are worth, and
        // they are not this end's to choose: the analogue modem's clock is its
        // own, so its symbols arrive a fraction of a symbol away from the
        // network's sample instants. A linear equaliser is at its worst at half
        // a symbol (`receiver`'s own table: 19 dB against 166 dB on the
        // instants), which is where an Agere SV92PP's TRN1u fits, and the
        // prefilter this end designs is the inverse of *what the line does* --
        // not of the line and this end's timing recovery together. Inverting the
        // second asks the far end to undo an interpolator it never ran, and what
        // arrives afterwards is ISI at the level spacing.
        let received = align_fractional(&known, &received);
        let Some(channel) = Channel::estimate(&known, &received, 48) else {
            eprintln!("V92TRACE: no channel estimate ({} symbols)", received.len());
            return None;
        };
        let limits = self.settings.limits;
        // 8.8.3 makes the precoder, the prefilter and the rate this end's job,
        // and CPd is how the far end is told what to run: an end that sends no
        // CPd has told it nothing, and the far end's SUVu acknowledgement stays
        // clear for ever (docs/27 section 0.5.5, docs/28 P2), so phase 4 never
        // starts and no B1u is ever transmitted. A design that cannot be made
        // legal is therefore answered with the identity -- which is legal in
        // every field of Table 30 and honest about what will happen to the far
        // end's symbols -- and the log says which and why. Silence is not one
        // of the answers.
        let (mut designed, mut degraded) = self.upstream_design(&channel, &limits);
        // Diagnostic: what the far end does with the filters in CPd, and what
        // happens when it is asked for none at all.
        //   none  -> z2 = [1]: pass-through, no filtering
        //   probe -> z2 = [0.8, 0.6]: two taps and unit energy, so 8.8.3's
        //            `mean square of (v G) = 1` still holds and the CPd that
        //            carries it is legal in every field -- but the far end's own
        //            output is then filtered by a section whose response is
        //            1.4 at DC and 0.2 at 4 kHz, which no far end can produce
        //            by accident and no receiver can mistake for the line.
        //            It answers one question and only one: whether the far end
        //            runs the prefilter it is sent. See docs/16 section 8f.3.
        //   a    -> z2 - delta ("x plus a correction")
        //   b    -> delta - z2 ("x minus the inverse")
        let mut sent_scale = designed.scale;
        match std::env::var("V92_PRE").ok().as_deref() {
            Some("none") => {
                designed.filters =
                    crate::v92::Filters { z1: Vec::new(), p1: Vec::new(), z2: vec![1.0], p2: Vec::new() };
                // The far end is told to run the section we sent, at the
                // amplitude we sent it at, so nothing was scaled: what the
                // design's own scale folds back is not in play. The log said
                // 12.586 here while the CPd said z2 = [1], which is a number
                // that reads as a receiver correction that is not happening.
                sent_scale = 1.0;
            }
            Some("probe") => {
                designed.filters = crate::v92::Filters {
                    z1: Vec::new(),
                    p1: Vec::new(),
                    z2: vec![0.8, 0.6],
                    p2: Vec::new(),
                };
                sent_scale = 1.0;
            }
            Some("a") => {
                if let Some(first) = designed.filters.z2.first_mut() {
                    *first -= 1.0;
                }
            }
            Some("b") => {
                designed.filters.z2.iter_mut().for_each(|value| *value = -*value);
                if let Some(first) = designed.filters.z2.first_mut() {
                    *first += 1.0;
                }
            }
            _ => {}
        }
        // The prefilter was scaled down to fit Q0.15; the receiver must fold
        // that scale back in when it reads B1u's levels. That is only true for
        // the design as it was made: `none` and `probe` replace it with a
        // section that is already on its final amplitude.
        self.upstream_scale = sent_scale;
        self.upstream_delay = designed.delay;
        // The rate this end asks for. The analogue modem's Ja says which rungs
        // it allows; this end takes the slowest of those, because the number
        // that decides how far up the ladder a line carries is the *noise*, and
        // the only figure for it available here is the two-point fit's -- which
        // on a telephone line is mostly the 4000 Hz the line does not have, not
        // noise. Asking for a rung the receiver cannot then demodulate would end
        // the call in Phase 4, which is exactly where V.92's last attempt this
        // far stopped.
        const SLOWEST: u32 = 1;
        // The bench's stress arm, and nothing else. `V92_UP_RATE=48000` pins
        // the rung naming that rate so that the *receive* path can be measured
        // at a spacing the design would otherwise refuse. Unset -- which is
        // every call a user makes -- the two lines below are the only ones that
        // run, and `ja` is the same `1` it has always been. An unparsable or
        // off-ladder rate is a bench mistake and leaves the default alone
        // rather than silently choosing something else.
        let pinned = std::env::var("V92_UP_RATE").ok().and_then(|rate| {
            let rate = rate.trim().parse::<u32>().ok()?;
            (1..=crate::v92::UP_RATES as u8).rev().find(|&drn| crate::v92::up_rate(drn) == rate)
        });
        let ja = match pinned {
            Some(drn) => 1u32 << (drn - 1),
            None => {
                let ja = self.ja.upstream_rates().unwrap_or(0) & SLOWEST;
                if ja == 0 { SLOWEST } else { ja }
            }
        };
        let trellis = Trellis::Sixteen;
        // The noise the design is told about, and the one number in this file
        // that is a reading of the line rather than of a fit. TRN1u is a
        // two-point stream with two thirds of its power at 4000 Hz, where a
        // telephone line has a null: the residual of a fit against it is mostly
        // that null and not noise, so it reads 13 dB on a line whose own SNR is
        // 40. Taking it at its word would say no rung of the ladder carries any
        // real line at all, which is what it said here first.
        //
        // So it is used as a *floor* on the assumed SNR and the rung is the
        // slowest the schedule allows (`ja`), searched down from there: a slower
        // rung than the line carries costs bandwidth, a faster one costs the
        // call, and there is no measurement here that would justify the risk of
        // the second.
        // The rung search at one assumed noise, with 6 dB taken off the noise
        // each time the ladder refuses: the fit's own noise is an estimate of
        // the line at 4000 Hz, where a loop has a null, so it is a floor and
        // not a reading (see above).
        let snr = self.rx.snr_db();
        let rung = |designed: &crate::v92::design::Design| {
            let mut noise = 10f64.powf(-snr / 10.0);
            for _ in 0..9 {
                if let Some(found) = choose(designed, noise, ja, trellis) {
                    return Some((found, noise));
                }
                noise /= 4.0;
            }
            // A pinned rung that 6.4.2's spacing judgement refused. Table 30 and
            // the class rules are still asked; the spacing question is answered
            // in the log instead of by refusing, because the arm exists to
            // measure what that judgement was protecting.
            let drn = pinned?;
            let noise = 10f64.powf(-snr / 10.0);
            let found = pin(designed, drn, trellis, noise)?;
            let room = if found.sigma > 0.0 { 3f64.sqrt() / (4.5 * found.sigma) } else { f64::INFINITY };
            eprintln!(
                "V92TRACE: upstream rate pinned to drn {drn} ({} bit/s): design sigma {:.2e}, the spacing rule allows LC <= {room:.0}, this set has LC {} (N {})",
                found.parameters.rate(),
                found.sigma,
                found.parameters.sets[0].len(),
                2 * found.parameters.sets[0].len()
            );
            Some((found, noise))
        };
        let mut found = rung(&designed);
        if found.is_none() && degraded.is_none() {
            // Nothing the rate ladder will carry comes out of this design, so
            // the far end would be told nothing at all. Ask it to run no
            // filters instead: the line's ISI is then this receiver's to undo,
            // and phase 4 at least begins. The rung does not move -- `ja` below
            // is the slowest the analogue modem allows either way.
            let residual = designed.residual_db();
            degraded = Some("no rung of the 6.1 ladder carries what the design leaves");
            designed = crate::v92::design::identity(&channel);
            // No filter is being sent, so nothing was scaled: the far end's
            // output is on the levels' own scale.
            self.upstream_scale = 1.0;
            self.upstream_delay = designed.delay;
            eprintln!(
                "V92TRACE: identity fallback: residual {residual:.1} dB, fit noise {:.2e}, ja {ja:#x}",
                channel.noise
            );
            found = rung(&designed);
        }
        if let Some(reason) = degraded {
            self.note(format!("V.92: upstream design degraded to identity ({reason})"));
        }
        let Some((mut choice, noise)) = found else {
            eprintln!(
                "V92TRACE: no upstream rung carries this line (fit noise {:.2e}, residual {:.1} dB, ja {ja:#x})",
                channel.noise,
                designed.residual_db()
            );
            return None;
        };
        // Diagnostic: force a four-point constellation, so that what the far
        // end puts on the line can be counted in levels. A CPd it obeys and a
        // CPd it ignores look nothing alike.
        if std::env::var_os("V92_TINY").is_some() {
            choice.parameters.moduli = [4u8; UP_INTERVALS];
            choice.parameters.sets = vec![vec![1, 3, 5, 7]];
        }
        if tracing() {
            eprintln!(
                "V92TRACE: upstream design assumed noise {noise:.2e}, scale {:.2}, residual {:.1} dB",
                designed.scale,
                designed.residual_db()
            );
        }
        Some(choice.parameters.clone())
    }

    /// 9.6.1.1.5-6: B1d out, and the receiver conditioned for B1u.
    ///
    /// The downstream's mapping comes from the analogue modem's CPu, which is
    /// the same table V.90's data mode reads (V.92 8.8.1 points at V.90 8.6.1),
    /// and the upstream's decoder from this end's own CPd.
    fn begin_data(&mut self) {
        self.note("V.92: Ed out".into());
        let upstream = self.parameters.as_ref().map_or(0, Parameters::rate);
        // The receiver is *not* reconditioned here. 9.6.1.1.6 has it done
        // "after receiving E2u", and that is not pedantry: E2u is a data frame
        // of TRN2u's four points (8.7.2), so it still needs the equaliser
        // TRN2u trained and the four-point slicer that feeds its feedback,
        // and B1u's twelve-interval frame begins on the *next* symbol, which
        // is the only thing that says where frame interval 0 and the first
        // trellis frame are. Conditioning here instead read E2u with the data
        // mode's constellation, and started B1u's frame count at a symbol
        // some hundreds early -- a whole number of symbols the far end never
        // agreed to.
        // A boundary this end already has is not one Ed going out can take
        // away: the wait is for a boundary that has not been found, and
        // `begin_b1u` has a decoder running on one. Re-arming the wait here
        // would time out a call that has its B1u and fail it in the open two
        // seconds later.
        let waiting = self.parameters.is_some() && self.upstream_data.is_none();
        self.b1u_pending = waiting;
        self.b1u_deadline = waiting.then_some(self.now + (B1U_WAIT * FS) as u64);
        let mut downstream = 0;
        if let Some(cpu) = self.cpu.as_ref()
            && let Some(mapping) = crate::v90::encoder::Mapping::from_cp(cpu)
        {
            // What B1d goes out on: CPu's rate, its six intervals' sets and
            // the sign bits it spends on shaping.
            if tracing() {
                eprintln!(
                    "V92TRACE: B1d mapping k {} s {} ld {} shaping {:?} sets {:?}",
                    mapping.k,
                    mapping.redundancy.data_bits(),
                    mapping.lookahead,
                    mapping.shaping,
                    mapping
                        .sets
                        .iter()
                        .map(|set| (set.points().len(), set.points().first().copied(), set.points().last().copied()))
                        .collect::<Vec<_>>()
                );
            }
            downstream = mapping.rate();
            self.downstream = Some(crate::v90::encoder::Encoder::new(mapping, self.settings.law));
            // 8.6.1: the data-mode scrambler starts afresh at B1d, and B1d is
            // 48 data frames of scrambled ones before the data starts.
            self.downstream_scrambler = Scrambler::new(Mode::Call);
            self.b1d_left = B1D_FRAMES;
            self.note(format!("V.92: B1d at {downstream}"));
        } else {
            self.note("V.92: no CPu yet, B1d waits".into());
        }
        // Data mode is on the line now (9.6.1.1.5-6): the stack above starts
        // handing DTE bits down and taking recovered bits up from here.
        self.status = Status::Connected { downstream, upstream };
        self.stage = Stage::Phase4;
        self.source.change(Out::B1d);
        self.fill_downstream();
    }

    /// Start listening for E2u (9.6.2.1.4).
    ///
    /// The analogue modem transmits E2u "after [it] has sent a CPu or SUVu
    /// sequence with the acknowledgement bit set and it has received a CPd or
    /// SUVd sequence with the acknowledgement bit set or Ed" -- so Ed is one way
    /// in, not the way in, and a far end that keys on the CPd it received sends
    /// E2u while this end is still finishing its own SUVd/CPd exchange. Measured
    /// on `/tmp/hsf-v90-interop/20261024-063632-a41c-off-d1`: the far end's E2u
    /// ends at symbol 108 733, this end's last CPd goes out at 18.63 s and its
    /// Ed at 19.21 s, so the whole of the real E2u falls in the stretch an
    /// Ed-only arm is deaf to. The cost of arming early is a candidate opened on
    /// a false zero run inside the exchange, and there is a gate for exactly
    /// that: the candidate has to read as B1u (see [`Self::score_b1u`]) or it is
    /// handed back by [`Self::abandon_b1u`] and the hunt goes on.
    ///
    /// Nothing is armed before `parameters` exists: `begin_b1u` needs CPd's
    /// constellation and would return without building a decoder, which would
    /// leave the search disarmed with no boundary at all.
    fn arm_e2u_search(&mut self) {
        if self.parameters.is_none() || self.upstream_data.is_some() {
            return;
        }
        // Only a *new* arm restarts the count: this is called on every SUVd
        // and CPd this end sends, and a zero run being counted across one of
        // them is E2u, not a reason to start again. (Measured on
        // `/tmp/p0corr/ident.raw`: without this, this end's own SUVd at
        // symbol 111 429 reset a thirty-six-bit run two symbols before it
        // ended, and the E2u at 111 432 was never seen at all.)
        if !self.awaiting_e2u {
            self.e2u_zeroes = 0;
        }
        self.awaiting_e2u = true;
    }

    /// 9.6.1.1.6: condition the receiver for B1u, on the symbol E2u ended
    /// before.
    ///
    /// Three things happen together and are all about the same symbol: the
    /// slicer moves to CPd's data mode constellation, the equaliser becomes
    /// the memoryless scale the far end's own precoder and prefilter left the
    /// channel at (6.4.2 -- they equalise at the transmitter, so a decision
    /// feedback section here would equalise a second time), and the decoder
    /// starts a data frame at what is now symbol 0.
    fn begin_b1u(&mut self) {
        let Some(parameters) = self.parameters.clone() else { return };
        let mut levels: Vec<f64> = Vec::new();
        for set in &parameters.sets {
            for &point in set {
                let level = parameters.gain * f64::from(point);
                levels.push(level);
                levels.push(-level);
            }
        }
        levels.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        levels.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        // The slicer moves to CPd's constellation and the equaliser stays: the
        // prefilter the far end runs is this end's design of it, and on a line
        // with insertion loss it is never exact.
        self.rx.set_data_mode(levels);
        self.upstream_data = Some(Decoder::new(&parameters));
        // Where B1u's frame interval 0 is, in symbols after the one E2u's zero
        // run was found to have ended on.
        //
        // Zero, and the zero is a measurement rather than a derivation: the
        // detector fires on the symbol carrying the first non-zero bit after
        // the run (see `symbol_upstream`), and 8.7.1 puts B1u's frame interval
        // 0 on the symbol after E2u's last zero -- the same symbol. Measured
        // both ways up on the two captures that matter here: on HSF
        // `20261024-063632-a41c-off-d1` the run ends on symbol 108 733, symbol
        // 108 734 carries the first non-zero bit *and* is the first symbol of
        // the data mode's constellation (its level is 0.33 from the nearest of
        // TRN2u's four points, where the run's symbols are 0.01 from them); on
        // `/tmp/p0corr/ident.raw` the same two numbers are 111 431 and 111 432.
        // It was 1 while the detector fired six symbols inside the run, which
        // is where the "one symbol after the completion" reading came from.
        const B1U_ORIGIN_AFTER_E2U: usize = 0;
        // Data frames of B1u the decoder is not given, while the equaliser
        // settles onto the data-mode constellation.
        //
        // B1u's first symbols decode wrong and the stretch is anchored to the
        // line, not to the frame index: shifting the origin one data frame
        // later moves the damage with it, one frame for one frame, so the
        // receiver needs a fixed number of symbols before its decisions are
        // the constellation's. Measured on a capture of an Agere SV92PP: the
        // first seventeen of B1u's forty-eight data frames carry ~50% wrong
        // bits and everything after them is exact, and dropping sixteen to
        // eighteen of them leaves one frame with a few wrong bits and the rest
        // clean.
        //
        // Dropping them costs nothing, and that is what makes this the right
        // place to do it rather than a workaround: 8.7.1 defines B1u as "48
        // data frames of scrambled binary ones", so those symbols carry no
        // information at all -- they exist to let the far end's receiver
        // settle, and ours is entitled to the same. The count is in whole data
        // frames because the twelve-symbol frame phase and the four-symbol
        // trellis phase must survive the drop: a skip of anything but a
        // multiple of twelve desynchronises the trellis frame and destroys the
        // whole stream (measured: ~50% of every bit, at every such skip).
        //
        // The equaliser is not gated: `symbol_upstream` withholds symbols from
        // `upstream_data` and the receiver behind it keeps running, which is
        // the same rule `upstream_drop_until` follows for TRN2u.
        const B1U_ACQUIRE_FRAMES: usize = 18;
        // The design's own delay is *not* taken off here, and that is a
        // measurement rather than a preference.
        //
        // [`Design::delay`](crate::v92::design::Design::delay) is the position
        // of the response's main tap: a symmetric seventeen-tap z2 puts it at
        // eight, so the delay is real, and it was added here on the reasoning
        // that E2u goes out before the prefilter and B1u after it. That is not
        // so: the prefilter takes effect at CPd (8.8.3) and delays the E
        // exchange by exactly what it delays B1u, so the completion symbol this
        // origin is measured from already carries the delay -- and adding it
        // again rotates the twelve-symbol data frame and the four-symbol
        // trellis phase by `delay` mod 12.
        //
        // Measured on `/tmp/p0corr/ident.raw` with eceb3c2's z2 design, whose
        // delay is 8: with the term, `longest_ones` 109, 0 bytes, B1u 393 ones
        // of 540; without it, 7128, 2063 bytes, B1u 568 of 576. Eight is not a
        // multiple of twelve, so this is not a rounding question -- one of the
        // two placements is simply wrong, and the line says which.
        //
        // `V92_B1U_SKIP` still forces the whole number, so a wrong origin is
        // measured rather than argued about.
        let _ = self.upstream_delay;
        // The symbols between the boundary symbol and the first one the
        // decoder is fed: `[Self::symbol_upstream]` drops `b1u_skip` symbols
        // after the one `begin_b1u` ran on and feeds the decoder from the next,
        // so the first symbol fed is `b1u_skip + 1` after the boundary. The `-1`
        // is that one: without it the acquisition skip is 217 symbols, which is
        // one more than eighteen data frames, and the frame phase would be a
        // symbol out. (It went unnoticed because the arm every B1u number was
        // ever taken on -- an Agere SV92PP -- idles on ones, and a stream of the
        // same thirty-six bits every frame reads the same at any phase.)
        self.b1u_skip = std::env::var("V92_B1U_SKIP")
            .ok()
            .and_then(|text| text.parse().ok())
            .unwrap_or(
                B1U_ORIGIN_AFTER_E2U + B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1,
            );
        if std::env::var_os("V92_RX_BITS").is_some() || tracing() {
            // Printed after the assignment, not before: this line said
            // `b1u_skip 0` on every call for as long as it existed, because it
            // was written above the line that sets the field.
            eprintln!(
                "V92B1U begin at now={} gain={} sets={} upstream_delay={} origin={} acquire={} b1u_skip={}",
                self.now,
                parameters.gain,
                parameters.sets.len(),
                self.upstream_delay,
                B1U_ORIGIN_AFTER_E2U,
                B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS,
                self.b1u_skip
            );
        }
        // `b1u_pending` and the deadline stay armed: a candidate that does not
        // read as B1u (see `decide_b1u_polarity`) hands the hunt back, and the
        // deadline is what ends the exchange if none of them does.
        // Both ways up, until B1u says which one it is: 8.7.1 makes those 48
        // data frames known, so the polarity that reads them as ones is the
        // line's and the other is an equaliser that TRN2u's own seed left
        // inverted. The window is the rest of B1u, and its bits are B1u's --
        // which carry nothing -- so both decoders' output is dropped while it
        // runs.
        self.b1u_probe = Some(Decoder::new(&parameters));
        // `V92_ROT`: the class rotation is read off B1u rather than taken from
        // `PAIR_FIRST_INDEX_OFFSET`, and it is read off the part of B1u this
        // decoder is actually fed -- the symbol after the acquisition skip,
        // counted from B1u's own first symbol. Off by default.
        //
        // Only the decoder reading the line as received is given it. The
        // polarity measurement above compares the two decoders' ones counts,
        // and it can only do that if they differ by polarity and nothing else:
        // given a measured rotation, the negated copy of an Agere reads 529
        // ones of 576 instead of 450, and the lead the verdict needs is gone
        // (`docs/48 §5`). A call whose polarity turns out inverted therefore
        // keeps the constant -- which is the behaviour it had.
        if let Some(decoder) = self.upstream_data.as_mut() {
            decoder.measure_rotation_from(self.b1u_skip + 1);
            // Above 24 000 B1u's frames are read against 8.7.1's known content
            // instead of against 6.4.1's chain, which is not the far end's
            // there (`docs/74`). It stops with B1u: nothing after it is known,
            // and the clause's chain is the only thing that can read it.
            decoder.read_by_content(true);
        }
        self.b1u_ones = [0; 2];
        self.b1u_scored = [0; 2];
        self.b1u_scoring = (B1U_FRAMES * crate::v92::UP_INTERVALS).saturating_sub(self.b1u_skip);
        self.b1u_polarity = 1.0;
        self.note(format!(
            "V.92: B1u from symbol {}, {} bit/s, scale {:.3}, taps {:?}",
            self.now,
            parameters.rate(),
            self.upstream_scale,
            self.rx.taps()
        ));
    }

    /// Keep B1d's codeword queue fed from the downstream encoder (V.90 8.6.1).
    fn fill_downstream(&mut self) {
        if !self.source.needs_codes() {
            return;
        }
        let law = self.settings.law;
        let mut octets = Vec::new();
        if let Some(encoder) = self.downstream.as_mut() {
            for _ in 0..B1D_FILL_FRAMES {
                // 8.6.1/V.90, which 8.8.1/V.92 adopts whole: "B1d consists of
                // 48 data frames of scrambled ones transmitted at the end of
                // start-up using the selected data mode constellation
                // parameters ... The symbols in the first data frame of B1d
                // shall have the magnitudes resulting from mapping the first
                // D scrambled ones after initializing the scrambler to zero".
                // The far end reads exactly that many symbols before any of
                // them is data: an Agere SV92PP's data mode decoder is
                // initialised with a 288-symbol (48 x 6) countdown and drops
                // everything it decodes until it has run out. Sending the
                // terminal's bits there loses the first 288 symbols of the
                // call's data and moves every bit after them, which is a
                // greeting that never arrives and a V.42 that never opens.
                let b1d = self.b1d_left > 0;
                if b1d {
                    self.b1d_left -= 1;
                }
                // After B1d the DTE's bits, and the idle pattern (ones) when
                // there are none: V.90's data mode idles on ones, and the far
                // end's scrambling chain needs a stream whether or not the
                // terminal is typing.
                let frame = encoder.next_frame(|| {
                    let bit = if b1d { true } else { self.outbound.pop_front().unwrap_or(true) };
                    // What goes into the scrambler, which is the stream V.42 owns:
                    // 0x7E flags, the ADP, then the terminal's bits.
                    if let Some(file) = tx_trace_file()
                        && let Ok(mut file) = file.lock()
                    {
                        use std::io::Write as _;
                        let _ = write!(file, "{}", u8::from(bit));
                        let _ = file.flush();
                    }
                    self.downstream_scrambler.scramble(bit)
                });
                octets.extend(frame.octets(law));
            }
        }
        if !octets.is_empty() {
            self.source.send_codes(octets);
        }
    }

    /// One upstream symbol: through the reader, then into whichever sequence
    /// finder is listening.
    fn symbol_upstream(&mut self, symbol: Symbol) {
        let level = symbol.value;
        // 9.6.1.1.6: once E2u has ended, the receiver is B1u's -- the data mode
        // modulation rather than four points -- and every symbol from there is
        // a data frame's worth of the decoder's.
        if self.upstream_data.is_some() {
            // The acquisition window first, then the polarity measurement, and
            // only then the decoder that will carry the call: both windows are
            // B1u's, and both are dropped.
            //
            // A candidate costs 576 symbols to confirm, so the symbol B1u
            // really begins on can fall inside a candidate's window: measured
            // on `/tmp/p0corr/g712.raw`, the detector fires at 115366 and B1u
            // begins at 115510, a hundred and forty-four symbols later. Keeping
            // the hunt alive through the window to catch it was tried and is
            // *not* the answer: on both proven-V.92 Agere captures
            // (`012503-mask3`, `012713-mask3c`) the data-mode bit stream throws
            // spurious zero runs at 116102 and 120594, each of which restarted
            // the probe and cost the call its connection -- 7128 and 2063 bytes
            // became nothing. A candidate therefore owns the window it opened,
            // and a boundary it swallowed is one this end fails on rather than
            // guesses at.
            if self.b1u_skip > 0 {
                self.b1u_skip -= 1;
                return;
            }
            if self.b1u_scoring > 0 {
                self.score_b1u(symbol);
                return;
            }
        }
        // The polarity the measurement above found, then whatever diagnostic
        // transform `V92_B1U_ADJ` asks for (see `adjust_b1u_level`). Done
        // before the decoder is borrowed, because the transform keeps state.
        if let Some(file) = b1u_levels_file()
            && let Ok(mut file) = file.lock()
        {
            use std::io::Write;
            let _ = writeln!(file, "{} {level:.9} data", self.now);
        }
        write_b1u_line(&symbol, self.now, "data");
        let level = self.adjust_b1u_level(self.b1u_polarity * level);
        if let Some(decoder) = self.upstream_data.as_mut() {
            decoder.feed(level);
            let bits = decoder.take_bits();
            if let Some(file) = b1u_trace_file()
                && let Ok(mut file) = file.lock()
            {
                use std::io::Write;
                let text: String = bits.iter().map(|b| if *b { '1' } else { '0' }).collect();
                let _ = writeln!(file, "{} {level:.6} {} {text}", self.now, bits.len());
            }
            if let Some(file) = rx_bits_file()
                && let Ok(mut file) = file.lock()
            {
                use std::io::Write;
                let text: String = bits.iter().map(|b| if *b { '1' } else { '0' }).collect();
                let _ = writeln!(file, "{} {} {}", self.now, bits.len(), text);
            }
            if !bits.is_empty() {
                self.upstream_bits += bits.len() as u64;
                self.recovered.extend(bits);
            }
            return;
        }
        let Some(reader) = self.upstream.as_mut() else { return };
        {
            use std::sync::atomic::{AtomicU64, Ordering};
            static N: AtomicU64 = AtomicU64::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed);
            if std::env::var_os("V92_RX_BITS").is_some() && matches!(n, 0 | 4000 | 40000 | 400000) {
                eprintln!("V92B1U no data decoder yet: symbol {n} at now={}", self.now);
            }
        }
        let bits: Vec<bool> = reader.feed(level).into_iter().collect();
        let dropped = self.now < self.upstream_drop_until;
        // Set when this symbol was E2u's last: B1u's frame interval 0 is the
        // one after it.
        let mut started = false;
        if let Some(file) = up_trace_file() {
            use std::io::Write;
            let mut out = String::with_capacity(bits.len() + 12);
            for bit in &bits {
                out.push(if *bit { '1' } else { '0' });
            }
            if let Ok(mut file) = file.lock() {
                let _ = writeln!(file, "{} {level:.6} {}", self.now, out);
            }
        }
        for bit in bits {
            if dropped {
                self.upstream_dropped += 1;
                continue;
            }
            if let Some(finder) = self.up_finder.as_mut()
                && let Some(family) = finder.feed(bit)
            {
                self.up_sequence(family);
            }
            // 8.7.2: E2u is a data frame of scrambled *zeros*, and a data
            // frame in the upstream direction is twelve *symbols* (6.4, 8.7.1)
            // -- so a frame's worth is twelve times the bits a symbol carries,
            // twenty-four on TRN2u's four points and thirty-six on its eight,
            // and not the twelve *bits* this used to count. Twelve bits is six
            // symbols, which put the boundary six symbols inside the far end's
            // own E2u before the arming was fixed.
            //
            // The boundary is where the run *ends*, not where a count in the
            // middle of it is reached: 8.7.1 puts B1u's frame interval 0 on the
            // symbol after the one carrying E2u's last zero, and a far end's
            // E2u need not be exactly one frame long. Measured on the HSF
            // capture `20261024-063632-a41c-off-d1` the run is symbols
            // 108 711-108 733 -- twenty-three symbols, against the clause's
            // twelve, and the far end's data mode starts on 108 734 -- so the
            // run's end is exact and anything counted from its start is not.
            // The length is therefore only a floor, which is what keeps the
            // short zero stretches an SUVu or a CPu carries from opening a
            // candidate: on that capture the runs long enough to open one are
            // 106 410, 106 483, 106 550 and 108 734, and only the last is E2u.
            //
            // 8.7.2's "extended by a single symbol if bit 29 of CPd is set"
            // needs no term of its own here, for the same reason: the extension
            // lengthens the run and the run's end is what is read.
            if self.awaiting_e2u {
                if bit {
                    let e2u = (UP_INTERVALS * self.upstream_size.bits()) as u32;
                    if self.e2u_zeroes >= e2u {
                        // This symbol carries the first non-zero bit after a
                        // frame's worth of zeros, so it is the symbol after the
                        // one E2u's last zero was on: B1u's frame interval 0
                        // (8.7.1), and what the twelve-symbol frame and the
                        // four-symbol trellis frame are counted from.
                        let now = self.now;
                        // Diagnostic (`V92_E2U_FROM`): the symbol a zero run
                        // has to end at or after to count as E2u. The question
                        // "would this arm work if the boundary were the right
                        // one" cannot be answered by a run that starts the
                        // receiver a hundred millimetres of line early, so the
                        // knob makes the boundary a given and leaves the
                        // equaliser as the only variable.
                        if now < e2u_from() {
                            self.e2u_zeroes = 0;
                            continue;
                        }
                        if let Some(exchange) = self.exchange.as_mut() {
                            exchange.peer_e(now);
                        }
                        self.awaiting_e2u = false;
                        self.note("V.92: E2u".into());
                        started = true;
                    } else {
                        // Too short to be E2u: not a boundary, and the next run
                        // counts from zero.
                        self.e2u_zeroes = 0;
                    }
                } else {
                    self.e2u_zeroes = self.e2u_zeroes.saturating_add(1);
                }
            }
        }
        // The symbol after E2u's last one is B1u's frame interval 0 (8.7.1), so
        // the decoder is built now and fed from here on.
        if started {
            self.begin_b1u();
        }
    }

    /// `V92_B1U_ADJ`: what the data-mode level stream needs before it is a
    /// constellation at all. Empty is the only behaviour a call uses; `flip`
    /// negates, `dc` subtracts a running mean, and `shift:N` subtracts N lattice
    /// steps (2G, of the CPd gain).
    ///
    /// It exists because the residual a broken data mode sees is not always the
    /// residual a metric sees: a level stream offset by half a lattice step
    /// slices *better* than a correct one and decides wrong about half the
    /// time, so "how much of the failure is a constant offset" has to be a
    /// switch and not an argument. Measured on `/tmp/p0corr/g712.raw` at the
    /// true B1u origin, neither `dc` nor `shift:0.5` changes B1u's reading
    /// (420 ones of 576 either way), which says the residual there is not a
    /// constant offset.
    fn adjust_b1u_level(&mut self, level: f64) -> f64 {
        let steps = 2.0 * self.parameters.as_ref().map_or(1.0, |p| p.gain);
        match self.b1u_adjust.as_str() {
            "flip" => -level,
            "dc" => {
                let mean = self.b1u_history.iter().sum::<f64>() / self.b1u_history.len().max(1) as f64;
                let out = level - mean;
                if self.b1u_history.len() >= 512 {
                    self.b1u_history.pop_front();
                }
                self.b1u_history.push_back(level);
                out
            }
            other if other.starts_with("shift:") => {
                let count: f64 = other["shift:".len()..].parse().unwrap_or(0.0);
                level - count * steps
            }
            _ => level,
        }
    }

    /// B1u through both polarities' decoders at once, so that the call can say
    /// which way up the data mode is instead of inheriting it from TRN2u.
    ///
    /// Why this is needed at all: every sequence before B1u is antipodal or
    /// differentially encoded, so a line read upside down decodes them
    /// identically -- Ru, Su, TRN1u and the exchange all came through correct
    /// on the calls this repairs. TRN2u is the first constellation the polarity
    /// could be read from, and it is the one place it cannot be trusted: its
    /// differential sign encoder starts from a seed the exchange left behind,
    /// not from a reset (8.7.6 resets the *scrambler* at E1u, not the sign
    /// chain), so on an Agere SV92PP it arrives either way up from call to call
    /// (measured: one call's TRN2u is the other's, sample for sample, negated,
    /// while the two calls' B1u is identical to 1e-7). The Phase 4 fit pins the
    /// data-mode equaliser to whichever polarity TRN2u arrived with, at exactly
    /// the same residual either way -- 33.7 dB measured on both -- and a data
    /// mode read through the inverted one is on the right lattice and carries
    /// the wrong bits.
    ///
    /// B1u is the first thing on the line whose polarity is the data mode's
    /// own: 8.7.1 zeroes the scrambler, the differential encoder, the precoder
    /// and the prefilter at its first symbol and sends 48 data frames of ones.
    /// So the polarity that reads those frames as ones is the line's, and it is
    /// measurable by decoding them both ways up -- which is what this does. The
    /// bits are B1u's, which carry no information, so both are dropped.
    fn score_b1u(&mut self, symbol: Symbol) {
        let level = symbol.value;
        if let Some(file) = b1u_levels_file()
            && let Ok(mut file) = file.lock()
        {
            use std::io::Write;
            let _ = writeln!(file, "{} {level:.9} score", self.now);
        }
        write_b1u_line(&symbol, self.now, "score");
        // The same transform the data path takes (`V92_B1U_ADJ`), so that a
        // diagnostic asked for here is measured on the reading the criterion
        // uses and not on a reading nothing else sees. Empty is the default and
        // does nothing.
        let level = self.adjust_b1u_level(level);
        for (which, sign) in [1.0, -1.0].into_iter().enumerate() {
            let decoder = if which == 0 {
                self.upstream_data.as_mut()
            } else {
                self.b1u_probe.as_mut()
            };
            let Some(decoder) = decoder else { continue };
            decoder.feed(sign * level);
            let bits = decoder.take_bits();
            self.b1u_ones[which] += bits.iter().filter(|bit| **bit).count() as u64;
            self.b1u_scored[which] += bits.len() as u64;
            if let Some(file) = b1u_bits_file()
                && let Ok(mut file) = file.lock()
            {
                use std::io::Write;
                let text: String = bits.iter().map(|b| if *b { '1' } else { '0' }).collect();
                let _ = writeln!(file, "{} {} {text}", self.now, if which == 0 { 1 } else { -1 });
            }
        }
        self.b1u_scoring -= 1;
        if self.b1u_scoring == 0 {
            self.decide_b1u_polarity();
        }
    }

    /// Which way up B1u came, from the two readings [`Self::score_b1u`] took.
    ///
    /// The margin is not a taste: B1u's data is *all* ones, so the right
    /// polarity reads every bit as one and the wrong one reads a fixed wrong
    /// pattern in its place (measured on an Agere: 30 of every 36 bits, and no
    /// frame of all ones). An eighth of the window is far outside both, and a
    /// window where the two readings are that close is one where neither was
    /// reading B1u -- so it keeps what the receiver had rather than inventing a
    /// polarity out of noise.
    fn decide_b1u_polarity(&mut self) {
        let scored = self.b1u_scored[0].min(self.b1u_scored[1]);
        let (straight, flipped) = (self.b1u_ones[0], self.b1u_ones[1]);
        // The polarity verdict is unchanged; what is new is that it is also the
        // boundary's verdict. See [`B1U_CONFIRM_EIGHTHS`] for why the ratio and
        // the lead are both needed, and for the arm that shows one is not
        // enough.
        let inverted = std::env::var("V92_B1U_POLARITY").ok().as_deref() != Some("off")
            && scored > 0
            && flipped > straight + scored / B1U_POLARITY_LEAD;
        let (best, other) = (straight.max(flipped), straight.min(flipped));
        // `V92_B1U_GATE=off` accepts every candidate, which is what measuring
        // the data mode below a boundary takes: the criterion lives above the
        // thing being measured, so a run that cannot pass it still has to be
        // able to say what it reads.
        let reads_as_b1u = std::env::var("V92_B1U_GATE").ok().as_deref() == Some("off")
            || (scored > 0
                && best * 8 >= scored * B1U_CONFIRM_EIGHTHS
                && best >= other + scored / B1U_POLARITY_LEAD);
        if !reads_as_b1u {
            // Not a boundary. This is the whole point of the criterion: a
            // receiver that goes on to read data from a place it has just
            // failed to read B1u from does not need the far end's agreement, it
            // needs a boundary it actually has.
            self.abandon_b1u(best, other, scored);
            return;
        }
        if inverted {
            // The equaliser that was reading *this* polarity all along, not the
            // one that read the other and would have a window of the wrong sign
            // in its Viterbi's pipeline: its first forty trellis frames would be
            // decided from a mixture.
            std::mem::swap(&mut self.upstream_data, &mut self.b1u_probe);
            self.b1u_polarity = -1.0;
        }
        if let Some(decoder) = self.upstream_data.as_mut() {
            decoder.read_by_content(false);
        }
        self.b1u_probe = None;
        self.b1u_pending = false;
        self.b1u_deadline = None;
        self.awaiting_e2u = false;
        let verdict = if inverted { "inverted, negating" } else { "as received" };
        self.note(format!(
            "V.92: B1u confirmed at {scored} bits: polarity {verdict} ({straight} ones straight, \
             {flipped} negated), {:.1} % ones",
            100.0 * best as f64 / scored as f64
        ));
    }

    /// Give a candidate boundary back and resume the hunt for E2u.
    ///
    /// Everything [`Self::begin_b1u`] did to the receiver is undone except the
    /// equaliser, which is the fit's and was not a guess: the slicer goes back
    /// to TRN2u's four points (8.7.2 makes E2u one of them), the data-mode
    /// decoders are dropped, and the zero-run detector is armed again. The
    /// deadline set when Ed went out is untouched, so a run of candidates that
    /// all fail costs the wait and then fails the exchange in the open.
    ///
    /// The one deadline that is set here is for the candidate opened before Ed
    /// went out -- which the wider arming in [`Self::arm_e2u_search`] makes
    /// possible: without it, a first candidate that fails while Ed is still
    /// unsent would leave a hunt with nothing to end it.
    fn abandon_b1u(&mut self, best: u64, other: u64, scored: u64) {
        let now = self.now;
        self.note(format!(
            "V.92: candidate B1u at {now} does not read as B1u ({best}/{scored} ones one way up, \
             {other} the other), still hunting E2u"
        ));
        self.upstream_data = None;
        self.b1u_probe = None;
        self.b1u_ones = [0; 2];
        self.b1u_scored = [0; 2];
        self.b1u_scoring = 0;
        self.b1u_polarity = 1.0;
        self.rx.set_slicer(Slicer::FourPoint);
        self.awaiting_e2u = true;
        self.e2u_zeroes = 0;
        if self.b1u_deadline.is_none() {
            self.b1u_pending = true;
            self.b1u_deadline = Some(self.now + (B1U_WAIT * FS) as u64);
        }
    }

    /// One upstream sequence, as 8.7.2-8.7.6 delivered it.
    fn up_sequence(&mut self, family: CpFamily) {
        let now = self.now;
        match family {
            CpFamily::Suvu(suvu) => {
                self.note(format!("V.92: SUVu, ack {}", suvu.ack));
                // 9.6.2.1.4's other half, and the half that decides *when*:
                // the analogue modem transmits E2u "after [it] has sent a CPu
                // or SUVu sequence with the acknowledgement bit set". An
                // acknowledged SUVu from the far end is therefore the last
                // thing that happens before E2u can, and it is the only
                // observable event on the far end's own side of the clause.
                // Both far ends measured here ack as they converge, and
                // neither waits for *this* end's ack bit, which is clear on
                // every CPd until a CPu has arrived: on the HSF capture the
                // last acknowledged SUVu is at symbol 108 704 and E2u ends at
                // 108 733; on `/tmp/p0corr/ident.raw` they are 111 304 and
                // 111 431.
                if suvu.ack {
                    self.arm_e2u_search();
                }
                if let Some(exchange) = self.exchange.as_mut() {
                    exchange.peer_suv(
                        PeerSuv { ack: suvu.ack, silence: suvu.silence, wait_for_cp: suvu.wait_for_cpu },
                        now,
                    );
                }
            }
            CpFamily::Cpu(cp) => {
                let (inner, outer, gain, seen) = self.rx.measured_levels();
                if seen > 0 {
                    self.note(format!(
                        "V.92: upstream four points measure {inner:.3}/{outer:.3}, gain {gain:.3} over {seen}"
                    ));
                }
                self.note(format!(
                    "V.92: CPu drn {}, ack {}, {:.1} kbit/s",
                    cp.drn,
                    cp.acknowledge,
                    crate::v90::sequences::data_rate(cp.drn).unwrap_or(0) as f64 / 1000.0
                ));
                let ack = cp.acknowledge;
                self.cpu = Some(cp);
                if let Some(exchange) = self.exchange.as_mut() {
                    exchange.peer_cp(PeerCp { ack }, now);
                }
            }
            // CPus is a rate renegotiation's short form (Table 24); training
            // never sees one.
            CpFamily::Cpus(_) | CpFamily::Cpt(_) => {}
        }
    }

    /// Record what happened, and say it out loud when the trace is on: a bench
    /// run's only account of what this end decided.
    fn note(&mut self, text: String) {
        if tracing() {
            eprintln!("V92TRACE: {text}");
        }
        self.notes.push(text);
    }

    fn su_reversal(&mut self, at: u64) {
        match self.stage {
            Stage::AwaitSu => {
                // D7: the Su-to-S-bar-u reversal. 9.5.1.1.7 does not end Jp
                // here: the receiver carries on receiving Su -- the run after
                // the 24.5T S-bar-u -- and keeps measuring, because that run's
                // phase is the one that already carries the half symbol. Jp
                // goes out as soon as the measurement is in (see `timers`).
                self.stage = Stage::AwaitSuBar;
                self.su_from = Some(at + SU_AFTER_S_BAR);
                self.note(format!("V.92: Su reversed at {at}, measuring the next run"));
            }
            Stage::AwaitSuBar => {
                // D9: the (24 + epsilon)T S-bar-u that answers Jp. Circuit 107
                // comes on before Jp' goes out.
                self.circuit107 = true;
                self.source.change(Out::JpPrime);
                self.stage = Stage::Dil;
                // D10: the second TRN1u starts 24T plus epsilon after this
                // reversal and its first symbol is upstream interval 0. The fit
                // that will read CPt is taken inside it, once [`REFIT_WINDOW`]
                // of its symbols are in -- found by correlation, because the
                // hunt reports the reversal it turns on up to a period late
                // (see `Receiver::align_trn1u`).
                self.refit_from = Some(at + TRN_START);
                self.trn2_estimate = self.refit_from;
                self.design_at = self.refit_from.map(|at| at + DESIGN_AFTER);
                self.refit_tries = 0;
                self.reader = TwoPointReader::differential();
                self.cps = CpFinder::v92(JA_UNIT_BITS);
                self.cp_len = 0;
                self.cp_pos = 0;
                self.e1u_zeroes = 0;
                if let Some(diag) = self.diag.as_mut() {
                    // Phase 3's CPt read starts again here, so the stream, the
                    // hits and the verdict are this read's and not a last one's.
                    diag.bits.clear();
                    diag.dropped = 0;
                    diag.first_symbol = None;
                    diag.hits = 0;
                    diag.first_hit = None;
                    diag.last_hit = None;
                    diag.frame = None;
                    diag.exited = false;
                }
                self.note("V.92: second S-bar-u, 107 on".into());
            }
            _ => {}
        }
    }

    fn symbol(&mut self, symbol: Symbol) {
        // 8.7.2-8.7.6: Phase 4's upstream arrives four points to a symbol, and
        // what it carries is SUVu, CPu, E2u and then B1u's payload.
        if self.upstream.is_some() || self.upstream_data.is_some() {
            self.symbol_upstream(symbol);
        }
        match self.stage {
            Stage::ReadJa => {
                if let Some(bit) = self.reader.feed(symbol.value) {
                    self.ja_bits += 1;
                    self.ja_ones += u64::from(bit);
                    if tracing() && self.ja_bits % 400 == 0 {
                        eprintln!(
                            "V92TRACE: Ja reader {} bits, {} ones, value {:.3}",
                            self.ja_bits, self.ja_ones, symbol.value
                        );
                        self.ja_ones = 0;
                    }
                    if let Some(descriptor) = self.ja.feed(bit) {
                        self.heard_ja(descriptor);
                    }
                }
            }
            Stage::Dil | Stage::Ri | Stage::RiBar => {
                let Some(bit) = self.reader.feed(symbol.value) else {
                    return;
                };
                self.cpt_bit(bit);
            }
            _ => {}
        }
    }

    fn heard_ja(&mut self, descriptor: Descriptor) {
        // 9.5.1.1.3: "may wait for up to 500 ms and shall then transmit Sd".
        self.source.dil = descriptor.symbols().collect();
        let mut end = 0;
        self.source.dil_ends = descriptor
            .ucodes
            .iter()
            .map(|&u| {
                end += descriptor.segment_length(u);
                end
            })
            .collect();
        self.note(format!(
            "V.92: Ja read, {} DIL segments, upstream rates {:05x}",
            descriptor.ucodes.len(),
            self.ja.upstream_rates().unwrap_or(0)
        ));
        let no_dil = descriptor.is_empty();
        self.descriptor = Some(descriptor);
        self.source.change(Out::Sd);
        self.stage = Stage::AwaitSu;
        // DR2 runs from TRN1d's start, and everything between here and there
        // has to come off the deadline: Sd is 384T, S-bar-d 48T, and TRN1d is
        // as long as this end was built to send it. Counting only TRN1d's
        // 2040T minimum leaves the allowance short by the rest of TRN1d --
        // 2.9 s with `Habits::LIVE` -- so a far end that uses the 5000 ms
        // 9.5.2.1.6 allows it before Su is cut off, and the call retrains
        // instead of connecting. That is what an Agere SV92PP on an 8 kHz
        // line did: it waited, and this end had already gone back to tone A.
        let trn1d = (self.settings.habits.trn1d * FS) as u64;
        self.su_deadline = Some(
            self.now
                + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64
                + trn1d
                + ((SU_WITHIN + self.settings.round_trip) * FS) as u64,
        );
        if no_dil && !ZERO_DIL_USES_SCR {
            self.source.change(Out::Ri);
        }
        self.rx.restart_hunt();
        self.ja_deadline = None;
    }

    fn heard_cp(&mut self, cp: Cp) {
        if self.cpt.is_none() {
            self.note(format!("V.92: CPt drn {}, {} set(s)", cp.drn, cp.constellations.len()));
        }
        // Phase 4's constellation is CPt's and nothing else's (V.90 8.6.5), so
        // this is the moment the downstream's coder can be built: `drn + 8` bits
        // a data frame over CPt's sets. Everything TRN2d, SUVd, CPd and Ed send
        // from here on goes through it.
        // Only while Phase 4 has not started: installing a coder in the middle
        // of TRN2d would put the differential encoder back to zero under a
        // signal that is already going out.
        if !cp.data_mode && !self.source.in_phase4() && self.source.set_cpt(&cp) && tracing() {
            eprintln!(
                "V92TRACE: phase 4 coder at {} bit/s, {} bits a frame",
                cp.frame_bits() as u32 * 8000 / 6,
                cp.frame_bits()
            );
        }
        self.cpt = Some(cp);
        match self.stage {
            Stage::Dil => {
                // 9.5.1.1.12: "finish the current DIL segment and send Ri",
                // which the source does at the next segment boundary.
                self.source.change(Out::Ri);
                self.stage = Stage::Ri;
                self.cp_pos = 0;
                self.e1u_zeroes = 0;
            }
            // With no DIL asked for, Ri has gone out already and this CPt is
            // the answer to it: R-bar-i ends Phase 3 (9.5.2.1.10 read from this
            // side, whose CPt is the analogue modem's).
            Stage::Ri if self.no_dil() => {
                self.source.change(Out::RiBar);
                self.stage = Stage::RiBar;
                self.e1u_zeroes = 0;
            }
            _ => {}
        }
    }

    /// One decoded bit of the CPt series, and where it falls in the repetition.
    ///
    /// 8.5.2 makes E1u "a data frame of scrambled, differentially encoded
    /// zeroes ... used to signal the end of CPt", and 9.5.1.1.13 has it arrive
    /// in the place the next CPt would have begun. Counting *any* twelve
    /// zeroes in a row is not enough to see it: every CPt has seventeen ones in
    /// front of it and then sixteen-bit words, and the reserved and shaping
    /// fields of Table 23 are whole words of zeroes -- so a run of sixteen
    /// comes round with every repetition, and an E1u that was looked for that
    /// way would be found in the middle of the very series it is meant to end
    /// (which is exactly what an earlier version of this did, and why Ri went
    /// out for one symbol instead of 384). So the length of one repetition is
    /// taken from the CPt that was read, and only the twelve bits where its
    /// synchronisation belongs are counted.
    /// Whether the analogue modem's Ja asked for no DIL at all, which puts
    /// 9.5.1.1.13's SCR path and 9.5.2.1.10's order in force.
    fn no_dil(&self) -> bool {
        self.descriptor.as_ref().is_some_and(Descriptor::is_empty)
    }

    fn cpt_bit(&mut self, bit: bool) {
        self.cp_bits += 1;
        self.cp_ones += u64::from(bit);
        let now = self.now;
        if let Some(diag) = self.diag.as_mut() {
            diag.first_symbol.get_or_insert(now);
            diag.push(bit);
            if let Some(file) = cpt_bits_file()
                && let Ok(mut file) = file.lock()
            {
                use std::io::Write;
                let _ = writeln!(file, "{now} {}", u8::from(bit));
            }
        }
        if tracing() && self.cp_bits % 400 == 0 {
            eprintln!("V92TRACE: CPt reader {} bits, {} ones", self.cp_bits, self.cp_ones);
            self.cp_ones = 0;
        }
        if let Some(cp) = self.cps.feed(bit) {
            // A sequence is found by its CRC, so the bit just fed is its last:
            // the next one is the first of the next repetition.
            self.cp_len = cp.to_bits_in(Layout::V92, UP_INTERVALS).len();
            self.cp_pos = 0;
            self.since_cpt = 0;
            self.e1u_zeroes = 0;
            if let Some(diag) = self.diag.as_mut() {
                let at = diag.count().saturating_sub(1);
                diag.hits += 1;
                diag.first_hit.get_or_insert(at);
                diag.last_hit = Some(at);
                // The frame the far end is actually sending, which is what
                // another arm's stream can be read against.
                diag.frame = Some(cp.to_bits_in(Layout::V92, UP_INTERVALS));
            }
            self.heard_cp(cp);
            return;
        }
        self.since_cpt = self.since_cpt.saturating_add(1);
        if self.cp_len == 0 {
            // E1u is not looked for at all until one CPt has been found by its
            // CRC, so on an arm that never finds one this is the whole of
            // Phase 3's exit -- and it used to say nothing whatever.
            if self.diag.is_some() && self.cp_bits % DIAG_EVERY == 0 {
                self.phase3_line("still no CPt CRC hit");
            }
            return;
        }
        if self.cp_pos < E1U_ZEROES {
            self.e1u_zeroes = if bit { 0 } else { self.e1u_zeroes + 1 };
            // Twelve zeros only mean E1u while CPt is actually being received:
            // one repetition of it, plus the twelve, is as long ago as the last
            // CRC hit can be. Silence is not E1u.
            let fresh = self.since_cpt <= self.cp_len + E1U_ZEROES;
            if self.diag.is_some()
                && self.e1u_zeroes == E1U_ZEROES
                && !(fresh && self.cpt.is_some() && self.stage == Stage::Ri)
            {
                // Twelve zeroes arrived where E1u belongs and it was not taken:
                // the four conditions are `cp_len != 0` (implied here), fresh,
                // `cpt.is_some()` and `stage == Ri`, and this says which of the
                // three that can fail did.
                let why = format!(
                    "twelve zeroes where E1u belongs, refused: fresh {fresh}, cpt {}, \
                     stage {:?}",
                    self.cpt.is_some(),
                    self.stage
                );
                self.phase3_line(&why);
            }
            if self.e1u_zeroes == E1U_ZEROES
                && fresh
                && self.cpt.is_some()
                && self.stage == Stage::Ri
            {
                self.note("V.92: E1u".into());
                self.source.change(Out::RiBar);
                self.stage = Stage::RiBar;
                // 8.7.6: TRN2u resets the scrambler and starts the differential
                // encoder from the last sign bit of the E1u just read, so this
                // is the only symbol from which the upstream can be read at all.
                // Starting the reader at Phase 4, three seconds of TRN2u later,
                // leaves the descrambler at an arbitrary point in its sequence
                // and every bit after it is garbage.
                self.begin_upstream();
            }
        }
        self.cp_pos += 1;
        if self.cp_pos >= self.cp_len {
            self.cp_pos = 0;
            self.e1u_zeroes = 0;
        }
    }

    /// One diagnostic line, when `V92_PHASE3_DIAG` asked for them.
    fn diag_line(&self, text: &str) {
        if self.diag.is_some() {
            eprintln!("V92DIAG: {text}");
        }
    }

    /// Phase 3's exit, once: the E1u test's verdict, or the reason it was never
    /// reached.
    fn phase3_exit(&mut self) {
        let why = {
            let Some(diag) = self.diag.as_ref() else { return };
            if diag.exited || diag.first_symbol.is_none() {
                return;
            }
            if diag.hits == 0 {
                "left with no CPt CRC hit at all, so E1u was never looked for".to_owned()
            } else if self.stage == Stage::Phase4 {
                "left on E1u".to_owned()
            } else {
                format!("left without E1u after {} hits", diag.hits)
            }
        };
        if let Some(diag) = self.diag.as_mut() {
            diag.exited = true;
        }
        self.phase3_line(&why);
    }

    /// Where Phase 3's exit stands, and -- when no CPt has checked -- how far
    /// the stream is from being one: see [`phase3_diag`].
    fn phase3_line(&mut self, why: &str) {
        let (cp_bits, cp_pos, cp_len, e1u_zeroes, since_cpt, stage) =
            (self.cp_bits, self.cp_pos, self.cp_len, self.e1u_zeroes, self.since_cpt, self.stage);
        let cpt = self.cpt.is_some();
        let (hits, first_hit, last_hit, first_symbol, jp, scan, frame, count) = {
            let Some(diag) = self.diag.as_ref() else { return };
            (
                diag.hits,
                diag.first_hit,
                diag.last_hit,
                diag.first_symbol,
                diag.jp,
                cpt_diag::closest(&diag.bits, diag.dropped),
                diag.frame.as_ref().map(Vec::len),
                diag.count(),
            )
        };
        eprintln!(
            "V92DIAG: phase 3 {why}, symbol {}: cp_bits {cp_bits}, cp_pos {cp_pos}, cp_len \
             {cp_len}, e1u_zeroes {e1u_zeroes}, since_cpt {since_cpt}, cpt {cpt}, stage {stage:?}; \
             CPt read from symbol {first_symbol:?}, hits {hits} at bits {first_hit:?}..{last_hit:?} \
             of {count}, frame {frame:?} bits; Jp (lag, epsilon) {jp:?}; rx {:.1} dB",
            self.now,
            self.rx.snr_db()
        );
        match scan.closest {
            Some(best) => eprintln!(
                "V92DIAG: closest CPt frame: {} bit errors (sync {}, starts {}, crc {}) at bit \
                 {} of {count}, {} blocks ({} bits); best sync error anywhere {}, {} windows \
                 within {} of {} scanned",
                best.errors,
                best.sync,
                best.starts,
                best.crc,
                best.at,
                best.blocks,
                best.length(),
                scan.min_sync,
                scan.near,
                cpt_diag::NEAR,
                scan.bits
            ),
            None => eprintln!(
                "V92DIAG: no CPt frame within {} sync errors anywhere in {} bits; best sync \
                 error anywhere {} (a CPt has 0, a stream that is not one averages 8.5); \
                 {} windows examined",
                cpt_diag::NEAR,
                scan.bits,
                scan.min_sync,
                scan.near
            ),
        }
    }
}

impl Default for Modem {
    fn default() -> Self {
        Self::new(Settings {
            law: Law::Mu,
            uinfo: 79,
            far_md: 0,
            round_trip: 0.0,
            wide: true,
            limits: FilterLimits::from_info1a(0, 0, 0),
            habits: Habits::PROMPT,
            probing: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v90::encoder::{Decoder as CodeDecoder, Frame as CodeFrame};
    use crate::v90::sequences::Layout;
    use crate::v90::sign::Redundancy;
    use crate::v92::sequences::{DownFamily, DownFinder};
    use crate::v92::up_signals::{ru, su, Trn1u, TwoPointSender};

    /// A CPt of the shape the analogue modem sends in Phase 3: one constellation
    /// used in all six intervals, at the rate `drn` names.
    fn cpt(drn: u8, points: std::ops::RangeInclusive<u8>) -> Cp {
        let mask = points.fold(0u128, |m, u| m | 1 << u);
        Cp {
            data_mode: false,
            drn,
            redundancy: Redundancy::None,
            trn1d_ratio: 1 << 13,
            intervals: [0; INTERVALS],
            constellations: vec![mask],
            ..Cp::default()
        }
    }

    /// The analogue modem's receiving half of the Phase 4 downstream: slice the
    /// line against **CPt's** constellations, undo 5.4's differential coding,
    /// then descramble and look for a sequence exactly as the far end does.
    ///
    /// This is the reader the bench says matters, and the one these tests used
    /// to lack. A two-point reader reads a two-point line back perfectly and
    /// says nothing whatever about whether the far end can, which is how a
    /// Phase 4 downstream carrying one bit a symbol passed its own tests for a
    /// week while an Agere SV92PP sliced four bits a frame out of it and never
    /// found a frame sync.
    struct FarEnd {
        cp: Cp,
        decoder: CodeDecoder,
        scrambling: Scrambler,
        finder: DownFinder,
        ucodes: [u8; INTERVALS],
        positive: [bool; INTERVALS],
        at: usize,
        /// The magnitudes seen, so a test can say what the line carried.
        levels: Vec<f64>,
    }

    impl FarEnd {
        fn new(cp: &Cp) -> Self {
            let mapping = CodeMapping::from_cp(cp).expect("a CPt that fits its own rate");
            Self {
                cp: cp.clone(),
                decoder: CodeDecoder::new(mapping),
                // The downstream's polynomial is the digital modem's (6.3), so
                // the descrambler is the `Call` one.
                scrambling: Scrambler::new(Mode::Call),
                finder: DownFinder::new(),
                ucodes: [0; INTERVALS],
                positive: [false; INTERVALS],
                at: 0,
                levels: Vec::new(),
            }
        }

        /// One line sample in; the sequence it completes, if any.
        fn feed(&mut self, level: f64) -> Option<DownFamily> {
            self.levels.push(level);
            let points = self.cp.points(self.at);
            let (ucode, _) = points
                .iter()
                .map(|&u| (u, (ucode::level(Law::Mu, u) - level.abs()).abs()))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .expect("a constellation with no points in it");
            self.ucodes[self.at] = ucode;
            self.positive[self.at] = level >= 0.0;
            self.at += 1;
            if self.at < INTERVALS {
                return None;
            }
            self.at = 0;
            let frame = CodeFrame { ucodes: self.ucodes, positive: self.positive };
            for bit in self.decoder.frame(frame) {
                let out = self.scrambling.descramble(bit);
                if let Some(family) = self.finder.feed(out) {
                    return Some(family);
                }
            }
            None
        }
    }

    fn settings() -> Settings {
        Settings {
            law: Law::Mu,
            uinfo: 79,
            far_md: 0,
            round_trip: 0.0,
            wide: true,
            limits: FilterLimits::from_info1a(0, 0, 0),
            habits: Habits::PROMPT,
            probing: None,
        }
    }

    /// V92-63, end to end: an L1 through the analyzer becomes a `Reading`, the
    /// reading is put on the settings the way phase 2's handover puts it there
    /// (`v90::startup`), and it is still there -- and logged -- at the point
    /// where the upstream design is computed.
    ///
    /// This is the log a bench call is audited from, so the test prints it.
    #[test]
    fn the_l1_reading_survives_to_the_upstream_design() {
        // A synthetic L1 through a line 15 dB down and falling 6 dB, with
        // noise under it, at the V.92 digital modem's own rate.
        const FS: f64 = 8000.0;
        let mut analyzer = crate::v34::probe::Analyzer::new(FS);
        let mut seed = 17u32;
        for i in 0..8000 {
            let t = i as f64 / FS;
            let signal: f64 = crate::v34::probe::TONES
                .iter()
                .map(|&(f, phase)| {
                    let gain = 10f64.powf((-15.0 - 6.0 * (f - 150.0) / 3600.0) / 20.0);
                    gain * (std::f64::consts::TAU * f * t + phase.to_radians()).cos()
                })
                .sum::<f64>()
                * 0.2;
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            let noise = (f64::from(seed) / f64::from(u32::MAX) - 0.5) * 0.002;
            analyzer.feed(signal + noise);
        }
        let reading = analyzer.reading().expect("an L1 of one second is a reading");
        assert_eq!(reading.tones.len(), 21);

        // The handover, as `v90::startup` does it: the reading goes on the
        // settings the phase 3 modem is built from.
        let mut settings = settings();
        settings.probing = Some(reading);
        let mut modem = Modem::new(settings);
        // The reading's path is what this test is about, and which path is the
        // default is a separate question with its own test: say it here so the
        // survival of the reading is measured rather than inherited.
        modem.prefilter = Prefilter::Reading;
        let notes = modem.take_notes();
        assert!(
            notes.iter().any(|line| line.contains("L1 probing retained")),
            "the modem cannot see the reading: {notes:?}"
        );

        // ...and at the point the upstream design is computed, where it used to
        // be dropped.
        let channel = Channel { taps: vec![1.0, -0.2, 0.05], noise: 1e-6, signal: 1.0 };
        let limits = FilterLimits::from_info1a(3, 3, 3);
        let (designed, degraded) = modem.upstream_design(&channel, &limits);
        for line in modem.take_notes() {
            eprintln!("{line}");
        }
        assert!(degraded.is_none(), "{degraded:?}");
        assert_eq!(designed.filters.z2.len(), crate::v92::design::READING_TAPS);
        designed.filters.fits(&limits).unwrap_or_else(|error| panic!("{error}"));
        // Level first: the design does not spend what the receiver's cliff is
        // one decibel of.
        let power: f64 = designed.filters.z2.iter().map(|tap| tap * tap).sum();
        assert!(power >= 0.9995, "the design attenuates: sum z^2 {power:.4}");
    }

    /// Which prefilter the design is built from must not change quietly.
    ///
    /// It has changed twice: to `Fit` when a paired bench batch blamed the
    /// reading for a regression that was really `Design::delay` being counted
    /// twice in B1u's frame origin, and back to `Reading` when that was found
    /// (see `prefilter_for`). A default that moves with the last alarming
    /// number is a default nobody can reason about, so this pins it.
    #[test]
    fn the_prefilter_default_is_the_reading_and_the_fit_is_opt_in() {
        assert_eq!(prefilter_for(None), Prefilter::Reading, "the default moved");
        assert_eq!(prefilter_for(Some("fit")), Prefilter::Fit, "the opt-out does not work");
        // Anything unrecognised is the default rather than a third state: a
        // typo in an environment variable must not quietly change the design.
        assert_eq!(prefilter_for(Some("Fit")), Prefilter::Reading);
        assert_eq!(prefilter_for(Some("")), Prefilter::Reading);
    }

    /// The Phase 4 sequences this end puts on the line have to be readable by
    /// the far end as the sequences they are, which is a claim about the
    /// scrambler's state, the differential encoder's and the sequence's own
    /// padding -- none of which `sequences`' own tests cover, because they pair
    /// a sequence with its own parser.
    ///
    /// A bench run said what getting this wrong costs: an Agere SV92PP read the
    /// first thing this end sent in Phase 4 as **Jd'**, which is the digital
    /// modem's *retrain request*, and put the whole call back through V.34
    /// Phase 2 to its death. So the reader the far end has -- CPt's
    /// constellations, 5.4's differential decoder, the descrambler and the
    /// sequence finder -- reads this end's own output here.
    #[test]
    fn the_phase_4_sequences_read_back_as_themselves() {
        let settings = settings();
        let cpt = cpt(16, 16..=47);
        let mut source = Source::new(&settings);
        assert!(source.set_cpt(&cpt), "CPt refused by the phase 4 coder");
        source.change(Out::Trn2d);
        // 9.6.1.1.1's least, so the scrambler and the differential encoder are
        // exactly as far along as they will be on the wire.
        for _ in 0..TRN2D_LEAST {
            source.next();
        }
        let mut far = FarEnd::new(&cpt);
        let mut found = None;
        let suvd = Suvd { silence: false, ack: true };
        source.send(Out::Suvd, suvd.to_bits(INTERVALS));
        for _ in 0..400 {
            if let Some(family) = far.feed(source.next()) {
                found = Some(family);
                break;
            }
        }
        match found {
            Some(DownFamily::Suvd(read)) => assert_eq!(read, suvd),
            other => panic!("SUVD did not read back: {other:?}"),
        }
    }

    /// TRN2d is the encoder of 5.4 on CPt's constellation, not a two-point
    /// stream, and a Phase 4 reader gets scrambled ones out of it.
    ///
    /// 8.6.5/V.90: TRN2d "is generated by applying scrambled binary ones to the
    /// encoder of 5.4. The constellation set to be used is passed in CPt. The
    /// scrambler, differential encoder and spectral shape filter memory shall be
    /// initialized to zero prior to transmitting TRN2d." 8.8.2/8.8.3/8.8.5 put
    /// Ed, CPd and SUVd on "the corresponding TRN2d modulation" -- the same
    /// encoder, carrying on.
    ///
    /// A two-point stream at U_INFO is what this end used to send, and it is
    /// what a Conexant HSF cannot read: its Phase 4 receiver conditions on the
    /// constellation CPt named, so over a level outside it, at a peak 4.2 dB
    /// below the SUVd that follows, it never acquired. Measured on the bench:
    /// forty-five SUVu sequences with the acknowledge bit clear, no CPu, and
    /// 9.7.2.1's retrain, twice; with TRN2d through the encoder it sends
    /// seventy-eight SUVu with the acknowledge bit set, takes this end's CPd',
    /// and both ends go on to Ed, B1d and B1u. See docs/17.
    #[test]
    fn trn2d_is_scrambled_ones_through_the_encoder_of_5_4() {
        let settings = settings();
        let cpt = cpt(16, 16..=47);
        let mut source = Source::new(&settings);
        assert!(source.set_cpt(&cpt), "CPt refused by the phase 4 coder");
        source.change(Out::Trn2d);
        let levels: Vec<f64> = (0..600).map(|_| source.next()).collect();

        // Every symbol is a point of the set CPt named, and more of them than a
        // two-point signal can reach.
        let mut seen: Vec<i64> = levels.iter().map(|l| (l.abs() * 512.0) as i64).collect();
        seen.sort_unstable();
        seen.dedup();
        assert!(seen.len() > 1, "TRN2d carried only {seen:?}: a two-point stream");
        for (i, level) in levels.iter().enumerate() {
            let points = cpt.points(i % INTERVALS);
            assert!(
                points.iter().any(|&u| (ucode::level(Law::Mu, u) - level.abs()).abs() < 0.5),
                "TRN2d symbol {level} is not a point of CPt's interval {}",
                i % INTERVALS
            );
        }

        // The far end's reader -- CPt's slicer, 5.4's differential decoder and
        // the descrambler -- reads ones, which is what 8.6.5 put there and what
        // a frame sync is found on.
        let mapping = CodeMapping::from_cp(&cpt).expect("a CPt that fits its own rate");
        let mut decoder = CodeDecoder::new(mapping);
        let mut descrambling = Scrambler::new(Mode::Call);
        let mut bits: Vec<bool> = Vec::new();
        let mut ucodes = [0u8; INTERVALS];
        let mut positive = [false; INTERVALS];
        for (i, level) in levels.iter().enumerate() {
            let points = cpt.points(i % INTERVALS);
            let (ucode, _) = points
                .iter()
                .map(|&u| (u, (ucode::level(Law::Mu, u) - level.abs()).abs()))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .expect("a constellation with no points in it");
            ucodes[i % INTERVALS] = ucode;
            positive[i % INTERVALS] = *level >= 0.0;
            if i % INTERVALS == INTERVALS - 1 {
                let frame = CodeFrame { ucodes, positive };
                for bit in decoder.frame(frame) {
                    bits.push(descrambling.descramble(bit));
                }
            }
        }
        let d = cpt.frame_bits();
        let settling = d;
        assert!(bits.len() > 2 * d, "only {} bits read back", bits.len());
        assert!(
            bits[settling..].iter().all(|&b| b),
            "the far end read {} zeros out of TRN2d, so it is not scrambled ones",
            bits[settling..].iter().filter(|&&b| !b).count()
        );
    }

    /// With no CPt there is no encoder to go out on, and the fallback is still
    /// the two-point signal at U_INFO that Phase 3's TRN1d uses -- but Phase 4
    /// cannot get past its first symbol without a CPt, so this is a shape the
    /// code keeps and the Recommendation does not have.
    #[test]
    fn trn2d_without_a_cpt_falls_back_to_u_info() {
        let settings = settings();
        let mut source = Source::new(&settings);
        source.change(Out::Trn2d);
        let levels: Vec<f64> = (0..600).map(|_| source.next()).collect();
        let mut seen: Vec<i64> = levels.iter().map(|l| (l.abs() * 512.0) as i64).collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(
            seen,
            vec![(ucode::level(Law::Mu, settings.uinfo) * 512.0) as i64],
            "the no-CPt fallback is not the two-point stream at U_INFO"
        );
    }

    /// The line a Phase 4 exchange actually puts out: whole sequences, repeated
    /// -- not one symbol of each.
    ///
    /// The exchange asks for a new sequence every time one ends, and the only
    /// thing `Source` has to get right in that dance is not to end one before
    /// its bits are done. It used to clear the queue in `send`, which ended
    /// whatever sequence was in the middle of going out, so each sequence got
    /// exactly one symbol onto the line -- and since bit 0 of both SUVd and CPd
    /// is a frame-sync one, what actually went out was a run of scrambled ones
    /// with no frame sync in it anywhere. An Agere SV92PP read that as nothing
    /// at all, kept answering SUVu with the acknowledge bit clear, and ten
    /// seconds later raised `V34 exception: 60096212`.
    #[test]
    fn a_phase_4_exchange_repeats_whole_sequences() {
        let settings = settings();
        let cpt = cpt(16, 16..=47);
        let mut source = Source::new(&settings);
        assert!(source.set_cpt(&cpt));
        source.change(Out::Trn2d);
        for _ in 0..TRN2D_LEAST {
            source.next();
        }
        let suvd = Suvd { silence: false, ack: false };
        let mut far = FarEnd::new(&cpt);
        let mut found = 0;
        source.send(Out::Suvd, suvd.to_bits(INTERVALS));
        for _ in 0..4000 {
            let level = source.next();
            // Exactly what `phase4_out` does: as soon as the last one ended,
            // ask for the same sequence again (9.6.1.1.1 repeats the SUVd
            // until a CPu arrives).
            if source.take_finished().is_some() {
                source.send(Out::Suvd, suvd.to_bits(INTERVALS));
            }
            if let Some(DownFamily::Suvd(read)) = far.feed(level) {
                assert_eq!(read, suvd);
                found += 1;
            }
        }
        // Four thousand symbols of a seventy-two bit sequence (a whole number
        // of 24-bit data frames): a good many of them, less the reader's own
        // settling and the frames the coder holds back.
        assert!(found > 25, "only {found} SUVd sequences reached the line");
    }

    /// And CPd, which is the one Phase 4 sequence with state in it: if its bits
    /// do not read back, the far end cannot use the parameters and will not
    /// acknowledge them (9.6.2.1.3) -- which on the bench looks exactly like a
    /// far end that has not heard anything at all.
    #[test]
    fn cpd_reads_back_as_the_parameters_it_was_built_from() {
        let settings = settings();
        // The slowest rung of 6.1's ladder, which is what this end asks for
        // first: K = 36 bits over twelve intervals, eight levels each.
        let parameters = {
            let mut taps = vec![0.0f64; 16];
            taps[0] = 1.0;
            let channel = Channel::estimate(&[1.0, -1.0, 1.0, 1.0, -1.0, 1.0], &[1.0, -1.0, 1.0, 1.0, -1.0, 1.0], 4)
                .expect("a flat channel");
            let designed = design(&channel, &settings.limits).expect("designable");
            choose(&designed, 1e-6, 1, Trellis::Sixteen).expect("the slowest rung").parameters
        };
        assert_eq!(parameters.drn, 1);
        let cpd = Cpd::from(&parameters);
        // The sequence itself first, so that a failure below says whether it is
        // the bits or the framing around them.
        let bits = cpd.to_bits(INTERVALS);
        let direct = Cpd::from_bits(&bits);
        assert!(direct.is_some(), "Cpd::from_bits refused its own bits, {} of them", bits.len());
        assert_eq!(direct.as_ref().map(|c| c.drn), Some(cpd.drn));
        assert_eq!(direct.as_ref().map(|c| c.moduli), Some(cpd.moduli));
        // And through the finder, with no line in between.
        let mut wire = DownFinder::new();
        let mut through = None;
        for &bit in &bits {
            if let Some(family) = wire.feed(bit) {
                through = Some(family);
                break;
            }
        }
        assert!(through.is_some(), "the finder missed a CPd of {} bits", bits.len());
        let cpt = cpt(16, 16..=47);
        let mut source = Source::new(&settings);
        assert!(source.set_cpt(&cpt));
        source.change(Out::Trn2d);
        for _ in 0..TRN2D_LEAST {
            source.next();
        }
        let mut far = FarEnd::new(&cpt);
        let mut found = None;
        source.send(Out::Cpd, cpd.to_bits(INTERVALS));
        // CPd is the longest sequence in the protocol -- a full 6.4 chain --
        // so the window is its own length and a good margin over.
        for _ in 0..(4 * bits.len() + 400) {
            if let Some(family) = far.feed(source.next()) {
                found = Some(family);
                break;
            }
        }
        match found {
            Some(DownFamily::Cpd(read)) => {
                assert_eq!(read.drn, cpd.drn, "drn");
                assert_eq!(read.ack, cpd.ack, "ack");
                assert_eq!(read.moduli, cpd.moduli, "moduli");
                assert_eq!(read.sets, cpd.sets, "constellation sets");
            }
            other => panic!("CPD did not read back: {other:?}"),
        }
    }

    /// TRN1d's minimum, 2040T (V.90 8.4.5): what [`Habits`] must not go under,
    /// and what the deadline in `heard_ja` has to leave room for.
    const TRN1D_MINIMUM: usize = 340 * INTERVALS;

    const RU_END: u64 = 384;
    const RU_BAR_END: u64 = 408;
    const TRN1_END: u64 = 408 + 2400;
    const JA_END: u64 = TRN1_END + 400;
    const SU_START: u64 = JA_END + 100;

    /// The analogue modem's side of Phase 3, as the plan's V92-27 script
    /// builds it: Ru, R-bar-u, TRN1u, Ja, silence, Su, S-bar-u, TRN1u, CPt,
    /// E1u.
    struct Script {
        trn: Trn1u,
        ja: Option<TwoPointSender>,
        cp: Option<TwoPointSender>,
        second_trn: Trn1u,
    }

    impl Script {
        fn new(wanted: &Descriptor, rates: u32, cp: Option<Cp>) -> Self {
            let trn = Trn1u::new();
            // Burn the sender's state forward so that `after` takes it over
            // with the same register the far end's descrambler will have.
            let mut ja = TwoPointSender::after(Trn1u::new());
            ja.preamble();
            ja.push(&wanted.to_bits_in(Some(rates)));
            let cp = cp.map(|cp| {
                let mut sender = TwoPointSender::after(Trn1u::new());
                sender.preamble();
                sender.push(&cp.to_bits_in(Layout::V92, UP_INTERVALS));
                sender
            });
            Self { trn, ja: Some(ja), cp, second_trn: Trn1u::new() }
        }

        fn next(&mut self, n: u64) -> f64 {
            if n < RU_END {
                return ru(n, false);
            }
            if n < RU_BAR_END {
                return ru(n, true);
            }
            if n < TRN1_END {
                return self.trn.next_symbol();
            }
            if n < JA_END {
                if let Some(ja) = self.ja.as_mut()
                    && let Some(level) = ja.next_symbol()
                {
                    return level;
                }
                return 0.0;
            }
            if n < SU_START {
                return 0.0;
            }
            // Su 144T, S-bar-u 24T, Su again (which is where Jp is heard),
            // the (24 + epsilon)T S-bar-u that answers it, TRN1u, CPt, E1u.
            let bar1 = SU_START + 144;
            let su2 = bar1 + 24;
            let bar2 = su2 + 400;
            let trn2 = bar2 + 40;
            let cpt = trn2 + 2400;
            if n < bar1 {
                return su(n, false);
            }
            if n < su2 {
                return su(n, true);
            }
            if n < bar2 {
                return su(n, false);
            }
            if n < trn2 {
                return su(n, true);
            }
            if n < cpt {
                return self.second_trn.next_symbol();
            }
            if let Some(cp) = self.cp.as_mut()
                && let Some(level) = cp.next_symbol()
            {
                return level;
            }
            0.0
        }
    }

    fn run(script: &mut Script, symbols: u64) -> Modem {
        let mut modem = Modem::new(settings());
        for n in 0..symbols {
            let level = script.next(n);
            modem.step(level * 8000.0);
        }
        modem
    }

    /// Phase 3 runs from Ru to R-bar-i and into Phase 4 over a clean line: Ja
    /// is read, CPt is read, circuit 107 comes on, and TRN2d follows.
    #[test]
    fn phase_3_runs_from_ru_to_phase_4() {
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let mut script = Script::new(&Descriptor::none(), 0x7ffff, Some(cp));
        let modem = run(&mut script, SU_START + 144 + 24 + 400 + 40 + 2400 + 1600);
        let notes = modem.clone().take_notes();
        assert!(modem.descriptor().is_some(), "no Ja: {notes:?}");
        assert!(modem.cpt().is_some(), "no CPt: {notes:?}");
        assert_eq!(modem.upstream_rates(), Some(0x7ffff), "mask: {notes:?}");
        assert!(modem.circuit107(), "circuit 107 never came on: {notes:?}");
        assert_eq!(modem.stage, Stage::Phase4, "phase {} notes {notes:?}", modem.phase());
    }

    /// 9.5.1.2.2: Su within 5100 ms of the start of TRN1d, however long TRN1d
    /// is. The deadline is armed when Ja is read, so it has to carry Sd,
    /// S-bar-d and TRN1d itself -- otherwise a live 3.6 s TRN1d leaves the far
    /// end about 4 s instead of the 5.1 s it is allowed, and a far end that
    /// waits before Su (9.5.2.1.6 allows 5000 ms) is cut off.
    #[test]
    fn the_su_deadline_carries_the_whole_of_trn1d() {
        let mut settings = settings();
        settings.habits = Habits::LIVE;
        assert!(settings.habits.trn1d * FS >= TRN1D_MINIMUM as f64, "TRN1d is at least 2040T");
        let habits = settings.habits;
        let mut modem = Modem::new(settings);
        let ja_read = 8000u64;
        modem.now = ja_read;
        modem.heard_ja(Descriptor::none());
        let deadline = modem.su_deadline.expect("a deadline from Ja");
        // TRN1d starts after Sd and S-bar-d, and Su is due 5.1 s after that.
        let expected = ja_read
            + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64
            + (habits.trn1d * FS) as u64
            + (SU_WITHIN * FS) as u64;
        assert_eq!(deadline, expected);
        // Which is 5.1 s after TRN1d's start, and not 5.1 s after Ja.
        let trn1d_start = ja_read + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64;
        assert!(deadline - trn1d_start >= (SU_WITHIN * FS) as u64);
    }

    /// Ja is read once and its descriptor kept, with the mask that came in it.
    #[test]
    fn ja_is_read_once_and_its_upstream_mask_is_kept() {
        let wanted = Descriptor::none();
        let mut script = Script::new(&wanted, 0x0000f, None);
        let modem = run(&mut script, JA_END + 50);
        let notes = modem.clone().take_notes();
        assert!(modem.descriptor().is_some(), "Ja not read: {notes:?}");
        assert_eq!(modem.upstream_rates(), Some(0x0000f), "{notes:?}");
    }

    /// The information sequence this end sends is a Jd the far end can read
    /// back, and it advertises every downstream rate (Table 21).
    #[test]
    fn the_jd_we_send_reads_back_as_a_jd_with_every_rate() {
        let bits = settings().jd_bits();
        let read = J::from_bits(&bits).expect("our own Jd does not check");
        let J::Jd(jd) = read else { panic!("our own Jd reads as a Jp") };
        assert_eq!(jd.rates, Jd::ALL_RATES);
        assert!(!jd.sixteen_in_training);
        assert_eq!(jd.lookahead, Habits::PROMPT.lookahead);
    }

    /// A descriptor that asks for segments puts them on the line, and the DIL
    /// ends in whole segments.
    #[test]
    fn a_descriptor_with_segments_puts_dil_on_the_line() {
        let wanted = crate::v90::dil::design(Law::Mu, 79);
        assert!(!wanted.is_empty(), "the DIL design asks for no segments");
        let mut settings = settings();
        settings.law = Law::Mu;
        let mut source = Source::new(&settings);
        source.dil = wanted.symbols().collect();
        let mut end = 0;
        source.dil_ends = wanted
            .ucodes
            .iter()
            .map(|&u| {
                end += wanted.segment_length(u);
                end
            })
            .collect();
        assert!(source.dil_ends.windows(2).all(|w| w[0] <= w[1]));
        assert_eq!(*source.dil_ends.last().unwrap(), source.dil.len());
        let length = source.dil.len();
        // V.90 8.4.1, adopted whole by 8.6.1: the *entire* sequence is repeated
        // until the far end terminates it, so two passes are two identical
        // passes and the DIL is still on the line afterwards.
        source.change(Out::Dil);
        let produced: Vec<f64> = (0..2 * length).map(|_| source.next()).collect();
        assert!(produced.iter().any(|v| v.abs() > 0.0), "the DIL was silent");
        assert_eq!(produced[..length], produced[length..], "the DIL did not repeat");
        assert_eq!(source.out(), Out::Dil, "the DIL left the line with nothing to replace it");

        // 9.5.1.1.12: a CPt part way through a pass ends the DIL on the next
        // segment boundary (V.90 8.4.1), not at once and not at the end of the
        // pass.
        let into = 7;
        let into = into.min(length / 2);
        for _ in 0..into {
            source.next();
        }
        source.change(Out::Ri);
        let next_end = source
            .dil_ends
            .iter()
            .copied()
            .find(|&end| end > into)
            .expect("a segment boundary after the seventh symbol");
        let mut sent = into;
        loop {
            source.next();
            if source.out() != Out::Dil {
                break;
            }
            sent += 1;
            assert!(sent <= next_end, "the DIL ran past the segment boundary it had reached");
        }
        assert_eq!(sent, next_end, "the DIL stopped mid-segment");
        assert_eq!(source.out(), Out::Ri, "the DIL was not followed by Ri");
    }
}
