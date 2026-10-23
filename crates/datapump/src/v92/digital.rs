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
use crate::v92::design::{Channel, design};
use crate::v92::epsilon;
use crate::v92::exchange::{Context, Exchange};
use crate::v92::receiver::{ALIGN_WINDOW, Heard, Receiver, Slicer, Symbol, TRAIN_WINDOW};
use crate::v92::sequences::{
    CpFamily, Cpd, J, JP_PRIME_BITS, Jp, Suvd, UpFinder, trn2u_unit,
};
use crate::v92::up_signals::{Trn2uReader, Trn2uSize, TwoPointReader};
use crate::v92::upchoice::choose;
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
#[derive(Debug, Clone, Copy, PartialEq)]
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
        }
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
    #[must_use]
    pub fn needs_codes(&self) -> bool {
        self.out == Out::B1d && self.codes.len() < 2 * INTERVALS
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
                            let (u, positive) = ucode::from_octet(self.law, octet);
                            return self.level(u, positive);
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
                    // Sent once, not repeated (V.90 8.4.1): what follows it is
                    // the analogue modem's CPt, and 9.5.1.1.12 cuts the DIL
                    // short at a segment boundary when that comes first.
                    if self.count >= self.dil.len() && self.at_frame_boundary() {
                        let next = self.pending.take().unwrap_or(Out::Silence);
                        self.start(next);
                        continue;
                    }
                    let at = self.count;
                    // "The sequence shall be terminated on a DIL-segment
                    // boundary" (V.90 8.4.1).
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
                    // 8.6.5/V.90: TRN2d is "scrambled binary ones" -- a
                    // **binary** signal, one bit a symbol, the scrambled bit
                    // being the symbol's sign. It does not go through the
                    // encoder of 5.4 and it does not use CPt's constellation;
                    // SUVd, CPd and Ed after it do (8.8.2, 8.8.3, 8.8.5).
                    //
                    // Sending it on CPt's constellation, through the trellis and
                    // the precoder, is what this end used to do and what an
                    // Agere SV92PP cannot read: its Phase 4 receiver is a plain
                    // slicer with no Viterbi, so its descrambled stream over
                    // TRN2d had no run of ones in it anywhere, its detector
                    // never acquired and it never ran on to the SUVd. Measured:
                    // with TRN2d binary it detects the SUVd at once, sends its
                    // CPu, detects the CPd and goes to data mode. See §0.0j.
                    self.sign = self.scrambler.scramble(true);
                    return self.level(self.uinfo, self.sign);
                }
            }
        }
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
    /// Zero bits since Ed went out, for E2u: twelve (or thirteen) descrambled
    /// zeros are what tells it from anything else (8.5.2, 8.7.2).
    e2u_zeroes: u32,
    /// Whether Ed has gone out, which is the only time E2u can be expected.
    awaiting_e2u: bool,
    /// The downstream data encoder, once CPu has said what to encode (V.90
    /// 8.6.1).
    downstream: Option<crate::v90::encoder::Encoder>,
    /// The upstream data decoder, in the parameters this end's own CPd named.
    upstream_data: Option<Decoder>,
    /// Upstream data bits the decoder has produced since B1u began.
    upstream_bits: u64,
    /// Largest sample seen since the last trace line.
    peak: f64,
    notes: Vec<String>,
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
        Self {
            settings,
            source: Source::new(&settings),
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
            refit_tries: 0,
            ja_bits: 0,
            ja_ones: 0,
            cp_bits: 0,
            cp_ones: 0,
            cp_len: 0,
            cp_pos: 0,
            e1u_zeroes: 0,
            since_cpt: 0,
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
            upstream_drop_until: 0,
            upstream_dropped: 0,
            up_finder: None,
            cpd: None,
            parameters: None,
            cpu: None,
            e2u_zeroes: 0,
            awaiting_e2u: false,
            upstream_bits: 0,
            downstream: None,
            upstream_data: None,
            peak: 0.0,
            notes: Vec::new(),
        }
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    pub fn status(&self) -> Status {
        self.status
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
            if self.rx.train_on_trn2u(TRAIN_WINDOW) && self.rx.install_phase_four() {
                self.upstream_drop_until = self.now + TRN2U_DROP_AFTER_FIT;
                self.note(format!(
                    "V.92: phase 4 receiver fitted on TRN2u, {:.1} dB",
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
            // Where the second TRN1u actually begins, by correlation: the hunt
            // reports the reversal up to a period late and the far end's whole
            // grid has moved by epsilon since the first fit, so the estimate is
            // good to tens of symbols. A least-squares fit at the wrong
            // alignment is a fit against the wrong sequence, and that is what
            // kept this refit from ever happening on a real line.
            let found = match self.rx.align_trn1u(at, REFIT_SEARCH, ALIGN_WINDOW) {
                // The alignment is verified by the correlation, not by the fit:
                // a two-point stream is two thirds of its power at 4000 Hz, and
                // a telephone line has nothing there, so *no* fit of TRN1u
                // reaches the figure a line with a flat top would give -- a
                // real one measures 15 dB and a harness with a 3.4 kHz filter
                // 13. What the refit has to get right is *when*, and the
                // correlation says that to a fraction of a symbol.
                Some((aligned, quality)) if quality > REFIT_QUALITY => {
                    // 9.5.1.1.10: "the digital modem shall keep a modulo 12 data
                    // frame interval count from the first symbol of TRN1u" --
                    // the *second* one, and this is the first the alignment of
                    // its first symbol is known to a symbol.
                    self.rx.set_frame_origin(base + aligned);
                    self.trn2_from = Some(base + aligned);
                    // 9.6.1.1.1's design is scheduled here: see `design_at`.
                    self.design_at = self.trn2_from.map(|at| at + DESIGN_AFTER);
                    let taken = self.rx.train_verified(aligned, TRAIN_WINDOW);
                    if tracing() {
                        eprintln!(
                            "V92TRACE: refit at {aligned}, quality {quality:.3} -> {taken} at {:.1} dB (was {baseline:.1})",
                            self.rx.snr_db()
                        );
                    }
                    taken.then(|| self.rx.snr_db())
                }
                _ => None,
            };
            match found {
                // And a fit that reads the line much worse than the one in place
                // is not a refit: the taps would be worse for the CPt than what
                // they replaced.
                Some(snr) if snr > baseline - REFIT_SLACK_DB => {
                    self.refit_from = None;
                    self.note(format!("V.92: refit at the new phase, {snr:.1} dB"));
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
            if kind == SequenceKind::E {
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
            }
            // 8.8.2: Ed is two data frames of scrambled zeros, at CPt's rate.
            SequenceKind::E => {
                self.source.send(Out::Ed, self.source.ed_bits());
                self.e2u_zeroes = 0;
                self.awaiting_e2u = true;
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

    /// 6.4 from the channel the second TRN1u measured: the fastest rung of the
    /// upstream ladder this line carries, or the slowest it will take.
    fn design_upstream(&self, from: u64) -> Option<Parameters> {
        const FIT: usize = DESIGN_AFTER as usize;
        let known = reference(0, FIT, Mode::Answer)?;
        let received = self.rx.history_range(from, from + known.len() as u64);
        if received.len() < FIT / 2 {
            return None;
        }
        let Some(channel) = Channel::estimate(&known, &received, 48) else {
            eprintln!("V92TRACE: no channel estimate ({} symbols)", received.len());
            return None;
        };
        let limits = self.settings.limits;
        let designed = match design(&channel, &limits) {
            Ok(designed) => designed,
            Err(why) => {
                eprintln!("V92TRACE: no upstream design: {why}");
                return None;
            }
        };
        // The rate this end asks for. The analogue modem's Ja says which rungs
        // it allows; this end takes the slowest of those, because the number
        // that decides how far up the ladder a line carries is the *noise*, and
        // the only figure for it available here is the two-point fit's -- which
        // on a telephone line is mostly the 4000 Hz the line does not have, not
        // noise. Asking for a rung the receiver cannot then demodulate would end
        // the call in Phase 4, which is exactly where V.92's last attempt this
        // far stopped.
        const SLOWEST: u32 = 1;
        let ja = self.ja.upstream_rates().unwrap_or(0) & SLOWEST;
        let ja = if ja == 0 { SLOWEST } else { ja };
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
        let mut noise = 10f64.powf(-self.rx.snr_db() / 10.0);
        let mut choice = None;
        for _ in 0..9 {
            choice = choose(&designed, noise, ja, trellis);
            if choice.is_some() {
                break;
            }
            noise /= 4.0;
        }
        let Some(choice) = choice else {
            eprintln!(
                "V92TRACE: no upstream rung carries this line (noise {noise:.2e}, fit noise {:.2e}, residual {:.1} dB, ja {ja:#x})",
                channel.noise,
                designed.residual_db()
            );
            return None;
        };
        if tracing() {
            eprintln!(
                "V92TRACE: upstream design assumed noise {noise:.2e}, scale {:.2}, residual {:.1} dB",
                designed.scale,
                designed.residual_db()
            );
        }
        Some(choice.parameters)
    }

    /// 9.6.1.1.5-6: B1d out, and the receiver conditioned for B1u.
    ///
    /// The downstream's mapping comes from the analogue modem's CPu, which is
    /// the same table V.90's data mode reads (V.92 8.8.1 points at V.90 8.6.1),
    /// and the upstream's decoder from this end's own CPd.
    fn begin_data(&mut self) {
        self.note("V.92: Ed out".into());
        if let Some(parameters) = self.parameters.take() {
            self.upstream_data = Some(Decoder::new(&parameters));
        }
        if let Some(cpu) = self.cpu.as_ref()
            && let Some(mapping) = crate::v90::encoder::Mapping::from_cp(cpu)
        {
            let rate = mapping.rate();
            self.downstream = Some(crate::v90::encoder::Encoder::new(mapping, self.settings.law));
            self.note(format!("V.92: B1d at {rate}"));
        } else {
            self.note("V.92: no CPu yet, B1d waits".into());
        }
        self.stage = Stage::Phase4;
        self.source.change(Out::B1d);
        self.fill_downstream();
    }

    /// Keep B1d's codeword queue fed from the downstream encoder (V.90 8.6.1).
    fn fill_downstream(&mut self) {
        if !self.source.needs_codes() {
            return;
        }
        let law = self.settings.law;
        let mut octets = Vec::new();
        if let Some(encoder) = self.downstream.as_mut() {
            for _ in 0..8 {
                // The DTE's own bits are not wired up here yet: a constant
                // stream keeps the mapper's own state turning, which is what the
                // far end's equaliser and the scrambling chain need.
                let frame = encoder.next_frame(|| false);
                octets.extend(frame.octets(law));
            }
        }
        if !octets.is_empty() {
            self.source.send_codes(octets);
        }
    }

    /// One upstream symbol: through the reader, then into whichever sequence
    /// finder is listening.
    fn symbol_upstream(&mut self, level: f64) {
        // 9.6.1.1.6: after Ed the receiver is conditioned for B1u, which is the
        // data mode modulation rather than four points.
        if let Some(decoder) = self.upstream_data.as_mut() {
            decoder.feed(level);
            let bits = decoder.take_bits();
            if !bits.is_empty() {
                self.upstream_bits += bits.len() as u64;
            }
            return;
        }
        let Some(reader) = self.upstream.as_mut() else { return };
        let bits: Vec<bool> = reader.feed(level).into_iter().collect();
        let dropped = self.now < self.upstream_drop_until;
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
            // 8.7.2: E2u is a data frame of scrambled *zeros*, so twelve
            // descrambled zeros after Ed is what it is. 9.6.1.1.4 terminates
            // the exchange on it, and 9.6.1.1.6 conditions for B1u.
            if self.awaiting_e2u {
                if bit {
                    self.e2u_zeroes = 0;
                } else {
                    self.e2u_zeroes += 1;
                    if self.e2u_zeroes == UP_INTERVALS as u32 {
                        let now = self.now;
                        if let Some(exchange) = self.exchange.as_mut() {
                            exchange.peer_e(now);
                        }
                        self.awaiting_e2u = false;
                        self.note("V.92: E2u".into());
                    }
                }
            }
        }
    }

    /// One upstream sequence, as 8.7.2-8.7.6 delivered it.
    fn up_sequence(&mut self, family: CpFamily) {
        let now = self.now;
        match family {
            CpFamily::Suvu(suvu) => {
                self.note(format!("V.92: SUVu, ack {}", suvu.ack));
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
                self.note("V.92: second S-bar-u, 107 on".into());
            }
            _ => {}
        }
    }

    fn symbol(&mut self, symbol: Symbol) {
        // 8.7.2-8.7.6: Phase 4's upstream arrives four points to a symbol, and
        // what it carries is SUVu, CPu, E2u and then B1u's payload.
        if self.upstream.is_some() || self.upstream_data.is_some() {
            self.symbol_upstream(symbol.value);
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
            self.heard_cp(cp);
            return;
        }
        self.since_cpt = self.since_cpt.saturating_add(1);
        if self.cp_len == 0 {
            return;
        }
        if self.cp_pos < E1U_ZEROES {
            self.e1u_zeroes = if bit { 0 } else { self.e1u_zeroes + 1 };
            // Twelve zeros only mean E1u while CPt is actually being received:
            // one repetition of it, plus the twelve, is as long ago as the last
            // CRC hit can be. Silence is not E1u.
            let fresh = self.since_cpt <= self.cp_len + E1U_ZEROES;
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
        }
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

    /// The Phase 4 downstream is CPt's rate: `drn + 8` bits over six codewords,
    /// through the encoder of 5.4, and `drn 16` is twenty-four of them -- four
    /// bits a symbol. A two-point stream is one bit a symbol, and a far end
    /// slicing CPt's constellation out of that reads noise however clean the
    /// line is; that is the shape of the bug this pins.
    #[test]
    /// TRN2d is a binary signal; the sequences after it are on CPt's
    /// constellation.
    ///
    /// 8.6.5/V.90 makes TRN2d "scrambled binary ones": one bit a symbol, the
    /// scrambled bit being the symbol's sign, and no encoder of 5.4 in the way.
    /// 8.8.2/8.8.3/8.8.5 put Ed, CPd and SUVd on "the corresponding TRN2d
    /// modulation", which is where CPt's constellation and the encoder come in.
    ///
    /// Sending TRN2d on CPt's constellation, through the trellis and the
    /// precoder, is what this end used to do and what an Agere SV92PP cannot
    /// read: its Phase 4 receiver is a plain slicer with no Viterbi, so over a
    /// sixteen-point TRN2d its descrambled stream had no run of ones in it
    /// anywhere, its sequence detector never acquired, and it sat in its
    /// wait-for-an-SUVd state until `V34 exception 60096212` ten seconds later.
    /// With TRN2d binary the same call had it detect the SUVd, send its CPu,
    /// detect the CPd and go to data mode. See §0.0j of the handover.
    #[test]
    fn trn2d_is_binary_and_the_sequences_are_not() {
        let settings = settings();
        let cpt = cpt(16, 16..=47);
        let mut source = Source::new(&settings);
        assert!(source.set_cpt(&cpt));
        source.change(Out::Trn2d);
        let magnitudes = |levels: &[f64]| {
            let mut seen: Vec<i64> = levels.iter().map(|l| (l.abs() * 512.0) as i64).collect();
            seen.sort_unstable();
            seen.dedup();
            seen
        };
        let trn2d: Vec<f64> = (0..600).map(|_| source.next()).collect();
        assert_eq!(
            magnitudes(&trn2d).len(),
            1,
            "TRN2d carried {:?}, so it is not the two-point signal 8.6.5 asks for",
            magnitudes(&trn2d)
        );
        // And the sequences are on CPt's constellation: more points than a
        // binary signal has. TRN2d runs its least first, so the change takes
        // effect at a frame boundary of its own and not before.
        for _ in 0..TRN2D_LEAST {
            source.next();
        }
        source.send(Out::Suvd, Suvd { silence: false, ack: false }.to_bits(INTERVALS));
        let suvd: Vec<f64> = (0..200).map(|_| source.next()).collect();
        assert!(
            magnitudes(&suvd).len() > 4,
            "the SUVd carried only {:?}, so it is not on CPt's constellation",
            magnitudes(&suvd)
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
        let mut modem = Modem::new(settings);
        let ja_read = 8000u64;
        modem.now = ja_read;
        modem.heard_ja(Descriptor::none());
        let deadline = modem.su_deadline.expect("a deadline from Ja");
        // TRN1d starts after Sd and S-bar-d, and Su is due 5.1 s after that.
        let expected = ja_read
            + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64
            + (settings.habits.trn1d * FS) as u64
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
        // And the source sends exactly one DIL pass before moving on.
        source.change(Out::Dil);
        let mut produced: Vec<f64> = Vec::new();
        for _ in 0..source.dil.len() + 8 {
            produced.push(source.next());
        }
        assert!(produced.iter().any(|v| v.abs() > 0.0), "the DIL was silent");
    }
}
