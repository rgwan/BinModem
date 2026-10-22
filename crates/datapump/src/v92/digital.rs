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
//!   digital modem can do about where the network's A/D samples.
//! - **Hear the second S-bar-u**, end Jp, raise circuit 107, and send **Jp'**
//!   and then the **DIL** the analogue modem asked for (9.5.1.1.9-9.5.1.1.11).
//! - **Read CPt** behind the DIL, answer **Ri**, and on **E1u** answer
//!   **R-bar-i** and go to Phase 4 with TRN2d (9.5.1.1.12).
//!
//! Two readings are named constants rather than buried: the epsilon is zero
//! until [`crate::v92::epsilon`] measures one, which is honest on the
//! straight-wire rigs this is proved on; and the zero-DIL branch is parsed and
//! taken but not separately timed, because no far end in reach asks for it.

use std::collections::VecDeque;

use crate::v32::{Mode, Scrambler};
use crate::v34::info::Info1aPcmUp;
use crate::v90::sequences::{Cp, CpFinder, Descriptor, DescriptorFinder, Jd, JA_UNIT_BITS};
use crate::v90::ucode::{self, Law};
use crate::v92::receiver::{Heard, Receiver, Symbol};
use crate::v92::sequences::{J, Jp, JP_PRIME_BITS};
use crate::v92::up_signals::TwoPointReader;

/// The network's rate: one codeword per 125 microseconds (5/V.92).
pub const FS: f64 = 8000.0;

/// The downstream data frame interval count, which V.92 does not change.
const INTERVALS: usize = 6;

/// Sd and S-bar-d, in frames: 384T and 48T (V.90 8.4.4).
const SD_FRAMES: usize = 64;
const SD_BAR_FRAMES: usize = 8;

/// TRN1d: "a minimum of 2040T" (V.90 8.4.5), in whole frames.
const TRN1D_FRAMES: usize = 340;

/// R-bar-i: "4 repetitions of the 6-symbol sequence" (V.90 8.6.4).
const R_BAR_FRAMES: usize = 4;

/// TRN2d: "at least 2040T" (9.6.1.1), in whole frames.
const TRN2D_FRAMES: usize = 340;

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

/// 9.5.1.2.2: Su within 5100 ms plus a round trip of the start of TRN1d.
const SU_WITHIN: f64 = 5.1;

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
    pub habits: Habits,
}

impl Settings {
    /// From INFO1a as the analogue modem sent it (Table 18).
    #[must_use]
    pub fn new(law: Law, asked: &Info1aPcmUp, round_trip: f64, wide: bool) -> Self {
        Self { law, uinfo: asked.uinfo, far_md: asked.md_length, round_trip, wide, habits: Habits::default() }
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
        self.out == Out::Trn2d
    }

    fn level(&self, ucode: u8, positive: bool) -> f64 {
        ucode::level(self.law, ucode) * if positive { 1.0 } else { -1.0 }
    }

    fn start(&mut self, out: Out) {
        self.out = out;
        self.count = 0;
        self.bits.clear();
        match out {
            // "The first symbol of Sd is defined to be transmitted in data
            // frame interval 0" (V.90 8.4.4).
            Out::Sd => self.symbol = 0,
            Out::Trn1d | Out::Trn2d => self.scrambler.reset(),
            Out::JpPrime => self.bits.extend([false; JP_PRIME_BITS]),
            _ => {}
        }
    }

    fn at_frame_boundary(&self) -> bool {
        self.symbol.is_multiple_of(INTERVALS as u64)
    }

    /// Ask for `out` at the next place the signal now on the line can end.
    pub fn change(&mut self, out: Out) {
        self.pending = Some(out);
    }

    /// One codeword out.
    pub fn next(&mut self) -> f64 {
        let level = self.next_level();
        if self.out != Out::Silence {
            self.symbol += 1;
        }
        level
    }

    fn next_level(&mut self) -> f64 {
        let w = 16 + self.uinfo;
        loop {
            match self.out {
                Out::Silence => {
                    if let Some(next) = self.pending.take() {
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
                                let next = self.pending.take().unwrap_or(Out::Silence);
                                self.start(next);
                                continue;
                            }
                            // "shall finish the current Jd before sending Jp"
                            // (9.5.1.1.8) and the same for Jp': a whole
                            // repetition, not a bit count.
                            _ => match self.pending.take() {
                                Some(next) => {
                                    self.start(next);
                                    continue;
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
                Out::Dil => {
                    if self.dil.is_empty() {
                        let next = self.pending.take().unwrap_or(Out::Silence);
                        self.start(next);
                        continue;
                    }
                    let at = self.count % self.dil.len();
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
                    if !bar
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
                Out::Trn2d => {
                    if self.count == TRN2D_FRAMES * INTERVALS {
                        // 9.6.1.1 sends at least this much TRN2d and the
                        // exchange of SUVd and CPd follows it (V92-35). Until
                        // that lands, the line stays on TRN2d's own signal
                        // rather than inventing one.
                        self.count = 0;
                    }
                    self.count += 1;
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
    /// Consecutive zero bits decoded, which is how E1u is seen (8.5.2).
    zeroes: usize,
    /// Bits the Ja reader has produced, and ones in the last stretch of them.
    ja_bits: u64,
    ja_ones: u64,
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
    /// Largest sample seen since the last trace line.
    peak: f64,
    notes: Vec<String>,
}

impl Modem {
    /// From the end of Phase 2, silent until the analogue modem says Ru.
    #[must_use]
    pub fn new(settings: Settings) -> Self {
        let ja_deadline = Some(((JA_WITHIN + settings.round_trip) * FS) as u64);
        Self {
            settings,
            source: Source::new(&settings),
            rx: Receiver::new(),
            stage: Stage::AwaitRu,
            reader: TwoPointReader::differential(),
            ja: DescriptorFinder::v92(),
            cps: CpFinder::v92(JA_UNIT_BITS),
            descriptor: None,
            cpt: None,
            trn_from: None,
            ja_bits: 0,
            ja_ones: 0,
            zeroes: 0,
            now: 0,
            ja_deadline,
            su_deadline: None,
            status: Status::Running,
            md_until: None,
            md_done: false,
            circuit107: false,
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
            Stage::Dil => "V.92 phase 3: DIL",
            Stage::Ri => "V.92 phase 3: Ri",
            Stage::RiBar => "V.92 phase 3: R-bar-i",
            Stage::Phase4 => "V.92 phase 4: TRN2d",
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
        self.rx.feed(input);
        while let Some(heard) = self.rx.heard() {
            self.heard(heard);
        }
        self.timers();
        let out = self.source.next();
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
                self.notes.push(format!("V.92: TRN1u trained at {:.1} dB", self.rx.snr_db()));
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
        self.notes.push(format!("V.92: {why}"));
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
                    self.notes.push(format!("V.92: Su at {at}"));
                }
            }
            Heard::SuReversal { at } => self.su_reversal(at),
            Heard::Trained { snr_db } => {
                if self.stage == Stage::Training {
                    self.notes.push(format!("V.92: TRN1u trained at {snr_db:.1} dB"));
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
                    self.notes.push("V.92: waiting out the MD INFO1a asked for".into());
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
        self.rx.restart_hunt();
        self.notes.push(format!("V.92: TRN1u expected from symbol {from}"));
    }

    fn su_reversal(&mut self, _at: u64) {
        match self.stage {
            Stage::AwaitSu => {
                // D7: the Su-to-S-bar-u reversal, which is what Jp's epsilon
                // is measured against.
                self.stage = Stage::AwaitSuBar;
                let epsilon = 0; // v92::epsilon's reading, which is not built.
                let jp = Jp {
                    epsilon,
                    eight_in_training: self.rx.snr_db() >= 20.0,
                    eight_in_renegotiation: false,
                };
                self.source.jp = jp.to_bits();
                // "shall finish the current Jd and then send Jp" (D8).
                self.source.change(Out::Jp);
                self.notes.push(format!("V.92: Jp with epsilon {epsilon}"));
            }
            Stage::AwaitSuBar => {
                // D9: the (24 + epsilon)T S-bar-u that answers Jp. Circuit 107
                // comes on before Jp' goes out.
                self.circuit107 = true;
                self.source.change(Out::JpPrime);
                self.stage = Stage::Dil;
                // D10: the second TRN1u starts 24T after this reversal and
                // its first symbol is upstream interval 0.
                self.reader = TwoPointReader::differential();
                self.cps = CpFinder::v92(JA_UNIT_BITS);
                self.zeroes = 0;
                self.notes.push("V.92: second S-bar-u, 107 on".into());
            }
            _ => {}
        }
    }

    fn symbol(&mut self, symbol: Symbol) {
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
                if let Some(cp) = self.cps.feed(bit) {
                    self.heard_cp(cp);
                }
                // 8.5.2: E1u is twelve decoded zeroes where a CPt's
                // synchronisation would be. A CPt ends on a twelve-symbol
                // boundary with at least one zero of fill, so only a run
                // longer than that counts.
                self.zeroes = if bit { 0 } else { self.zeroes + 1 };
                if self.zeroes >= E1U_ZEROES && self.cpt.is_some() && self.stage == Stage::Ri {
                    self.notes.push("V.92: E1u".into());
                    self.source.change(Out::RiBar);
                    self.stage = Stage::RiBar;
                }
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
        self.notes.push(format!(
            "V.92: Ja read, {} DIL segments, upstream rates {:05x}",
            descriptor.ucodes.len(),
            self.ja.upstream_rates().unwrap_or(0)
        ));
        let no_dil = descriptor.is_empty();
        self.descriptor = Some(descriptor);
        self.source.change(Out::Sd);
        self.stage = Stage::AwaitSu;
        // Sd starts the downstream half, and DR2 runs from TRN1d's start,
        // which is a fifth of the way into what Sd sends.
        self.su_deadline =
            Some(self.now + ((SU_WITHIN + self.settings.round_trip) * FS) as u64 + TRN1D_FRAMES as u64 * INTERVALS as u64);
        if no_dil && !ZERO_DIL_USES_SCR {
            self.source.change(Out::Ri);
        }
        self.rx.restart_hunt();
        self.ja_deadline = None;
    }

    fn heard_cp(&mut self, cp: Cp) {
        if self.cpt.is_none() {
            self.notes.push(format!("V.92: CPt drn {}, {} set(s)", cp.drn, cp.constellations.len()));
        }
        self.cpt = Some(cp);
        if self.stage == Stage::Dil {
            // 9.5.1.1.12: "finish the current DIL segment and send Ri", which
            // the source does at the next segment boundary.
            self.source.change(Out::Ri);
            self.stage = Stage::Ri;
            self.zeroes = 0;
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
            habits: Habits::PROMPT,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v90::sequences::Layout;
    use crate::v92::up_signals::{ru, su, Trn1u, TwoPointSender};

    fn settings() -> Settings {
        Settings {
            law: Law::Mu,
            uinfo: 79,
            far_md: 0,
            round_trip: 0.0,
            wide: true,
            habits: Habits::PROMPT,
        }
    }

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
                sender.push(&cp.to_bits_in(Layout::V92, 1));
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
