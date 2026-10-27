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
use crate::v92::receiver::{
    ALIGN_WINDOW, Heard, LadderStep, Receiver, Slicer, Symbol, TRAINED_SNR_DB, TRAIN_WINDOW,
};
use crate::v92::sequences::{
    CpFamily, Cpd, J, JP_PRIME_BITS, Jp, Suvd, UpFinder, trn2u_unit,
};
use crate::v34::probe::Reading;
use crate::v92::up_signals::{Trn2uReader, Trn2uSize, TwoPointReader};
use crate::v92::rung::{Bound, Sigma, apply_ceiling, ceiling_rung};
use crate::v92::upchoice::{choose, pin};
use crate::v92::{FilterLimits, LevelSet, Parameters, PeerCp, PeerSuv, SequenceKind, Trellis};

/// The network's rate: one codeword per 125 microseconds (5/V.92).
pub const FS: f64 = 8000.0;

/// The downstream data frame interval count, which V.92 does not change.
const INTERVALS: usize = 6;

/// Sd and S-bar-d, in frames: 384T and 48T (V.90 8.4.4).
const SD_FRAMES: usize = 64;
const SD_BAR_FRAMES: usize = 8;

/// 9.5.1.1.3's wait between reading `Ja` and putting `Sd` on the line, in
/// seconds: "After receiving a DIL descriptor of Ja, the digital modem **may
/// wait for up to 500 ms** and shall then transmit signal Sd for 384T and
/// signal S-d for 48T" (`v92.txt:2535-2536`; the same words are at
/// `v92.txt:2519-2520` in the copy docs/206 quoted from).
///
/// It is a *permission* and not an obligation: the digital modem may use any
/// part of it, and this end used none of it until the wait became a number
/// ([`Habits::sd_after_ja`]). The point of the clause is that `Sd` need not be
/// jammed into the tail of the far end's `Ja` -- the analogue modem terminates
/// `Ja` only after detecting the `Sd`-to-`S-bar-d` transition (9.5.2.1.3), so
/// how close our `Sd` starts to its `Ja` decides what the far end's own
/// detector has to work with.
///
/// The wait is spent in whole frames of [`INTERVALS`] codewords, so that the
/// signal which follows still begins on a frame boundary: see the silence
/// branch of `Source::next_level` and the reason it must (`Source::next`).
pub const SD_AFTER_JA_WITHIN: f64 = 0.5;

/// One frame of the wait, in milliseconds: [`INTERVALS`] codewords at [`FS`]
/// codewords a second, i.e. 0.75 ms.
pub const SD_FRAME_MS: f64 = INTERVALS as f64 * 1000.0 / FS;

/// R-bar-i: "4 repetitions of the 6-symbol sequence" (V.90 8.6.4).
const R_BAR_FRAMES: usize = 4;

/// `R-bar` in symbols: "R for 384T followed by R-bar for 24T" (9.8.1.1.1,
/// 9.8.1.1.4) is four repetitions of six.
const R_BAR_SYMBOLS: usize = R_BAR_FRAMES * INTERVALS;

/// How many symbols off the phase `Ru` locked on say the far end has turned
/// `Ru` into `R-bar-u`.
///
/// **[inferred]**, and the one number the fiducial below rests on: 8.5.5's
/// `R-bar-u` is `{-LU x3, +LU x3}`, so the first *three* symbols after the
/// transition disagree with `Ru`'s phase and no later symbol does. Waiting for
/// a whole period would put `Rd` on the line at the end of the 24T rather than
/// near its start.
const RU_BAR_DEVIATIONS: usize = 3;

/// Where the far end's renegotiation `TRN2u` begins, in symbols after this end
/// sees the `Ru`-to-`R-bar-u` transition.
///
/// 9.8.2.1.1 puts `Ru` at 384T and `R-bar-u` at 24T, and 9.8.2.2.3 (responding)
/// and 9.8.2.1.2 (initiating) put `TRN2u` straight after `R-bar-u`'s 24T. This
/// end's detector fires on the [`RU_BAR_DEVIATIONS`]-th disagreeing symbol, so
/// `R-bar-u` has `24 - 3` symbols left to run when it does, and the far end's
/// `TRN2u` starts here.
///
/// It is a **fiducial and not a guess**: the reader for the far end's `SUVu` is
/// built on this symbol, and one symbol either way would leave its descrambler
/// on the wrong phase and the `SUVu` unreadable -- the same silent failure the
/// downstream frame grid has (`Source::next`).
///
/// **[inferred]** from 9.8.2.1.1/9.8.2.2.3 plus [`RU_BAR_DEVIATIONS`]; no
/// signal is involved that 8.5.5 and 8.7.6 do not define.
pub const TRN2U_AFTER_RU_BAR: u64 = (R_BAR_SYMBOLS - RU_BAR_DEVIATIONS) as u64;

/// Whether this end asks for 9.8.1.1.3's silent period in its own `SUVd`.
///
/// **[our choice]**, and the clause leaves it open twice over: 9.8.1.1.2 enters
/// the silent-period branch only "if bit 32 is set in **either** `SUVd` or
/// `SUVu`", and 9.8.1.1.5 has the digital modem "wait to receive another
/// `SUVu`" when the bit is clear. `false` is Figure 15's shape -- "Rate
/// renegotiation with no silence initiated by the digital modem": `SUVd`,
/// `SUVd`, `CPd` -- which reaches the `CPd` without a handshake whose only
/// purpose is a retrain of `Rt` this end does not need, since what is changing
/// is the far end's upstream and not this end's downstream. The far end can
/// still take this end into the branch by setting bit 32 in its own `SUVu`, and
/// that path *is* implemented and tested; the alternative reading is `true`,
/// which puts 9.8.1.1.3's `SUVd'`, `Ed`, silence and `Rt` on every
/// renegotiation, as Figures 16 and 17 draw.
const RENEG_REQUEST_SILENCE: bool = false;

/// Rd and Rt, in frames: "transmit signal Rd for 384T and then R-bar-d for
/// 24T" (9.8.1.1.1, 9.8.1.2.2), and the same pair for Rt (9.8.1.1.4, .5).
///
/// 384T is 64 data frame intervals of [`INTERVALS`] codewords, the length
/// 8.4.4/V.90 already gives Sd; 24T is [`R_BAR_FRAMES`] repetitions of the
/// six-symbol pattern, which is exactly the 24T 9.8.1.1.1 asks for. Neither is
/// a free parameter of this end's: they are counted out of the clause.
const R_FRAMES: usize = 64;

/// The number of data frame *intervals* a downstream data frame carries, which
/// is what V.90 8.6.4 counts Rd's codewords over ("the data mode constellation
/// of each data frame interval"). V.92 leaves it at twelve (6.4).
const DOWN_INTERVALS: usize = 12;

/// The period of `Rf`'s sign pattern, in symbols.
///
/// 8.8.4 (`v92.txt:1957-1959`): "**Rf** is transmitted by repeating the
/// 12-symbol sequence containing the PCM codewords with the sign pattern
/// `+ + - - + + - - + + - -`", and "**R-bar-f** consists of 2 repetitions of
/// the 12-symbol sequence ... with the sign pattern
/// `- - + + - - + + - - + +`". Both are `+ + - -` repeated, so the pattern's
/// period is four symbols and its phase is fixed by "the left-most sign is
/// transmitted first": the first two symbols of `Rf` are positive and the next
/// two negative. (Read off the source line, not from memory: an earlier ticket
/// nearly shipped `- - - + + -` for V.90's `R-bar`, a PDF hyphenation
/// artefact of the em dashes.)
const RF_SIGN_PERIOD: usize = 4;

/// TRN2d in a rate renegotiation: "shall then transmit TRN2d for **up to**
/// 16008T" (9.8.1.1.2).
///
/// "Up to" is a ceiling and not a target: 16008T is 2668 data frame intervals,
/// and a digital modem that sent exactly that on every renegotiation would
/// spend a third of a second per rung. What the ceiling bounds is how long the
/// far end may be kept in TRN2d before SUVd starts; this end cuts TRN2d at the
/// first frame boundary at or after [`Source::trn2d_symbols`], and the test
/// pins that the emitted run is within the ceiling.
const TRN2D_MOST: usize = 16008;

/// What 8.5.5 says a run of `Ru` looks like, as a sign pattern: "repeating the
/// 6-symbol sequence {+LU, +LU, +LU, -LU, -LU, -LU}".
///
/// One period, as `true` for positive. `Ru`'s bar is the negation, which is the
/// same array read inverted.
const RU_PERIOD: [bool; 6] = [true, true, true, false, false, false];

/// How many periods of `Ru` this end wants to see before it believes the far
/// end is renegotiating.
///
/// **[our choice]**, and the one threshold in the detector. 9.8.2.1.1 gives the
/// far end 384T of `Ru` to spend, so a detector may take as long as it likes
/// inside that; four periods is 24 of those 384 symbols. The bound in the other
/// direction is false positives: a data mode symbol is a legal point of the
/// data constellation, so a *six*-symbol match on signs alone happens by
/// chance one time in sixty-four, and this end would answer a lucky stretch of
/// data with `Rd`. Four periods on signs *and* constant magnitude is what
/// 8.5.5's "the same structure used while transmitting 2 point TRN1u signal"
/// buys -- a two-point signal holds |level| fixed for all six symbols, which
/// the differential encoder in data mode does not.
const RU_LOCK_PERIODS: usize = 4;

/// How far the six magnitudes of one period may drift and still be the same
/// two-point signal, as a fraction of the louder one.
///
/// **[our choice]**. A two-point signal is exact in the ideal; this is the
/// slack a real line's noise and this end's own AGC need. Wide enough that a
/// `Ru` read through a noisy line still locks, narrow enough that a data mode
/// symbol -- whose intervals differ in magnitude by far more than this -- does
/// not.
const RU_MAGNITUDE_SLACK: f64 = 0.35;

/// How much of a window the period-six component must account for before the
/// far end is taken to be sending `Ru`, as the amplitude of the six-symbol
/// component over the window's rms. **`[our choice]`**, and it is the one
/// threshold that reads 8.5.5 off the line instead of off the clause's ideal
/// waveform.
///
/// **Why a second reading of 8.5.5 is needed at all.** `RU_MAGNITUDE_SLACK`
/// above asks for six *equal* magnitudes, which is what 8.5.5's
/// `{+LU, +LU, +LU, -LU, -LU, -LU}` is at the modulator's input. What the Far
/// end actually puts on the line is that sequence as a period-six wave, and the
/// wave comes back through the far end's own transmit path. Measured on
/// `/home/hikari/runs-214/164250-sd108ab170-r214/agr-tx.raw`, the far end's
/// `Ru` at record sample 192 090 is **three** magnitudes, not one --
/// 4 683 / 4 650 / 33 -- because the only component of a 1 333.33 Hz
/// ({fs}/6) period-six wave left below Nyquist is the fundamental: the
/// measured 6-symbol pattern is a pure 5 389-amplitude sine at a constant
/// -30.4 deg for 46.5 ms, third harmonic 0.3 against a fundamental of 5 388.8.
/// A detector that insists on six equal magnitudes can therefore never fire on
/// this far end, which is why `docs/224`'s `Ru detected` was never printed.
///
/// The two forms bracket the threshold: a raw `{+ + + - - -}` square wave has a
/// six-symbol component of 4/pi = 1.27 times its rms, the fundamental-only form
/// has sqrt(2) = 1.414, and data mode levels -- measured over the same file,
/// three separate data stretches, every third symbol -- reach 0.831 at
/// [`RU_TONE_SYMBOLS`] and no more. The threshold sits 1.3 times above the
/// third and 1.15 times below the first.
const RU_TONE_DOMINANCE: f64 = 1.10;

/// Symbols of history the period-six reading is taken over.
///
/// **[our choice]** 48, eight periods of 8.5.5's six. It is the shortest window
/// that separates the two populations by a clear margin, and it is bounded from
/// below by a race that is measured and not hypothetical: 9.6.1.1.2's
/// data-mode-loss detector (`DATA_MODE_LOSS_FRAMES` of twelve symbols) calls
/// `Ru` "a symbol outside the constellation CPd declared" and fires 96 symbols
/// in -- `runs-223/181911-ab170-fpe-v92` did exactly that at 25.000 and sent
/// `Ri` at a far end that was asking for a rate renegotiation. Forty-eight
/// symbols is 6 ms of 8.5.5's 384T, half the loss detector's own latency, and
/// leaves 336 for `R-bar-u` and the frame boundary `Rd` waits for.
///
/// The false-positive bound at this length: over three data stretches of the
/// same recording the six-symbol component reaches 0.831. At 24 symbols it
/// reaches 0.987 and at 96 it falls to 0.561 -- 48 is where the loss-detector
/// race and that spread cross.
const RU_TONE_SYMBOLS: usize = 48;

/// 9.8.1.2.1's receiver conditioning, as a small state machine.
///
/// "After detecting `Ru`, the digital modem shall clamp circuit 104 to binary
/// one and **condition its receiver to detect the `Ru`-to-`R-bar-u`
/// transition**" (9.8.1.2.1). Two states, because the clause asks for two
/// things in order: `Ru` seen, then the transition away from it.
///
/// The detector is a **side channel**. It is fed the same received levels the
/// B1u decoder is fed and it returns nothing to that decoder, so a call with no
/// renegotiation in it puts exactly the same symbols through exactly the same
/// arithmetic it did before this existed. `no_ru_in_data_mode_never_fires_the_
/// responder` pins that.
#[derive(Debug, Clone)]
pub struct RuDetector {
    /// The signs of the last four `Ru` periods, one bit a symbol, oldest in the
    /// most significant position: 1 positive.
    ///
    /// A shift register and not a six-symbol window, because [`RU_PERIOD`] has
    /// period six and a window slid one symbol at a time only equals it on one
    /// of every six symbols. Matching it *at whichever phase the far end
    /// started on* is what [`RU_SHIFTED`] is for.
    signs: u32,
    /// The magnitudes of the last [`RU_PERIOD`] symbols, for the two-point
    /// test.
    magnitudes: [f64; 6],
    /// Symbols seen since the last reset, saturating at 24.
    filled: usize,
    /// 9.8.1.2.1's conditioning is in force: `Ru` has been detected, and what
    /// is wanted now is the `Ru`-to-`R-bar-u` transition.
    locked: bool,
    /// The phase of the six-symbol cycle `Ru` locked on, 0..6.
    phase: usize,
    /// Consecutive symbols since then that disagree with that phase.
    deviations: usize,
    /// The last [`RU_TONE_SYMBOLS`] levels, oldest at [`Self::filled`]'s ring
    /// position. Used only for 8.5.5's period-six reading.
    recent: [f64; RU_TONE_SYMBOLS],
    /// Where the next level goes in [`Self::recent`].
    at: usize,
    /// The six-symbol component's phase when the period-six reading locked.
    tone_phase: Option<f64>,
    /// Symbols the period-six reading has held the far end to `Ru` for.
    tone_run: usize,
    /// The period-six reading has already reported the end of `Ru`, once.
    tone_fired: bool,
    /// Symbols fed, saturating at [`RU_TONE_SYMBOLS`]: the ring is only a
    /// window once it has been filled once.
    seen: usize,
}

impl Default for RuDetector {
    fn default() -> Self {
        Self {
            signs: 0,
            magnitudes: [0.0; 6],
            filled: 0,
            locked: false,
            phase: 0,
            deviations: 0,
            recent: [0.0; RU_TONE_SYMBOLS],
            at: 0,
            tone_phase: None,
            tone_run: 0,
            tone_fired: false,
            seen: 0,
        }
    }
}

/// The 24 symbols of four `Ru` periods, oldest symbol in the top bit.
const RU_LOCKED: u32 = 0b111_000_111_000_111_000_111_000;

/// Every one of the six phases [`RU_LOCKED`] can legitimately start on.
///
/// "the 6-symbol sequence {+LU, +LU, +LU, -LU, -LU, -LU}" (8.5.5) is a *cyclic*
/// sequence -- the far end begins it at a data frame boundary, which is not a
/// boundary of this end's slicing -- so a receiver that only recognised one
/// phase would miss five renegotiations in six.
/// One `Ru` period, oldest symbol in bit 5, and its six phases.
///
/// The transition of 9.8.1.2.2 is detected on *one* period and not on four:
/// 9.8.2.1.1 gives the far end 24T of `R-bar-u`, which is four periods, and a
/// receiver that needed all four would put `Rd` on the line at the last symbol
/// of it rather than at its start.
const RU_ONE: u32 = 0b111_000;
const RU_ONE_SHIFTED: [u32; 6] = [
    RU_ONE,
    (RU_ONE << 1 | RU_ONE >> 5) & 0x3F,
    (RU_ONE << 2 | RU_ONE >> 4) & 0x3F,
    (RU_ONE << 3 | RU_ONE >> 3) & 0x3F,
    (RU_ONE << 4 | RU_ONE >> 2) & 0x3F,
    (RU_ONE << 5 | RU_ONE >> 1) & 0x3F,
];

const RU_SHIFTED: [u32; 6] = [
    RU_LOCKED,
    ((RU_LOCKED << 1) | (RU_LOCKED >> 23)) & 0xFF_FFFF,
    ((RU_LOCKED << 2) | (RU_LOCKED >> 22)) & 0xFF_FFFF,
    ((RU_LOCKED << 3) | (RU_LOCKED >> 21)) & 0xFF_FFFF,
    ((RU_LOCKED << 4) | (RU_LOCKED >> 20)) & 0xFF_FFFF,
    ((RU_LOCKED << 5) | (RU_LOCKED >> 19)) & 0xFF_FFFF,
];

impl RuDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether 9.8.1.2.1's conditioning is in force.
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Forget everything: no `Ru` is in progress.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// One received data mode symbol, as the level the slicer saw.
    ///
    /// `Ru` is "the same structure used while transmitting 2 point TRN1u
    /// signal" (8.5.5), so it is read off level rather than off bits: the B1u
    /// decoder is slicing these same symbols against the data constellation,
    /// and a two-point signal is not a legal codeword sequence of that
    /// constellation in general. Nothing here touches that decoder.
    pub fn feed(&mut self, level: f64) -> RuEvent {
        let magnitude = level.abs();
        self.signs = ((self.signs << 1) | u32::from(level >= 0.0)) & 0xFF_FFFF;
        self.magnitudes.rotate_left(1);
        self.magnitudes[5] = magnitude;
        self.filled = (self.filled + 1).min(24);
        self.recent[self.at] = level;
        self.at = (self.at + 1) % RU_TONE_SYMBOLS;
        self.seen = (self.seen + 1).min(RU_TONE_SYMBOLS);
        // 8.5.5's period-six structure, read as the wave the far end actually
        // sends it as. Checked first because it is the reading that survives
        // the far end's transmit path (`RU_TONE_DOMINANCE`), while the
        // two-point magnitude test below is the reading that survives an ideal
        // modulator. A `Ru` satisfies whichever of the two its transmitter
        // leaves intact; a data mode symbol satisfies neither.
        if self.seen >= RU_TONE_SYMBOLS && !self.tone_fired {
            match self.period_six() {
                RuEvent::None => {}
                event => return event,
            }
        }
        if self.filled < 24 {
            return RuEvent::None;
        }
        // 8.5.5's two-point structure, one period at a time: all six magnitudes
        // of a period within a fraction of each other. A data mode symbol has
        // the intervals of its own constellation, which are not.
        let (loud, quiet) = self
            .magnitudes
            .iter()
            .fold((0.0_f64, f64::MAX), |(hi, lo), &m| (hi.max(m), lo.min(m)));
        if quiet <= 0.0 || (loud - quiet) / loud > RU_MAGNITUDE_SLACK {
            return RuEvent::None;
        }
        let ru = RU_SHIFTED.contains(&self.signs);
        if self.locked {
            // 9.8.1.2.1's conditioning: the transition *away* from `Ru` is
            // what 9.8.1.2.2 releases `Rd` on, and it is only a transition
            // because `Ru` came first -- which is what `locked` records.
            //
            // The **phase** is what makes it a transition. 8.5.5's pattern is
            // three positives then three negatives, so its negation is the
            // same cyclic sequence started three symbols later: `{+ + + - - -}`
            // read from offset 3 *is* `{- - - + + +}`. A receiver that only
            // compared patterns would therefore see no difference at all
            // between `Ru` and `R-bar-u`, which is exactly why 9.8.1.2.1
            // conditions the receiver rather than merely watching for a
            // sequence. What is tracked here is the phase `Ru` locked on; a
            // symbol that disagrees with it is the far end flipping over.
            let want = (RU_SHIFTED[self.phase] >> 23) & 1;
            self.phase = (self.phase + 1) % 6;
            if u32::from(level >= 0.0) == want {
                self.deviations = 0;
            } else {
                self.deviations += 1;
                // Three symbols off the locked phase is half a period and is
                // not noise: `R-bar-u` is 24T, so waiting for a whole period
                // would put `Rd` on the line at the end of it rather than at
                // its start.
                if self.deviations >= RU_BAR_DEVIATIONS {
                    return RuEvent::RuToRuBar;
                }
            }
            return RuEvent::None;
        }
        if ru {
            self.locked = true;
            // The phase `Ru` locked on, so that the transition can be read
            // against it.
            self.phase = RU_SHIFTED.iter().position(|&p| p == self.signs).unwrap_or(0);
            self.deviations = 0;
            return RuEvent::RuDetected;
        }
        RuEvent::None
    }

    /// 8.5.5's `Ru` read as a period-six wave: the amplitude and phase of the
    /// six-symbol component over [`RU_TONE_SYMBOLS`] symbols, against the
    /// window's rms.
    ///
    /// "Signal `Ru` is transmitted by repeating the 6-symbol sequence
    /// {+LU, +LU, +LU, -LU, -LU, -LU}" (`v92.txt:1325`) is a statement about
    /// period, not about harmonics, and the detector may hold the far end to
    /// what the clause guarantees: one period is six symbols, so the wave has
    /// exactly one component below the Nyquist of a symbol rate of one symbol a
    /// sample. At 8 000 symbols a second that component is at 1 333.33 Hz, and
    /// that is what the bench's far end puts on the line (`RU_TONE_DOMINANCE`).
    ///
    /// The transition of 9.8.1.2.1 is read the same way. `R-bar-u` is the
    /// negation, which for a single component is a 180-degree step of its
    /// phase -- and, measured, this far end does not send one at all (`docs/224`
    /// [verified]: phase constant to 0.3 degree over its 384T of `Ru`, then
    /// data). So the end of `Ru` is taken as the transition too, because a far
    /// end that spends its 384T and goes back to data has made the same
    /// statement in the only way its transmitter makes it.
    fn period_six(&mut self) -> RuEvent {
        let n = RU_TONE_SYMBOLS as f64;
        let (mut re, mut im, mut power) = (0.0_f64, 0.0_f64, 0.0_f64);
        for k in 0..RU_TONE_SYMBOLS {
            let v = self.recent[(self.at + k) % RU_TONE_SYMBOLS];
            let theta = -2.0 * std::f64::consts::PI * k as f64 / RU_PERIOD.len() as f64;
            re += v * theta.cos();
            im += v * theta.sin();
            power += v * v;
        }
        let rms = (power / n).sqrt();
        if rms <= 0.0 {
            return RuEvent::None;
        }
        let amplitude = 2.0 * (re * re + im * im).sqrt() / n;
        let phase = im.atan2(re);
        if amplitude / rms >= RU_TONE_DOMINANCE {
            if self.tone_phase.is_none() {
                self.tone_phase = Some(phase);
                self.tone_run = 0;
                if !self.locked {
                    self.locked = true;
                    self.phase = 0;
                    self.deviations = 0;
                    return RuEvent::RuDetected;
                }
                return RuEvent::None;
            }
            self.tone_run += 1;
            let turned = (phase - self.tone_phase.unwrap_or(phase)).abs();
            let turned = turned.min(2.0 * std::f64::consts::PI - turned);
            if turned > 2.0 * std::f64::consts::PI / 3.0 {
                self.tone_fired = true;
                self.tone_phase = None;
                return RuEvent::RuToRuBar;
            }
            return RuEvent::None;
        }
        // The six-symbol component is gone. If it held for a period at least,
        // the far end has left `Ru` without inverting it: 9.8.1.2.2's
        // transition, in the only form this far end sends it.
        if self.tone_phase.is_some() {
            let held = self.tone_run;
            self.tone_phase = None;
            if held >= RU_PERIOD.len() {
                self.tone_fired = true;
                return RuEvent::RuToRuBar;
            }
        }
        RuEvent::None
    }
}

/// What a run of received symbols says about rate renegotiation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuEvent {
    /// Nothing yet: this symbol neither completed `Ru` nor left it.
    None,
    /// 9.8.1.2.1: `Ru` detected, the receiver conditioned, circuit 104 clamped.
    RuDetected,
    /// 9.8.1.2.2: the `Ru`-to-`R-bar-u` transition, which is what releases
    /// `Rd`.
    RuToRuBar,
}


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

/// 9.5.2.1.7's S-bar-u, in symbols and tenths: 24.5T. [`BLIND_SU`] counts from
/// here, because a far end that never reverses never sends it either.
const SU_BAR_TENTHS: u64 = 245;

/// How far behind the onset the hunt's `Su` report is when it names a location:
/// [`HELD_PERIODS`](crate::v92::receiver) periods of the six-symbol shape, give
/// or take where in the period the anchor was taken.
const SU_REPORT_LAG: u64 = 48;

/// How far past the `Su` **onset** Phase 3 keeps to the far end's timetable
/// before it goes on without 9.5.2.1.7's `S-bar-u` (docs/131).
///
/// 9.5.2.1.6 and 9.5.2.1.7 make the far end's obligation a **timetable** --
/// Su for 144T (8.5.6), `S-bar-u` for 24.5T, then Su again until Jp -- and the
/// Recommendation puts **one** deadline on this end: 9.5.1.2.2's 5.1 s plus a
/// round trip, for Su's *arrival*. There is none for *detecting* the
/// `S-bar-u`, so waiting for an event a band-limited line cannot present is a
/// wait the Recommendation never asked for; the schedule is the permission:
/// 144T + 24.5T, plus the two periods the hunt's anchor may take to be sure of
/// a step ([`STRAY_PERIODS`](crate::v92::receiver)), counted from the onset
/// (`S0`, see `Modem::su_advance`) and not from the hunt's report of it.
///
/// Measured: this bench's Agere reverses nine periods after its Su is reported
/// (54 symbols, `hsf-su.md` section 3), well inside the schedule, so the
/// detector confirms it there and this bound is never reached on a compliant
/// peer. The HSF reverses at 144T on the `BRIDGE_ANTIALIAS=0` arm, but on the
/// default `minphase` arm its decimator removes Su's 4 kHz component (-78 dB at
/// 4.0 kHz) and there is no step on the line for any detector to see; the hunt
/// itself is scale-invariant (docs/122), so "wait for the event" is the wrong
/// criterion on every band-limited line and this is the right one (docs/131).
const SU_TIMETABLE: u64 = crate::v92::SU_SYMBOLS as u64
    + SU_BAR_TENTHS / 10
    + 2 * crate::v92::up_signals::RU_PERIOD as u64;

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

/// How far past the point where the upstream stopped repeating every six
/// samples the second TRN1u's first symbol may lie, and the step the ladder of
/// fits walks that stretch on.
///
/// A conforming far end's TRN1u begins at once: 9.5.2.1.10 puts it (24 + epsilon)T
/// after the S-bar-u, and the S-bar-u is itself one period of six, so the point
/// [`Receiver::su_ends`] measures is the point TRN1u starts at. A far end that
/// answers a Jp it was never asked for answers from its own clock instead, and
/// on this bench's HSF the gap between its tone stopping and its TRN1u's first
/// fit is measured at seventy-one symbols (`20261025-173736-hsf-bl1`, 12.9686 s
/// to 12.9806 s, and the fit's own plateau begins at 12.9757 s).
///
/// 128 is that gap plus a whole feed-forward section, because the fit cannot see
/// TRN1u until the section can be a delay line -- which is also why the step is
/// eight: the plateau is [`TRAIN_TAPS`] wide, so eight always lands on it. The
/// ladder is therefore seventeen fits.
const REFIT_SWEEP: u64 = 128;
const REFIT_LADDER: u64 = 8;

/// The fit the timetable's estimate has to read for the refit to keep the
/// alignment it gives, in decibels: the receiver's own gate for "a linear fit of
/// the second TRN1u describes this line well enough to read a two-point CPt"
/// ([`TRAINED_SNR_DB`]).
///
/// Both figures are the same measurement -- one least-squares fit of the line
/// against the sequence this end generates, over the same window -- so the
/// comparison between them is fair. What this decides is *whose* alignment is
/// trusted, and the tie goes to the estimate, because it is the one the 3-of-3
/// Agere call is measured on: on this bench that call's estimate fits at 13.5 dB
/// and its measured alignment at 15.8, and both read CPt, so a working far end's
/// frame origin is not moved to gain two decibels. The HSF's estimate fits at
/// 6.4 dB -- under the gate, which is why its CPt was never read at all -- and
/// its measured alignment at 14.5.
const REFIT_ESTIMATE_DB: f64 = TRAINED_SNR_DB;

/// How far back the end of the six-symbol upstream is looked for, and the window
/// and the hold its measurement is taken with.
///
/// The refit fires [`REFIT_WINDOW`] in and will wait at most [`REFIT_WAIT`], so
/// four thousand covers both with the history's own second in hand. The window
/// is four periods of six (24 symbols, 3 ms): long enough for the measure to sit
/// at 1.000 on Su and at a third of that on TRN1u -- measured, it reads 1.000 to
/// 12.9685 s and 0.16 by 12.9695 s -- and short enough to put the boundary
/// inside four symbols. The hold is two periods, so one lost period inside Su is
/// not the end of it.
const REFIT_MEASURE: u64 = 4_096;
const SU_ENDS_WINDOW: usize = 24;
const SU_ENDS_HOLD: usize = 48;

/// How long the refit will wait, after the reversal, for a measured second
/// TRN1u before taking the line as it stands.
///
/// 300 ms is what this bench measures a far end taking to answer a Jp it did not
/// expect (the HSF), and it is the same allowance the `Su` tolerance makes. The
/// wait is not free -- a CPt that arrives during it cannot be read -- so it is
/// bounded rather than open-ended: past it the refit runs exactly where the
/// timetable says, which is what it did before the measurement existed.
const REFIT_WAIT: u64 = 2_400;

/// How often the refit looks at the line while it is waiting, in symbols.
///
/// The boundary is found by scanning history, which costs a few hundred thousand
/// multiplies: affordable once, not once a symbol. Sixty-four symbols is eight
/// milliseconds, a tenth of the resolution the wait is worth.
const REFIT_POLL: u64 = 64;

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

/// The clause's own allowance for the B1u hunt, in seconds of line time:
/// 9.6.1.2.1 -- *"If the digital modem does not receive B1u within 20 s plus 6
/// round-trip delays from the end of INFO1a, the digital modem shall initiate a
/// retrain according to 9.7.1.1"* (`v92.txt:2770`). The analogue modem's
/// symmetric half is 9.6.2.2.1 (`v92.txt:2799`).
///
/// **This used to be 2.0 s, counted from Ed, and that was ours, not the
/// clause's.** The clause bounds the *whole* of Phase 3 plus Phase 4 from the
/// end of INFO1a -- `analogue.rs`'s `START_UP` reads it the same way, and its
/// comment says so ("the timer that covers all of Phase 3 and Phase 4"). Two
/// seconds after Ed is a limit the standard never imposed: shorter than the
/// clause by an order of magnitude, and on `docs/289`'s `drn 2` call it ended
/// the hunt at 25.180 s while this end's candidates were still the wrong ones.
///
/// The anchor matters as much as the number. Because the clause counts from the
/// **end of INFO1a**, an arm at Ed must spend what is *left* of the clause's
/// budget, not a fresh one: `start_up_by` is absolute in `now` units, exactly
/// like `ja_deadline`, whose clause (9.5.1.2.1) has the same origin. A wait
/// restarted at Ed would exceed the clause, which is the other way to get this
/// wrong. Renegotiation re-anchors `start_up_origin`, because 9.8's exchange is
/// a new Phase 4 with its own reference.
///
/// What it is *not*: an invitation to condition the receiver on a timer. It
/// used to do that "regardless", which is the one thing a timer cannot know:
/// B1u's frame interval 0 is a symbol of the far end's, not of this end's clock,
/// and a receiver that conditions on its own timer reads a data frame from a
/// boundary that never existed. Measured on `/tmp/p0corr/g712.raw`, that path
/// delivered 14 kB of wrong bytes and dropped a V.42 that never opened; a call
/// that cannot find B1u still fails in the open, where the retrain and the
/// fallback the start-up already has can act on it. A longer hunt only moves the
/// deadline; it never invents a boundary.
pub const B1U_WAIT: f64 = 20.0;

/// 9.6.1.2.1's "plus 6 round-trip delays" (`v92.txt:2770`), in round trips.
pub const B1U_WAIT_RTD: f64 = 6.0;

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

/// Symbols in one B1u (8.7.1: forty-eight data frames of twelve symbols).
pub const B1U_SYMBOLS: usize = B1U_FRAMES * crate::v92::UP_INTERVALS;

/// How long E2u is, in symbols (8.7.2), from the flag this end puts in its own
/// CPd -- one place, so that the sequence this end sends and the window the
/// receiver searches for B1u's boundary cannot disagree. See
/// [`crate::v92::upchoice::EXTEND_E2U`].
pub const E2U_SYMBOLS: usize = crate::v92::upchoice::E2U_SYMBOLS;

/// How far back from the run's end the boundary search looks, in symbols.
///
/// [`E2U_SYMBOLS`], which is what makes the candidate set complete (see
/// [`Modem::begin_b1u`]), or zero for `V92_B1U_SEARCH=off` -- the runs's end
/// taken as B1u's frame interval 0 with no search at all, which is what every
/// B1u number published before `/dev/shm/reports/e2u-boundary.md` was taken
/// with and is kept as the arm that measures the search.
fn b1u_deepest() -> usize {
    if std::env::var("V92_B1U_SEARCH").ok().as_deref() == Some("off") { 0 } else { E2U_SYMBOLS }
}

/// The level stream of a candidate's B1u, and where the zero run that opened
/// the candidate ended.
///
/// 8.7.2's E2u is the thing the run is *made of*, and 8.7.1 puts B1u's frame
/// interval 0 on the symbol after E2u's last one -- but the run is read
/// through TRN2u's four points and its descrambler, and the first symbols of
/// B1u do not stop reading as zeros because E2u has ended. So the run's end is
/// where the *candidates* are counted from, not where B1u is taken to begin:
/// see [`Modem::begin_b1u`].
#[derive(Debug, Clone)]
pub struct B1uSearch {
    /// The symbol the run ended on: the first symbol carrying a non-zero bit
    /// after E2u's frame of scrambled zeros. B1u's frame 0 is at or before it.
    pub end: u64,
    /// One level per symbol from `end + 1`, up to [`B1U_SYMBOLS`], with the
    /// same symbols as they arrived beside them: the symbols past the winning
    /// candidate's window are the first symbols of the *data* mode, and they
    /// have to be handed to the decoder that carries the call rather than
    /// dropped (`Modem::b1u_backlog`).
    levels: Vec<f64>,
    symbols: Vec<Symbol>,
}

/// One candidate boundary's reading of B1u: the two decoders 8.7.1's known
/// content is read with, and what they read.
#[derive(Debug)]
struct B1uReading {
    /// Symbols back from the run's end that this candidate puts B1u's frame
    /// interval 0 on. Zero is "the run ended on the last symbol of E2u"; what
    /// the search measures is how far the run overran E2u into B1u.
    back: usize,
    /// Absolute symbol of B1u's frame interval 0, as this candidate claims.
    origin: u64,
    /// Ones each decoder read: `[0]` the line as it arrived, `[1]` it negated.
    ones: [u64; 2],
    /// Bits each of them read.
    scored: [u64; 2],
    /// And the bits themselves, kept for `V92_B1U_BITS` on the winner.
    bits: [Vec<bool>; 2],
    straight: Decoder,
    probe: Decoder,
}

impl B1uReading {
    /// The better of the two polarities -- 8.7.1 makes B1u all ones, so this
    /// is what the boundary is judged on.
    fn best(&self) -> u64 {
        self.ones[0].max(self.ones[1])
    }

    /// The other one, which is the far end's seed read the wrong way up if the
    /// candidate is a boundary at all.
    fn other(&self) -> u64 {
        self.ones[0].min(self.ones[1])
    }

    /// Bits read, whichever polarity read fewer.
    fn scored(&self) -> u64 {
        self.scored[0].min(self.scored[1])
    }
}

/// Whether a candidate's reading is B1u at all: 8.7.1 makes it all ones, so
/// three quarters of the window has to read as ones in the better of the two
/// polarities and that polarity has to lead the other by an eighth of the
/// window (see [`B1U_CONFIRM_EIGHTHS`] for why both).
///
fn reads_as_b1u(reading: &B1uReading) -> bool {
    let scored = reading.scored();
    scored > 0
        && reading.best() * 8 >= scored * B1U_CONFIRM_EIGHTHS
        && reading.best() >= reading.other() + scored / B1U_POLARITY_LEAD
}

/// Which of the candidates is believed, as an index into `readings`.
///
/// The **nearest the run's end** that reads as B1u, not the one that reads it
/// best. 8.7.1 puts B1u's frame interval 0 on the symbol after E2u's last one,
/// so the run's end is the reading that needs no explanation; a boundary
/// further back is a claim that the run overran E2u, and 8.7.2 sanctions
/// exactly one symbol of that (`upchoice::EXTEND_E2U`). The score's maximum on
/// its own cannot carry that claim: twelve symbols back is the *same* frame
/// phase -- twelve is a data frame, and four divides it -- so 8.7.1's content
/// separates the two by only the one frame of differential reflection and the
/// 23 bits of descrambler priming the shift costs, and on a synthesised line
/// with no noise at all the twin reads 569 against the true boundary's 540
/// (which of the two the chain's `d(-1) = 0` suits is not a property of the
/// line). Believing the twin would put the whole data mode twelve symbols out,
/// which no V.42 survives. So the gate answers *whether* a candidate is a
/// boundary and the order answers *which* one.
///
/// `gate_off` has nothing to decide between candidates with, so it reports the
/// best-reading one: that is the instrument's question ("what does the best
/// boundary in the window read"), not the call's.
fn choose_b1u_boundary(readings: &[B1uReading], gate_off: bool) -> Option<usize> {
    if gate_off {
        // Nothing to decide *between* candidates with the gate off: the
        // question that knob leaves is "what does the best boundary in this
        // window read", not "which boundary is it".
        return best_b1u_reading(readings);
    }
    choose_b1u_boundary_by(readings, &reads_as_b1u, false)
}

/// [`choose_b1u_boundary`], with the verdict on one reading passed in.
///
/// The rule is separated from [`reads_as_b1u`] so that it can be tested on a
/// line whose polarities are not the bench's -- a synthesised one has no noise,
/// so the *negated* reading of its B1u is the far end's own pattern read
/// backwards and comes out 476 of 576 ones, which is close enough to the true
/// boundary's 540 that the lead the gate demands is not there. What this rule
/// has to get right is the *order*, not the threshold.
fn choose_b1u_boundary_by(
    readings: &[B1uReading],
    accept: &dyn Fn(&B1uReading) -> bool,
    fall_back_to_best: bool,
) -> Option<usize> {
    if let Some(nearest) = (0..readings.len()).find(|&index| accept(&readings[index])) {
        return Some(nearest);
    }
    fall_back_to_best.then(|| best_b1u_reading(readings)).flatten()
}

/// The best-reading candidate: best polarity, then the wider lead over the
/// other one. Ties keep the nearest, because the comparison is strict.
fn best_b1u_reading(readings: &[B1uReading]) -> Option<usize> {
    let lead = |reading: &B1uReading| reading.best().saturating_sub(reading.other());
    let mut best: Option<usize> = None;
    for index in 0..readings.len() {
        let Some(champion) = best else {
            best = Some(index);
            continue;
        };
        if (readings[index].best(), lead(&readings[index])) > (readings[champion].best(), lead(&readings[champion])) {
            best = Some(index);
        }
    }
    best
}

/// The symbols the committed data decoder still has to be given, in time
/// order: the ones between the end of B1u and the end of the collected window.
///
/// A candidate `back` symbols behind the run's end puts B1u's last symbol at
/// `window - 1 - back`, so the window holds `back` symbols of the far end's
/// *data* past it. The decoders are fed from B1u's frame 0 and stop at the end
/// of the window (`last = B1U_SYMBOLS - 1 - back`), so a decoder fed only the
/// window is fed `back` symbols short and starts the data mode `back` symbols
/// into a data frame -- which is a call that reads B1u perfectly and delivers
/// nothing. This is that `back`-symbol carry-over, and it is empty for the
/// candidate at the run's end.
fn b1u_backlog_symbols(search: &B1uSearch, back: usize) -> Vec<Symbol> {
    let first = B1U_SYMBOLS.saturating_sub(back);
    (first..B1U_SYMBOLS).filter_map(|index| search.symbols.get(index).copied()).collect()
}

/// The two decoders one candidate boundary is read with: B1u as it arrived and
/// B1u negated, built at the candidate's frame 0 and fed from `skip + 1`
/// symbols after it.
///
/// Everything here is a function of the *candidate's* own origin, which is
/// what lets thirteen of them be read at once: the memories 8.7.1 zeroes are
/// zeroed at the candidate's first symbol, the descrambler is primed with the
/// twenty-three bits of 8.7.1's known content that precede the symbol it is
/// fed from, and the class rotation is measured from that symbol.
fn b1u_decoders(parameters: &Parameters, skip: usize) -> (Decoder, Decoder) {
    let mut straight = Decoder::new(parameters);
    let mut probe = Decoder::new(parameters);
    // 8.7.1 makes B1u's payload known, so the far end's differential chain at
    // any of its frame boundaries is computable (`modulus::b1u_differential_at`
    // -- the scrambled ones through 6.4.1's steps 1 to 3, with the moduli they
    // are decoded with). This decoder does not start at B1u's frame 0: it is
    // fed `skip + 1` symbols after the boundary, a whole number of data
    // frames, so the first frame it decodes is B1u's frame
    // `(skip + 1) / 12` and the chain's value there is not the `d(-1)` 8.7.1
    // zeroes.
    //
    // **It is still not seeded by default, and that is a measurement.**
    // Seeding it with the computed value takes B1u from 537 ones of 576 to
    // 503, and every seed from 0 to 47 frames leaves exactly those two
    // outcomes, one per value of d (24 000, `~/v92up-runs/c24-4.agr-tx.raw`,
    // `/tmp/fe/ber` tags `seed-fix`, `seed18`): the two decodes differ in 36
    // bits of 576, all of them in the two frames the seed's own reflection
    // touches, and the better one is the value the untold decoder already
    // holds. So the chain this receiver starts on carries 8.7.1's zero, the
    // far end's d at frame 18 is the *complement* of the clause's chain if the
    // frame is 18 at all, and the bits B1u is short by are not a reflection
    // -- which is what the zero pattern shows: the first frame decoded reads
    // 16 ones of 36 and the other fifteen carry nineteen zeros between them.
    //
    // A second premise was falsified the same way and is recorded here so that
    // nobody re-derives it: sweeping the Viterbi's start state over all 0..63
    // leaves the same 537 at states 0/1/16 and worse elsewhere, so the deficit
    // is not the start state either. **The ceiling of any known-content
    // seeding is 550**; the rest needs the slicing better, not the state.
    //
    // `V92_B1U_SEED_FRAMES=<n>` is that experiment: seed 6.4.1's chain `n`
    // data frames into B1u. It is off unless asked for, and it is worth
    // nothing -- seeding with the computed value takes 537 to 503.
    let seed = std::env::var("V92_B1U_SEED_FRAMES").ok().and_then(|text| text.parse::<usize>().ok());
    if let Some(frames) = seed {
        let moduli = crate::v92::modulus::Moduli12::from(parameters);
        let chain = crate::v92::modulus::b1u_differential_at(frames, &moduli, parameters.bits());
        // The negated probe gets the other value: it reads the same symbols
        // through an inverted line, which inverts 6.4.1's reflection step with
        // it (`modulus`'s module comment), so its chain starts complemented.
        straight.seed_differential(chain);
        probe.seed_differential(!chain);
        if std::env::var_os("V92_RX_BITS").is_some() || tracing() {
            eprintln!(
                "V92B1U seed: frames_skipped={frames} moduli={:?} bits={} d={chain}",
                moduli.values(),
                parameters.bits()
            );
        }
    }
    // The same clause, applied to 6.3's descrambler, and unlike the
    // differential chain above this one is **on by default** because it is
    // worth bits: B1u frame 18 is not B1u frame 0 for the descrambler either,
    // its twenty-three-bit memory is fed entirely by bits 8.7.1 makes known,
    // and scoring the window against a register the far end never had costs
    // the first twenty-three bits of it. Measured on `c24-4` at 24 000:
    // **537 ones without, 550 with, exactly +13** (`/tmp/reports/b1u-seed.md`).
    //
    // This is a defect of the *instrument*, not of the receiver: the data mode
    // path starts at B1u's own first symbol with a zeroed descrambler (8.7.1)
    // and is not primed here. `V92_B1U_PRIME=0` restores the old reading,
    // which is what every published "B1u N/576" before today was taken with.
    if std::env::var("V92_B1U_PRIME").ok().as_deref() != Some("0") {
        let bits = parameters.bits();
        let first = (skip + 1) / crate::v92::UP_INTERVALS;
        let stream = crate::v92::modulus::b1u_scrambled_bits(first, bits);
        let keep = stream.len().min(23);
        let preceding: Vec<bool> = stream[stream.len() - keep..].to_vec();
        straight.seed_descrambler(&preceding);
        // The negated probe reads the same symbols through an inverted line,
        // which complements every bit it hands 6.3's descrambler (`modulus`'s
        // module comment), so its memory is the complement of the same window
        // -- the same treatment `V92_B1U_SEED_FRAMES` gives the differential
        // chain.
        let flipped: Vec<bool> = preceding.iter().map(|bit| !bit).collect();
        probe.seed_descrambler(&flipped);
    }
    // `V92_ROT`: the class rotation is read off B1u rather than taken from
    // `PAIR_FIRST_INDEX_OFFSET`, and it is read off the part of B1u this
    // decoder is actually fed -- the symbol after the acquisition skip,
    // counted from B1u's own first symbol. Off by default.
    //
    // Only the decoder reading the line as received is given it. The polarity
    // measurement compares the two decoders' ones counts, and it can only do
    // that if they differ by polarity and nothing else: given a measured
    // rotation, the negated copy of an Agere reads 529 ones of 576 instead of
    // 450, and the lead the verdict needs is gone (`docs/48 §5`). A call whose
    // polarity turns out inverted therefore keeps the constant -- which is the
    // behaviour it had.
    straight.measure_rotation_from(skip + 1);
    // `docs/264` sec 3: keep the window's decided indices. This is a *record*
    // of the decisions the decoder has already taken -- `keep_decisions` pushes
    // them after `decide` has returned -- so it cannot move a decision, and it
    // is drained once, by `B1uBins::measure`, at the boundary. The decoder that
    // goes on into data mode has it switched off again there, because the
    // alternative is a Vec that grows for the length of the call.
    straight.keep_decisions(true);
    // Above 24 000 B1u's frames *can* be read against 8.7.1's known content
    // instead of against 6.4.1's chain (`docs/74`), and it stops with B1u
    // because nothing after it is known. It is off unless `V92_BY_CONTENT=1`
    // asks for it: it is a workaround for a far end whose chain a payload
    // cannot follow, and the chain is the default.
    straight.read_by_content(true);
    (straight, probe)
}

/// Score 8.7.1's known content at every candidate boundary the run's end
/// allows, and hand them back for the choice.
///
/// `levels[0]` is the level of the symbol *after* the run's end, so candidate
/// `back` claims B1u's frame interval 0 on symbol `end - back` and its
/// decoders read `levels[skip - back]` through `levels[B1U_SYMBOLS - 1 -
/// back]`. The candidate set is `back` in `0..=deepest`, and `deepest` is
/// [`E2U_SYMBOLS`], which is what makes it complete: the run contains all
/// twelve symbols of 8.7.2's E2u, so B1u's frame 0 is at most twelve symbols
/// before the run's end, and every frame phase between there and the run's end
/// is one of the thirteen.
fn b1u_readings(
    parameters: &Parameters,
    end: u64,
    levels: &[f64],
    skip: usize,
    deepest: usize,
) -> Vec<B1uReading> {
    let mut out = Vec::with_capacity(deepest + 1);
    for back in 0..=deepest {
        let origin = end.saturating_sub(back as u64);
        // `skip` is the acquisition window, and it is measured from the
        // candidate's own origin: the candidate that starts `back` symbols
        // earlier has already had `back` of its window read.
        let first = skip.saturating_sub(back);
        let Some(last) = B1U_SYMBOLS.checked_sub(1 + back) else { continue };
        if first > last || last >= levels.len() {
            continue;
        }
        let (mut straight, mut probe) = b1u_decoders(parameters, skip);
        let mut ones = [0u64; 2];
        let mut scored = [0u64; 2];
        let mut bits: [Vec<bool>; 2] = [Vec::new(), Vec::new()];
        for &level in &levels[first..=last] {
            for (which, sign) in [1.0, -1.0].into_iter().enumerate() {
                let decoder = if which == 0 { &mut straight } else { &mut probe };
                decoder.feed(sign * level);
                let read = decoder.take_bits();
                ones[which] += read.iter().filter(|bit| **bit).count() as u64;
                scored[which] += read.len() as u64;
                bits[which].extend_from_slice(&read);
            }
        }
        out.push(B1uReading { back, origin, ones, scored, bits, straight, probe });
    }
    out
}

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
    /// How much of 9.5.1.1.3's window this end uses between reading `Ja` and
    /// putting `Sd` on the line, in seconds: 0 to [`SD_AFTER_JA_WITHIN`].
    ///
    /// **0 is the behaviour every call on this bench had before the number
    /// existed** -- `Sd` starts at the first frame boundary after `Ja` is read
    /// -- so the default changes nothing until a value is chosen. What it is
    /// for is the arrangement of our own transmission in time, which the clause
    /// explicitly leaves to the digital modem; see [`Self::sd_after_ja_frames`]
    /// for the frame grid the wait is kept on.
    pub sd_after_ja: f64,
    /// Jd's bits 49:50, "the maximum spectral shaping look-ahead".
    pub lookahead: u8,
}

impl Habits {
    /// Short TRN1d, for the tests and for the second end of our own call.
    pub const PROMPT: Self = Self { trn1d: 0.3, sd_after_ja: 0.0, lookahead: 1 };

    /// What a live server sends: the analogue modem's equaliser wants the
    /// time, and the clause's own ceiling is four seconds. None of 9.5.1.1.3's
    /// 500 ms is used, which is what this end has always done.
    pub const LIVE: Self = Self { trn1d: 3.6, sd_after_ja: 0.0, lookahead: 1 };

    /// The live server's habits, with 9.5.1.1.3's wait an operator setting
    /// rather than a constant: `BINMODEM_V92_SD_AFTER_JA_MS` if it is set,
    /// [`Self::LIVE`] (zero) otherwise.
    ///
    /// The clause gives a window, not a value -- "may wait for up to 500 ms"
    /// (`v92.txt:2535-2536`) -- so how much of it to use is a policy number and
    /// tuning it must not mean patching a constant. `docs/209` gave TRN1d's
    /// length the same shape for the same reason. Out-of-window and
    /// off-the-grid values are the clamp's business, not the operator's
    /// ([`Self::sd_after_ja_frames`]).
    #[must_use]
    pub fn live() -> Self {
        let mut habits = Self::LIVE;
        if let Ok(text) = std::env::var("BINMODEM_V92_SD_AFTER_JA_MS")
            && let Ok(ms) = text.trim().parse::<f64>()
            && ms.is_finite()
        {
            habits.sd_after_ja = ms / 1000.0;
        }
        habits
    }

    /// 9.5.1.1.3's wait as this end will spend it: **clamped into the clause's
    /// window and snapped to the frame grid**, in frames of [`INTERVALS`]
    /// codewords.
    ///
    /// The grid is not a convenience. Silence occupies frame intervals like
    /// anything else and a signal that starts mid-frame rotates the six-symbol
    /// pattern of everything measured against the frame count (`Source::next`
    /// records what that cost: an Agere SV92PP answered a rotated `Ri` with two
    /// seconds of nothing and then a retrain). A wait of a whole number of
    /// frames leaves the residue exactly where it was and moves only the
    /// placement.
    ///
    /// Clamped, so that no setting can send the wait out of conformance: the
    /// largest whole frame inside 500 ms is 666 frames = 499.5 ms, since 667
    /// frames is 500.25 ms and past the clause.
    #[must_use]
    pub fn sd_after_ja_frames(&self) -> usize {
        let ms = self.sd_after_ja * 1000.0;
        let ms = if ms.is_finite() { ms.clamp(0.0, SD_AFTER_JA_WITHIN * 1000.0) } else { 0.0 };
        let frames = (ms / SD_FRAME_MS).round();
        (frames as usize).min(Self::sd_after_ja_frames_most())
    }

    /// The most frames 9.5.1.1.3's window holds.
    #[must_use]
    pub fn sd_after_ja_frames_most() -> usize {
        (SD_AFTER_JA_WITHIN * FS / INTERVALS as f64).floor() as usize
    }

    /// The wait in codewords, which is what the source counts in.
    #[must_use]
    pub fn sd_after_ja_symbols(&self) -> usize {
        self.sd_after_ja_frames() * INTERVALS
    }
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
    /// Rd, 384T: signal R on the *data mode* constellation of each data frame
    /// interval, "as passed in CP" (V.90 8.6.4, reached from 8.8.4/V.92;
    /// 9.8.1.1.1, 9.8.1.2.2).
    Rd,
    /// R-bar-d, 24T: "4 repetitions of the 6-symbol sequence ... with the sign
    /// pattern - - - + + +" -- the same codewords, the pattern inverted
    /// (V.90 8.6.4).
    RdBar,
    /// Rt, 384T: signal R on the *training* constellation of each data frame
    /// interval, "as passed in CPt" (V.90 8.6.4; 9.8.1.1.4, 9.8.1.1.5).
    ///
    /// Not the same signal as [`Out::Rd`]: a different constellation, and the
    /// one place 9.8 can silently send the wrong level.
    Rt,
    /// R-bar-t, 24T.
    RtBar,
    /// Rf, 384T: 8.8.4's repeated 12-symbol sequence "with the sign pattern
    /// + + - - + + - - + + - -", each symbol "the highest power PCM codeword
    /// from the data mode constellation of each data frame interval as passed
    /// in CPu" (`v92.txt:1957-1962`) -- 9.9.1.2.2's answer to a fast
    /// parameter exchange.
    Rf,
    /// R-bar-f, 24T: "2 repetitions of the 12-symbol sequence ... with the
    /// sign pattern - - + + - - + + - - + +" (8.8.4, `v92.txt:1959-1961`):
    /// the same codewords, the pattern inverted.
    RfBar,
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
    /// Whether the `Ri` on the line is [`Source::ri_opener`]'s and counts its
    /// own 384T out, rather than waiting for an `E1u` that will not come.
    ri_timed: bool,
    /// Codewords of silence still to go out before `pending` may start. It is
    /// 9.5.1.1.3's wait and nothing else; zero everywhere but after `Ja`.
    hold: usize,
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
    /// The last codeword B1d put on the line, and its sign: what the
    /// queue-empty arm repeats rather than emitting a level that is not a data
    /// symbol at all (`docs/221`). Cleared whenever a signal starts.
    last_code: Option<(u8, bool)>,
    /// How long TRN2d lasts: "at least 2040T" (9.6.1.1.1).
    trn2d_symbols: usize,
    /// Set when a Phase 4 sequence has run out of bits: see
    /// [`Source::take_finished`].
    finished: Option<Out>,
    /// Phase 4's own coder, from CPt; `None` until the analogue modem's CPt
    /// has been read, which is the only thing that says what constellation
    /// Phase 4 goes out on (V.90 8.6.5).
    phase4: Option<Down4>,
    /// 8.8.5: the coder a fast parameter exchange's `SUVd`, `CPd` and `Ed` go
    /// out on -- "**During Fast Parameter Exchange it is transmitted using the
    /// preceding data mode modulation**" (`v92.txt:1963-1966`). Built from
    /// **CPu**, the constellation `B1d` was going out on, and not from `CPt`:
    /// a far end in a fast parameter exchange has stayed in the data mode
    /// modulation (9.9 has no `TRN2d` in it), so `CPt`'s coder is the wrong one.
    data4: Option<Down4>,
    /// [`Source::data4`] is the coder in force: a fast parameter exchange is
    /// the procedure in flight rather than 9.8's rate renegotiation.
    fpe: bool,
    /// 9.8's two R signals, already resolved to one codeword per downstream
    /// data frame interval: `Rd` out of the data mode constellation CPd named
    /// and `Rt` out of the training constellation CPt named (V.90 8.6.4).
    ///
    /// Resolved once, at [`Source::arm_renegotiation`], rather than per symbol:
    /// the two constellations are the whole difference between the signals and
    /// a per-symbol lookup is where they would get mixed up.
    reneg: Option<RenegPoints>,
}

/// The two R signals of 9.8, one codeword a data frame interval.
///
/// V.90 8.6.4 keeps them apart on purpose: "**Rd** denotes signal R using the
/// highest power PCM codeword from the **data mode** constellation of each data
/// frame interval as passed in **CP**", while "**Rt** denotes signal R using the
/// highest power PCM codeword from the **training** constellation of each data
/// frame interval as passed in **CPt**". Two different constellations, two
/// different codewords, and 9.8.1.2.2 asks for `Rd` while 9.8.1.1.4 and
/// 9.8.1.1.5 ask for `Rt`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RenegPoints {
    /// Twelve codewords, interval 0 to 11, from CPd's data mode sets.
    pub rd: [u8; DOWN_INTERVALS],
    /// Twelve codewords, interval 0 to 11, from CPt's training sets.
    pub rt: [u8; DOWN_INTERVALS],
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
            hold: 0,
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
            last_code: None,
            trn2d_symbols: TRN2D_LEAST,
            finished: None,
            phase4: None,
            data4: None,
            fpe: false,
            reneg: None,
            ri_timed: false,
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
    /// 8.8.5's **Fast Parameter Exchange** coder: `SUVd`, `CPd` and `Ed` are
    /// "transmitted using the preceding data mode modulation" (`v92.txt:1963-1966`),
    /// so they go out through `CPu`'s encoder and not `CPt`'s.
    ///
    /// Built afresh, which is also 9.9.1.2.3's "initialize the scrambler,
    /// differential encoder and spectral shaping filter memory to zero"
    /// (`v92.txt:3083-3085`) applied to the coder that is actually used: a new
    /// `Encoder` is `Differential::new()` and `Shaper::new()`
    /// (`crates/datapump/src/v90/encoder.rs:154-162`), and there is no reset
    /// anywhere else (`encoder.rs:129-133`). The scrambler is `Source`'s own,
    /// used by [`Source::phase4_codeword`] for the sequences' bits, and it is
    /// zeroed here for the same clause.
    pub fn begin_fpe(&mut self, cpu: &Cp) -> bool {
        let Some(mapping) = crate::v90::encoder::Mapping::from_cp(cpu) else { return false };
        self.fpe = true;
        self.data4 = Some(Down4 { coder: Encoder::new(mapping, self.law), samples: VecDeque::new() });
        self.scrambler.reset();
        true
    }

    /// Back to data mode: the fast parameter exchange is over and `CPt`'s coder
    /// is the corresponding one again for whatever 9.6.1.1 does next.
    pub fn end_fpe(&mut self) {
        self.fpe = false;
        self.data4 = None;
    }

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
        let coder = if self.fpe { self.data4.as_ref() } else { self.phase4.as_ref() };
        coder.is_some_and(|p| !p.samples.is_empty())
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
        // 8.8.5: the same coder, whichever constellation is the "corresponding"
        // one for the procedure in flight -- `CPt`'s during Training and Rate
        // Renegotiation, **the preceding data mode's during Fast Parameter
        // Exchange** (`v92.txt:1963-1966`).
        let mut phase4 = if self.fpe { self.data4.take()? } else { self.phase4.take()? };
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
        if self.fpe {
            self.data4 = Some(phase4);
        } else {
            self.phase4 = Some(phase4);
        }
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

    /// Give 9.8's two R signals their codewords, out of the constellations
    /// V.90 8.6.4 sends them out on.
    ///
    /// `data` is the highest-power codeword of each downstream data frame
    /// interval as **CPd** passed it; `training` is the same out of **CPt**.
    /// The caller resolves both because it is the caller that holds CPd and
    /// CPt; what this keeps is the pair, so that no later symbol can pick the
    /// wrong one.
    pub fn arm_renegotiation(&mut self, data: [u8; DOWN_INTERVALS], training: [u8; DOWN_INTERVALS]) {
        self.reneg = Some(RenegPoints {
            rd: data,
            rt: training,
        });
    }

    /// 9.5.1.1.12's `Ri` for 384T followed by `R-bar-i` for 24T, with the bar
    /// counted out here rather than by an `E1u`.
    ///
    /// In Phase 3 the bar is the far end's to ask for: `E1u` ends the `CPt`
    /// series and this end answers `R-bar-i` (`cpt_bit`), which is why nothing
    /// in the `Ri` arm ever counted 384T. Off the re-entry there is no `CPt`
    /// and no `E1u` -- the far end has simply stopped sending data -- so the
    /// clause's own lengths are the only thing left to run the signal for, and
    /// they are `Ri` for 384T and `R-bar-i` for 24T (`docs/216` §3).
    pub fn ri_opener(&mut self) {
        self.ri_timed = true;
        self.change(Out::Ri);
    }

    /// 9.8.1.1.2's "up to 16008T": how long TRN2d may run before the `SUVd`
    /// the clause puts after it goes out.
    ///
    /// The bound is a **countdown**, not a check: the `TRN2d` arm takes a
    /// pending change only at a frame boundary at or after this many symbols,
    /// so asking for `SUVd` while TRN2d is on the line cuts it at exactly this
    /// count instead of letting it run on. Without it the arm's exit condition
    /// (`count >= trn2d_symbols && at_frame_boundary && pending`) never fires in
    /// a renegotiation and the far end never sees an `SUVd` -- `docs/214` §6's
    /// first gap.
    ///
    /// 16008T is 2668 whole data frame intervals, so the cut lands on the frame
    /// boundary the `SUVd` after it must start on.
    pub fn bound_trn2d(&mut self, symbols: usize) {
        self.trn2d_symbols = symbols;
    }

    /// How long TRN2d is being held for, in symbols.
    #[must_use]
    pub fn trn2d_bound(&self) -> usize {
        self.trn2d_symbols
    }

    /// Whether the signal waiting to go out may only begin on a data frame
    /// boundary.
    ///
    /// "The rate renegotiation procedure ... shall be initiated only on the
    /// boundary of a data frame. Similarly, a modem shall only respond to a
    /// rate renegotiation on the boundary of a data frame" (`v92.txt:2839`),
    /// and 9.8.1.1.1 and 9.8.1.2.2 each repeat it of `Rd` itself: "The signal
    /// Rd shall begin on the boundary of a data frame".
    ///
    /// Only the 9.8 signals are named. Everything else in this end's Phase 4
    /// takes its place on whatever symbol the signal before it ran out on, and
    /// changing that would be a change to every call, not to this one -- see
    /// [`Source::next`] for the bug that made the frame grid load-bearing.
    fn pending_needs_frame_boundary(&self) -> bool {
        matches!(
            self.pending,
            // `Ri` is here for `docs/216`'s re-entry and nothing else: out of
            // data mode the `B1d` arm is where a queued signal waits, and
            // every frame-counted signal after `Ri` -- the whole of Phase 4 --
            // is measured against the grid `Ri` starts on. In Phase 3 `Ri` is
            // never pending from `B1d`, so a call that does not fall back is
            // unchanged.
            // `Rf` for the same reason as `Rd`: 9.9.1.2.2 names the boundary
            // outright -- "The signal `Rf` shall begin on the boundary of a
            // data frame" (`v92.txt:3081-3082`).
            Some(Out::Rd | Out::RdBar | Out::Rt | Out::RtBar | Out::Ri | Out::RiBar | Out::Rf | Out::RfBar)
        )
    }

    /// One codeword of Rd or Rt, from the interval [`Source::symbol`] is in.
    ///
    /// The sign comes from V.90 8.6.4's "repeating the 6 symbol sequence ...
    /// with the sign pattern + + + - - -", so it repeats every [`INTERVALS`];
    /// the *codeword* comes from "each data frame interval", which is
    /// [`DOWN_INTERVALS`] of them. Two periods of different length, which is
    /// why they are counted apart.
    fn r_codeword(&self, training: bool, positive: bool) -> f64 {
        let Some(reneg) = self.reneg.as_ref() else {
            // No constellation to name, and no codeword that means "nothing".
            // `arm_renegotiation` (`2066`) is called before either `Rd` or `Rt`
            // is ever asked for (`5780`, `5837`), so this is a caller that
            // skipped it -- and a U_INFO `Rd` is -18 dB below the data mode
            // peak, the same shape as `docs/218`'s Ucode 2: a level that
            // silently means "no answer" is a level a far end reads as a
            // signal. Loud instead of silent, and the value unchanged so that
            // no path a bench call reaches today moves.
            debug_assert!(
                false,
                "9.8's Rd/Rt asked for before arm_renegotiation: no constellation to take \
                 a codeword from (v90.txt:1841-1848)"
            );
            return self.level(self.uinfo, positive);
        };
        let points = if training { &reneg.rt } else { &reneg.rd };
        let interval = (self.symbol % DOWN_INTERVALS as u64) as usize;
        let ucode = points[interval.min(DOWN_INTERVALS - 1)];
        self.level(ucode, positive)
    }

    /// Is the emitted signal of 9.8 the 24T bar, and has it run its 24T?
    fn r_bar_ended(&self) -> bool {
        self.count >= R_BAR_FRAMES * INTERVALS
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
                if let Some(data4) = self.data4.as_mut() {
                    data4.samples.clear();
                }
            }
            _ => self.last.clear(),
        }
        if out != Out::B1d {
            self.codes.clear();
        }
        // A signal beginning is not a repeat of the last codeword of the one
        // before it, whatever it was.
        self.last_code = None;
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
        self.hold = 0;
    }

    /// Ask for `out` after `symbols` more codewords of silence: 9.5.1.1.3's
    /// "may wait for up to 500 ms and shall then transmit signal Sd"
    /// (`v92.txt:2535-2536`).
    ///
    /// `symbols` is a whole number of frames ([`Habits::sd_after_ja_symbols`]
    /// guarantees it), so the wait only moves *when* `Sd` starts and leaves the
    /// frame grid -- and with it the six-symbol pattern every frame-counted
    /// signal is measured against -- exactly where it was. The wait is counted
    /// only while silence is on the line: it belongs to the gap after `Ja`, and
    /// a hand-over that finds some other signal in flight has nothing to wait
    /// in.
    pub fn change_after(&mut self, out: Out, symbols: usize) {
        self.pending = Some(out);
        self.hold = if self.out == Out::Silence { symbols } else { 0 };
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
                    // 9.5.1.1.3's wait, spent as silence on the frame grid: the
                    // pending signal starts at the first frame boundary at or
                    // after the hold runs out, so it is still whole frames
                    // (`Source::next`'s rotation) and only its placement moved.
                    if self.hold > 0 {
                        self.hold -= 1;
                        return 0.0;
                    }
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
                    //
                    // A renegotiation waiting in `pending` is the one thing
                    // here that may not start where the codeword queue happens
                    // to empty: `v92.txt:2839` and 9.8.1.2.2 both put `Rd` on
                    // a data frame boundary. The gate applies to nothing else,
                    // so an ordinary hand-over out of data mode is unchanged.
                    // A signal that has to begin on a data frame boundary does
                    // **not** wait for the codeword queue to run dry:
                    // `fill_downstream` refills it every symbol, so "empty" is a
                    // moment that need never come and the signal would sit in
                    // `pending` for ever. Measured on `runs-216`'s first call:
                    // `data mode lost` fired at 25.000 s, `Ri` was asked for --
                    // and never went out, because `codes` was never empty on a
                    // frame boundary. A frame boundary is the whole gate.
                    let startable = if self.pending_needs_frame_boundary() {
                        self.at_frame_boundary()
                    } else {
                        self.codes.is_empty()
                    };
                    if startable
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
                            // Kept, because `None` below is "the last codeword
                            // again" and this is the only place that knows it.
                            self.last_code = Some((u, !negative));
                            return self.level(u, !negative);
                        }
                        // Nothing queued: the **last codeword again**, which is
                        // what this comment said while the code sent U_INFO --
                        // a -18 dB hole in the middle of data mode, on the one
                        // path where every symbol is a data symbol. `None` here
                        // is a scheduling defect (`needs_codes`, `2040`), and a
                        // repeated data codeword is the only non-signal that
                        // keeps the frame grid and the constellation both
                        // right. Measured latent, not current: in
                        // `runs-218/172503-r218b/agr-rx.raw` the data mode
                        // window (190 000-199 988) has 3772 -- U_INFO -- in
                        // proportion with its neighbours, so the fallback did
                        // not fire there (`docs/221`).
                        None => {
                            let (u, positive) = self.last_code.unwrap_or((self.uinfo, self.sign));
                            return self.level(u, positive);
                        }
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
                    // `ri_opener`'s `Ri` runs the clause's 384T and then turns
                    // over, exactly where an `E1u` would have turned it over.
                    if !bar && self.ri_timed && self.count >= R_FRAMES * INTERVALS && self.at_frame_boundary() {
                        self.ri_timed = false;
                        self.start(Out::RiBar);
                        continue;
                    }
                    // "+ + + - - -": UINFO in every interval, and the bar the
                    // other way up (V.90 8.6.4).
                    let interval = (self.symbol % INTERVALS as u64) as usize;
                    self.count += 1;
                    return self.level(self.uinfo, (interval < 3) ^ bar);
                }
                // 9.8.1.1.1 and 9.8.1.2.2: "transmit signal Rd for 384T and
                // then R-bar-d for 24T", the same pair for Rt. Four arms and
                // not two because the *codeword* differs between them -- CPd's
                // data mode constellation against CPt's training one (V.90
                // 8.6.4) -- and the sign pattern between R and its bar.
                Out::Rd | Out::RdBar | Out::Rt | Out::RtBar => {
                    let bar = matches!(self.out, Out::RdBar | Out::RtBar);
                    let training = matches!(self.out, Out::Rt | Out::RtBar);
                    // "The signal Rd shall begin on the boundary of a data
                    // frame" (9.8.1.1.1, 9.8.1.2.2). Asserted rather than
                    // assumed: a signal started off-frame that rotated a
                    // frame-counted grid is the bug recorded at the top of
                    // docs/214 and at `digital.rs:1781-1782`, and it is silent
                    // on this side of the line.
                    if self.count == 0 {
                        debug_assert!(
                            self.at_frame_boundary(),
                            "9.8.1.2.2: {} began {} symbols into a data frame, not on its boundary",
                            if training { "Rt" } else { "Rd" },
                            self.symbol % INTERVALS as u64
                        );
                    }
                    // The bar is 24T and then the signal after it: 9.8.1.1.2's
                    // TRN2d for the responder, 9.8.1.1.4's SUVd for the
                    // initiator. R is 384T.
                    let ends = if bar {
                        self.r_bar_ended()
                    } else {
                        self.count >= R_FRAMES * INTERVALS
                    };
                    if ends && self.at_frame_boundary() {
                        // 9.8.1.2.2 and 9.8.1.1.1 each name the pair: R for
                        // 384T and *then* R-bar for 24T. With nothing asked
                        // for past the bar, the bar is followed by 9.8.1.1.2's
                        // TRN2d; with nothing asked for past the R, by its own
                        // bar.
                        let next = self.pending.take().unwrap_or(if bar {
                            Out::Trn2d
                        } else if training {
                            Out::RtBar
                        } else {
                            Out::RdBar
                        });
                        self.start(next);
                        continue;
                    }
                    self.count += 1;
                    let interval = (self.symbol % INTERVALS as u64) as usize;
                    // R-bar is "4 repetitions of the 6-symbol sequence ... with
                    // the sign pattern - - - + + +" (`v90.txt:1835`): the same
                    // codewords, the pattern inverted, and "neither R nor R-bar
                    // are differentially encoded" -- so the polarity is the
                    // literal one, not the encoder's. (An earlier draft of this
                    // comment had `- - - + + -`, a PDF hyphenation artifact of
                    // the em dashes; it is read off the source line.)
                    return self.r_codeword(training, (interval < 3) ^ bar);
                }
                // 9.9.1.2.2: "After detecting the RM-to-RM-bar transition, the
                // digital modem shall transmit signal Rf for 384T and then
                // R-bar-f for 24T. The signal Rf shall begin on the boundary of
                // a data frame" (`v92.txt:3079-3082`).
                //
                // The codeword is 8.8.4's -- "the highest power PCM codeword
                // from the data mode constellation of each data frame interval
                // as passed in CPu" (`v92.txt:1961-1962`) -- the same source
                // `Rd` already resolves (`docs/218`), read here through
                // `r_codeword`'s data-mode branch rather than through a second
                // resolution of the same message. `renegotiation_points` fills
                // `reneg.rd` from `self.downstream`'s mapping, i.e. from the
                // encoder B1d actually goes out on.
                Out::Rf | Out::RfBar => {
                    let bar = self.out == Out::RfBar;
                    // "The signal Rf shall begin on the boundary of a data
                    // frame": asserted, like `Rd`'s, because a signal started
                    // off the grid is silent on this side of the line and
                    // rotates every interval after it.
                    if self.count == 0 {
                        debug_assert!(
                            self.at_frame_boundary(),
                            "9.9.1.2.2: Rf began {} symbols into a data frame, not on its boundary",
                            self.symbol % INTERVALS as u64
                        );
                    }
                    // Rf is 384T and R-bar-f is 24T: 384T is 8.8.4's 12-symbol
                    // sequence 32 times, 24T its two repetitions.
                    let ends = if bar {
                        self.count >= R_BAR_FRAMES * INTERVALS
                    } else {
                        self.count >= R_FRAMES * INTERVALS
                    };
                    if ends && self.at_frame_boundary() {
                        let next = self.pending.take().unwrap_or(if bar {
                            // 9.9.1.2.3's SUVd is asked for by `RenegTail`
                            // before the bar ends; this fallback is reached
                            // only by a source driven without a tail.
                            Out::Silence
                        } else {
                            Out::RfBar
                        });
                        self.start(next);
                        continue;
                    }
                    // "+ + - -" with the left-most sign first, and the bar the
                    // other way up. The sign is the literal one: 8.8.4 gives
                    // `Rf` a sign pattern rather than a differential encoding,
                    // exactly as V.90 8.6.4 does for `R`/`R-bar`.
                    let positive = (self.count % RF_SIGN_PERIOD) < RF_SIGN_PERIOD / 2;
                    self.count += 1;
                    return self.r_codeword(false, positive ^ bar);
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
        // The table, not a hand-derived maxima: `Mapping::common_loudest` is
        // the single owner of "a codeword legal in all six of CPt's
        // intervals", so this cannot drift from the levels Phase 4 actually
        // emits. U_INFO is the fallback for a Phase 4 with no CPt, which is a
        // Phase 4 that cannot go out on the encoder either.
        self.phase4
            .as_ref()
            .and_then(|phase4| phase4.coder.mapping().common_loudest())
            .unwrap_or(self.uinfo)
    }
}

/// How the digital modem learns that the far end has **left data mode**.
///
/// **[our choice]**, and the threshold is the one thing on this path the
/// clauses do not give. V.92 has no signal for "I have stopped sending data":
/// 9.8.1 is a rate renegotiation, which starts with an `Ru` the Agere measured
/// for `docs/216` never sends anywhere in a call, and 9.7 is a retrain, which
/// starts with Tone A. What is left is the property the clauses *do* define the
/// far end's upstream by: 6.4.2 and 8.7.1 put every B1u symbol on a point of
/// the constellation this end declared in `CPd`, so a symbol outside that
/// constellation's loudest point by more than one constellation step is not a
/// B1u symbol at all -- and a sustained run of them is a far end that is not
/// sending B1u.
///
/// That is why this test is a magnitude compare and not a pattern match. A
/// pattern would be a property of the far end's generator (`V34TxDataReset` /
/// `DisableScrambler` / `SetPatternGenerator` at `.text+0x5e530`), which is a
/// property of its *implementation*; a level outside the constellation it
/// agreed to is a property of the protocol. Cost is one comparison a symbol.
///
/// The margin of one whole constellation step is measured, not chosen: on
/// `/tmp/agr-v90-interop/164250-sd108ab170-r214/agr-tx.raw` (the far end's own
/// transmit, 24 000 bit/s, `CPd` over 8 points, gap 2 LSB) the data mode's
/// loudest symbol in 9 440 symbols of data is 8 931, the loudest point of the
/// table is 10 746, and no symbol at all reaches the 12 179 threshold; the
/// generator pattern that follows reaches 22 303 and has 15 of its 40 levels
/// above it.
// ---------------------------------------------------------------------------
// 9.9.1.2.1/.2: RM, and the RM-to-RM-bar transition
// ---------------------------------------------------------------------------

/// What one received data mode symbol says about 9.9's `RM`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RmEvent {
    /// Nothing: data mode as usual.
    None,
    /// 9.9.1.2.1: "After detecting `RM`" -- the far end has begun a fast
    /// parameter exchange.
    RmDetected,
    /// 9.9.1.2.2: the `RM`-to-`RM`-bar transition, which is what `Rf` answers.
    RmToRmBar,
}

/// `RM`'s pattern as Table 25 gives it, one byte a symbol: 1 is
/// `K_i = M_i - 1` (the loudest point of the interval's set) and 0 is
/// `K_i = 0` (the quietest).
///
/// **Generated from the table's own rows and not typed from memory**: Table 25
/// (`v92.txt:1630-1647`) lists `K_i = M_i - 1` for `i` in `{0,1,4,5,8,9}` and
/// `K_i = 0` for `i` in `{2,3,6,7,10}`, with `u11 = 0`. The twelve-symbol
/// pattern is therefore `{max, max, min, min}` three times: period four, the
/// loud pair first.
pub const RM_K_LOUD: [bool; 12] = {
    let mut k = [false; 12];
    let mut i = 0;
    while i < 12 {
        // Rows 0,1,4,5,8,9 and row 11 (u11 = 0) -- i.e. every i whose row in
        // Table 25 is `M_i - 1` for 0..11, plus `u11 = 0`.
        k[i] = matches!(i, 0 | 1 | 4 | 5 | 8 | 9);
        i += 1;
    }
    k
};

/// Table 26's `RM`-bar, the same way: `K_i = 0` for `i` in `{0,1,4,5,8,9}`,
/// `K_i = M_i - 1` for `i` in `{2,3,6,7,10,11}` (`v92.txt:1654-1667`).
pub const RM_BAR_K_LOUD: [bool; 12] = {
    let mut k = [false; 12];
    let mut i = 0;
    while i < 12 {
        k[i] = matches!(i, 2 | 3 | 6 | 7 | 10 | 11);
        i += 1;
    }
    k
};

/// How many symbols of `RM`'s period-four pattern must agree before the far end
/// is taken to be sending `RM`. **`[our choice]`**, **24 symbols** -- six whole
/// periods of Table 25's four-symbol pattern, i.e. detection latency of 24
/// symbols (3 ms of `RM`'s 384T, which is 6 % of the signal).
///
/// Two bounds and both are measured, not guessed. The lower one is chance: a
/// data mode's own symbols reach both ends of the constellation continually, so
/// a `{loud,loud,quiet,quiet}` alignment happens by accident, and on a 40-level
/// constellation a twelve-symbol run (three periods) locks about once in two
/// thousand trials -- which is once in a few seconds of data. Twenty-four
/// symbols is 2^-24 a trial per phase. The upper one is the far end's own bar:
/// its `V90DataModeDecoder` detector needs ">= 40 complement pairs" and four
/// period-4 hits (`docs/220`, `[verified]`), so 24 symbols of evidence is under
/// what the other end asks for itself and over what chance produces. Timing is
/// in symbols, never wall clock.
const RM_SYMBOLS: usize = 24;

/// How many symbols of Table 26's pattern must agree before 9.9.1.2.2's
/// transition is taken. **`[our choice]`**
///
/// One period, four symbols, for the same reason [`RU_BAR_DEVIATIONS`] is
/// three: `R-bar-M` is 24T, two periods of Table 26's twelve, and a detector
/// that waited for all of it would put `Rf` on the line at the end of the
/// 24T rather than at its start. The transition is the *complement* of the
/// pattern `RM` locked on, so these four symbols are the four that Table 26
/// moves the loud pair to.
const RM_BAR_SYMBOLS: usize = 4;

/// 9.9.1.2.1's receiver conditioning and 9.9.1.2.2's transition, on the
/// received data mode level stream.
///
/// **What is detectable without a trellis decoder, and why this is it.** `RM`
/// "shall be trellis encoded" (8.7.4, `v92.txt:1666-1667`) and goes out on "the
/// constellation parameters used for the data mode", so its *symbols* are not
/// Table 25's values -- they are whatever the trellis code and the prefilter
/// put on the line. What Table 25 does fix is the **modulus encoder output**,
/// and that is what the magnitude carries: `K_i = M_i - 1` puts the symbol at
/// the loudest point of interval `i`'s set and `K_i = 0` at the quietest, so
/// the received magnitudes show `{loud, loud, quiet, quiet}` three times over.
/// That signature is what this reads: **no trellis decoder, no modulus
/// decoder**, only the two ends of the constellation the far end's own `CPu`
/// (and this end's CPd) declared.
///
/// `[inferred]` in exactly one place: that the trellis code and the prefilter
/// leave the loud/quiet distinction of the modulus encoder output intact. The
/// margins are what make that safe -- `hi` is the midpoint of the constellation
/// table's top two points and `lo` of its bottom two, so a symbol only has to
/// be in the right half of the range, not at the right point.
///
/// A **side channel**, like [`RuDetector`] and [`DataModeLoss`]: it is fed the
/// same levels the B1u decoder is fed and returns nothing to it, so a call with
/// no `RM` in it puts exactly the same symbols through exactly the same
/// arithmetic it did before.
#[derive(Debug, Clone)]
pub struct RmDetector {
    /// Above this a symbol is near the loud end of the constellation.
    hi: f64,
    /// Below this it is near the quiet end.
    lo: f64,
    /// The last four classifications, oldest first; `None` is neither end.
    window: [Option<bool>; 4],
    /// Symbols classified since the last reset, saturating at four.
    filled: usize,
    /// The phase of the four-symbol cycle `RM` locked on, if it has.
    phase: Option<usize>,
    /// Symbols in a row that agree with that phase, counting from the symbol
    /// that established it.
    agreed: usize,
    /// Symbols in a row that agree with the *complement* -- 9.9.1.2.2's
    /// transition.
    inverted: usize,
    /// `RM` seen: 9.9.1.2.1's conditioning is in force.
    heard: bool,
    /// The transition has been reported, once.
    fired: bool,
}

impl RmDetector {
    /// A detector against the level table [`Modem::begin_b1u`] built -- the
    /// same table [`DataModeLoss`] is built from, because both ask the same
    /// question of it: which levels are points of the constellation the far
    /// end agreed to send.
    #[must_use]
    pub fn new(levels: &[f64]) -> Option<Self> {
        let mut magnitudes: Vec<f64> =
            levels.iter().map(|l| l.abs()).filter(|l| l.is_finite() && *l > 0.0).collect();
        magnitudes.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let top = *magnitudes.last()?;
        let bottom = *magnitudes.first()?;
        if top <= bottom {
            return None;
        }
        // The midpoint of the top two and of the bottom two points: a symbol
        // has to be in the right half of the constellation's range, not at the
        // right point, which is what survives the trellis code and the line.
        let second_top = magnitudes.iter().rev().nth(1).copied().unwrap_or(top);
        let second_bottom = magnitudes.get(1).copied().unwrap_or(bottom);
        Some(Self {
            hi: (top + second_top) / 2.0,
            lo: (bottom + second_bottom) / 2.0,
            window: [None; 4],
            filled: 0,
            phase: None,
            agreed: 0,
            inverted: 0,
            heard: false,
            fired: false,
        })
    }

    /// Whether 9.9.1.2.1's conditioning is in force: `RM` has been seen.
    #[must_use]
    pub fn is_locked(&self) -> bool {
        self.heard
    }

    pub fn reset(&mut self) {
        let hi = self.hi;
        let lo = self.lo;
        *self = Self {
            hi,
            lo,
            window: [None; 4],
            filled: 0,
            phase: None,
            agreed: 0,
            inverted: 0,
            heard: false,
            fired: false,
        };
    }

    /// One received data mode symbol, as the level the slicer saw.
    pub fn feed(&mut self, level: f64) -> RmEvent {
        if self.fired {
            return RmEvent::None;
        }
        let magnitude = level.abs();
        let class = if !magnitude.is_finite() || magnitude <= self.lo {
            Some(false)
        } else if magnitude >= self.hi {
            Some(true)
        } else {
            None
        };
        self.window.rotate_left(1);
        self.window[3] = class;
        if self.filled < 4 {
            self.filled += 1;
        }
        // Table 25's pattern, at whichever phase it started on: `RM` begins on
        // a data frame boundary of the far end's, which is not a boundary of
        // this end's slicing.
        let pattern = [true, true, false, false];
        if !self.heard {
            let Some(phase) = self.phase else {
                if self.filled < 4 {
                    return RmEvent::None;
                }
                let found = (0..4).find(|p| {
                    (0..4).all(|k| self.window[k] == Some(pattern[(p + k) % 4]))
                });
                match found {
                    Some(p) => {
                        self.phase = Some(p);
                        self.agreed = 4;
                    }
                    None => return RmEvent::None,
                }
                return RmEvent::None;
            };
            let expected = pattern[(phase + self.agreed) % 4];
            if class == Some(expected) {
                self.agreed += 1;
                if self.agreed >= RM_SYMBOLS {
                    self.heard = true;
                    return RmEvent::RmDetected;
                }
            } else {
                // Off the pattern: `RM` did not happen here, and the next four
                // symbols may establish it again.
                self.phase = None;
                self.agreed = 0;
            }
            return RmEvent::None;
        }
        // 9.9.1.2.1: "condition its receiver to detect the RM-to-RM-bar
        // transition". Table 26 moves the loud pair two places, so the expected
        // symbol is the complement of `RM`'s own.
        let phase = self.phase.unwrap_or(0);
        let counted = self.agreed + self.inverted;
        let expected = pattern[(phase + counted) % 4];
        if class == Some(!expected) {
            self.inverted += 1;
            if self.inverted >= RM_BAR_SYMBOLS {
                self.fired = true;
                return RmEvent::RmToRmBar;
            }
        } else {
            self.inverted = 0;
            if class == Some(expected) {
                self.agreed += 1;
            }
        }
        RmEvent::None
    }
}

#[derive(Debug, Clone)]
pub struct DataModeLoss {
    /// The loudest level of the data mode constellation CPd declared, plus one
    /// constellation step. Above this a level is not a point of that
    /// constellation by any reading.
    threshold: f64,
    /// Symbols seen in the data frame now open.
    in_frame: usize,
    /// Whether the data frame now open has carried an illegal symbol yet.
    frame_illegal: bool,
    /// Data frames in a row that carried an illegal symbol.
    failed: usize,
    /// Set once, so that a far end that stays out of data mode fires once and
    /// the re-entry is not restarted under itself.
    fired: bool,
}

/// How many consecutive data frames must carry an illegal symbol before the
/// far end is taken to have left data mode. **`[our choice]`**
///
/// Eight data frames is 96 symbols, 12 ms. The two bounds are the far end's and
/// this end's: it has to be short against the 4 000 samples (0.5 s) the far
/// end's own data-mode receive state waits before it gives up (`docs/213`,
/// `V92ClientDataModeRxState` at `.text+0x5dea0` sets `+0x11698` to
/// `RTDEd + 4000`), and long against a burst of line noise, which is what a
/// single failed frame is. On the measured call above every one of those eight
/// frames fails on the first symbol of it (37 % of the generator's symbols are
/// above the threshold, so a frame of twelve that carries none has probability
/// 0.003), which leaves the whole 0.5 s of the far end's wait for the re-entry
/// itself.
const DATA_MODE_LOSS_FRAMES: usize = 8;

impl DataModeLoss {
    /// A detector against the level table [`Modem::begin_b1u`] built, or `None`
    /// when that table has no step to measure against.
    #[must_use]
    pub fn new(levels: &[f64]) -> Option<Self> {
        let mut sorted: Vec<f64> = levels.iter().copied().filter(|l| l.is_finite()).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let top = *sorted.last()?;
        let step = sorted
            .windows(2)
            .map(|w| w[1] - w[0])
            .filter(|d| *d > 0.0)
            .fold(f64::INFINITY, f64::min);
        if !step.is_finite() || step <= 0.0 {
            return None;
        }
        Some(Self { threshold: top + step, in_frame: 0, frame_illegal: false, failed: 0, fired: false })
    }

    /// The level above which a symbol is not a point of the declared
    /// constellation.
    #[must_use]
    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    /// Whether the far end has been declared out of data mode.
    #[must_use]
    pub fn fired(&self) -> bool {
        self.fired
    }

    /// One received data mode symbol. `true` on the symbol the condition
    /// becomes sustained, and never again after it.
    ///
    /// A data frame is twelve symbols (Figure 1) and fails on **one** illegal
    /// symbol in it: the test is not "how many errors", it is "was this frame a
    /// point sequence of the constellation at all", and one symbol that is not
    /// is enough to answer no.
    pub fn feed(&mut self, level: f64) -> bool {
        if self.fired {
            return false;
        }
        self.frame_illegal |= !level.is_finite() || level.abs() > self.threshold;
        self.in_frame += 1;
        if self.in_frame < UP_INTERVALS {
            return false;
        }
        // One whole data frame has been read: it either carried a symbol that
        // is not a point of the constellation or it did not.
        self.in_frame = 0;
        if std::mem::take(&mut self.frame_illegal) {
            self.failed += 1;
        } else {
            self.failed = 0;
        }
        if self.failed >= DATA_MODE_LOSS_FRAMES {
            self.fired = true;
            return true;
        }
        false
    }
}

// ---------------------------------------------------------------------------
// 9.8.1.1.2 to 9.6.1.1.2: the tail after Rd and R-bar-d
// ---------------------------------------------------------------------------

/// Where 9.8's tail has got to.
///
/// The clause chain is 9.8.1.2.3 -> 9.8.1.1.2 -> .3 -> .4/.5 -> 9.6.1.1.2, and
/// every step of it hands the next one a signal this end has already got an
/// emitter for. What was missing was the machine that asks for them in the
/// clause's order, which is this and [`RenegTail::step`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenegStage {
    /// 9.8.1.2.2 and 9.8.1.1.2's first half: `Rd` 384T, `R-bar-d` 24T and then
    /// `TRN2d` are on [`Source`]'s own arms, and nothing is asked for until
    /// `TRN2d` is on the line.
    Trn2d,
    /// 9.8.1.1.2/.3: `SUVd` sequences. Which of the two they are is the bit-32
    /// branch, and it is decided here and nowhere else.
    Suvd,
    /// 9.8.1.1.3: `Ed` has gone out and the line is Ucode 0 until an `SUVu`
    /// with bit 32 clear (9.8.1.1.4).
    Silence,
    /// 9.8.1.1.4/.5: `Rt` for 384T then `R-bar-t` for 24T.
    Rt,
    /// 9.6.1.1.2: the exchange -- `SUVd` sequences and the one `CPd`.
    Cp,
    /// 9.6.1.1.4's `Ed` was the last thing in the clause: B1d again.
    Done,
}

/// The tail of 9.8: what to ask [`Source`] for after `R-bar-d`, and when.
///
/// One machine for the initiating and responding halves, because 9.8.1.2.3 says
/// so outright -- "The digital modem shall then proceed according to 9.8.1.1.2"
/// -- and 9.8.1.1.4/.5 end at 9.6.1.1.2 like everything else. The exchange is
/// the same [`Exchange`] Phase 4 uses, with [`Context::Renegotiation`], which is
/// what selects 9.8's silent period and refuses CPd bit 29's one-symbol E2u
/// extension (`Context::may_extend_e2u`).
#[derive(Debug, Clone)]
pub struct RenegTail {
    stage: RenegStage,
    /// 9.8.1.1.3's `SUVd'` -- bit 33 set -- has gone out in the silence
    /// handshake, which is what "the current SUVd" has to be before the `Ed`.
    sent_ack_suvd: bool,
    /// 9.8.1.1.3's `Ed` has been asked for.
    ed_asked: bool,
    /// And the silence behind it.
    silence_asked: bool,
}

impl RenegTail {
    /// A tail at 9.8.1.2.3, with the exchange 9.6.1.1.2 is entered from.
    ///
    /// This is `Exchange::new(Context::Renegotiation, ...)`. It is a **live**
    /// caller at last: `docs/214` §6 gap 3 recorded that the context was
    /// constructible and had none.
    /// Returns the tail **and** the exchange it drives, because the exchange is
    /// what the far end's `SUVu` is reported to ([`Exchange::peer_suv`]) and the
    /// owner has to hold it for the reader that does the reporting.
    pub fn new(round_trip: f64, fs: f64) -> (Self, Exchange) {
        let mut exchange = Exchange::new(Context::Renegotiation, round_trip, fs);
        // 9.6.1.1.3's repeats, interleaved with SUVd as 9.6.2.1.2's CPu needs.
        exchange.repeat_between_suvs(REPEAT_CP_GAP);
        // `CPD_WAITS_FOR_PARKED_TX`: 18 ms after its `CPu` the far end's chain
        // is mid-sequence and the acknowledgement it sets never leaves it; one
        // window later it is parked and `a001` comes out.
        exchange.hold_cp_for_parked_tx();
        // 9.8.1.1.2's "either SUVd or SUVu"; see [`RENEG_REQUEST_SILENCE`].
        exchange.request_silence(RENEG_REQUEST_SILENCE);
        (
            Self { stage: RenegStage::Trn2d, sent_ack_suvd: false, ed_asked: false, silence_asked: false },
            exchange,
        )
    }

    /// A tail that begins at **9.9.1.2.3's** `SUVd`, with no `TRN2d` before it.
    ///
    /// 9.9.1.2.3 asks for exactly three things -- "initialize the scrambler,
    /// differential encoder and spectral shaping filter memory to zero and
    /// transmit `SUVd` sequences with **bit 32 clear** and proceed according to
    /// 9.6.1.1.2" -- and no `TRN2d` at all: the far end has *stayed* in the data
    /// mode modulation (8.8.5: during a fast parameter exchange `SUVd` is
    /// "transmitted using the preceding data mode modulation"), so the
    /// `TRN2d` that 9.8.1.1.2 wants has nothing to retrain. This constructor is
    /// the entry at [`RenegStage::Suvd`] for that, and it is the only
    /// difference from [`RenegTail::new`]: no `TRN2d` stage, and bit 32 of this
    /// end's own `SUVd` clear (9.9.1.2.3 says so outright, rather than leaving
    /// it to [`RENEG_REQUEST_SILENCE`]).
    pub fn from_suvd(round_trip: f64, fs: f64) -> (Self, Exchange) {
        let mut exchange = Exchange::new(Context::Renegotiation, round_trip, fs);
        exchange.repeat_between_suvs(REPEAT_CP_GAP);
        // "with bit 32 clear" (9.9.1.2.3): no silent period is requested.
        exchange.request_silence(false);
        (
            Self { stage: RenegStage::Suvd, sent_ack_suvd: false, ed_asked: false, silence_asked: false },
            exchange,
        )
    }

    #[must_use]
    pub fn stage(&self) -> RenegStage {
        self.stage
    }

    #[must_use]
    pub fn is_done(&self) -> bool {
        self.stage == RenegStage::Done
    }

    /// One of this end's Phase 4 sequences has just run out of bits.
    ///
    /// 9.8.1.1.3's `Ed` is followed by silence and 9.6.1.1.4's by B1d, which is
    /// the whole difference between the two stages that end on an `Ed`.
    fn sequence_ended(&mut self, done: Out, exchange: &mut Exchange, at: u64) {
        // 9.8.1.1.3's Ed is **not** the exchange's E: 9.8.1.1.4 carries on to
        // `Rt` and only then to 9.6.1.1.2, so reporting it as
        // `SequenceKind::E` would set `Exchange`'s `finished` and every later
        // `next_sequence` would answer `B1` -- no `CPd` at all after the
        // silence.
        if done == Out::Ed && self.ed_asked {
            self.stage = RenegStage::Silence;
            return;
        }
        let kind = match done {
            Out::Cpd => SequenceKind::Cp,
            Out::Ed => SequenceKind::E,
            _ => SequenceKind::Suv,
        };
        exchange.sequence_ended(kind, at);
        // 9.6.1.1.4's Ed is the end of the clause.
        if done == Out::Ed {
            self.stage = RenegStage::Done;
        }
    }

    /// One symbol of the tail: what ended, then what the clause wants next.
    ///
    /// This is the whole of the per-symbol work, so that the driver in
    /// [`Modem::reneg_out`] and the tests below drive the same code: a machine
    /// proved in one shape and used in another is the bug `docs/214` records of
    /// the two-point TRN2d.
    fn push(
        &mut self,
        source: &mut Source,
        exchange: &mut Exchange,
        cpd: Option<&Cpd>,
        now: u64,
    ) -> Option<Out> {
        // "complete sending the current SUVd" (9.8.1.1.3): a sequence that has
        // run out has ended, and the acknowledge bit of 9.6.1.1.2 changes at a
        // boundary and nowhere else.
        if let Some(finished) = source.take_finished() {
            self.sequence_ended(finished, exchange, now);
        }
        self.step(source, exchange, cpd, now)
    }

    /// One symbol: whatever the clause asks of the signal now in flight.
    ///
    /// Returns the sequence this asked the source for, or `None` when the
    /// signal on the line is one the source runs by itself.
    fn step(
        &mut self,
        source: &mut Source,
        exchange: &mut Exchange,
        cpd: Option<&Cpd>,
        now: u64,
    ) -> Option<Out> {
        match self.stage {
            RenegStage::Done => None,
            RenegStage::Trn2d => {
                // 9.8.1.1.2: "shall then transmit TRN2d for up to 16008T
                // followed by SUVd sequences". The 16008T is
                // [`Source::bound_trn2d`]'s countdown: the SUVd is asked for
                // now and the source holds it until the count is spent.
                if source.out() != Out::Trn2d {
                    return None;
                }
                self.stage = RenegStage::Suvd;
                self.step(source, exchange, cpd, now)
            }
            RenegStage::Suvd | RenegStage::Cp => {
                // "unless bit 32 is set in either SUVd or SUVu" (9.8.1.1.2) --
                // the one place the two directions interact. Bit 32 of our own
                // SUVd is [`RENEG_REQUEST_SILENCE`]; bit 32 of the far end's
                // SUVu is `Exchange::peer_asked_silence`, latched for the whole
                // exchange because 9.8.1.1.4 tests the bit that *was* set.
                //
                // Re-tested on **every** symbol and not once at the start of the
                // stage: the far end's bit arrives with its SUVu, which is after
                // this end's first SUVd, so a branch taken at the first SUVd
                // would never see it.
                let silence = exchange.silence_requested() || exchange.peer_asked_silence();
                if !silence {
                    return self.exchange_step(source, exchange, cpd, now);
                }
                // 9.8.1.1.3: "shall then transmit SUVd sequences with bit 33
                // set. After receiving an SUVu sequence with bit 33 set or E2u,
                // the digital modem shall complete sending the current SUVd and
                // then transmit Ed followed by silence."
                if self.ed_asked {
                    // The Ed is on the line; "followed by silence" is asked for
                    // behind it, so the source starts the silence at the frame
                    // boundary the Ed ends on and never repeats the Ed.
                    if source.out() == Out::Ed && !self.silence_asked {
                        self.silence_asked = true;
                        source.change(Out::Silence);
                    }
                    return None;
                }
                // "complete sending the current SUVd and then transmit Ed": an
                // SUVd that is only *pending* is not a current one -- the
                // sequence in flight may still be TRN2d's countdown, and asking
                // for the Ed there would displace the SUVd and never put one on
                // the line at all.
                // And it comes after an `SUVd'`, not before one: 9.8.1.1.3's
                // order is "SUVd with bit 33 set" **then** "after receiving an
                // SUVu with bit 33 set ... Ed".
                if self.sent_ack_suvd && exchange.peer_acked() && source.out() == Out::Suvd {
                    self.ed_asked = true;
                    source.send(Out::Ed, source.ed_bits());
                    return Some(Out::Ed);
                }
                if exchange.sending().is_some() {
                    return None;
                }
                exchange.sequence_started(SequenceKind::Suv);
                let bits = Suvd { silence: true, ack: true }.to_bits(INTERVALS);
                source.send(Out::Suvd, bits);
                self.sent_ack_suvd = true;
                Some(Out::Suvd)
            }
            RenegStage::Silence => {
                if source.out() != Out::Silence {
                    // The Ed is still finishing: the silence was asked for when
                    // it started, and the source takes it at the boundary.
                    if !self.silence_asked {
                        self.silence_asked = true;
                        source.change(Out::Silence);
                    }
                    return None;
                }
                // 9.8.1.1.4: "If bit 32 of SUVu was set, the digital modem shall
                // wait to receive SUVu with bit 32 clear." `peer_silence` follows
                // the *latest* SUVu and not a latch, which is what makes this
                // the wait the clause describes.
                if exchange.peer_silence() {
                    return None;
                }
                // The silent period is over and 9.6.1.1.2 starts again behind
                // `Rt`: `Exchange::silence_ended` drops the acknowledge state
                // bit 33 carried *inside* the silence, so that it means "your
                // CP was received" again afterwards (Tables 27 and 31;
                // `SILENCE_CLEARS_ACK_STATE`).
                exchange.silence_ended();
                // "After receiving SUVu with bit 32 clear, the digital modem shall
                // transmit Rt for 384T followed by R-bar-t for 24T and SUVd."
                self.stage = RenegStage::Rt;
                source.change(Out::Rt);
                Some(Out::Rt)
            }
            RenegStage::Rt => {
                match source.out() {
                    // The bar is 9.8.1.1.4's last R signal and the SUVd comes
                    // straight after it; the Cp stage asks for it, and the
                    // source holds it for the bar's 24T.
                    Out::RtBar => {
                        // 9.8.1.1.4: "...and SUVd. The digital modem shall then
                        // proceed according to 9.6.1.1.2." The exchange is the
                        // same one, so what changes is only that 9.8.1.1.3's Ed
                        // is behind us.
                        self.stage = RenegStage::Cp;
                        self.ed_asked = false;
                        self.step(source, exchange, cpd, now)
                    }
                    // Rt or R-bar-t: the source's own arm counts them out.
                    Out::Rt | Out::Trn2d => None,
                    // Still waiting for the silence to end on a frame boundary.
                    _ => None,
                }
            }
        }
    }

    /// 9.8.1.1.2's other half and 9.6.1.1.2: the exchange's own order, which is
    /// `SUVd` until the far end's `SUVu` arrives and one `CPd` after it.
    fn exchange_step(
        &mut self,
        source: &mut Source,
        exchange: &mut Exchange,
        cpd: Option<&Cpd>,
        now: u64,
    ) -> Option<Out> {
        if exchange.sending().is_some() {
            return None;
        }
        let kind = exchange.next_sequence(now);
        let ack = exchange.sequence_started(kind);
        match kind {
            SequenceKind::Suv => {
                let bits = Suvd { silence: exchange.silence_requested(), ack }.to_bits(INTERVALS);
                source.send(Out::Suvd, bits);
                Some(Out::Suvd)
            }
            // 9.6.1.1.2: "After receiving an SUVu sequence, the digital modem
            // shall transmit a single CPd".
            SequenceKind::Cp => {
                let Some(cpd) = cpd else {
                    // Nothing to declare (9.6.1.1.2 allows the wait): keep the
                    // SUVs going rather than send a CPd that says nothing.
                    let bits = Suvd { silence: false, ack }.to_bits(INTERVALS);
                    source.send(Out::Suvd, bits);
                    return Some(Out::Suvd);
                };
                let mut cpd = cpd.clone();
                cpd.ack = ack;
                source.send(Out::Cpd, cpd.to_bits(INTERVALS));
                Some(Out::Cpd)
            }
            SequenceKind::E => {
                source.send(Out::Ed, source.ed_bits());
                Some(Out::Ed)
            }
            _ => None,
        }
    }
}

/// Which of the aim's two measurements to take: `V92_FE_AIM=first`, `second`,
/// `both` (default) or `off`.
///
/// Two measurements, two targets, because the two stretches they are taken on
/// are read by two different things. The first is on the second TRN1u, before
/// CPt, and maximises the *linear* fit CPt is read with; the second is on
/// TRN2u, and maximises the four-point decision-feedback fit the data mode is
/// read with. Keeping them separable is what makes "the mechanism is wrong"
/// and "the criterion chooses badly" two measurements rather than one argument.
/// How far to move the **far end's** transmitter phase away from what Jp would
/// otherwise ask for: `V92_JP_OFFSET`, in symbols.
///
/// This is the only way to move the sampling instant that V.92 gives us, and it
/// is why a captured `*.raw` cannot answer a phase question: the far end has
/// already chosen its instant when the recording was made. Table 22 bits 18:33
/// carry epsilon as `[0, 1)` of a symbol, so the offset is added modulo one and
/// the same value is handed to `expect_step`, which is what our own receiver
/// waits for.
///
/// Off by default and diagnostic only; a bench sweep sets it and reads the
/// residual phase out of the meter below.
/// Whether `V92_SU_BLIND=1` armed the go-now override: off unless asked for.
///
/// It used to be the product's way past a far end that never reverses. It is
/// now only an **override** on top of Phase 3's timetable (docs/131): with it
/// set, the Su stage is left the moment the hunt reports Su instead of on the
/// schedule, which is what a bench arm wants when it is asking what a far end
/// does with a Jp it never waited for.
fn blind_su() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| {
        matches!(
            std::env::var("V92_SU_BLIND").ok().as_deref(),
            Some("1") | Some("yes") | Some("on") | Some("true")
        )
    })
}

/// Whether Phase 3's timetable is armed: on unless `V92_SU_TOLERANCE=off`.
///
/// The name is the old switch's: the timetable replaced the coherence gate that
/// used to sit behind it (`docs/131`), and `off` is still the way back -- with
/// it off, the Su stage is left only by 9.5.2.1.7's reversal or by 9.5.1.2.2,
/// which is what this end did before either.
fn su_tolerance() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| {
        !matches!(
            std::env::var("V92_SU_TOLERANCE").ok().as_deref(),
            Some("0") | Some("off") | Some("no") | Some("false") | Some("none")
        )
    })
}

/// Whether `V92_CODE_GRID` armed the one-per-call field dump (docs/114 R6).
///
/// One variable for the whole report -- CPu bit 35, bit 128, bits 52:67, the
/// mask blocks, SUVu bits 27:31 and the `K` this end fitted from them -- because
/// the question they exist to answer is asked with all of them at once: what
/// does this far end say its code grid is, and does the number this end read out
/// of it agree with the levels on the line. `eprintln!` and nothing else: no
/// path below reads these, and a call without the variable set is bit-identical
/// to one made before this existed.
fn code_grid_log() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| {
        !matches!(
            std::env::var("V92_CODE_GRID").ok().as_deref(),
            None | Some("0") | Some("off") | Some("no") | Some("false")
        )
    })
}

/// The upstream slicer's table for one constellation set, which the level set
/// and nothing else chooses.
///
/// `code` is docs/114 R4: `round(K a(eta))`, the far end's codes brought to
/// this end's scale. `uniform` is the odd lattice's own points, the table every
/// call before `fa36308` sliced on -- and it is returned by *ignoring* `k`, not
/// by passing it a one that might one day stop being one. That is the whole
/// point of the gate: on the odd lattice there is no scale to fit, a fit
/// through the origin against `2 eta + 1` returns whatever the far end's mask
/// averages to (433.75 on the replay corpus), and reading B1u at that scale is
/// what took the default arm from 576 of 576 to 327 of 576. A `k` this arm
/// cannot see cannot do that again.
fn up_table(levelset: LevelSet, set: &[u16], k: f64) -> Vec<u16> {
    match levelset {
        LevelSet::Uniform => set.to_vec(),
        LevelSet::Code => crate::v92::codegrid::scaled(set, k),
    }
}

/// The levels `B1u` is sliced against, in equaliser units, ascending, deduplicated.
///
/// Two things decide it and they are not the same decision:
///
/// * the **set**, from `up_table`: the odd lattice `2 eta + 1` on
///   [`LevelSet::Uniform`] whatever `k` says, the far end's codes `round(k a)`
///   on [`LevelSet::Code`] -- `docs/216` and `docs/114` R4;
/// * the **lattice**, from [`crate::v92::receiver::lattice_levels`], and only
///   when the set is the code grid. CPd's points are G.711 code values when the
///   far end's constellation is its codec's output (`docs/242` sec 2); on the odd
///   lattice they are not, the far end cannot emit them and must not, and a
///   table snapped to the nearest code is a table displaced by half a chord step:
///   36 LSB at 1 920 LSB on drn 1's 349-LSB spacing, a fifth of the decision
///   half-gap. That is `docs/257`'s regression: the snap was applied to
///   every set here, it was validated only as a fit figure (36.5 -> 35.7 dB,
///   `docs/242` sec 2) and never as a payload, and the call it broke decoded
///   nothing in either direction.
///
/// The gain is CPd's `G`: the far end transmits at `G a(eta)` and 8.8.3's power
/// identity is what makes that one number this end's equaliser unit.
fn data_mode_levels(parameters: &Parameters, k: f64) -> Vec<f64> {
    let mut levels: Vec<f64> = Vec::new();
    for set in &parameters.sets {
        for point in up_table(parameters.levelset, set, k) {
            let level = parameters.gain * f64::from(point);
            levels.push(level);
            levels.push(-level);
        }
    }
    if parameters.levelset == LevelSet::Code {
        levels = crate::v92::receiver::lattice_levels(&levels);
    }
    levels.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    levels.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    levels
}

fn jp_offset() -> f64 {
    std::env::var("V92_JP_OFFSET").ok().and_then(|t| t.parse().ok()).unwrap_or(0.0)
}

/// Measure the line's phase and **report it** without installing anything:
/// `V92_FE_AIM=meter`.
///
/// The converter's two measurements are the only phase instrument this receiver
/// has, and until now they could only run when the delay line was in the path
/// and being written to. Metering separates the two: the line exists (so the
/// aimers have raw history to replay) but nothing is ever installed, so what
/// comes out is the phase *the far end chose*, in symbols, which is the number
/// `V92_JP_OFFSET` is swept against.
fn fe_metering() -> bool {
    matches!(std::env::var("V92_FE_AIM").ok().as_deref(), Some("meter") | Some("measure"))
}

fn fe_aim_wants(second: bool) -> bool {
    match std::env::var("V92_FE_AIM").ok().as_deref() {
        Some("meter") | Some("measure") => false,
        Some("off") | Some("0") | Some("none") => false,
        Some("first") | Some("trn1u") => !second,
        Some("second") | Some("trn2u") => second,
        _ => true,
    }
}

/// How many symbols of the training the aim is allowed to look at:
/// `V92_FE_WINDOW`, defaulting to whatever the caller would fit.
///
/// The criterion's dynamic range is a few per cent and it is measured on a
/// thousand symbols, so how much line it is given is an experimental variable
/// and not a constant: on TRN2u the aim may look at the whole stretch the fit
/// will be built from, and TRN2u has twice the fit's window on the line by the
/// time the fit is taken.
fn fe_window(default: usize) -> usize {
    std::env::var("V92_FE_WINDOW").ok().and_then(|t| t.parse().ok()).unwrap_or(default)
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
    /// Where the hunt last reported Su in the stage that is waiting for its
    /// reversal. It is [`SU_TIMETABLE`]'s anchor -- `S0` is this less
    /// [`SU_REPORT_LAG`] -- and nothing in the decode reads it.
    su_found: Option<u64>,
    /// Where the hunt reported the Su-to-S-bar-u reversal, once it has. Only
    /// recorded here: it confirms the timetable and re-anchors the epsilon
    /// window, and it is not what lets the stage advance (docs/131).
    su_seen: Option<u64>,
    /// The last `held`/`coherence` the hunt reported for a Su this end is
    /// waiting on: evidence for the log line, and a permission for nothing.
    su_evidence: Option<(u32, f64)>,
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
    /// The next symbol count at which the refit may look at the line again.
    ///
    /// The refit waits for the far end to begin its second TRN1u, and it finds
    /// that by measuring the line (see `Receiver::su_ends`), which costs a scan
    /// of history: affordable once, not once a symbol. `None` is "look as soon
    /// as the window is in".
    refit_poll: Option<u64>,
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
    /// 9.8.1.2.1's receiver conditioning: the `Ru` detector that runs *during
    /// data mode*, where the B1u decoder above is what is being fed and
    /// `up_finder` is not (see `docs/213` section 6). A side channel: it is
    /// told the level the decoder is told and it answers with nothing the
    /// decoder sees.
    ru_detector: RuDetector,
    /// The far end's upstream against the constellation `CPd` declared: the
    /// detector behind `docs/216`'s re-entry. `None` until `begin_b1u` has a
    /// level table to measure against.
    data_loss: Option<DataModeLoss>,
    /// 9.9.1.2.1/.2's `RM` detector: the far end's **fast parameter exchange**,
    /// which is a different procedure from 9.8's rate renegotiation and answers
    /// with `Rf`/`R-bar-f` rather than `Rd`/`R-bar-d` (`docs/222`).
    rm_detector: Option<RmDetector>,
    /// 9.9.1.2.1's clamp has been applied for *this* exchange: one note, one
    /// clamp, however many `RM` symbols follow.
    rm_responder_started: bool,
    /// `docs/216` §3: `Ri`/`R-bar-i` have gone out and the far end's `TRN2u`
    /// is due. The re-entry's one difference from 9.8's tail is where the
    /// upstream reader is armed, because there is no `Ru`-to-`R-bar-u`
    /// transition to be the fiducial for it.
    reentry: bool,
    /// 9.9's fast parameter exchange is the procedure in flight, not 9.8's:
    /// its tail is `RenegTail::from_suvd`'s and it never sends `TRN2d`.
    fpe: bool,
    /// 9.8.1.2.1's clamp has been done for this renegotiation, so that a run of
    /// `Ru` does not clamp and log once a period.
    ru_responder_started: bool,
    /// 9.8.1.1.2 to 9.6.1.1.2: the tail after `R-bar-d`, from TRN2d's end to the
    /// one `CPd`. `None` outside a rate renegotiation, and that is also the gate
    /// on the far end's `SUVu` reader below.
    reneg: Option<RenegTail>,
    /// The symbol the far end's renegotiation `TRN2u` begins on, once `Ru` has
    /// become `R-bar-u`. See [`TRN2U_AFTER_RU_BAR`].
    reneg_trn2u_from: Option<u64>,
    /// The CPd this end will send, and the parameters behind it, which are also
    /// what the upstream decoder needs once B1u starts (6.4).
    cpd: Option<Cpd>,
    parameters: Option<Parameters>,
    /// The analogue modem's CPu, once it has arrived: the downstream's data
    /// mode parameters (9.6.1.1.5).
    cpu: Option<Cp>,
    /// `K`, the far code to this end's level scale, as
    /// [`codegrid::estimate_from_mask`] read it out of the last CPu (docs/114
    /// R2a/R4). One is "the far end's codes are this end's numbers", which is
    /// what a CPd designed on the code grid assumes, so it is also what a call
    /// with no mask at all uses.
    up_scale: f64,
    /// Where [`Self::up_scale`] came from, for the log: `"mask"`, `"design"`
    /// or `"design (bit 128)"`.
    up_scale_source: &'static str,
    /// SUVu bits 27:31 as the far end last reported them: `None` is the printed
    /// 16, "no measurement has been taken". Kept only for the one-per-call
    /// dump (`V92_CODE_GRID`), never for a decision -- `docs/114` R5.
    suvu_level: Option<f64>,
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
    ///
    /// Absolute in `now` units and anchored at `start_up_origin`, because
    /// 9.6.1.2.1 counts from the end of INFO1a rather than from Ed.
    b1u_deadline: Option<u64>,
    /// The `now` that clause 9.6.1.2.1's 20 s is counted from: zero, which is
    /// the end of Phase 2 and so the end of INFO1a (`Modem::new` is built
    /// there), and re-set to the `Ru` that opens a 9.8 renegotiation, whose
    /// Phase 4 is a new exchange with its own reference.
    start_up_origin: u64,
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
    /// The design the upstream rung is chosen from, held from the moment it is
    /// built to the moment the measurement that chooses a rung exists.
    ///
    /// The design has to be made from the second TRN1u, while that stream is
    /// still inside the receiver's history; the rung cannot be chosen until
    /// TRN2u has been fitted, whose residual is the first decision-point
    /// reading of the line (`Self::decide_upstream_rung`). So the design is
    /// kept across that gap and the rung is chosen from it, rather than the
    /// design being made twice or from a stale window.
    upstream_plan: Option<Design>,
    /// Whether a CPd has gone out on the line. A rung chosen after that cannot
    /// be told to the far end, so the provisional one stands.
    cpd_sent: bool,
    /// The same decoder on the other polarity, kept while B1u is being read
    /// both ways up: see [`Self::b1u_polarity`] and `decide_b1u_polarity`.
    b1u_probe: Option<Decoder>,
    /// Ones each decoder has read out of B1u: `[0]` is the line as it arrived,
    /// `[1]` is it negated.
    b1u_ones: [u64; 2],
    /// Bits each of them has read, for the margin the choice is made on.
    b1u_scored: [u64; 2],
    /// The level stream of a candidate boundary's B1u, being collected so that
    /// every candidate in [`E2U_SYMBOLS`] can be scored against 8.7.1's known
    /// content before one of them is believed. See [`Self::begin_b1u`].
    b1u_search: Option<B1uSearch>,
    /// Symbols already received that the committed data decoder has not been
    /// given yet.
    ///
    /// A candidate `back` symbols behind the run's end puts the end of B1u
    /// `back` symbols behind the end of the collected window, so the first
    /// `back` symbols of the data mode are inside the window and are dropped
    /// unless they are carried over: the run's end is 111 432 on one call and
    /// 111 578 on the next, and at `back = 2` that is two symbols of the far
    /// end's data. Nothing needs them -- they are data, not B1u -- but the
    /// decoder that carries the call does: without them its stream starts two
    /// symbols into a data frame, which is how a call that read B1u at 576 of
    /// 576 connects and then delivers 9 820 bytes of nothing.
    b1u_backlog: std::collections::VecDeque<Symbol>,
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

/// **The live `bin`/`adj` instrument** (`docs/264` sec 3): the fraction of
/// `B1u`'s symbols whose *decided index* differs from the index that was
/// **sent**, and the share of those errors that landed one index away.
///
/// This is `docs/253` sec 2's statistic, on the call path instead of in an
/// example, and it is measurable live for one reason and no other: **8.7.1
/// makes `B1u`'s payload known** -- forty-eight data frames of scrambled
/// ones -- so the far end's etas are a function of the clause and of the
/// rung, not of what the far end happens to be sending. `UpEncoder::b1u`
/// generates exactly the sequence CPd's own set will carry, and the
/// decoder's own frame indices (`take_decisions_indexed`: "index 0 is
/// `B1u`'s first four symbols") pair with it symbol for symbol, which is
/// why a dropped frame cannot shift the pairing.
///
/// **The chords** are the µ-law chords of the *sent* level, as `docs/253`
/// sec 2 defines them, and the rate reported is the **population-weighted**
/// one: each chord's own error rate times the share of the symbols it
/// carries. One global rate over a set whose chords differ by 64x in step
/// is the number this arm is not allowed to report.
///
/// **Cost**: one `Vec` of decided frames, already allocated by the decoder,
/// drained once per candidate at the boundary (thirteen candidates, ~134
/// frames each, 4 `i32` per frame), and one pass of 576 symbols to build the
/// sent sequence. Nothing is added to the per-symbol path: the instrument
/// reads what the decoder decided and never feeds it anything.
#[derive(Clone, Copy)]
pub(crate) struct B1uBins {
    /// Per chord: symbols sent, and the ones decided as another *class*.
    sent: [u64; 8],
    wrong: [u64; 8],
    total: u64,
    total_wrong: u64,
    /// Of the errors, the ones that landed on the adjacent index.
    adjacent: u64,
    /// Decisions that had no sent index to pair with (the decoder read past
    /// the end of `B1u`'s known content), so the pairing's own honesty is
    /// visible.
    unpaired: u64,
    /// The fewest symbols any *displaced* pairing of the same decisions got
    /// wrong, over every displacement of `1 ... 11` symbols either way
    /// ([`Self::TRUST_MARGIN`]). A rate is only published when the given
    /// origin is decisively better than every other one: an instrument that
    /// scores a displaced pairing the same as a right one cannot certify the
    /// right one, and `docs/285` sec 2 is the call that proved it.
    nearest_rival: u64,
    /// `eta_decided - eta_sent` over every paired symbol, in a fixed table of
    /// `-`[`Self::DELTA_MID`]` ..= +`[`Self::DELTA_MID`], with everything
    /// further out in [`Self::delta_far`]. This is what separates "wrong
    /// point" from "right point, wrong number": a difference that is a
    /// multiple of the interval's `Mi` (of `2Mi` at `k = 3`) is two members of
    /// one 6.4.2 class, which is the signature of a *representative* and not of
    /// a misread symbol, while a difference of one is a read error. `docs/285`
    /// sec 6 is the call that needed it.
    delta: [u64; 33],
    delta_far: u64,
    /// The largest `|eta|` the sent sequence and the decided stream reached.
    max_abs_sent: i32,
    max_abs_decided: i32,
    /// How many times `chord_of` found no level for an index and fell back to
    /// zero, which is the only thing that can make `chord=0` a strawman.
    chord_fallback: u64,
    /// The number of points in the set the chords are read off (`sets[0]`),
    /// so the fallback count above can be read against the range it needs.
    set_points: usize,
    /// Decisions whose index is not one the constellation holds at all.
    decided_out_of_set: u64,
    /// The direction of the level error the error is *about*: `true` when
    /// the far end is high.
    high: bool,
}

impl Default for B1uBins {
    fn default() -> Self {
        Self {
            sent: [0; 8],
            wrong: [0; 8],
            total: 0,
            total_wrong: 0,
            adjacent: 0,
            unpaired: 0,
            nearest_rival: 0,
            delta: [0; 33],
            delta_far: 0,
            max_abs_sent: 0,
            max_abs_decided: 0,
            chord_fallback: 0,
            set_points: 0,
            decided_out_of_set: 0,
            high: false,
        }
    }
}

/// `V92_B1U_DUMP`: the decisions `B1uBins::measure` is about to pair, with the
/// four `Parameters` facts `sent` is rebuilt from, so that a live call's
/// grading can be re-derived offline instead of re-run (`docs/287`).
///
/// Off unless the variable holds a path, and then one plain-text block per
/// `measure` call (one per polarity): `first`, `gain`, `moduli`, `set`, then
/// one `index position eta` triple per decided symbol. A diagnostic, so it may
/// not move a decision: it reads what the decoder already decided, after the
/// drain and never before it.
fn dump_decisions(parameters: &Parameters, decisions: &[(u64, [i32; 4])], first: usize) {
    let Some(path) = std::env::var_os("V92_B1U_DUMP") else { return };
    use std::io::Write as _;
    let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) else {
        return;
    };
    let set: Vec<String> = parameters.sets.first().map(|set| set.iter().map(|point| point.to_string()).collect()).unwrap_or_default();
    let _ = writeln!(file, "first {first}");
    let _ = writeln!(file, "gain {}", parameters.gain);
    let _ = writeln!(file, "moduli {}", parameters.moduli.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(" "));
    let _ = writeln!(file, "set {}", set.join(" "));
    for (index, etas) in decisions {
        for (position, &eta) in etas.iter().enumerate() {
            let _ = writeln!(file, "{index} {position} {eta}");
        }
    }
    let _ = file.flush();
}

impl B1uBins {
    /// Pair every decided index with the one 8.7.1 says was sent.
    ///
    /// `first` is the `B1u` symbol the decoder's frame 0 begins on, in the
    /// encoder's own numbering, and it has three terms:
    ///
    /// ```text
    /// first = skip + 1 + B - back
    /// ```
    ///
    /// `skip + 1` is where `b1u_decoders` puts the decoder relative to the
    /// boundary the candidate claims (the same `skip` that seeds the
    /// differential chain and the descrambler); `back` is the candidate's own
    /// claim -- `b1u_readings` feeds candidate `back` from `levels[skip -
    /// back]`; and `B` is how far before the run's end `B1u`'s frame 0 really
    /// is, which is the far end's E2u overrun and is **not** the candidate's
    /// `back` unless the search's winner is the true boundary.
    ///
    /// The call site hands `skip + 1 - back`, which is this expression with
    /// `B = 0` -- the clause's anchor, "B1u's frame interval 0 is the symbol
    /// after E2u's last one" (8.7.2), and *not* a measured fact: `begin_b1u`
    /// records the run overrunning E2u by two symbols on one Agere capture. The
    /// two terms are therefore only consistent when the far end agrees with the
    /// clause, and `docs/287` is the analysis of what happens when it does not:
    /// a wrong `B` displaces the pairing by whole data frames, which is the one
    /// displacement 8.7.1's content cannot see and the one `nearest_rival`'s
    /// sweep (`1 .. UP_INTERVALS`, i.e. 1 to 11) never tests.
    fn measure(parameters: &Parameters, decisions: &[(u64, [i32; 4])], first: usize, high: bool) -> Self {
        dump_decisions(parameters, decisions, first);
        let mut bins = Self { high, ..Self::default() };
        let Ok(mut encoder) = crate::v92::upstream::UpEncoder::new(parameters) else {
            return bins;
        };
        let sent: Vec<i32> = encoder.b1u().iter().map(|symbol| symbol.eta).collect();
        // **The shared representation is 6.4.2's equivalence class, not the
        // index** (`docs/285` sec 6). "The precoder identifies Ki with an
        // equivalence class E(Ki). The precoder selects a point u(n) from the
        // equivalence class E(Ki). The index of the constellation point u(n)
        // is denoted by y(n)" (v92.txt:449-451), and the class is "E(Ki) =
        // {a(n_k) | n_k = Ki + z_k Mi, z_k an integer} for k = 0, 1, 2; {a(n_k)
        // | n_k = 2Ki + 2 z_k Mi + (n_0 + n_1 + n_2 + Y0) mod 2, z_k an
        // integer} for k = 3" (v92.txt:458-459). The clause fixes the class and
        // leaves `z_k` to the precoder, so **the two indices this function used
        // to subtract are not the same number**: the encoder's `eta` is the
        // class member whose level is nearest `-c(n)` (`precoder.rs`,
        // `Constellation::choose`) and the decoder's is `Bests`' cheapest index
        // in its own residue class (`decoder.rs`, `Bests::new`), and each is
        // *a* member. Two members of one class differ by a multiple of `Mi`
        // (`2Mi` at k = 3) and carry the same `Ki`, which is the only thing
        // 6.4.1 reads back out of an index (`index_to_ki`); US6414989B1 col. 14
        // puts the licence plainly -- "the receiver need only be concerned with
        // the coset received and may ignore which coset member was
        // transmitted". So the comparison is `index_to_ki_class`, each interval
        // in its own radix, which is the convention
        // `examples/v92_b1u_class.rs` was corrected to at `b1d8223` after the
        // `+-Mi/2` / `max(moduli)` defect of `docs/274` sec 3. The numeric
        // difference is still kept, but only as `delta`: it says *how* the two
        // differ, and a multiple of `Mi` is the signature of a different member
        // and not of a misread symbol.
        let modulus_at = |symbol: usize| parameters.modulus(crate::v92::interval(symbol as u64));
        let position_at = |symbol: usize| crate::v92::trellis_index(symbol as u64);
        let class_of = |eta: i32, symbol: usize| -> u8 {
            crate::v92::precoder::index_to_ki_class(eta, modulus_at(symbol), position_at(symbol))
        };
        let sent_class: Vec<u8> =
            sent.iter().enumerate().map(|(symbol, &eta)| class_of(eta, symbol)).collect();
        let set = &parameters.sets[0];
        let constellation = crate::v92::precoder::Constellation::new(set);
        bins.set_points = set.len();
        bins.max_abs_sent = sent.iter().map(|eta| eta.abs()).max().unwrap_or(0);
        // The chord of a sent index, and whether the set had no level for it:
        // `unwrap_or(0)` is kept (`docs/253` sec 2 reads the chords of the sent
        // level and this instrument does not own that reading) but it is now
        // *counted*, because a fallback is the one way `chord=0 pop=100 %` can
        // be an artefact of the table rather than of the population.
        let chord_of = |eta: i32| -> (usize, bool) {
            let index = if eta >= 0 { eta as usize } else { (-eta - 1) as usize };
            let missing = index >= set.len();
            let value = f64::from(set.get(index).copied().unwrap_or(0));
            let full = (parameters.gain * value).clamp(0.0, 32767.0).round() as i32;
            let (code, _) = crate::v90::ucode::nearest(crate::v90::ucode::Law::Mu, full);
            (usize::from(code >> 4).min(7), missing)
        };
        let count = (decisions.len() * crate::v92::TRELLIS_FRAME) as u64;
        let wrong_at = |first: usize| -> Option<(u64, u64)> {
            let (mut wrong, mut paired) = (0u64, 0u64);
            for (index, etas) in decisions {
                for (position, &eta) in etas.iter().enumerate() {
                    let symbol = first + *index as usize * 4 + position;
                    if sent.get(symbol).is_none() {
                        continue;
                    }
                    paired += 1;
                    if class_of(eta, symbol) != sent_class[symbol] {
                        wrong += 1;
                    }
                }
            }
            // A displacement that cannot pair the same population is not a
            // rival reading of the same window, it is a shorter window.
            (paired == count).then_some((wrong, paired))
        };
        for (index, etas) in decisions {
            for (position, &eta) in etas.iter().enumerate() {
                let symbol = first + *index as usize * 4 + position;
                let Some(&sent_eta) = sent.get(symbol) else {
                    bins.unpaired += 1;
                    continue;
                };
                let (chord, missing) = chord_of(sent_eta);
                if missing {
                    bins.chord_fallback += 1;
                }
                bins.sent[chord] += 1;
                bins.total += 1;
                bins.count_delta(eta - sent_eta);
                if eta.abs() > bins.max_abs_decided {
                    bins.max_abs_decided = eta.abs();
                }
                if !constellation.holds(eta) {
                    bins.decided_out_of_set += 1;
                }
                if class_of(eta, symbol) != sent_class[symbol] {
                    bins.wrong[chord] += 1;
                    bins.total_wrong += 1;
                    if (eta - sent_eta).abs() == 1 {
                        bins.adjacent += 1;
                    }
                }
            }
        }
        // `docs/285` sec 2: the pairing's own honesty. B1u is forty-eight
        // *identical* data frames, so a whole-frame displacement is invisible
        // to everything that reads its content -- the ones count, the rotation,
        // the moduli -- and visible only here. The instrument therefore reads
        // the same decisions at every displacement of one to eleven symbols
        // either way and records the best of them: a right origin is
        // decisively better than every displaced one, and a displaced origin
        // has a rival that is decisively better than it. `unpaired` alone
        // could not see this, because a displacement keeps every index in
        // range and pairs every symbol -- which is how a call whose payload was
        // bit-perfect reported `unpaired=0` beside `bin=87.50%`.
        //
        // `docs/285` sec 6 leaves this in the *same* representation as the
        // number it guards: a displacement changes the interval and the trellis
        // position of every pair, so a class comparison is compared against the
        // class comparison a shift produces. `TRUST_MARGIN` and its verdict are
        // untouched.
        bins.nearest_rival = u64::MAX;
        for shift in 1..crate::v92::UP_INTERVALS {
            for displaced in [first.checked_sub(shift), Some(first + shift)] {
                let Some(displaced) = displaced else { continue };
                if let Some((wrong, _)) = wrong_at(displaced) {
                    bins.nearest_rival = bins.nearest_rival.min(wrong);
                }
            }
        }

        bins
    }

    /// Whether the rate this pairing reports can be published.
    ///
    /// The origin has to be decisively better than every displaced pairing of
    /// the same window ([`Self::TRUST_MARGIN`]), and every symbol has to have
    /// found its partner. An instrument that cannot say which of two origins a
    /// stream came from is not measuring the far end, it is measuring its own
    /// alignment, and `docs/285` sec 2 is the call that read 0.875 that way.
    fn trusted(&self) -> bool {
        self.total > 0
            && self.unpaired == 0
            && self.nearest_rival != u64::MAX
            && self.total_wrong * Self::TRUST_MARGIN <= self.nearest_rival
    }

    /// How much better than its nearest displaced rival a pairing has to be.
    ///
    /// Two: a right origin errs on the far end's symbols and a displaced one
    /// errs on nearly all of them, so four is a margin a call that is merely
    /// *noisy* still clears and a call that is *displaced* never does. A noisy
    /// window does not become a displaced one: `docs/285` sec 2's call read
    /// 0.875 -- chance -- against 0.000 for the right origin on the same
    /// decisions.
    const TRUST_MARGIN: u64 = 4;

    /// How wide the `eta - sent` table is either side of zero.
    const DELTA_MID: i32 = 16;

    /// One numeric index difference into the fixed table.
    fn count_delta(&mut self, delta: i32) {
        if delta.abs() > Self::DELTA_MID {
            self.delta_far += 1;
        } else {
            self.delta[(delta + Self::DELTA_MID) as usize] += 1;
        }
    }

    /// The population-weighted bin error rate, as a fraction.
    fn weighted(&self) -> f64 {
        if self.total == 0 {
            return f64::NAN;
        }
        let mut sum = 0.0;
        for chord in 0..8 {
            if self.sent[chord] == 0 {
                continue;
            }
            let share = self.sent[chord] as f64 / self.total as f64;
            sum += share * (self.wrong[chord] as f64 / self.sent[chord] as f64);
        }
        sum
    }

    /// What the comparison was handed, in the one number that says whether the
    /// instrument is reading a population or a table: the `eta - sent`
    /// histogram, then the set the chords are read off and the fallbacks in it.
    ///
    /// `docs/285` sec 6. Two wrong numbers with a delta of `+-Mi` are two
    /// members of one class; a delta that is a constant offset per position is
    /// a convention; a delta of one is a read error. Without this line the log
    /// says a rate and never says which of the three it is looking at.
    fn spread(&self, drn: u8) -> Vec<String> {
        let mut out = Vec::new();
        let mut parts = Vec::new();
        for (offset, &count) in self.delta.iter().enumerate() {
            if count == 0 {
                continue;
            }
            parts.push(format!("{:+}: {count}", offset as i32 - Self::DELTA_MID));
        }
        if self.delta_far != 0 {
            parts.push(format!("|d| > {}: {}", Self::DELTA_MID, self.delta_far));
        }
        if parts.is_empty() {
            parts.push("none".to_owned());
        }
        out.push(format!("V.92: BIN drn={drn} delta decided-sent: {}", parts.join(", ")));
        out.push(format!(
            "V.92: BIN drn={drn} set[0] {} points (indices -{} ..= {}), |eta| max sent {} decided {}, \
             chord unwrap_or fallbacks {}, decided outside the set {}",
            self.set_points,
            self.set_points,
            self.set_points as i32 - 1,
            self.max_abs_sent,
            self.max_abs_decided,
            self.chord_fallback,
            self.decided_out_of_set,
        ));
        out
    }

    /// The one line a call carries: the population, both rates, the adjacent
    /// share, and the per-chord table behind the weighted number.
    fn lines(&self, drn: u8) -> Vec<String> {
        let mut out = Vec::new();
        let pct = 100.0 * self.weighted();
        let adjacent = if self.total_wrong == 0 {
            "0".to_owned()
        } else {
            format!("{} of {} ({:.1} %)", self.adjacent, self.total_wrong, 100.0 * self.adjacent as f64 / self.total_wrong as f64)
        };
        out.push(format!(
            "V.92: BIN drn={drn} symbols={} bin={pct:.2}% adj={adjacent} unpaired={} \
             (decided 6.4.2 class against 8.7.1's sent class, population-weighted over the \u{b5}-law chords)",
            self.total, self.unpaired
        ));
        for chord in (0..8).rev() {
            if self.sent[chord] == 0 {
                continue;
            }
            out.push(format!(
                "V.92: BIN drn={drn} chord={chord} pop={:.1}% n={} bin={:.2}% nwrong={}",
                100.0 * self.sent[chord] as f64 / self.total.max(1) as f64,
                self.sent[chord],
                100.0 * self.wrong[chord] as f64 / self.sent[chord] as f64,
                self.wrong[chord],
            ));
        }
        out.extend(self.spread(drn));
        out
    }
}


/// What the rate ladder decided, in the numbers it decided on.
///
/// A rung being *refused* is a pass in this design, and it looks like
/// nothing happening: the measured residual and the ceiling it implies
/// therefore go in the log beside the rung that came out, so that a bench
/// call can be audited for the criterion and not only for its rate.
struct RateDecision {
    /// The Phase 3 fit's residual, in index steps of the LC = 8 grid, with
    /// the direction of its own error (`crate::v92::rung::Bound`).
    measured: Sigma,
    /// The highest rung that residual permits, or `None` when it permits
    /// none at all.
    ceiling: Option<u8>,
    /// The rungs on offer: the Ja mask AND the ceiling, or the pin when
    /// the bench set one.
    mask: u32,
    /// Whether it was the ceiling, rather than the mask, that left only the
    /// slowest rung -- i.e. whether this call is a refusal.
    empty: bool,
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
            su_found: None,
            su_seen: None,
            su_evidence: None,
            jp_sent: false,
            refit_from: None,
            refit_tries: 0,
            refit_poll: None,
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
            ru_detector: RuDetector::new(),
            data_loss: None,
            rm_detector: None,
            rm_responder_started: false,
            reentry: false,
            fpe: false,
            ru_responder_started: false,
            reneg: None,
            reneg_trn2u_from: None,
            cpd: None,
            parameters: None,
            cpu: None,
            up_scale: 1.0,
            up_scale_source: "design",
            suvu_level: None,
            e2u_zeroes: 0,
            awaiting_e2u: false,
            b1u_pending: false,
            b1u_deadline: None,
            start_up_origin: 0,
            upstream_bits: 0,
            outbound: VecDeque::new(),
            recovered: Vec::new(),
            downstream: None,
            b1d_left: 0,
            downstream_scrambler: Scrambler::new(Mode::Call),
            upstream_data: None,
            prefilter: prefilter_source(),
            upstream_plan: None,
            cpd_sent: false,
            upstream_scale: 1.0,
            upstream_delay: 0,
            b1u_skip: 0,
            b1u_probe: None,
            b1u_ones: [0; 2],
            b1u_scored: [0; 2],
            b1u_search: None,
            b1u_backlog: std::collections::VecDeque::new(),
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
        // `BINMODEM_V92_MAX_DRN` lifts the upstream cap for this call and this
        // call only (`rung::cap_override`). It is off by default and the
        // constant it lifts does not move, so a call that has it set is a
        // measurement above the cap rather than a working configuration --
        // which is a fact a transcript has to state at the top, in words that
        // cannot be mistaken for the ordinary cap line further down.
        if let Some(drn) = crate::v92::rung::cap_override() {
            modem.note(crate::v92::rung::cap_override_line(drn));
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
            // The origin `begin_upstream` took is where *this* end put the
            // reader on the line, which is an event of ours: the far end's
            // `TRN2u` begins when its transmitter comes on, and 9.6.2.1.1 puts
            // no signal of ours between the two (`docs/294` section 1). Where
            // the reader started early or late, the reference the fit reads is
            // a different stretch of 8.7.6's sequence and the fit describes
            // nothing -- which is the `no phase 4 fit on TRN2u` both HSF
            // attempts printed. The onset is measured on the history the fit
            // is about to read (`Receiver::signal_onset`), and it is used
            // nowhere else.
            if let Some(onset) = self.rx.signal_onset() {
                if onset != self.trn2u_from {
                    self.note(format!(
                        "V.92: TRN2u began at {onset}, {} symbols from the reader's origin {}",
                        onset as i64 - self.trn2u_from as i64,
                        self.trn2u_from
                    ));
                    self.trn2u_from = onset;
                }
            }
            // The **second** measurement, on the four-point training, taken
            // just before the fit that will inherit it: same window, same
            // reference, so what the aim maximises is what the data mode is
            // fitted with. `fe_install` rebuilds the history from the raw
            // stream, so the fit below is entirely on converted samples.
            if self.rx.fe_enabled() && fe_aim_wants(true) {
                let forced = std::env::var("V92_FE_DELTA").ok().and_then(|t| t.parse::<f64>().ok());
                // The aim's reference is Table 28's four points, and it is
                // only the far end's sequence when this end asked for four
                // (Jp bit 48, 8.7.6). Today `upstream_size` is fixed at four,
                // so this is a guard against the day it is not: a four-point
                // reference fitted against an eight-point line would give a
                // plausible number for a phase that is not being measured.
                let four = self.upstream_size == Trn2uSize::Four;
                if !four {
                    self.note("V.92: no aim at TRN2u: the reference is four points".into());
                }
                let aimed = forced.map(|d| (d, f64::NAN)).or_else(|| {
                    if four {
                        self.rx.fe_aim_four(self.trn2u_from, fe_window(TRAIN_WINDOW))
                    } else {
                        None
                    }
                });
                if let Some((delta, figure)) = aimed {
                    self.rx.fe_install(delta);
                    self.note(format!(
                        "V.92: fractional delay {delta:.3} at TRN2u (four-point residual {figure:.5} held out)"
                    ));
                }
            }
            if self.rx.fe_enabled() && fe_metering() {
                if let Some((delta, figure)) = self.rx.fe_aim_four(self.trn2u_from, fe_window(TRAIN_WINDOW)) {
                    self.note(format!(
                        "V.92: phase meter, TRN2u: {delta:.3} symbols (four-point residual {figure:.5})"
                    ));
                }
            }
            if self.rx.fe_enabled() && std::env::var_os("V92_FE_DEBUG").is_some() {
                self.rx.fe_report_four(self.trn2u_from, fe_window(TRAIN_WINDOW));
            }
            let fit_started = std::time::Instant::now();
            if self.rx.train_on_trn2u(TRAIN_WINDOW, self.trn2u_from) && self.rx.install_phase_four() {
                let fit_ms = fit_started.elapsed().as_secs_f64() * 1000.0;
                self.upstream_drop_until = self.now + TRN2U_DROP_AFTER_FIT;
                // What the fit replaced, on its own window: the four-point
                // figure and the design's own, side by side, so the call says
                // which constellation each was measured on instead of leaving
                // the reader to guess whether the two are comparable.
                // The figure, the alignment it was solved at, the window's own
                // best and the design's own figure, all four on one line: a
                // residual cannot be read as a fit when the peak it is measured
                // against is printed beside it (`docs/298` section 3).
                let design = self
                    .rx
                    .last_fit_pair()
                    .map_or(f64::NAN, |(held, _)| held);
                self.note(format!(
                    "V.92: phase 4 receiver fitted on TRN2u in {fit_ms:.2} ms, {:.1} dB at \
                     alignment {} (peak {:.1} dB), against {design:.1} dB for the design on the \
                     same window",
                    self.rx.snr_db(),
                    self.alignment_line(),
                    self.peak_db()
                ));
                // The measurement that chooses the upstream rung exists from
                // here: the receiver's own residual on a four-point stream with
                // data mode's symbol rate, filter and power. See
                // `decide_upstream_rung` for what the figure is and which way
                // each step of its conversion errs.
                self.decide_upstream_rung();
            } else {
                self.upstream_drop_until = self.upstream_drop_until.min(self.now);
                let design = self.rx.last_fit_pair().map_or(f64::NAN, |(held, _)| held);
                self.note(format!(
                    "V.92: no phase 4 fit on TRN2u: the best alignment reads {:.1} dB against \
                     {:.1} dB for the window's own best (solved alignment {}, design \
                     {design:.1} dB on the same window)",
                    self.rx.last_fit_pair().map_or(f64::NAN, |(_, fitted)| fitted),
                    self.peak_db(),
                    self.alignment_line()
                ));
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
            // What the far end is actually asked for: epsilon as measured, plus
            // the bench's offset. Both the wire field and our own expectation
            // must carry the same number, because `expect_step` is what the
            // receive hunt waits for (9.5.2.1.8).
            let offset = jp_offset();
            let epsilon = if offset == 0.0 {
                epsilon
            } else {
                let wanted = epsilon::to_field(f64::from(epsilon) / 65536.0 + offset);
                self.note(format!(
                    "V.92: Jp epsilon {epsilon} -> {wanted} (bench offset {offset:+.4} symbols)"
                ));
                wanted
            };
            // Bits 48 and 49 are 0: 4-point upstream training constellations.
            // 8.7.6 (`v92.txt:1717`) says TRN2u "is a 4- or 8-point
            // constellation signal **as requested by the digital modem via Jp
            // bits 48 and 49**", and Table 22's own wording (`v92.txt:1420-1424`)
            // is `0 = 4-point; 1 = 8-point` for CPu, E2u, SUVu and TRN2u in
            // training (bit 48) and in rate renegotiation (bit 49). The clause
            // fixes neither: it asks which, and the answer is ours.
            //
            // Ours is 4-point, and it is the default every call this end has
            // made: the upstream training sequences are *received* here, and
            // every one of them is demodulated against Tables 28's 4-point maps
            // (TRN2u's sign, Table 29, is 8-point and unused). Asking for
            // 8-point without the receiver beside it would put a training
            // signal on the line that this end cannot read, which is the same
            // class of breakage `EXTEND_E2U`'s constant exists to prevent. So
            // the default is correct rather than merely harmless, and
            // `a_four_point_training_constellation_is_what_we_ask_for` pins it.
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
            && self.refit_poll.is_none_or(|at| self.rx.symbols() >= at)
        {
            let base = self.rx.history_base();
            let at = from.saturating_sub(base);
            let before = self.rx.fitted();
            let baseline = before.snr();
            // Where the six-symbol upstream actually stopped, measured on the
            // line: Su and S-bar-u are one period of six (8.5.6), the second
            // TRN1u is scrambled two-point (8.7.6), and the point between them
            // is the one thing about this boundary that can be *measured*
            // instead of assumed. `None` means the upstream has not stopped
            // repeating: the far end has not begun its TRN1u yet.
            let measured = self.rx.su_ends(REFIT_MEASURE, SU_ENDS_WINDOW, SU_ENDS_HOLD);
            // The timetable's own estimate of where the second TRN1u begins,
            // chosen by the correlation and the fit exactly as it always was.
            // This is the refit on every call whose far end is conforming: the
            // measurement below is a **recovery path and not a routine one**,
            // and it runs only where this fit says the line was not fitted at
            // all -- see [`REFIT_ESTIMATE_DB`]. A conforming far end therefore
            // pays exactly what it always paid, which is what keeps its frame
            // origin and its critical window where they were measured.
            let timed = self
                .rx
                .align_trn1u_by_fit(at, REFIT_SEARCH, ALIGN_WINDOW, REFIT_QUALITY, TRAIN_WINDOW);
            let usable = timed.is_some_and(|(_, fit)| fit >= REFIT_ESTIMATE_DB);
            // And the recovery: the search the line itself places, one rung per
            // look, because a fit is about a millisecond and this loop has eight
            // samples in one -- so the ladder is a rung at a time with the taps
            // handed back between rungs (`Receiver::ladder_step`). Seventeen
            // rungs is a hundred milliseconds of wall clock and no burst at all.
            let ladder = match (usable, measured) {
                (false, Some(onset)) => {
                    Some(self.rx.ladder_step(onset, REFIT_SWEEP, REFIT_LADDER, TRAIN_WINDOW))
                }
                _ => None,
            };
            // Which is a reason to wait, not a reason to fit: a fit taken here
            // is a fit against Su, and that is where 6.4 dB came from. Bounded
            // by [`REFIT_WAIT`], past which the line is taken as it stands --
            // and looked at once every [`REFIT_POLL`] symbols rather than once
            // a symbol, because the measurement is a scan of history.
            let busy = matches!(ladder, Some(LadderStep::Continue));
            let waiting = busy
                || (!usable && measured.is_none() && self.rx.symbols() < from + REFIT_WAIT);
            self.refit_poll = waiting.then(|| self.rx.symbols() + REFIT_POLL);
            if !waiting {
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
                // Aimed on *this* TRN1u, the second one: it follows our TRN1d, so
                // by now the far end's transmit clock is locked to our rate and the
                // upstream fractional phase is a property of the loop rather than a
                // transient -- which is what the first TRN1u gives, and why aiming
                // there produced 0.590 while this one produced 0.760 (`docs/83`
                // §3.5).
                // `fe_install` rebuilds the whole history from the raw stream, so
                // the refit taken below sees one phase throughout.
                // The **first** of the two measurements, and its target is CPt's
                // and not the eye's: a two-point CPt is read by the linear fit
                // taken here, so what has to be right is that the fit describes
                // the line at the phase it will be installed at. `fe_aim_linear`
                // maximises that fit; `docs/83` §3.6 measured why the eye criterion
                // is not a substitute -- on a two-point stream its whole curve
                // moves one per cent and it ranks the phase *against* the one the
                // data mode needs.
                if self.rx.fe_enabled() && fe_aim_wants(false) {
                    let forced = std::env::var("V92_FE_DELTA").ok().and_then(|t| t.parse::<f64>().ok());
                    let aimed = forced
                        .map(|d| (d, f64::NAN))
                        .or_else(|| self.rx.fe_aim_linear(at as usize, REFIT_WINDOW));
                    if let Some((delta, figure)) = aimed {
                        self.rx.fe_install(delta);
                        self.note(format!(
                            "V.92: fractional delay {delta:.3} at TRN1u 2 (linear fit {figure:.2} dB)"
                        ));
                    }
                }
                if self.rx.fe_enabled() && fe_metering() {
                    // Report only: the number is the far end's phase error, and the
                    // place to act on it is the next call's Jp, not this receiver.
                    if let Some((delta, figure)) = self.rx.fe_aim_linear(at as usize, REFIT_WINDOW) {
                        self.note(format!(
                            "V.92: phase meter, second TRN1u: {delta:.3} symbols (linear fit {figure:.2} dB)"
                        ));
                    }
                }
                if self.rx.fe_enabled() && std::env::var_os("V92_FE_DEBUG").is_some() {
                    self.rx.fe_report_two(at as usize, REFIT_WINDOW);
                }
                // The measured walk first: coarse to fine around the point where
                // the upstream stopped repeating (see `Receiver::align_trn1u_from`).
                // It returns an absolute symbol count, the estimate below is a
                // history index, and everything downstream is the index.
                let swept = match ladder {
                    Some(LadderStep::Done(at, fit)) => Some((at.saturating_sub(base), fit)),
                    _ => None,
                };
                let mut search: Option<(u64, f64)> = None;
                let chosen = match (timed, swept) {
                    // The estimate stands unless the line says the fit taken at it
                    // cannot read a two-point CPt at all: see [`REFIT_ESTIMATE_DB`].
                    (Some((timed_at, timed_fit)), Some((swept_at, swept_fit)))
                        if timed_fit < REFIT_ESTIMATE_DB && swept_fit > timed_fit =>
                    {
                        self.note(format!(
                            "V.92: second TRN1u at {}, measured from the end of the six-symbol \
                             upstream at {} ({swept_fit:.1} dB, against {timed_fit:.1} dB at {} for \
                             the timetable's estimate)",
                            base + swept_at,
                            measured.unwrap_or(0),
                            base + timed_at
                        ));
                        search = Some((swept_at, swept_fit));
                        Some(swept_at)
                    }
                    (Some((timed_at, timed_fit)), _) => {
                        search = Some((timed_at, timed_fit));
                        Some(timed_at)
                    }
                    (None, Some((swept_at, swept_fit))) => {
                        search = Some((swept_at, swept_fit));
                        Some(swept_at)
                    }
                    (None, None) => None,
                };
                let found = match chosen {
                    Some(aligned) => {
                        let fit = search.map_or(f64::NAN, |(_, fit)| fit);
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
            self.fail("no E2u that reads as B1u within 9.6.1.2.1's 20 s plus 6 round-trip delays of the end of INFO1a");
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
            // The aim is **not** taken here. `docs/83` §3.5, corrected by the
            // user against the Recommendation: this is the TRN1u *before* our
            // TRN1d, so the far end's transmit loop is not yet locked to our
            // rate and the fractional phase on the line is a transient rather
            // than a property of the loop. Measured on the same capture, the
            // same criterion reads 0.590 here and 0.760 on the second TRN1u.
            // The first legal measurement is the second TRN1u, below; the
            // second is TRN2u, in `timers`.
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
        // Phase 3 keeps to a **timetable**, not to an event (docs/131).
        //
        // 9.5.2.1.6/.7 make the far end's obligation a schedule: Su for 144T,
        // its 24.5T S-bar-u, then Su again until Jp. The Recommendation's one
        // deadline here is 9.5.1.2.2's 5.1 s plus a round trip for Su to
        // *arrive*, kept below and untouched; it asks for nothing to be
        // detected. And on a band-limited line there is nothing to detect --
        // this bench's HSF reverses at 144T on the antialias-off arm and is one
        // unmodulated tone on the default `minphase` arm, because its
        // decimator removes Su's 4 kHz component. The hunt is right and
        // scale-invariant (docs/122); the criterion was wrong.
        //
        // So the schedule advances the stage, from `S0` = the onset the hunt
        // reported minus its own lag, and `held`/`coherence` are evidence that
        // gets logged and permission for nothing. The reversal detector is
        // untouched and runs first: when it does fire it confirms the
        // schedule, re-anchors everything downstream on where it actually
        // landed, and its epsilon window opens on the Su *after* it
        // (9.5.1.1.9), as before.
        if let Some(found) = self.su_found {
            let s0 = found.saturating_sub(SU_REPORT_LAG);
            if self.su_seen.is_some() {
                self.su_advance(found);
            } else if self.stage == Stage::AwaitSu {
                // Evidence, once a timer tick: reported rather than
                // consulted. The longest hold seen and not the latest reading,
                // because the hunt re-anchors the moment it calls the turn and
                // would report the new anchor's zero from then on.
                if let Some(held) = self.rx.su_held() {
                    let longer = match self.su_evidence {
                        Some((best, _)) => held.0 >= best,
                        None => true,
                    };
                    if longer {
                        self.su_evidence = Some(held);
                    }
                }
                if blind_su() {
                    self.su_advance(found);
                } else if su_tolerance() && self.now >= s0 + SU_TIMETABLE {
                    self.su_advance(found);
                }
            }
        }
        if let Some(at) = self.su_deadline
            && self.now > at
            && self.stage == Stage::AwaitSu
            && self.source.out() == Out::Jd
        {
            self.su_deadline = None;
            // 9.5.1.2.2 gives 5100 ms from the start of TRN1d, and the three
            // ways to spend them -- no Su, Su that never turned over, Su that
            // turned over with a step this end did not expect -- are one line
            // of log and three different fixes. The hunt is still holding the
            // answer when the deadline arrives, so say it: a case that cost a
            // capture and an afternoon to tell apart is a case the log can
            // report for the price of one note (docs/97).
            if let Some((kind, _, held, stray, step)) = self.rx.hunt_state() {
                self.note(format!(
                    "V.92: the hunt was still holding {kind} after {held} periods \
                     ({stray} misses in a row, expecting a step of {step:.3})"
                ));
            } else {
                self.note("V.92: the hunt was holding nothing at the deadline".into());
            }
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
                    self.su_found = Some(at);
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

    /// Go on as if 9.5.2.1.7's `S-bar-u` had arrived, at the position it would
    /// have taken.
    ///
    /// [9.5.2.1.7] fixes *where* it goes: Jp's epsilon is measured on the Su
    /// that follows the reversal, and the far end's answer is (24 + epsilon)T
    /// after that ([9.5.2.1.8]), so the position is 144T after the onset and
    /// not wherever this end happened to notice anything -- which is why the
    /// timetable and the detector agree to the symbol when the detector does
    /// fire.
    fn proceed_as_if_reversed(&mut self, found: u64) {
        let at = found + crate::v92::SU_SYMBOLS as u64 - SU_REPORT_LAG;
        self.su_reversal(at);
    }

    /// The one line Phase 3 gives to account for its Su, and the advance the
    /// timetable owes this end once it is due (docs/131).
    ///
    /// Called from [`Self::timers`] and never from the symbol path: what it
    /// reports about the line -- `held`, `coherence`, the reversal symbol -- is
    /// evidence for the log, and none of it decides anything.
    fn su_advance(&mut self, found: u64) {
        let s0 = found.saturating_sub(SU_REPORT_LAG);
        let scheduled = s0 + crate::v92::SU_SYMBOLS as u64;
        let seen = self.su_seen.take();
        self.su_found = None;
        let evidence = match self.su_evidence.take() {
            Some((held, coherence)) => {
                format!("Su held {held} periods, coherent at {coherence:.3}")
            }
            None => "Su held for no whole period".to_string(),
        };
        let reversal = match seen {
            Some(at) => format!(
                "reversal seen at {at} (S0+{}T, {:+}T off the schedule)",
                at.saturating_sub(s0),
                at as i64 - scheduled as i64
            ),
            None => "reversal absent".to_string(),
        };
        let advance = match seen {
            Some(_) => "advanced on the reversal".to_string(),
            None => format!("advanced on timetable at S0+{SU_TIMETABLE}T"),
        };
        self.note(format!(
            "phase 3: Su at S0 {s0}; {reversal}; {evidence}; {advance}; epsilon window open"
        ));
        if seen.is_none() {
            self.proceed_as_if_reversed(found);
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
        self.su_found = None;
        self.su_seen = None;
        self.su_evidence = None;
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
        // 9.8's tail is not Phase 4's: 9.6.1.1.5's B1d is not where a
        // renegotiation ends, and the exchange it runs is Context::Renegotiation.
        if self.reneg.is_some() {
            self.reneg_out();
            return;
        }
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
                self.cpd_sent = true;
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
        // `CPD_WAITS_FOR_PARKED_TX`: the far end answers a `CPd` only when its
        // own transmit chain is parked, so the one `CPd` waits one window after
        // its `CPu` instead of chasing it.
        exchange.hold_cp_for_parked_tx();
        // And it waits for the `CPu` itself, which is the same thing one step
        // earlier: the `CPd` is what acknowledges the far end's `CPu`
        // (9.6.1.1.2's bit 33), so a `CPd` released before that `CPu` has been
        // decoded carries the bit clear *and* lands inside the sequence it is
        // answering. The renegotiation's exchange already waits; the initial
        // Phase 4 did not, and its `CPd` therefore went out unacknowledged
        // whenever the far end's `CPu` was decoded a moment too late -- on 3
        // of 3 calls at answer point 170 it was 20 ms too late (CPd 22.340,
        // `ack false`; `CPu` 22.360), and the far end fell forward with
        // `0x6009521a` 1.002 s after data mode. The wait is bounded by
        // [`Exchange::peer_cp_wait`].
        exchange.wait_for_peer_cp(true);
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
        let from = self.rx.symbols();
        self.begin_upstream_at(from);
    }

    /// What the last TRN2u fit solved its alignment at, in the receiver's own
    /// symbol instants, out of how many alignments were tried.
    ///
    /// A fit is only a fit at the alignment the four points themselves pick:
    /// `docs/297` printed `1.3 dB` on a window that peaks at `18.75` dB and
    /// called it `fitted`, which is a residual read as a success. The number is
    /// printed on the same line as the decibels so that the split between
    /// `origin` and the channel delay is visible rather than absorbed by
    /// `CHANNEL_DELAY`.
    fn alignment_line(&self) -> String {
        self.rx
            .last_alignment()
            .map(|(lag, _, tried)| format!("{lag} of {tried}"))
            .unwrap_or_else(|| "at no alignment".to_owned())
    }

    /// The best any alignment on the same window reads, in decibels: the figure
    /// a residual is measured against.
    fn peak_db(&self) -> f64 {
        self.rx.last_alignment().map_or(f64::NAN, |(_, peak, _)| peak)
    }

    /// [`Self::begin_upstream`] with 8.7.6's fiducial named by the caller.
    ///
    /// `from` is the absolute symbol index of TRN2u's **first** symbol, which
    /// is the symbol after the last `E1u` symbol the reader counted. The E1u
    /// path knows it (`symbol.raw + 1`); every other caller -- a rate
    /// renegotiation's fiducial, the SCR path, the timetable -- reaches here
    /// with no symbol in hand and passes the newest sample, which is what this
    /// took before and is right for them because nothing between them and
    /// TRN2u is being counted off a centred decision.
    fn begin_upstream_at(&mut self, from: u64) {
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
        let start = from;
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
        // 9.6.1.2.1 is not conditioned on anything this end receives: "If the
        // digital modem does not receive B1u within 20 s plus 6 round-trip
        // delays from the end of INFO1a, the digital modem shall initiate a
        // retrain". The arm it had was `enter_data_mode`'s and
        // `abandon_b1u`'s, and both need an upstream sequence to have been
        // decoded -- so a Phase 4 whose reader decodes nothing at all has no
        // deadline and waits for ever, which is what `docs/298` section 1
        // found: `TRN2d done, SUVd out` and no further line until the
        // harness's own connect timeout. Armed here, from `start_up_by`'s
        // absolute origin, a silent reader becomes the retrain the clause asks
        // for instead of a call that says nothing.
        if self.b1u_deadline.is_none() {
            self.b1u_pending = true;
            self.b1u_deadline = Some(self.start_up_by());
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
                        "V92TRACE: CPd drn {} gain {:.4} moduli {:?} sets {:?} filters z1 {} p1 {} z2 {} p2 {} limits {:?}/{:?}/{:?} coeffs z1 {:?} p1 {:?} z2 {:?} p2 {:?}",
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
                        self.settings.limits.most,
                        f.z1,
                        f.p1,
                        f.z2,
                        f.p2
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
                // The level set is named here, once per call, because it is the
                // one thing about this CPd a batch has to be able to tell from
                // the log: the two arms of an A/B differ in these points and in
                // nothing else, so `levelset uniform` and `levelset code` are
                // what separates them without an environment dump.
                self.note(format!(
                    "V.92: upstream drn {}, {:.1} kbit/s, levelset {}",
                    parameters.drn,
                    f64::from(parameters.rate()) / 1000.0,
                    parameters.levelset.name()
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
        // ---- The rung is not decided here ----
        //
        // The rate this end asks for has two inputs. The analogue modem's Ja
        // says which rungs it will offer at all (Table 20); the *measurement*
        // says which of those the line will carry, and that is a ceiling
        // (`crate::v92::rung`):
        //
        //     mask = (Ja mask) AND (the rungs the measured residual permits)
        //
        // A ceiling can only take a rung away, and nothing raises one by
        // assuming a better line than the one that was measured. What is wrong
        // with the number that used to be used here is not its direction, it is
        // that it is not a measurement of the line.
        //
        // **What was here, and why it was sixteen times too pessimistic.** The
        // rung was chosen from `Receiver::snr_db()` as it stood after the Phase 3
        // TRN1u fit, through `Sigma::from_design`. `read_window` defines that
        // figure as
        //
        //     snr_db = 10 log10( mean(reference^2) / mean(error^2) )
        //
        // over the fit's own window, one value per symbol. TRN1u's reference is
        // its two points at `+-LU` = `+-1` (`up_signals::LU`), so its mean square
        // is exactly one, and the decibel figure *is* a per-symbol residual
        // variance in the units the levels are in -- the units in which one
        // half-cell is `G` and one index step is `2G`. Taken at face value, with
        // no bandwidth factor anywhere and no fitted constant,
        //
        //     sigma = sqrt(10^(-snr/10)) / (2G)          index steps
        //
        // and at the 13.5 dB the second TRN1u fits at on this bench that is
        // **0.975 index steps**. The same call's own B1u *levels* read 0.0887
        // offline. The arithmetic was therefore not the defect; the *quantity*
        // is. TRN1u is a two-point stream whose transitions this end's
        // fractional-delay aim lands half a symbol off the network's instants, so
        // its fit residual is dominated by uncancelled ISI and by sampling phase
        // -- the receiver's own note is "19 dB against 166 dB on the instants" --
        // and that residual is a property of TRN1u, not of the line: the same
        // receiver reads 36.7 dB on the four-point TRN2u twenty seconds later.
        // No bandwidth or matched-filter argument maps one onto the other,
        // because what differs is not noise through another bandwidth, it is ISI
        // the data mode does not have. So the figure is not converted. It is
        // replaced.
        //
        // **What chooses the rung instead** is the same receiver's residual on a
        // stream with data mode's character: TRN2u (8.7.6) is four points at
        // unit mean square, the same symbol rate and the same transmit filter as
        // B1u, and the Phase 4 receiver's fit against it reads the residual at
        // the very output the slice will read. It exists 2400 symbols into
        // TRN2u, and a CPd follows TRN2u in protocol order (9.6.1.1: SUVd,
        // TRN2u/SUVu, then CPd), so the rung is chosen there, in
        // `Self::decide_upstream_rung` -- the one place a rung is chosen on a
        // measurement.
        //
        // Until then a CPd has to exist, because a line whose TRN2u this
        // receiver cannot fit must still be told something. What "nothing
        // measured yet" permits is the ladder's slowest, so that is the
        // provisional CPd built here, through `pin` rather than `choose` for the
        // reason the floor always was: `choose`'s spacing sanity rule is for a
        // caller with no measurement, and it must not veto the one rung the call
        // cannot do without.
        const SLOWEST_DRN: u8 = 1;
        let trellis = Trellis::Sixteen;
        // The bench's stress arm, and nothing else. `V92_UP_RATE=48000` pins the
        // rung naming that rate so that the *receive* path can be measured at a
        // spacing the design would otherwise refuse. Unset -- which is every call
        // a user makes -- nothing below runs, and the mask is Ja's, narrowed by
        // the measurement. An unparsable or off-ladder rate is a bench mistake
        // and leaves the default alone rather than silently choosing something
        // else. A pinned rung consults no measurement, so it is built here and no
        // measurement is waited for.
        let pinned = std::env::var("V92_UP_RATE").ok().and_then(|rate| {
            let rate = rate.trim().parse::<u32>().ok()?;
            (1..=crate::v92::UP_RATES as u8).rev().find(|&drn| crate::v92::up_rate(drn) == rate)
        });
        // A build that cannot make the slowest rung cannot make any of them, and
        // a design that is legal in no field of Table 30 at all is answered with
        // the identity rather than with silence (see `upstream_design`'s note).
        let build_floor = |designed: &Design| pin(designed, SLOWEST_DRN, trellis, 0.0);
        let mut floor = if let Some(drn) = pinned { pin(&designed, drn, trellis, 0.0) } else { build_floor(&designed) };
        if floor.is_none() && degraded.is_none() {
            let residual = designed.residual_db();
            degraded = Some("no rung of the 6.1 ladder carries what the design leaves");
            designed = crate::v92::design::identity(&channel);
            // No filter is being sent, so nothing was scaled: the far end's
            // output is on the levels' own scale.
            self.upstream_scale = 1.0;
            self.upstream_delay = designed.delay;
            eprintln!(
                "V92TRACE: identity fallback: residual {residual:.1} dB, channel noise {:.2e}",
                channel.noise
            );
            floor = build_floor(&designed);
        }
        if let Some(reason) = degraded {
            self.note(format!("V.92: upstream design degraded to identity ({reason})"));
        }
        let Some(mut choice) = floor else {
            eprintln!(
                "V92TRACE: the ladder's slowest rung cannot be built from this design (residual {:.1} dB)",
                designed.residual_db()
            );
            return None;
        };
        // Diagnostic: force a four-point constellation, so that what the far end
        // puts on the line can be counted in levels. A CPd it obeys and a CPd it
        // ignores look nothing alike.
        if std::env::var_os("V92_TINY").is_some() {
            choice.parameters.moduli = [4u8; UP_INTERVALS];
            choice.parameters.sets = vec![vec![1, 3, 5, 7]];
        }
        if tracing() {
            eprintln!(
                "V92TRACE: upstream design scale {:.2}, residual {:.1} dB; the rung waits for the four-point fit",
                designed.scale,
                designed.residual_db()
            );
        }
        if let Some(drn) = pinned {
            self.note(format!(
                "V.92: upstream rung pinned to drn {drn} at {} bit/s (V92_UP_RATE, no measurement consulted)",
                choice.parameters.rate()
            ));
            return Some(choice.parameters);
        }
        self.upstream_plan = Some(designed);
        self.note(format!(
            "V.92: the rung waits for the four-point measurement -- nothing before TRN2u reads the \
             line at the slice; CPd provisionally drn {} at {} bit/s",
            choice.parameters.drn,
            choice.parameters.rate()
        ));
        Some(choice.parameters)
    }

    /// The rung, chosen where it has an input: the Phase 4 receiver's residual on
    /// TRN2u.
    ///
    /// # The conversion, and the assumption behind it
    ///
    /// `Receiver::snr_db()` after `train_on_trn2u` is `read_window`'s figure for
    /// the window the Phase 4 receiver was fitted on: `10 log10(mean(reference^2)
    /// / mean(error^2))`, one value per symbol, against Table 28's four points,
    /// whose mean square is exactly one (`(1/5 + 9/5) / 2`). So
    ///
    /// ```text
    /// sigma_abs    = sqrt(10^(-snr/10))        the levels' own units
    /// sigma_steps  = sigma_abs / (2 G(8))      index steps on the LC = 8 grid
    /// ```
    ///
    /// and the assumption is the one that makes that legitimate: the reference's
    /// mean square is one, so the ratio is already a variance in the levels' own
    /// units; `G` is 8.8.3's gain for a uniform set brought to mean square one;
    /// and the residual is measured at the equaliser's output, one sample per
    /// symbol, which is exactly where the slice will read it. No bandwidth, no
    /// matched filter and no constant from anywhere else enter.
    ///
    /// **Which way each step errs.** The ratio-to-amplitude step and the `2G`
    /// step are exact given those two conventions. The window is not, and it errs
    /// one way only:
    ///
    /// 1. the fit is in-sample, so the residual it reads is *lower* than the
    ///    residual on symbols the fit never saw -- low;
    /// 2. the decision-feedback section is fed the *known* symbols throughout
    ///    TRN2u, and one wrong decision feeding the next is exactly what a DFE
    ///    does in data mode, so training never shows that propagation -- low;
    /// 3. the far end's precoder is off during TRN2u (9.6.1.1 puts it in the
    ///    data mode), so the ISI it does not cancel is invisible here and lands
    ///    in data mode only -- low;
    /// 4. TRN2u is at 21 s and B1u at 26 s, so the line may move in between, in
    ///    either direction -- unmeasured, and the only step that is not one-way.
    ///
    /// Steps 1 to 3 are all one way, so this is a [`Bound::Lower`]: the
    /// data-mode residual is **no smaller** than the number, and by how much is
    /// measurable rather than argued -- on the c24-4 capture this reads 0.0674
    /// index steps (36.7 dB) where the same call's B1u *levels* read 0.0887
    /// (33.4 dB), a factor of 1.32 in sigma and 3.3 dB in the ratio. That is why
    /// the bound is not [`Bound::Reading`] and certainly not `Upper`: a training
    /// stream cannot see what only decisions produce.
    ///
    /// [`Bound::Lower`] is then spent the way `crate::v92::rung` says to spend
    /// it -- at the slowest rung, because a lower bound cannot justify anything
    /// faster -- and that is the honest outcome rather than a failure of the
    /// criterion: the only residual that exists before CPd is one measured on
    /// known symbols, so no pre-CPd measurement can license a rung the line has
    /// not already been watched carrying. The number is *reported* either way,
    /// so a call says what the line looked like and what it was allowed.
    /// **The ladder, on the live decision** (`docs/264` sec 2): which rungs the
    /// level measured on this call permits, by `docs/261` sec 4's per-rung
    /// tolerance, with every rung's verdict written to the log in
    /// `rung::Reading::line`'s own form.
    ///
    /// # Why the level and not the rate
    ///
    /// `CPd` carries the rung (9.6.1.1.2) and goes out **before** `B1u` exists,
    /// so at the instant this decision is taken there is no `B1u` window for the
    /// bin error rate to be measured over. The one measurement there is, is the
    /// level: the four-point fit's gain against the design's own four points,
    /// which is the far end's distance from the `1/G` this end named in the
    /// previous `CPd` (`docs/256` sec 1.2; 8.8.3 makes `1/G` the level the
    /// design's points arrive at). So the ladder is read with `symbols = 0` --
    /// no rate, `instrument=level` -- and `admit` says exactly that rather than
    /// reporting a zero it did not measure.
    ///
    /// The residual sigma is *not* the gate. It is a `Bound::Lower` on the
    /// data-mode residual (see `decide_upstream_rung`), it refuses rungs on
    /// evidence that cannot justify any, and on the bench it floors the ladder
    /// at `drn 1` whatever the level does (`docs/256` sec 3.1) -- a criterion
    /// that answers "24 000" before it looks. It stays in the log as an
    /// advisory number beside the gate, so that a call shows both and neither is
    /// inferred from the other.
    ///
    /// # The cap
    ///
    /// The mask this returns is **cut to
    /// `rung::MEASURED_WORKING_MAX_DRN`** before it is returned, whatever the
    /// ladder granted: the ladder is printed in full -- one `LADDER` line per
    /// rung, saying what each instrument says -- and then
    /// `rung::cap_line` says what that grant would have been and what the call
    /// actually gets (`docs/266` sec 4: 33 333 and 25 333 do not carry data,
    /// only 24 000 is measured working; `docs/269` sec 3: why `drn 2` fails is
    /// open, and `drn 7` fails too). The tolerance table and `admit` are kept
    /// exactly as they were: they are the instrument, not the licence.
    ///
    /// # When there is no level
    ///
    /// A call whose four-point measurement never ran has no `delta` to judge, so
    /// there is nothing for the tolerance to be about: the ladder is not
    /// consulted, the sigma mask stands as it did, and the log says so. That is
    /// the only branch in which the old criterion still decides.
    fn upstream_ladder(&mut self, ja: u32, measured: Sigma, ceiling: Option<u8>) -> (u32, RateDecision) {
        let (inner, outer, level_gain, seen) = self.rx.measured_levels();
        let mut mask = 0u32;
        if seen > 0 {
            // The far end's distance from `1/G`, as a fraction: the four-point
            // gain the receiver settled on is the ratio between the amplitude
            // arriving and the design's own `1/sqrt(5)`, `3/sqrt(5)`, so one
            // means the far end put the design's points on the line.
            let delta = level_gain - 1.0;
            for drn in 1..=crate::v92::UP_RATES as u8 {
                if ja & (1 << (drn - 1)) == 0 {
                    continue;
                }
                let lc = crate::v92::rung::lc_for(drn);
                // The declared set's own `G` for this rung, which is the number
                // `CPd` carries: `1/G` is then the level the tolerance is
                // measured from, and `delta` is the same relative error at every
                // rung. Taken from the declaration rather than from the odd
                // lattice's closed form, so the `level=` and `1/G=` fields on the
                // LADDER line are in the same currency as the set that goes out
                // (`docs/310` sec 2.1: at 24 000 the code grid asks for 1/G of
                // 1 620 LSB where the lattice's closed form names 9.22 index
                // units). The *verdict* does not move: `level_lsb` is built from
                // the same gain, so `delta` -- and the tolerance it is compared
                // with -- is the same number either way.
                let gain = crate::v92::upchoice::declared_gain(lc)
                    .unwrap_or_else(|| crate::v92::upchoice::gain_for(lc));
                let reading = crate::v92::rung::Reading {
                    drn,
                    lc,
                    level_lsb: gain.recip() * (1.0 + delta),
                    gain,
                    bin: 0.0,
                    crossing: 0.0,
                    // No `B1u` yet: this is a `CPd` decision.
                    symbols: 0,
                };
                let verdict = crate::v92::rung::admit(&reading);
                if verdict.granted() {
                    mask |= 1 << (drn - 1);
                }
                self.note(format!("V.92: {}", reading.line(verdict)));
            }
            // What the instruments granted, printed as it always was, and then
            // the cap: the ladder is read in full -- one `LADDER` line per
            // rung, and the summary line carries the mask all of them make --
            // and the mask the call is *decided* on is that grant cut to
            // `MEASURED_WORKING_MAX_DRN`. The instruments are the instrument;
            // they are not the licence (`docs/266` sec 4, `docs/269` sec 3).
            let granted = mask;
            mask = crate::v92::rung::cap_to_measured(granted);
            self.note(format!(
                "V.92: upstream rate: level {delta:+.2}% against 1/G (four points {inner:.3}/{outer:.3} \
                 over {seen}), tolerance per rung from docs/261 sec 4, mask {granted:#07x}"
            ));
            self.note(crate::v92::rung::cap_line(granted));
        } else {
            // The one branch in which the old sigma criterion still decides,
            // and the cap binds it exactly as it binds the ladder.
            let granted = apply_ceiling(ja, measured);
            mask = crate::v92::rung::cap_to_measured(granted);
            self.note(
                "V.92: no four-point level to judge (the ladder needs one); the sigma mask stands, and \
                 it is a lower bound that can only refuse"
                    .into(),
            );
            self.note(crate::v92::rung::cap_line(granted));
        }
        (mask, RateDecision { measured, ceiling, mask, empty: false })
    }

    fn decide_upstream_rung(&mut self) {
        let Some(designed) = self.upstream_plan.take() else { return };
        if self.cpd_sent {
            // A CPd has already gone out with the provisional rung. The far end
            // has it, and telling it a different one on a repeat is a rate this
            // end did not offer when it mattered.
            self.note(
                "V.92: the four-point measurement arrived after CPd went out; the provisional rung stands"
                    .into(),
            );
            return;
        }
        let trellis = Trellis::Sixteen;
        let snr_db = self.rx.snr_db();
        let residual = 10f64.powf(-snr_db / 20.0);
        let measured = Sigma::absolute(residual, Bound::Lower);
        // `choose` and `pin` take the noise as a *power*, because their own
        // sanity rule works in variances; the same reading supplies it.
        let noise = 10f64.powf(-snr_db / 10.0);
        let ja = self.ja.upstream_rates().unwrap_or(0);
        // The ceiling the sigma reading implies. It is a **`Bound::Lower`** on
        // the data-mode residual (the doc comment above is why), so it is not
        // evidence a rung may be spent on: `docs/264` sec 2 keeps the number and
        // its refusals in the log as an *advisory* and stops it from deciding.
        // What decides is the ladder below.
        let ceiling = ceiling_rung(measured);
        let advisory = apply_ceiling(ja, measured);
        let (allowed, decision) = self.upstream_ladder(ja, measured, ceiling);
        // A ceiling that leaves nothing does not license a rung below the
        // ladder's slowest: 24 000 is what a measurement that permits nothing
        // still gets, because the alternative is no call at all. Which of the
        // two left it there is what the log carries.
        let (mask, empty) = if allowed == 0 { (1u32, true) } else { (allowed, false) };
        let decision = RateDecision { empty, ..decision };
        let found = if empty {
            pin(&designed, 1, trellis, noise)
        } else {
            choose(&designed, noise, mask, trellis).or_else(|| pin(&designed, 1, trellis, noise))
        };
        let Some(choice) = found else {
            self.note(
                "V.92: no rung of the ladder could be built from the design this measurement was made \
                 against; keeping the provisional rung"
                    .into(),
            );
            return;
        };
        self.parameters = Some(choice.parameters.clone());
        self.cpd = Some(Cpd::from(&choice.parameters));
        // What the sigma reading says, kept and not spent. It is a `Lower`
        // bound on the data-mode residual, so it can refuse a rung and can never
        // justify one; the ladder above is what decided, and this line is what
        // keeps the two numbers comparable on one call (`docs/264` sec 2).
        let advisory_ceiling = match ceiling {
            Some(drn) => format!("drn {drn}"),
            None => "none".to_string(),
        };
        self.note(format!(
            "V.92: advisory (not the gate): sigma {:.4} index steps at LC {} from the four-point fit at \
             {snr_db:.1} dB ({:?}) would allow {advisory_ceiling} and mask {advisory:#07x}; the ladder, \
             capped at drn {}, grants mask {:#07x}; {}",
            decision.measured.steps(),
            crate::v92::rung::REFERENCE_LC,
            decision.measured.bound(),
            crate::v92::rung::effective_max_drn(),
            decision.mask,
            // Which criterion took the ceiling above, by name, and what the
            // mean-square screen would have said instead (`docs/277` sec 1).
            // A reader who compares a residual with `sigma_max` can see here
            // that it decided nothing -- the defect `docs/274` sec 2.2 was.
            decision.measured.ceiling_note(),
        ));
        if decision.empty {
            // The event the ceiling exists to make visible: every rung the far
            // end offered was refused, and the refusal is in the ladder's own
            // lines above with the number that refused it.
            self.note(format!(
                "V.92: the level refuses every rung the far end offered, drn 1 included; keeping the \
                 slowest, drn 1 at {} bit/s (the cap is not what emptied this mask -- it can only \
                 refuse above drn {})",
                crate::v92::up_rate(1),
                crate::v92::rung::MEASURED_WORKING_MAX_DRN
            ));
        }
        if let Some(drn) = self.parameters.as_ref().map(|p| p.drn) {
            self.note(format!(
                "V.92: upstream rung settled: drn {drn}, {:.1} kbit/s (this is what CPd carries)",
                f64::from(crate::v92::up_rate(drn)) / 1000.0
            ));
        }
    }

    /// 9.6.1.1.5-6: B1d out, and the receiver conditioned for B1u.
    ///
    /// The downstream's mapping comes from the analogue modem's CPu, which is
    /// the same table V.90's data mode reads (V.92 8.8.1 points at V.90 8.6.1),
    /// and the upstream's decoder from this end's own CPd.
    ///
    /// Phase 4's `Ed` is only **one** of the two ways into 9.6.1.1.5, and this
    /// function is that way's name for it: the body is
    /// [`Self::enter_data_mode`], which 9.8's renegotiation calls too. Two
    /// copies of "the data mode is up" is exactly what lost the far end's
    /// `LAPM` after a renegotiation -- `ru_ended` un-latched the state (9.8.1.2.1
    /// takes the data mode down) and the way back never re-ran the latch, so
    /// the pump reported `Phase4` for the rest of the call, the modem above read
    /// that as `Retraining` and never rebuilt its stack, V.42's clocks were held
    /// at zero with it (`lib.rs:1924`), and no `LAPM` frame went downstream
    /// again: the far end's T401 expired 1.008 s into its data mode.
    fn begin_data(&mut self) {
        self.note("V.92: Ed out".into());
        self.enter_data_mode();
    }

    /// The data mode is up: 9.6.1.1.5's `B1d` out, and everything that follows
    /// from it, in the one place, for every way in.
    ///
    /// What it latches, and each is why that lives here rather than at a call
    /// site:
    ///
    /// * `Status::Connected` -- the only thing `v90::startup`'s
    ///   `pcm::Status::Connected` arm (`startup.rs:534`) turns into
    ///   `Progress::Connected`, which is what makes the modem above build its
    ///   V.42/LAPM stack (`lib.rs:2011-2141`) and stop holding V.42's clocks
    ///   (`lib.rs:1924`).
    /// * `B1d` itself: the encoder rebuilt from CPu, 8.6.1's scrambler
    ///   initialised to zero, and `B1D_FRAMES` of scrambled ones counted down
    ///   -- a renegotiation's `B1d` is a *new* `B1d`, and the far end drops
    ///   that many symbols through a countdown this starts.
    /// * 9.6.1.1.6's `B1u` arm -- `b1u_pending` and its 2 s deadline -- for the
    ///   `E2u` that follows every one of these entries.
    /// 9.6.1.2.1's deadline as an absolute instant in `now` units: the end of
    /// INFO1a (`start_up_origin`) plus *"20 s plus 6 round-trip delays"*
    /// (`v92.txt:2770`).
    ///
    /// Absolute rather than relative to the arm, so that arming it late in
    /// Phase 4 does not buy back a budget the clause already spent -- the same
    /// shape as `ja_deadline`, whose clause has the same origin. The bound it
    /// gives can therefore be in the past when it is armed, and that is the
    /// clause answering, not a bug: a Phase 3 that used the whole 20 s leaves
    /// no time for the hunt, and the retrain is then what 9.6.1.2.1 asks for.
    fn start_up_by(&self) -> u64 {
        self.start_up_origin
            + ((B1U_WAIT + B1U_WAIT_RTD * self.settings.round_trip) * FS) as u64
    }

    fn enter_data_mode(&mut self) {
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
        self.b1u_deadline = waiting.then_some(self.start_up_by());
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
        // docs/114 R4: the slicer's levels are `round(K a(eta))` for the far
        // code to this end's scale `K`, not `a(eta)` as designed. With CPd on
        // the code grid the design's own points *are* code values, so `K` away
        // from one means the far end's codes are arriving at this end's scale
        // times `K`, and a table at `K` costs the spacing the scale error takes
        // and adds nothing. With `K = 1` -- no mask read, or bit 128 set -- the
        // table is the design's, which is the behaviour every earlier call had.
        //
        // **What the level set decides, and what it is not allowed to decide.**
        // R4 is about a far end whose CPd points *are* G.711 code values; on the
        // odd lattice they are `a(eta) = 2 eta + 1`, there is no scale to fit and
        // `K` has nothing to say. So the choice below is a choice of *path* and
        // not a second computation: the uniform arm takes the line every call
        // before `fa36308` took, point for point, and never reads `self.up_scale`
        // at all. That is what makes it a restoration rather than a coincidence
        // of `K` happening to be one -- `K` is not one on a mask the far end
        // fills in, and a fit against the odd lattice returns 433.75, which read
        // B1u at 327 of 576.
        let levels = data_mode_levels(&parameters, self.up_scale);
        self.note(format!(
            "V.92: upstream level table at K {:.5} ({}) over {} point(s), gap {:.0} LSB",
            self.up_scale,
            self.up_scale_source,
            parameters.sets.first().map_or(0, Vec::len),
            crate::v92::codegrid::min_gap(parameters.sets.first().map_or(&[][..], Vec::as_slice)),
        ));
        // The slicer moves to CPd's constellation and the equaliser stays: the
        // prefilter the far end runs is this end's design of it, and on a line
        // with insertion loss it is never exact.
        self.rx.set_data_mode(levels.clone());
        // `docs/216` §2: the same table, as the threshold above which a symbol
        // is not a point of the constellation the far end agreed to send.
        self.data_loss = DataModeLoss::new(&levels);
        // 9.9.1.2.1/.2: the same table, as `RM`'s loud/quiet signature
        // (`RmDetector`, Table 25/26). A side channel like the two around it.
        self.rm_detector = RmDetector::new(&levels);
        // Where B1u's frame interval 0 is, in symbols after the one E2u's zero
        // run was found to have ended on.
        //
        // **Not on the run's end, and this is the measurement that says so.**
        // The detector fires on the symbol carrying the first non-zero bit
        // after the run (see `symbol_upstream`), and 8.7.1 does put B1u's
        // frame interval 0 on the symbol after E2u's last *symbol* -- but the
        // run is not E2u. It is read through TRN2u's four points and its
        // descrambler, and it ends where the far end's data-mode symbols stop
        // reading as zeros through them, which is not the same symbol:
        //
        // * `/tmp/agr-v90-interop/152741-prime-b/agr-tx.raw` (the call that
        //   connects): the run's last zero is symbol 111 431 and B1u begins on
        //   111 432 -- the run's end, and 550 of 576 ones when read from
        //   there.
        // * `/tmp/agr-v90-interop/152653-prime-a/agr-tx.raw` (the calls that
        //   do not): the run's last zero is 111 577 and the detector fires on
        //   111 578, but the data mode's constellation starts on **111 576**
        //   -- the level alphabet changes there, from TRN2u's two magnitudes
        //   (0.44, 1.34) to CPd's odd lattice (0.10, 0.30, 0.51, 0.71) -- so
        //   the run overran E2u by two symbols, B1u was read two symbols into
        //   its own frame 0, and the whole call read 311 of 576 ones, which is
        //   chance. Every same-shaped candidate in that call then failed the
        //   same way and it dropped with "no E2u that reads as B1u".
        //
        // Two symbols is not a tolerance to be tuned: it is how many of B1u's
        // first symbols happen to descramble to zero through TRN2u's slicer,
        // and it is a different number on every far end and every call (zero
        // on prime-b, two on prime-a, and the old `V92_E2U_FROM` measurements
        // moved the fire 720 symbols the *other* way when the arming was
        // wrong). So it is measured per candidate rather than assumed: the run
        // end gives the *set*, 8.7.1's known content gives the *answer*.
        //
        // The set is complete because 8.7.2 makes E2u exactly one data frame:
        // the run contains all of it, so B1u's frame 0 is at most
        // `E2U_SYMBOLS` symbols before the run's end, and the thirteen
        // candidates `end - 0 ..= end - E2U_SYMBOLS` are every symbol that
        // claim can put it on. They are also every *phase*: twelve symbols is
        // the twelve-symbol data frame and, through 12 and 4, the four-symbol
        // trellis frame as well, so exactly one candidate can be the far end's
        // frame interval 0 and the others read a wrong frame phase, which is
        // what makes the score separate them by 550 against 300.
        //
        // The cost is one bounded search at the boundary and nothing on the
        // per-symbol path: thirteen candidates x the 361-symbol scoring
        // window, once, on a level stream this end has to collect anyway. A
        // candidate that reads as ones commits; if none does, the hunt goes on
        // exactly as it did.
        //
        // `V92_B1U_SEARCH=off` takes the candidate set back to `back = 0`
        // alone, which is the behaviour every B1u number published before
        // `/dev/shm/reports/e2u-boundary.md` was taken with.
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
        //
        // The number itself lives beside the chain it seeds, because it is also
        // `b1u_differential_at`'s argument: dropping whole data frames is what
        // makes the first frame decoded here B1u's frame 18 and not its frame
        // 0 (`modulus::B1U_ACQUIRE_FRAMES`).
        const B1U_ACQUIRE_FRAMES: usize = crate::v92::modulus::B1U_ACQUIRE_FRAMES;
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
            .unwrap_or(B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1);
        if std::env::var_os("V92_RX_BITS").is_some() || tracing() {
            // Printed after the assignment, not before: this line said
            // `b1u_skip 0` on every call for as long as it existed, because it
            // was written above the line that sets the field.
            eprintln!(
                "V92B1U begin at now={} gain={} sets={} upstream_delay={} candidates={} b1u_skip={}",
                self.now,
                parameters.gain,
                parameters.sets.len(),
                self.upstream_delay,
                b1u_deepest(),
                self.b1u_skip
            );
        }
        // `b1u_pending` and the deadline stay armed: a candidate that does not
        // read as B1u (see `decide_b1u_boundary`) hands the hunt back, and the
        // deadline is what ends the exchange if none of them does.
        //
        // The window is B1u's forty-eight data frames, collected whole and
        // then read at every candidate boundary in it (`b1u_readings`). The
        // bits are B1u's, so they carry nothing: both decoders' output is
        // dropped once the reading is taken, whichever boundary wins.
        self.b1u_search = Some(B1uSearch {
            end: self.now,
            levels: Vec::with_capacity(B1U_SYMBOLS),
            symbols: Vec::with_capacity(B1U_SYMBOLS),
        });
        self.b1u_ones = [0; 2];
        self.b1u_scored = [0; 2];
        self.b1u_polarity = 1.0;
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
        //
        // A candidate costs B1u's 576 symbols to confirm, so the symbol B1u
        // really begins on can fall inside another candidate's window: measured
        // on `/tmp/p0corr/g712.raw`, the detector fires at 115366 and B1u
        // begins at 115510, a hundred and forty-four symbols later. Keeping the
        // hunt alive through the window to catch it was tried and is *not* the
        // answer: on both proven-V.92 Agere captures (`012503-mask3`,
        // `012713-mask3c`) the data-mode bit stream throws spurious zero runs
        // at 116102 and 120594, each of which restarted the probe and cost the
        // call its connection -- 7128 and 2063 bytes became nothing. A
        // candidate therefore owns the window it opened, and a boundary it
        // swallowed is one this end fails on rather than guesses at.
        //
        // What the window is *for* has changed: it is the stretch every
        // candidate boundary is read over (`collect_b1u`), and the choice
        // between them is 8.7.1's known content rather than the run counter
        // that opened them. See [`Self::begin_b1u`].
        if self.b1u_search.is_some() {
            self.collect_b1u(symbol);
            return;
        }
        // The far end's data, from the symbol after B1u: the decoder that
        // carries the call, on the polarity the measurement found.
        //
        // The symbols at the *front* of this queue come before the live ones:
        // a boundary the search placed `back` symbols behind the run's end
        // leaves the first `back` symbols of the data inside the window it
        // collected (see [`Self::b1u_backlog`]).
        if self.upstream_data.is_some() {
            if let Some(pending) = self.b1u_backlog.pop_front() {
                self.b1u_symbol(pending);
                return;
            }
            self.b1u_symbol(symbol);
            return;
        }
        // 9.8.1.1.2: a rate renegotiation is in progress, so the far end is out
        // of data mode as well and what it sends is its TRN2u and then SUVu.
        // Reading it is the same reader and finder Phase 3 uses, and this is the
        // one place outside Phase 3 that either is fed. The gate is the
        // renegotiation itself: no `Ru`, no tail, no feed.
        if self.reneg.is_some() || self.reneg_trn2u_from.is_some() {
            self.reneg_symbol(symbol);
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

    /// One symbol of the far end's data mode, on the polarity the boundary
    /// measurement found.
    ///
    /// The same path `V92_B1U_ADJ` and both bit traces have always been on; it
    /// is a function so that the symbols a boundary search placed inside its
    /// own window (`b1u_backlog`) and the live ones cannot be read differently.
    /// 9.8.1.2.1: "After detecting `Ru`, the digital modem shall clamp circuit
    /// 104 to binary one and condition its receiver to detect the `Ru`-to-
    /// `R-bar-u` transition."
    ///
    /// Circuit 104 is the one the *data* direction of the DTE interface runs
    /// on, and this end has no such line: the clamp is modelled where it shows
    /// -- the downstream data mode signal stops and the transmitter is reset,
    /// which is the same thing `agrmodemlib.o`'s `V92ClientDataModeRxState`
    /// does at `.text+0x5e530` when the client side of this procedure starts
    /// (`V34TxDataReset` / `DisableScrambler` / `SetPatternGenerator`). The
    /// receiver conditioning is `RuDetector::locked`.
    fn ru_detected(&mut self) {
        if !self.clamp_circuit_104("9.8.1.2.1") {
            return;
        }
        self.note(
            "V.92: 9.8.1.2.1 -- the far end is renegotiating (Ru detected in data mode); \
             circuit 104 clamped and the receiver conditioned for the Ru-to-R-bar-u transition"
                .to_string(),
        );
    }

    /// 9.8.1.2.2: "After detecting the `Ru`-to-`R-bar-u` transition, the digital
    /// modem shall transmit signal `Rd` for 384T and then `R-bar-d` for 24T.
    /// The signal `Rd` shall begin on the boundary of a data frame."
    ///
    /// Then 9.8.1.2.3 hands the rest to 9.8.1.1.2, which is the initiator's own
    /// tail: `TRN2d` for up to 16008T, `SUVd`, and the exchange of 9.6.1.1.2.
    fn ru_ended(&mut self) {
        let (data, training) = self.renegotiation_points();
        self.source.arm_renegotiation(data, training);
        // §9.8.1.2.3 chains into §9.8.1.1.2, whose first signal is TRN2d; the
        // 24T of R-bar-d takes its place at the frame boundary `Source` waits
        // for (`pending_needs_frame_boundary`). TRN2d in a renegotiation is
        // capped by 9.8.1.1.2's "up to 16008T" rather than the 2040T of
        // 9.6.1.1.1, and that cap is a **countdown**: without it the TRN2d arm's
        // exit condition never fires and TRN2d runs on for ever -- `docs/214`
        // §6 gap 1. The SUVd after it is asked for by [`RenegTail`], which also
        // owns 9.6.1.1.2's exchange.
        self.source.bound_trn2d(TRN2D_MOST);
        self.source.change(Out::Rd);
        let (tail, mut exchange) = RenegTail::new(self.settings.round_trip, FS);
        // A far end this call has already heard send a CPu sends another; the
        // CPd waits for it, so that it carries bit 33 and lands one window
        // after the CP it answers rather than inside it
        // (`CPD_WAITS_FOR_PARKED_TX`).
        exchange.wait_for_peer_cp(self.cpu.is_some());
        self.reneg = Some(tail);
        // The exchange 9.6.1.1.2 is entered from, and the one the far end's
        // SUVu is reported to while the tail runs.
        self.exchange = Some(exchange);
        // 9.8.2.1.1 and 9.8.2.2.3: the far end's TRN2u begins where its 24T of
        // R-bar-u ends, and this end's own detector is the fiducial for it.
        self.reneg_trn2u_from = Some(self.now + TRN2U_AFTER_RU_BAR);
        // Out of data mode: 9.8.1.2.1 clamped circuit 104, so the B1u decoder is
        // not reading data any more, and the finder that reads the far end's
        // SUVu is armed here -- and only here. A call with no `Ru` in it never
        // reaches this function, which is what keeps the data path identical.
        self.upstream_data = None;
        self.upstream = None;
        self.up_finder = None;
        self.upstream_drop_until = 0;
        self.awaiting_e2u = false;
        self.e2u_zeroes = 0;
        self.b1u_pending = false;
        self.b1u_deadline = None;
        // 9.8's Phase 4 is a new exchange, so 9.6.1.2.1's clock starts again
        // here rather than at the end of this call's INFO1a.
        self.start_up_origin = self.now;
        self.b1u_search = None;
        self.data_loss = None;
        // 9.8.1.2.1 clamps circuit 104 and takes the data mode down: from here
        // to 9.6.1.1.5's `B1d` the line carries `Rd`, `TRN2d`, `SUVd` and
        // `CPd`, and there is no data mode to be connected to. `begin_data`
        // latched `Status::Connected` at the first data mode (`:5874`) and
        // nothing used to un-latch it, so the pump went on reporting a
        // connected call through the whole of 9.8: the modem above built a
        // V.42 stack over a line that had just stopped carrying B1u, its
        // detection phase failed, and the CONNECT it had been holding was
        // released for a call that never got past `SUVd`. 9.6.1.1.5's
        // `begin_data` sets `Connected` again, and that one is real.
        self.status = Status::Phase4;
        self.note(
            "V.92: 9.8.1.2.2 -- Ru-to-R-bar-u seen; sending Rd for 384T and R-bar-d for 24T \
             from the next data frame boundary, then 9.8.1.1.2's TRN2d (up to 16008T) and SUVd"
                .to_string(),
        );
    }

    /// The far end has left data mode: 9.6.1.1 from the top, entered with
    /// 9.5.1.1.12's `Ri` because that is the state it went back to.
    ///
    /// The trigger is [`DataModeLoss`] and it is `[our choice]`; everything
    /// asked for here is the clause's:
    ///
    /// * 9.5.1.1.13/9.5.2.1.10: `Ri` is what the analogue modem conditions on
    ///   to receive `CPt` -- or, in the fallback state measured for `docs/216`,
    ///   `V92ClientPhase4RxWaitRiBarState`, what it waits for to re-open Phase
    ///   4 at all.
    /// * 9.6.1.1.1: `TRN2d` "for at least 2040T", which the source's own
    ///   `Ri`->`R-bar-i`->`TRN2d` chain starts, and whose upper bound here is
    ///   9.8.1.1.2's `up to 16008T` -- the same countdown 9.8.1.2.3 uses.
    /// * 9.6.1.1.2/.3/.4: one `CPd` after the far end's `SUVu`, then `Ed`,
    ///   which is [`RenegTail`] and is reused rather than copied.
    fn data_mode_lost(&mut self) {
        if self.reneg.is_some() {
            return;
        }
        let threshold = self.data_loss.as_ref().map_or(0.0, DataModeLoss::threshold);
        self.note(format!(
            "V.92: data mode lost -- {} consecutive data frames carried a symbol outside the \
             constellation CPd declared (over {threshold:.2}); sending Ri 384T and R-bar-i 24T \
             on the data frame grid, then 9.6.1.1 again",
            DATA_MODE_LOSS_FRAMES
        ));
        let (data, training) = self.renegotiation_points();
        self.source.arm_renegotiation(data, training);
        // The opener, and it is **`Ri`**, not `Rd`: `Ri` is the signal the far
        // end's own state is named for (`V92ClientPhase4RxWaitRiBarState`,
        // `0x5dea0`, `docs/216` §1). Both were tried on the bench and both were
        // silent (`docs/216` §5): `Ri` 384T + `R-bar-i` 24T at 24.998 s
        // (`runs-216/165935`) and `Rd` 384T + `R-bar-d` 24T at 24.998 s
        // (`runs-216/170104`), each 408 symbols on the line and no `R detected`
        // from either. What is committed is the one the far end's own state
        // name asks for; the A/B is measured and recorded, not pending.
        self.source.ri_opener();
        self.source.bound_trn2d(TRN2D_MOST);
        let (tail, mut exchange) = RenegTail::new(self.settings.round_trip, FS);
        // A far end this call has already heard send a CPu sends another; the
        // CPd waits for it, so that it carries bit 33 and lands one window
        // after the CP it answers rather than inside it
        // (`CPD_WAITS_FOR_PARKED_TX`).
        exchange.wait_for_peer_cp(self.cpu.is_some());
        self.reneg = Some(tail);
        self.exchange = Some(exchange);
        // 9.8.1.1.2's 16008T is 9.8's ceiling and not 9.6.1.1.1's 2040T floor:
        // the floor is satisfied by the tail's own exchange, which cannot send
        // SUVd before the far end's SUVu, and the ceiling is what keeps TRN2d
        // from running on for ever (`Source::bound_trn2d`).
        self.reentry = true;
        self.reneg_trn2u_from = None;
        // Out of data mode: the B1u decoder is dropped and the reader for the
        // far end's TRN2u/SUVu is armed when our own TRN2d begins (`reneg_out`).
        self.upstream_data = None;
        self.upstream = None;
        self.up_finder = None;
        self.upstream_drop_until = 0;
        self.awaiting_e2u = false;
        self.e2u_zeroes = 0;
        self.b1u_pending = false;
        self.b1u_deadline = None;
        self.b1u_search = None;
        self.b1u_probe = None;
    }

    /// "Clamp circuit 104 to binary one", for **both** of the procedures that
    /// ask it -- 9.8.1.2.1's `Ru` and 9.9.1.2.1's `RM` (`v92.txt:3076`,
    /// `3079-3081`).
    ///
    /// One mechanism and not two, and the same one `docs/214` introduced for
    /// 9.8.1.2.1: circuit 104 is the line the *data* direction of the DTE
    /// interface runs on, and this end has no such line, so the clamp is
    /// modelled where it shows -- the downstream data mode signal stops and the
    /// transmitter is reset, which is what `agrmodemlib.o`'s
    /// `V92ClientDataModeRxState` does at `.text+0x5e530`
    /// (`V34TxDataReset` / `DisableScrambler` / `SetPatternGenerator`).
    /// Returns whether it clamped now, so that a hundred `RM` symbols and a
    /// hundred `Ru` periods report once.
    fn clamp_circuit_104(&mut self, clause: &'static str) -> bool {
        if self.ru_responder_started {
            return false;
        }
        self.ru_responder_started = true;
        self.note(format!(
            "V.92: {clause} -- circuit 104 clamped and the receiver conditioned for the \
             transition to the bar of the R signal the far end is sending"
        ));
        true
    }

    /// 9.9.1.2.1: "After detecting `RM`, the digital modem shall clamp circuit
    /// 104 to binary one and condition its receiver to detect the `RM`-to-
    /// `RM`-bar transition" (`v92.txt:3079-3081`).
    ///
    /// The conditioning *is* [`RmDetector`]: once it has heard `RM` it stops
    /// matching Table 25's pattern and starts matching Table 26's complement of
    /// it, which is 9.9.1.2.1's second half. Nothing else about the receiver
    /// changes -- `RM` goes out on "the constellation parameters used for the
    /// data mode" (8.7.4, `v92.txt:1664-1666`), so the slicer that was reading
    /// `B1u` is already the right one.
    fn rm_detected(&mut self) {
        if self.reneg.is_some() {
            return;
        }
        self.clamp_circuit_104("9.9.1.2.1");
        self.note(
            "V.92: 9.9.1.2.1 -- RM detected in data mode; the far end has begun a fast \
             parameter exchange, receiver conditioned for the RM-to-RM-bar transition"
                .to_string(),
        );
    }

    /// 9.9.1.2.2: "After detecting the `RM`-to-`RM`-bar transition, the digital
    /// modem shall transmit signal `Rf` for 384T and then `R-bar-f` for 24T. The
    /// signal `Rf` shall begin on the boundary of a data frame."
    ///
    /// Then 9.9.1.2.3: "initialize the scrambler, differential encoder and
    /// spectral shaping filter memory to zero and transmit `SUVd` sequences with
    /// bit 32 clear and proceed according to 9.6.1.1.2."
    ///
    /// **Why this and not 9.8's `Rd`.** The two procedures are cross-linked by
    /// the Recommendation itself: 9.9.1.1.2 ends "If signal `Ru` is detected
    /// the modem shall proceed according to 9.8.1.2.1" and 9.9.2.1.2 ends "If
    /// signal `Rd` is detected, the modem shall proceed according to
    /// 9.8.2.2.1" (`v92.txt:3074-3075`, `3097-3098`). `Rd`/`Ru` are the
    /// **fallback** branches of a modem that has *not* moved to 9.9; a far end
    /// that is sending `RM` has, and no `Rd` it receives is looked for. That is
    /// the whole of why `runs-216/170104`'s `Rd`, at the data mode maximum and
    /// at the right 3772/7932 levels, produced nothing at all.
    fn rm_ended(&mut self) {
        if self.reneg.is_some() {
            return;
        }
        let (data, training) = self.renegotiation_points();
        self.source.arm_renegotiation(data, training);
        // The three resets of 9.9.1.2.3, applied here rather than at the
        // `R-bar-f`-to-`SUVd` boundary the clause puts them at: `Rf` and
        // `R-bar-f` are **not** produced through any of the three memories.
        // 8.8.4 gives them a literal sign pattern and one codeword an interval,
        // so the memories they reset are first used by the `SUVd` after them --
        // which is the boundary the clause is protecting. See
        // [`Self::reset_data_mode_memory`].
        self.reset_data_mode_memory();
        // 8.8.5's "the preceding data mode modulation": the sequence coder for
        // this exchange is CPu's, not CPt's. Without it the SUVd after `R-bar-f`
        // goes out on the *training* constellation, which the far end's data
        // mode slicer is not reading -- `docs/222` §7's gap.
        if let Some(cpu) = self.cpu.clone() {
            if !self.source.begin_fpe(&cpu) {
                self.note("V.92: 8.8.5 -- no CPu mapping for the fast parameter exchange's SUVd".into());
            }
        }
        // 9.9.1.2.2's opener: `Rf` 384T then `R-bar-f` 24T, on the data frame
        // grid (`Source::pending_needs_frame_boundary`).
        self.source.change(Out::Rf);
        // 9.9.1.2.3's tail is **9.6.1.1.2 directly**: no `TRN2d`, because the
        // far end never left the data mode modulation (8.8.5: during a fast
        // parameter exchange a short information sequence is "transmitted using
        // the preceding data mode modulation"). `RenegTail::from_suvd` is that
        // entry and it requests no silent period -- "bit 32 clear".
        let (tail, exchange) = RenegTail::from_suvd(self.settings.round_trip, FS);
        self.reneg = Some(tail);
        self.exchange = Some(exchange);
        self.fpe = true;
        self.reentry = false;
        self.reneg_trn2u_from = None;
        // Out of data mode: the B1u decoder is dropped, and the reader for the
        // far end's `SUVu` is armed now -- 9.9's exchange has no `TRN2u` in
        // front of it (Figure 19: `RM`, `R-bar-M`, `SUVu`, `SUVu`, `SUVu`,
        // `CPu`, ...), so there is no fiducial to wait for the way 9.8's has.
        self.upstream_data = None;
        self.upstream = None;
        self.up_finder = None;
        self.upstream_drop_until = 0;
        self.awaiting_e2u = false;
        self.e2u_zeroes = 0;
        self.b1u_pending = false;
        self.b1u_deadline = None;
        self.b1u_search = None;
        self.b1u_probe = None;
        self.data_loss = None;
        self.rm_detector = None;
        self.begin_upstream();
        self.note(
            "V.92: 9.9.1.2.2 -- RM-to-RM-bar seen; sending Rf for 384T and R-bar-f for 24T \
             from the next data frame boundary, then 9.9.1.2.3's SUVd (bit 32 clear) and 9.6.1.1.2"
                .to_string(),
        );
    }

    /// 9.9.1.2.3: "initialize the scrambler, differential encoder and spectral
    /// shaping filter memory to zero" (`v92.txt:3083-3085`).
    ///
    /// All three live in the **data mode** transmitter, not in Phase 4's:
    ///
    /// * the scrambler is `self.downstream_scrambler`
    ///   (`crates/datapump/src/v92/digital.rs:3809`, built afresh by
    ///   `begin_data` at `:4038-4039`) -- 8.6.1's data mode scrambler;
    /// * the differential encoder is `v90::encoder::Encoder`'s `signs:
    ///   Differential` (`crates/datapump/src/v90/encoder.rs:144`, built by
    ///   `Encoder::new` at `:159`) and the spectral shaping filter memory is the
    ///   same encoder's `shaper: Shaper` (`encoder.rs:144`, `Shaper::new` at
    ///   `:155`). `Encoder` says of itself, at `encoder.rs:129-133`:
    ///   "**Built afresh wherever the Recommendation starts the coding again**
    ///   -- TRN2d and B1d both begin with 'the scrambler, differential encoder
    ///   and spectral shape filter memory ... initialized to zero' -- so
    ///   **there is no reset**."
    ///
    /// So the shaping memory has **never** been reset at a sequence boundary and
    /// could not have been: the only way this end zeroes it is to rebuild the
    /// encoder, which is what 8.6.5/TRN2d does at
    /// `digital.rs:2216-2224` for Phase 4's coder. This does the same for the
    /// data mode's encoder, out of the same `Mapping` -- CPu's, the one B1d was
    /// going out on -- so nothing but the memories changes.
    fn reset_data_mode_memory(&mut self) {
        self.downstream_scrambler = Scrambler::new(Mode::Call);
        if let Some(encoder) = self.downstream.as_ref() {
            let mapping = encoder.mapping().clone();
            self.downstream = Some(crate::v90::encoder::Encoder::new(mapping, self.settings.law));
        }
        self.note(
            "V.92: 9.9.1.2.3 -- scrambler, differential encoder and spectral shaping filter \
             memory initialised to zero"
                .to_string(),
        );
    }

    /// 9.8.1.1.2 to 9.6.1.1.2, one symbol: the renegotiation's own tail.
    ///
    /// Separate from [`Self::phase4_out`] because 9.8's exchange is
    /// [`Context::Renegotiation`] and because what follows Ed is silence and
    /// then `Rt` rather than data mode. Everything before the tail -- `Rd`,
    /// `R-bar-d`, `TRN2d`, `Rt`, `R-bar-t` -- is on [`Source`]'s own arms, so
    /// what this drives is only the sequence-boundary decisions.
    fn reneg_out(&mut self) {
        let now = self.now;
        // 9.6.1.1.1: `TRN2d` is on the line, so the far end's `TRN2u` begins --
        // and 8.7.6 resets the scrambler at its first symbol, which is why the
        // reader is built here and not before. On 9.8's own path the fiducial
        // is the `Ru`-to-`R-bar-u` transition; a re-entry has no `Ru`, so the
        // end of our own `R-bar-i` is used instead (`[our choice]`, and the one
        // place this path can be a few symbols out: the far end has to notice
        // `R-bar-i` before it can answer with `TRN2u`).
        if self.reentry && self.source.out() == Out::Trn2d {
            self.reentry = false;
            self.begin_upstream();
            self.note("V.92: 9.6.1.1.1 -- TRN2d out; conditioning for the far end's TRN2u and SUVu".into());
        }
        let asked;
        let done;
        {
            let Self { source, reneg, exchange, cpd, .. } = self;
            let (Some(tail), Some(exchange)) = (reneg.as_mut(), exchange.as_mut()) else {
                return;
            };
            asked = tail.push(source, exchange, cpd.as_ref(), now);
            done = tail.is_done();
        }
        match asked {
            // 9.6.1.1.2: "After receiving an SUVu sequence, the digital modem
            // shall transmit a single CPd".
            Some(Out::Cpd) => {
                let drn = self.cpd.as_ref().map_or(0, |cpd| cpd.drn);
                self.note(format!(
                    "V.92: 9.6.1.1.2 -- the renegotiation's single CPd, drn {drn}"
                ));
            }
            // 9.8.1.1.4: "After receiving SUVu with bit 32 clear, the digital
            // modem shall transmit Rt for 384T followed by R-bar-t for 24T".
            Some(Out::Rt) => self.note(
                "V.92: 9.8.1.1.4 -- SUVu with bit 32 clear; sending Rt for 384T and \
                 R-bar-t for 24T, then SUVd"
                    .into(),
            ),
            _ => {}
        }
        if done {
            self.finish_renegotiation();
        }
    }

    /// 9.6.1.1.4's `Ed` was the last signal of the renegotiation: back to B1d,
    /// and re-arm the search that finds the far end's next B1u.
    ///
    /// The receiver, not the transmitter, is what this has to put right. Out of
    /// data mode this end dropped the B1u decoder (`ru_ended`), so B1u's return
    /// is the Phase 4 acquisition again: 9.6.1.1.6's E2u run, and B1u's frame
    /// interval 0 on the symbol after it. `awaiting_e2u` and `b1u_pending` are
    /// what `begin_data` arms, and they are armed here for the same reason.
    fn finish_renegotiation(&mut self) {
        self.reneg = None;
        self.exchange = None;
        self.reneg_trn2u_from = None;
        self.reentry = false;
        self.fpe = false;
        self.rm_responder_started = false;
        self.data_loss = None;
        self.ru_detector.reset();
        self.ru_responder_started = false;
        self.upstream_data = None;
        // The reader the `E2u` is read through is the one put on the line at the
        // far end's **`TRN2u`**, and Ed is not a place to start a new one.
        // 8.7.6: "The scrambler shall be reset at the beginning of `TRN2u`", and
        // 8.7.2: "`E2u` is a data frame of **scrambled**, differentially encoded
        // zeroes ... transmitted using the corresponding `TRN2u` modulation
        // during Training and Rate Renegotiation." So its zeroes decode to
        // zeroes only through the descrambler state that reset began and every
        // sequence since has advanced -- and a `Trn2uReader::new` here is a
        // descrambler starting again at zero, mid-stream, with nothing on the far
        // end's side to match it.
        //
        // Rebuilding it here was the whole of the renegotiation's `E2u` defect:
        // measured at answer point 171 (`runs-236c/203200-f171-1`), the
        // renegotiation's exchange completed at 36.540, the far end's `E2u` came
        // and no run of zeroes was ever read from it, and at 38.540 this end
        // failed the exchange with "no E2u that reads as B1u within 2 s of Ed" --
        // the far end answering `6009521a` at 41.880. Phase 4 never exposed it
        // because its reader is built once, at `TRN2u`'s first symbol
        // (`begin_upstream`), and nothing between there and its `E2u` replaces
        // it.
        //
        // A reader that is *not* on the line is the one case that still needs
        // building: 9.6.1.1.1's re-entry (`data_mode_lost`) arms its own at our
        // `TRN2d` (`reneg_out` -> `begin_upstream`), so this is a fallback for a
        // path that reached Ed without one, not the ordinary path.
        if self.upstream.is_none() {
            self.upstream = Some(Trn2uReader::new(self.upstream_size));
        }
        if self.up_finder.is_none() {
            self.up_finder = Some(UpFinder::new(trn2u_unit(self.upstream_size == Trn2uSize::Eight)));
        }
        self.upstream_drop_until = 0;
        self.awaiting_e2u = true;
        self.e2u_zeroes = 0;
        self.source.end_fpe();
        self.note("V.92: 9.8 rate renegotiation over -- B1d again".into());
        // 9.6.1.1.5-6 by the same door Phase 4 uses. The renegotiation took the
        // data mode down (`ru_ended` -> `Status::Phase4`), and *this* is the
        // signal that puts it back: `Ed` was the last one out, so `B1d` is what
        // follows, and the state that goes with `B1d` -- `Status::Connected`,
        // CPu's encoder, 8.6.1's scrambler, `B1D_FRAMES`, the 9.6.1.1.6 `B1u`
        // arm -- is `begin_data`'s body and not a second copy of it. Calling it
        // here is at worst idempotent; not calling it was `docs/238`'s defect.
        self.enter_data_mode();
    }

    /// One symbol of the far end's side of a renegotiation.
    ///
    /// This is the feed `docs/213` §6 named as missing: `up_finder` was fed in
    /// Phase 3 and never again, so a renegotiation's `SUVu` had no reader. It is
    /// legitimate here because 9.8.1.2.1 has taken this end *out of data mode* --
    /// the far end's `TRN2u` and `SUVu` are the only signals on its upstream --
    /// and it is gated on [`Self::reneg`], which only `ru_ended` sets.
    fn reneg_symbol(&mut self, symbol: Symbol) {
        if let Some(at) = self.reneg_trn2u_from {
            if self.now < at {
                return;
            }
            // 8.7.6: "The scrambler shall be reset at the beginning of TRN2u" --
            // so the reader is built *on* its first symbol, as Phase 4 builds it
            // on TRN2u's first symbol too. One symbol either way and the
            // descrambler sits on the wrong phase and no `SUVu` comes out.
            self.reneg_trn2u_from = None;
            self.upstream = Some(Trn2uReader::new(self.upstream_size));
            self.up_finder = Some(UpFinder::new(trn2u_unit(self.upstream_size == Trn2uSize::Eight)));
            self.upstream_drop_until = 0;
            self.note(format!("V.92: the far end's renegotiation TRN2u begins at {}", self.now));
        }
        let level = symbol.value;
        let Some(reader) = self.upstream.as_mut() else { return };
        let bits: Vec<bool> = reader.feed(level).into_iter().collect();
        for bit in bits {
            if let Some(finder) = self.up_finder.as_mut()
                && let Some(family) = finder.feed(bit)
            {
                self.up_sequence(family);
            }
        }
    }

    /// The two constellations V.90 8.6.4 separates `Rd` from `Rt` by.
    ///
    /// `Rd` is "the highest power PCM codeword from the **data mode**
    /// constellation of each data frame interval as passed in **CP**", `Rt` the
    /// same "from the **training** constellation ... as passed in **CPt**".
    /// Twelve intervals each, three different level sources, and the two arrays
    /// are never the same one.
    ///
    /// `Ri` is not here at all: it is "the single PCM codeword whose Ucode is
    /// U_INFO for all data frame intervals" (`v90.txt:1844`), a codeword and
    /// not a constellation, and it is emitted at `Source::uinfo`
    /// (`digital.rs:2487`).
    fn renegotiation_points(&self) -> ([u8; DOWN_INTERVALS], [u8; DOWN_INTERVALS]) {
        // Data mode, out of the constellation **we transmit**.
        //
        // The CP that carries the digital modem's data mode constellations is
        // **CPu**: `v92.txt:1474` -- "The long CPu sequence contains modulation
        // parameters for use by the digital modem in data mode" -- and 8.8.4's
        // `Rf` reads its own level out of it by name, "the highest power PCM
        // codeword from the data mode constellation of each data frame interval
        // as passed in **CPu**" (`v92.txt:1961-1962`). `Rd`'s "as passed in CP"
        // (`v90.txt:1841-1842`) is that same message under V.90's name for it.
        //
        // `begin_data` (`digital.rs:5254`) builds `self.downstream` out of that
        // CPu, so the mapping read here is the one `B1d` is actually going out
        // on. `Constellation::points()` holds Ucodes "sorted so that index 0 is
        // the largest" (`modulus.rs:103`), so the *first* of an interval's set
        // is its highest-power codeword -- taken from the encoder on the line
        // rather than from a second reading of the same message.
        //
        // It is **not** `self.cpd`. A CPd is "modulation parameters for use by
        // the analogue modem in data mode" (`v92.txt:1764`) -- the *upstream*
        // constellation -- and its points are `V92_LEVELSET`'s level indices
        // `a(eta) = 2eta + 1` (`upchoice.rs:429`), not Table 1 linear
        // magnitudes. Reading it here and inverting it with `ucode::nearest`
        // took the odd lattice's last index, 15 at LC 8, for a linear value and
        // made Ucode 2 = linear 16: the -66 dBFS `Rd` of
        // `runs-216/170104`.
        let mut data = [self.source.uinfo; DOWN_INTERVALS];
        if let Some(encoder) = self.downstream.as_ref() {
            let sets = &encoder.mapping().sets;
            for (i, slot) in data.iter_mut().enumerate() {
                if let Some(&loudest) = sets.get(i % INTERVALS).and_then(|s| s.points().first()) {
                    *slot = loudest;
                }
            }
        }
        // Training, out of CPt's own coder -- the encoder TRN2d, SUVd, CPd and
        // Ed go out on, which `set_cpt` built from the far end's CPt (8.6.5).
        // `Constellation::points()` is "sorted so that index 0 is the largest"
        // (`modulus.rs:103`), i.e. **loudest first**, so the highest-power
        // codeword of an interval's set is its *first* -- `.last()` is the
        // quietest and was what this read before `docs/218`; `Rt` is not
        // emitted on any path a bench call has reached, which is why nothing
        // measured it.
        let mut training = [self.source.uinfo; DOWN_INTERVALS];
        if let Some(phase4) = self.source.phase4.as_ref() {
            let sets = &phase4.coder.mapping().sets;
            for (i, slot) in training.iter_mut().enumerate() {
                if let Some(&loudest) = sets.get(i % INTERVALS).and_then(|s| s.points().first()) {
                    *slot = loudest;
                }
            }
        }
        (data, training)
    }

    fn b1u_symbol(&mut self, symbol: Symbol) {
        let level = symbol.value;
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
        // 9.8.1.2.1's conditioning, on the side: the detector is told the same
        // level the B1u decoder is about to be told, and nothing it answers
        // reaches that decoder. A call with no `Ru` in it runs this and then
        // runs exactly what it ran before -- `docs/214` pins that.
        match self.ru_detector.feed(level) {
            RuEvent::None => {}
            RuEvent::RuDetected => self.ru_detected(),
            RuEvent::RuToRuBar => self.ru_ended(),
        }
        // `docs/216` §2: the same level again, against the constellation CPd
        // declared. A side channel like the one above -- until it fires, the
        // decoder below is fed exactly what it was fed before.
        // `Ru` is two-point and every one of its symbols is outside the
        // constellation `CPd` declared, so 9.6.1.1.2's loss detector would call
        // the far end's rate renegotiation a lost data mode and answer it with
        // `Ri`. While 9.8.1.2.1's conditioning is in force the symbols are not
        // data mode symbols at all, so that detector is not shown them.
        let lost = !self.ru_detector.is_locked()
            && self.data_loss.as_mut().is_some_and(|loss| loss.feed(level));
        if lost {
            self.data_mode_lost();
            return;
        }
        // docs/222: 9.9.1.2.1/.2's `RM`, on the same level and with the same
        // side-channel discipline. `RM` is Table 25's period-four
        // `{max,max,min,min}` modulus encoder output, which is what makes it
        // visible without a trellis decoder; before it fires the decoder below
        // is fed exactly what it was fed before.
        match self.rm_detector.as_mut().map(|rm| rm.feed(level)) {
            Some(RmEvent::None) | None => {}
            Some(RmEvent::RmDetected) => self.rm_detected(),
            Some(RmEvent::RmToRmBar) => self.rm_ended(),
        }
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
    fn collect_b1u(&mut self, symbol: Symbol) {
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
        // does nothing. Applied once per symbol and not once per candidate,
        // because it keeps state: `dc` subtracts a running mean.
        let level = self.adjust_b1u_level(level);
        let Some(search) = self.b1u_search.as_mut() else { return };
        search.levels.push(level);
        search.symbols.push(symbol);
        if search.levels.len() >= B1U_SYMBOLS {
            self.decide_b1u_boundary();
        }
    }

    /// Which candidate boundary 8.7.1's known content reads as ones from, and
    /// the commit of that one.
    ///
    /// 8.7.1 makes B1u forty-eight data frames of scrambled *ones*, so the far
    /// end's payload over the whole of it is known, and the two readings of it
    /// at one candidate are enough to say whether that candidate is B1u's frame
    /// interval 0 at all: the right polarity reads ones, and a boundary that is
    /// a symbol or two out reads a wrong data-frame phase, which the
    /// differential chain, the descrambler and the Viterbi turn into about half
    /// ones -- the same distinction the polarity measurement already stands on.
    ///
    /// So the run's end does not decide anything by itself; it says where to
    /// look (see [`Self::begin_b1u`] for the measurement and for why the set is
    /// complete), and this picks the one candidate that reads B1u. The gate is
    /// the same one a single candidate had to pass: a search is not a licence
    /// to believe its best answer, so a best candidate that still does not read
    /// as ones hands the hunt back through [`Self::abandon_b1u`].
    fn decide_b1u_boundary(&mut self) {
        let Some(search) = self.b1u_search.take() else { return };
        let Some(parameters) = self.parameters.clone() else { return };
        let mut readings = b1u_readings(&parameters, search.end, &search.levels, self.b1u_skip, b1u_deepest());
        if readings.is_empty() {
            // A `V92_B1U_SKIP` that leaves no symbol of the window to read is
            // not a boundary either; hand the hunt back rather than leave it
            // waiting for a window that has already gone.
            self.abandon_b1u(search.end, search.end, 0, 0, 0, 0);
            return;
        }
        // `V92_B1U_GATE=off` accepts every candidate, which is what measuring
        // the data mode below a boundary takes: the criterion lives above the
        // thing being measured, so a run that cannot pass it still has to be
        // able to say what it reads. See [`choose_b1u_boundary`] for which of
        // the candidates is believed and why it is not the best-reading one.
        let gate_off = std::env::var("V92_B1U_GATE").ok().as_deref() == Some("off");
        if std::env::var_os("V92_RX_BITS").is_some() || tracing() {
            let scores: Vec<String> = readings
                .iter()
                .map(|reading| format!("back={} {}/{}", reading.back, reading.best(), reading.scored()))
                .collect();
            eprintln!("V92B1U candidates from end={} skip={}: {}", search.end, self.b1u_skip, scores.join(" "));
        }
        let Some(winner) = choose_b1u_boundary(&readings, gate_off) else {
            // Not a boundary, not even the best of them. This is the whole
            // point of the criterion: a receiver that goes on to read data from
            // a place it has just failed to read B1u from does not need the far
            // end's agreement, it needs a boundary it actually has.
            let Some(closest) = best_b1u_reading(&readings) else {
                self.abandon_b1u(search.end, search.end, 0, 0, 0, 0);
                return;
            };
            let failed = &readings[closest];
            // The rotation the failed candidate measured, for the same reason
            // the winner's is logged: a candidate that reads 547 of 576 has a
            // mapping that is nearly right, and the measurement is what says
            // whether "nearly" is the rotation's fault or not.
            if let Some(line) = failed.straight.rotation_line() {
                self.note(format!("V.92: {line}"));
            }
            self.abandon_b1u(search.end, failed.origin, failed.back, failed.best(), failed.other(), failed.scored());
            return;
        };
        let (back, origin) = (readings[winner].back, readings[winner].origin);
        let mut chosen = readings.swap_remove(winner);
        // `docs/264` sec 3: the window's decided indices, both polarities, taken
        // here because both decoders are still in hand and not yet committed to
        // the stream. They are a record of decisions already taken -- draining
        // them cannot move one -- and `keep_decisions` goes back off before
        // either decoder reads a symbol of the far end's data.
        // `docs/285` sec 2: the decoder `b1u_readings` built for this
        // candidate is fed from `levels[skip - back]`, so B1u's symbol at its
        // frame 0 is `skip + 1 - back` in the encoder's numbering. Without the
        // `- back` every decision is paired with a symbol the candidate's own
        // displacement away, and on the bench the winner is routinely a whole
        // data frame behind the run's end (twelve symbols, which B1u's
        // forty-eight identical frames make invisible to its ones count).
        let first = (self.b1u_skip + 1).saturating_sub(back);
        let high = self.rx.measured_levels().2 > 1.0;
        let mut straight = std::mem::replace(&mut chosen.straight, Decoder::new(&parameters));
        let mut probe = std::mem::replace(&mut chosen.probe, Decoder::new(&parameters));
        straight.keep_decisions(false);
        probe.keep_decisions(false);
        let bins_straight = B1uBins::measure(&parameters, &straight.take_decisions_indexed(), first, high);
        let bins_probe = B1uBins::measure(&parameters, &probe.take_decisions_indexed(), first, high);
        chosen.straight = straight;
        chosen.probe = probe;
        if let Some(file) = b1u_bits_file()
            && let Ok(mut file) = file.lock()
        {
            use std::io::Write;
            for (which, bits) in chosen.bits.iter().enumerate() {
                let text: String = bits.iter().map(|bit| if *bit { '1' } else { '0' }).collect();
                let sign = if which == 0 { 1 } else { -1 };
                let _ = writeln!(file, "{} {sign} {text}", chosen.origin);
            }
        }
        let rotation = chosen.straight.rotation_line().map(str::to_owned);
        self.upstream_data = Some(chosen.straight);
        self.b1u_probe = Some(chosen.probe);
        self.b1u_ones = chosen.ones;
        self.b1u_scored = chosen.scored;
        self.b1u_skip = 0;
        // The window ends `back` symbols after B1u does, and those symbols are
        // the far end's data: the winning decoder is fed from the end of B1u,
        // so they are next in its stream.
        self.b1u_backlog = b1u_backlog_symbols(&search, back).into();
        self.note(format!(
            "V.92: B1u from symbol {origin}, {} bit/s, run end {} back {back}, scale {:.3}, taps {:?}",
            parameters.rate(),
            search.end,
            self.upstream_scale,
            self.rx.taps()
        ));
        // What the class rotation was measured to be, and on what margin. One
        // line, whether or not it agreed with the constant: `547 of 576` is the
        // reading whose frame phase is right and whose mapping is nearly right,
        // and without this line nothing in the transcript separates that from a
        // line of noise or from a rotation that was never taken.
        if let Some(line) = rotation {
            self.note(format!("V.92: {line}"));
        }
        // The polarity verdict is unchanged, and it is the boundary's verdict:
        // it now runs on the chosen candidate's window instead of on the
        // window the run's end happened to open.
        self.decide_b1u_polarity();
        // `docs/264` sec 3: the `bin`/`adj` table, on the window that was just
        // read, through the polarity the call settled on -- and then the ladder
        // asked the same question again with the rate it could not have had
        // before `CPd`: the grant is confirmed or refused *by the instrument*,
        // in the ladder's own line, on the same call.
        let bins = if self.b1u_polarity < 0.0 { bins_probe } else { bins_straight };
        for line in bins.lines(parameters.drn) {
            self.note(line);
        }
        // `docs/278`'s rule -- the log prints which gate decided -- extends to
        // "the log does not print a number it cannot stand behind": a pairing
        // whose origin is not decisively better than a displaced one of the
        // same window is not a measurement of the far end, and the ladder is
        // not asked a question it would then answer with that number.
        //
        // Fails closed, like everything else here: with no rate the rung CPd
        // carries is the one that stands, and no grant is issued on a pairing
        // that could not certify its own alignment (`docs/285` sec 2).
        if !bins.trusted() {
            self.note(format!(
                "V.92: BIN pairing refused: unpaired={} wrong={} of {} against a nearest displaced rival \
                 of {} over {}-symbol shifts -- rate=n/a (pairing is not self-consistent, so the \
                 decided B1u frame phase is not the one this window was paired from)",
                bins.unpaired,
                bins.total_wrong,
                bins.total,
                bins.nearest_rival,
                crate::v92::UP_INTERVALS - 1,
            ));
            return;
        }
        let (_, _, level_gain, _) = self.rx.measured_levels();
        let lc = crate::v92::rung::lc_for(parameters.drn);
        let gain = crate::v92::upchoice::gain_for(lc);
        let reading = crate::v92::rung::Reading {
            drn: parameters.drn,
            lc,
            level_lsb: gain.recip() * level_gain,
            gain,
            bin: bins.weighted(),
            // The live receive path keeps the decided index, not the level
            // stream the crossing is measured on: `n/a`, not zero.
            crossing: f64::NAN,
            symbols: bins.total,
        };
        let verdict = crate::v92::rung::admit(&reading);
        self.note(format!("V.92: B1u confirms the ladder: {}", reading.line(verdict)));
        if !verdict.granted() {
            self.note(
                "V.92: the rung CPd carries is refused by B1u's own bin rate -- the number is above"
                    .to_owned()
                    + " and it is what the next renegotiation has to act on",
            );
        }
    }

    /// Which way up B1u came, from the two readings [`Self::decide_b1u_boundary`] took.
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
        // The gate itself now lives in [`Self::decide_b1u_boundary`], which is
        // the only caller: a candidate that does not read as B1u is abandoned
        // there, with the boundary it claimed in hand, before this runs. What
        // is left here is the verdict on a window that has already passed it.
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
        let best = straight.max(flipped);
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
    ///
    /// The symbol in the note is the candidate *boundary* that lost, not the
    /// symbol the verdict was reached on. It used to be `self.now`, which is
    /// the end of the 576-symbol window and so a number that looks like a
    /// boundary and is not one: `/dev/shm/reports/e2u-boundary.md` reads 722
    /// symbols of distance between one call's failing candidate and another's
    /// good one, and 576 of those 722 are this stamp.
    fn abandon_b1u(&mut self, end: u64, origin: u64, back: usize, best: u64, other: u64, scored: u64) {
        self.note(format!(
            "V.92: candidate B1u at {origin} (run end {end}, back {back}) does not read as B1u \
             ({best}/{scored} ones one way up, {other} the other), still hunting E2u"
        ));
        // A candidate that does not read as ones can be the wrong *boundary* or
        // the wrong pair-first offset. It is the boundary: the offset no longer
        // moves here. It used to rotate `{2, 3, 1}` between attempts, which was
        // a search for the value the constant already names, and the measurement
        // behind the constant (`docs/88` §9, `PAIR_FIRST_INDEX_OFFSET`) says it
        // does not vary from call to call. A candidate that does not read as
        // ones is blamed on the boundary again, as it was before the rotation.
        self.upstream_data = None;
        self.b1u_probe = None;
        self.b1u_search = None;
        self.b1u_ones = [0; 2];
        self.b1u_scored = [0; 2];
        self.b1u_polarity = 1.0;
        self.rx.set_slicer(Slicer::FourPoint);
        self.awaiting_e2u = true;
        self.e2u_zeroes = 0;
        if self.b1u_deadline.is_none() {
            self.b1u_pending = true;
            self.b1u_deadline = Some(self.start_up_by());
        }
    }

    /// One upstream sequence, as 8.7.2-8.7.6 delivered it.
    fn up_sequence(&mut self, family: CpFamily) {
        let now = self.now;
        match family {
            CpFamily::Suvu(suvu) => {
                self.note(format!("V.92: SUVu, ack {}", suvu.ack));
                // Table 27 bits 27:31, remembered for `V92_CODE_GRID`'s dump and
                // for nothing else: 0.25 dB and relative, so it cannot place a
                // point (docs/114 §1.5, R5). The Agere on this bench sends the
                // printed 16, "no measurement has been taken", on every frame.
                self.suvu_level = suvu.level;
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
                let first = self.cpu.is_none();
                // Which level set this call is running, read off the parameters
                // the design built and not off the environment: the code arm is
                // the only one bit 128 says anything about, and asking
                // `V92_LEVELSET` again here is what kept the fallback armed in
                // the arm meant to restore the old path.
                let code_arm = self.parameters.as_ref().is_some_and(|p| p.levelset == LevelSet::Code);
                self.read_code_grid(&cp, first);
                if first && cp.codec.is_some() && code_arm {
                    // docs/114 R3. Bit 128 says "the transmit constellations
                    // differ from the codec's D/A output constellations", so the
                    // mask below is *not* a level table for this far end's
                    // transmitter and this end must not read it as one. What is
                    // left is the behaviour every call had before the code grid
                    // existed: slice on the design's own levels, measured
                    // rather than reported. Named once per call, because a far
                    // end that has said it will say it on every repeat.
                    self.note(format!(
                        "V.92: CPu bit 128 set -- the far end's transmit levels are not its codec's, \
                         so the upstream slicer keeps the design's measured levels and no code table \
                         is built from its mask ({} masks, delta {})",
                        cp.constellations.len(),
                        cp.codec.as_ref().map_or(0, Vec::len)
                    ));
                }
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

    /// Read the far end's code grid out of a CPu: `K` for the upstream slicer
    /// (docs/114 R2a/R4) and, under `V92_CODE_GRID`, the one-per-call dump of
    /// the fields that report it (R6).
    ///
    /// `K` is the fit of this end's design against the codes the far end says it
    /// will put on the line -- the mask read through the law bit 35 names. A far
    /// end that snaps a uniform ladder onto its codes reports *fewer* codes than
    /// the design has points, which is why the fit pairs by position rather than
    /// by nearest (`codegrid::estimate_from_mask`).
    ///
    /// Nothing here changes what is computed per symbol: `K` is turned into the
    /// slicer's levels once, at `begin_b1u`, and this end's own levels are what
    /// is used when there is no mask to read or when bit 128 says the mask is
    /// not about the far end's transmit path.
    fn read_code_grid(&mut self, cp: &Cp, first: bool) {
        let Some(parameters) = self.parameters.as_ref() else {
            self.up_scale = 1.0;
            self.up_scale_source = "design (no CPd)";
            return;
        };
        let Some(set) = parameters.sets.first() else {
            self.up_scale = 1.0;
            self.up_scale_source = "design (no set)";
            return;
        };
        // **The gate, and it is the whole of the fix.** Everything below -- the
        // `K` fit, the level table `begin_b1u` builds at `K`, and the bit-128
        // fallback that decides whether the fit is allowed at all -- is
        // docs/114 R2a/R3/R4, and R1-R4 are a design on the far end's *code
        // grid*. On the odd lattice there is no scale to fit: the design's
        // points are the table, the far end's mask is a rate mask and not a
        // level list, and a least-squares fit through the origin against eight
        // points of `2 eta + 1` returns whatever the mask's codes happen to
        // average to -- 433.75 on the replay corpus, from a mask whose codes
        // run 256 to 2000. That number is meaningless and it is not harmless:
        // it was `K` on the uniform arm, which read B1u at 327 of 576 where
        // the same call at `cb51415` read 576 of 576.
        //
        // So the uniform arm does not fit, does not consult the mask, does not
        // look at bit 128, and leaves `K` at one. It is the path `cb51415` took:
        // no `read_code_grid` existed there, so the source below is named for
        // the arm rather than for a decision, and the dump further down says so.
        // What the mask says the far end's codes are, decoded through the law
        // bit 35 names. This is a read of the CPu and not a decision: it is
        // what the `V92_CODE_GRID` dump below prints either way, and it is
        // computed here so that the dump stays where it is.
        let law = crate::v92::codegrid::law(cp.a_law);
        let codes: Vec<u16> = cp
            .points(0)
            .iter()
            .map(|&ucode| crate::v92::codegrid::value_in(law, ucode))
            .filter(|&value| value > 0)
            .collect();
        // The fit and the bit-128 fallback, in the code arm only.
        if parameters.levelset == LevelSet::Uniform {
            self.up_scale = 1.0;
            self.up_scale_source = "design (uniform)";
        } else {
            let fitted =
                if cp.codec.is_some() { None } else { crate::v92::codegrid::estimate_from_mask(set, &codes) };
            match fitted {
                Some(k) if k.is_finite() && k > 0.0 => {
                    self.up_scale = k;
                    self.up_scale_source = "mask";
                }
                _ if cp.codec.is_some() => {
                    self.up_scale = 1.0;
                    self.up_scale_source = "design (bit 128)";
                }
                _ => {
                    self.up_scale = 1.0;
                    self.up_scale_source = "design (no mask)";
                }
            }
        }
        if !first || !code_grid_log() {
            return;
        }
        let blocks: Vec<String> = cp.constellations.iter().map(|mask| format!("{mask:032x}")).collect();
        let union = cp.constellations.iter().fold(0u128, |all, mask| all | mask);
        let popcount = union.count_ones();
        let highest = 127 - union.leading_zeros().min(127);
        eprintln!(
            "V92_CODE_GRID: bit35 codec {} bit128 {} bit52:67 {} ({} x TRN1d at the codec's D/A) \
             SUVu27:31 {} mask {} blocks [{}] popcount {} max Ucode {} design LC {} fit K {:.5} ({}) codes {}",
            if cp.a_law { "A-law" } else { "mu-law" },
            if cp.codec.is_some() { "set" } else { "clear" },
            cp.trn1d_ratio,
            f64::from(cp.trn1d_ratio) / 8192.0,
            match self.suvu_level {
                None => "16 (no measurement)".to_owned(),
                Some(level) => format!("{level:.2} dB"),
            },
            cp.constellations.len(),
            blocks.join(" "),
            popcount,
            highest,
            set.len(),
            self.up_scale,
            self.up_scale_source,
            codes.len()
        );
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
                //
                // The arithmetic of the detector is not this stage's to
                // change (docs/131). What the report is for now is to confirm
                // the timetable -- `su_advance` logs where it landed against
                // the schedule -- and to re-anchor the epsilon window on the
                // Su that follows it, which is why `su_from` is taken from
                // `at` and not from the schedule.
                self.su_seen = Some(at);
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
        //
        // A renegotiation is the one stretch where **neither** reader is on the
        // line: `ru_ended` drops both (`upstream` and `upstream_data`) so that
        // the far end's `TRN2u`/`SUVu` are read by `reneg_symbol`, which
        // `symbol_upstream` routes to on `reneg`/`reneg_trn2u_from`. Gating the
        // feed on the two readers alone therefore silences the upstream for the
        // whole of 9.8: no `SUVu` is decoded, `Exchange::peer_suv` is never
        // called, the tail stops at `RenegStage::Suvd` repeating `SUVd`, and the
        // far end never gets the `CPd` 9.6.1.1.2 owes it (docs/227).
        if self.upstream.is_some()
            || self.upstream_data.is_some()
            || self.reneg.is_some()
            || self.reneg_trn2u_from.is_some()
        {
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
                // A line with nothing on it hands the reader **no bits**. The
                // receiver says so with `decided: None` -- its equaliser has no
                // input to decide on, so it emits the quiet symbol and no
                // decision (`receiver.rs`, `equalise_and_slice`) -- and the
                // alternative, reading the quiet symbol's `0.0` as a level, is
                // twelve zeroes in the `CPt`'s own next-repetition slot: an
                // `E1u` the far end never sent. That is what ended Phase 3 off
                // the far end's legal pre-`TRN2u` idle in the HSF call
                // (`docs/294` §2, `docs/295` §1). A real `E1u` is a transmitted
                // data frame of zeroes and arrives as decided symbols, so it is
                // not this that refuses it.
                if symbol.decided.is_none() {
                    return;
                }
                let Some(bit) = self.reader.feed(symbol.value) else {
                    return;
                };
                self.cpt_bit(bit, symbol.raw);
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
        // 9.5.1.1.3's wait: how much of the clause's 500 ms this end uses, on
        // the frame grid so the following signal still begins on a boundary.
        let wait = self.settings.habits.sd_after_ja_symbols() as u64;
        if wait > 0 {
            self.note(format!(
                "V.92: waiting {}T (9.5.1.1.3) before Sd",
                wait
            ));
        }
        self.source.change_after(Out::Sd, wait as usize);
        self.stage = Stage::AwaitSu;
        // DR2 runs from TRN1d's start, and everything between here and there
        // has to come off the deadline: Sd is 384T, S-bar-d 48T, and TRN1d is
        // as long as this end was built to send it. Counting only TRN1d's
        // 2040T minimum leaves the allowance short by the rest of TRN1d --
        // 2.9 s with `Habits::LIVE` -- so a far end that uses the 5000 ms
        // 9.5.2.1.6 allows it before Su is cut off, and the call retrains
        // instead of connecting. That is what an Agere SV92PP on an 8 kHz
        // line did: it waited, and this end had already gone back to tone A.
        // `now` is the sample `Ja` was read on and every term below is a
        // duration this end spends before TRN1d starts: the 9.5.1.1.3 wait,
        // Sd, S-bar-d, and then TRN1d itself. The wait is a term here for the
        // same reason Sd and S-bar-d are: leave it out and the deadline is
        // armed `wait` symbols early, which for a 500 ms wait is the whole of
        // what the clause allows Su to be late by.
        let trn1d = (self.settings.habits.trn1d * FS) as u64;
        self.su_deadline = Some(
            self.now
                + wait
                + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64
                + trn1d
                + ((SU_WITHIN + self.settings.round_trip) * FS) as u64,
        );
        if no_dil && !ZERO_DIL_USES_SCR {
            // No Sd goes out on this path at all, so there is nothing for
            // 9.5.1.1.3's wait to precede and it is not spent: the deadline
            // above carries it, which can only make the deadline later.
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

    fn cpt_bit(&mut self, bit: bool, at: u64) {
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
                //
                // The first symbol of TRN2u is the one **after** the symbol this
                // bit arrived on, and that is `symbol.raw`, not the newest
                // sample: the equaliser is a centred one, so its decision for
                // the symbol the samples are centred on is only emitted
                // `ahead` symbols later (`receiver.rs`, `equalise_and_slice`,
                // `raw: instant = count - ahead`). Taking `rx.symbols()` here is
                // taking the newest sample and is `ahead` - 1 = 47 symbols late
                // at 96 forward taps -- which is 20 symbols past the far end of
                // the alignment the fit can absorb, and is what the HSF call's
                // `no phase 4 fit on TRN2u` was reading (`docs/297` section 2).
                self.begin_upstream_at(at.saturating_add(1));
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

    /// 9.6.1.2.1 (`v92.txt:2770`): *"If the digital modem does not receive B1u
    /// within 20 s plus 6 round-trip delays from the end of INFO1a, the digital
    /// modem shall initiate a retrain according to 9.7.1.1"*.
    ///
    /// The number was 2.0 s and counted from `Ed` until `docs/291`. Both halves
    /// of that were ours: the clause allows 20 s, and it counts them from the
    /// end of INFO1a rather than from the `Ed` this end sends at the end of
    /// Phase 4. Pinned here so the self-imposed limit cannot come back quietly.
    #[test]
    fn b1u_hunt_is_the_clauses_twenty_seconds_from_info1a() {
        assert_eq!(B1U_WAIT, 20.0, "9.6.1.2.1 allows 20 s, not 2 s");
        assert_eq!(B1U_WAIT_RTD, 6.0, "9.6.1.2.1 adds 6 round-trip delays");
        let mut zero = settings();
        zero.round_trip = 0.0;
        let modem = Modem::new(zero);
        // Anchored at the end of INFO1a (`now` = 0), not at the moment it is
        // armed: 20 s of line time, and never longer.
        assert_eq!(modem.start_up_by(), (20.0 * FS) as u64);
        let mut slow = settings();
        slow.round_trip = 0.05;
        let modem = Modem::new(slow);
        // 20 s + 6 x 50 ms, and no more than the clause's own sum.
        assert_eq!(modem.start_up_by(), ((20.0 + 6.0 * 0.05) * FS) as u64);
    }
    use crate::v90::encoder::{Decoder as CodeDecoder, Frame as CodeFrame};
    use crate::v90::sequences::Layout;
    use crate::v90::sign::Redundancy;
    use crate::v92::sequences::{DownFamily, DownFinder};
    use crate::v92::up_signals::{ru, su, Trn1u, TwoPointSender};
    use crate::v92::upstream::UpEncoder;
    use crate::v92::modulus::B1U_ACQUIRE_FRAMES;
    use crate::v92::Filters;

    /// A CPt of the shape the analogue modem sends in Phase 3: one constellation
    /// used in all six intervals, at the rate `drn` names.
    fn cpt(drn: u8, points: std::ops::RangeInclusive<u8>) -> Cp {
        let mask = points.fold(0u128, |m, u| m | 1 << u);
        Cp {
            data_mode: false,
            drn,
            redundancy: crate::v90::sign::Redundancy::None,
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
        /// Whether this far end's `Su` turns over at 144T as 9.5.2.1.7 says it
        /// does, or never turns over at all. Which of the two it is decides
        /// which of the two paths Phase 3 takes out of `AwaitSu`, and both are
        /// measured on this bench: the Agere reverses 54 symbols after its Su is
        /// reported, the HSF sends one unmodulated tone for 4.63 s.
        reverses: bool,
        /// Symbols of `Su` before its `S-bar-u`: 9.5.2.1.7's 144T on a
        /// conforming far end, and whatever a late one is scripted with.
        bar1_at: u64,
        /// Whether this far end sends `Su` at all. A far end that sends none is
        /// 9.5.1.2.2's case and nothing else's.
        su: bool,
        /// Whether the `E1u` that ends the `CPt` series has gone out. 8.5.2
        /// makes it "a data frame of scrambled, differentially encoded zeroes",
        /// sent after the last `CPt` on the same chain -- so this Script sends
        /// it, and does not go quiet where it belongs. Quiet was read as an
        /// `E1u` by this bench for as long as the reader was fed the receiver's
        /// quiet symbol, which is the defect `docs/295` §1 names.
        e1u: bool,
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
            Self {
                trn,
                ja: Some(ja),
                cp,
                second_trn: Trn1u::new(),
                e1u: false,
                reverses: true,
                bar1_at: crate::v92::SU_SYMBOLS as u64,
                su: true,
            }
        }

        /// The same, turning over `late` symbols after `Su` starts instead of at
        /// 9.5.2.1.7's 144T.
        fn reversing_at(late: u64, wanted: &Descriptor, rates: u32, cp: Option<Cp>) -> Self {
            Self { bar1_at: late, ..Self::new(wanted, rates, cp) }
        }

        /// A far end that never sends `Su` at all: 9.5.1.2.2's third case.
        fn without_su(wanted: &Descriptor, rates: u32) -> Self {
            Self { su: false, ..Self::new(wanted, rates, None) }
        }

        /// The same call from a far end that breaks 9.5.2.1.7's second clause:
        /// `Su` from its start to the point where it answers, with no
        /// `S-bar-u` in it anywhere.
        fn without_reversal(wanted: &Descriptor, rates: u32, cp: Option<Cp>) -> Self {
            Self { reverses: false, ..Self::new(wanted, rates, cp) }
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
            if !self.su {
                return 0.0;
            }
            let bar1 = SU_START + self.bar1_at;
            let su2 = bar1 + 24;
            let bar2 = su2 + 400;
            let trn2 = bar2 + 40;
            let cpt = trn2 + 2400;
            if self.reverses {
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
            } else {
                // One tone from the start of Su to the point where the far end
                // answers: no reversal, no step, nothing the hunt can call a
                // turn -- and it stops exactly where the tone was measured to
                // stop, when the far end hears a Jp it never asked for.
                if n < trn2 {
                    return su(n, false);
                }
            }
            if n < cpt {
                return self.second_trn.next_symbol();
            }
            if let Some(cp) = self.cp.as_mut() {
                if let Some(level) = cp.next_symbol() {
                    return level;
                }
                // The `CPt` series is over: 8.5.2's twelve zeroes, on the chain
                // the series was sent on, where the next `CPt` would have begun.
                if !self.e1u {
                    self.e1u = true;
                    cp.push(&[false; E1U_ZEROES]);
                }
                if let Some(level) = cp.next_symbol() {
                    return level;
                }
            }
            0.0
        }
    }

    fn run(script: &mut Script, symbols: u64) -> Modem {
        let mut modem = Modem::new(settings());
        let mut n = 0;
        drive(&mut modem, script, &mut n, symbols);
        modem
    }

    /// Carry an existing modem on along the same script, so that what a stage
    /// looked like part-way through can be asserted without running it twice.
    fn drive(modem: &mut Modem, script: &mut Script, n: &mut u64, symbols: u64) {
        for _ in 0..symbols {
            let level = script.next(*n);
            modem.step(level * 8000.0);
            *n += 1;
        }
    }

    /// The reversal symbol a phase 3 line reports, from the log.
    fn reversal_at(notes: &str) -> u64 {
        let tag = "reversal seen at ";
        let i = notes.find(tag).unwrap_or_else(|| panic!("no reversal in: {notes}")) + tag.len();
        notes[i..]
            .split(|c: char| !c.is_ascii_digit())
            .next()
            .and_then(|digits| digits.parse().ok())
            .unwrap_or_else(|| panic!("no symbol after `{tag}`: {notes}"))
    }

    /// The phase 3 line, of which there is exactly one per call.
    fn phase3_line(notes: &str) -> String {
        let count = notes.matches("phase 3:").count();
        assert_eq!(count, 1, "expected exactly one phase 3 line, found {count}: {notes}");
        notes
            .lines()
            .find(|line| line.contains("phase 3:"))
            .unwrap_or_default()
            .to_string()
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

    /// A CPd for drn 1 as `upchoice` would send it, and a `Modem` holding it.
    fn modem_with_cpd() -> Modem {
        let design = crate::v92::design::Design {
            filters: Filters { z2: vec![1.0], ..Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        // The code grid, named rather than read from `V92_LEVELSET`: these
        // tests are about docs/114 R2a/R3/R4, whose `K` fit and level table are
        // written against a design whose points are code values. The arm the
        // default sends is the odd lattice, and the same fit against it is a
        // different question (see `upchoice`'s switch).
        let parameters = crate::v92::upchoice::pin_with(
            &design,
            1,
            Trellis::Sixteen,
            1e-9,
            crate::v92::upchoice::LevelSet::Code,
        )
        .expect("drn 1 is pinnable")
        .parameters;
        let mut modem = Modem::default();
        modem.parameters = Some(parameters);
        modem
    }

    /// A CPu carrying `codes` in interval 0's mask, with bit 128 as asked.
    fn cpu_with_mask(codes: &[u8], differ: bool) -> Cp {
        let mask = codes.iter().fold(0u128, |mask, &ucode| mask | 1 << ucode);
        Cp {
            data_mode: true,
            drn: 1,
            intervals: [0; INTERVALS],
            constellations: vec![mask],
            codec: differ.then(|| vec![mask]),
            ..Cp::default()
        }
    }

    /// docs/114 R3: a far end that sets bit 128 says its transmit levels are not
    /// its codec's, and the mask it sends with that is not a level table. The
    /// call falls back to the design's own measured levels -- **the slice path
    /// does not move**, which is asserted as the table being bit-identical --
    /// and says so once, by name.
    #[test]
    fn a_far_end_that_sets_bit_128_falls_back_and_says_so() {
        use crate::v92::codegrid;
        use crate::v92::sequences::CpFamily;
        let mut modem = modem_with_cpd();
        let set = modem.parameters.clone().expect("a CPd").sets[0].clone();
        // The mask a code-transparent far end would report for this design.
        let codes: Vec<u8> = set.iter().map(|&point| codegrid::nearest_code(f64::from(point))).collect();

        modem.up_sequence(CpFamily::Cpu(cpu_with_mask(&codes, true)));
        let notes = modem.clone().take_notes().join("\n");
        assert!(notes.contains("bit 128 set"), "the fallback has to name itself: {notes}");
        assert_eq!(modem.up_scale, 1.0, "bit 128 must not move the slicer's scale");
        assert_eq!(modem.up_scale_source, "design (bit 128)");
        // And with K = 1 the table is the design's own, point for point: this is
        // what "the slice path is unchanged" means in code.
        assert_eq!(codegrid::scaled(&set, modem.up_scale), set, "the fallback changed the levels");
        // A second CPu on the same call does not repeat the event: it is one
        // event per call, not one per repeat.
        modem.up_sequence(CpFamily::Cpu(cpu_with_mask(&codes, true)));
        let again = modem.clone().take_notes().join("\n");
        assert_eq!(again.matches("bit 128 set").count(), 1, "{again}");
    }

    /// docs/114 R2a/R4: with bit 128 clear the mask *is* about the far end's
    /// transmitter, so `K` is fitted from it and the slicer's table is built at
    /// that scale rather than at one.
    #[test]
    fn a_code_transparent_mask_sets_k_and_the_table_follows_it() {
        use crate::v92::codegrid;
        use crate::v92::sequences::CpFamily;
        let mut modem = modem_with_cpd();
        let set = modem.parameters.clone().expect("a CPd").sets[0].clone();
        for k in [1.0f64, 0.98, 1.03] {
            // What the far end reports when its codes arrive at this end's scale
            // times k: the mask is a list of Ucodes, so the fit has to see them
            // as codes and not as levels.
            let mut codes: Vec<u8> =
                set.iter().map(|&point| codegrid::nearest_code(k * f64::from(point))).collect();
            codes.dedup();
            modem.cpu = None;
            modem.up_sequence(CpFamily::Cpu(cpu_with_mask(&codes, false)));
            assert_eq!(modem.up_scale_source, "mask", "k {k}: no fit was taken");
            assert!((modem.up_scale - k).abs() < 0.05 * k, "k {k} came back as {}", modem.up_scale);
            // The table is `round(K a(eta))`, and at K = 1 it is the design's own
            // -- so a call whose far end is on this end's scale is untouched.
            let table = codegrid::scaled(&set, modem.up_scale);
            if k == 1.0 {
                assert_eq!(table, set);
            } else {
                assert_ne!(table, set, "k {k}: the table did not move");
            }
        }
    }

    /// The gate: the uniform arm never consults the `K` table.
    ///
    /// `d25ca78` made the level set a switch at the *transmitter* and left the
    /// receiver fitting `K` in both arms, so the default arm sliced on a table
    /// scaled by a number fitted against a lattice it does not describe: 433.75
    /// out of the corpus mask, and B1u read 327 of 576 where `cb51415` read 576
    /// of 576. This pins the gate from the inside, three ways, because "the
    /// regression is gone" and "the test that says so can fail" are not the same
    /// claim:
    ///
    /// 1. the uniform arm's table ignores `K` at *any* value, the fitted one
    ///    included -- it is not a `K` that happens to be one;
    /// 2. a fit is still taken in the code arm, so (1) is not vacuous;
    /// 3. a uniform call fed the very mask that produced 433.75 leaves `K` at
    ///    one, which is the property the replay numbers rest on.
    #[test]
    fn the_uniform_arm_never_consults_the_k_table() {
        use crate::v92::codegrid;
        use crate::v92::sequences::CpFamily;
        let design = crate::v92::design::Design {
            filters: Filters { z2: vec![1.0], ..Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        // 433.75 is not a made-up number: it is what `estimate_from_mask`
        // returned on the replay corpus, where the far end's codes run 256 to
        // 2000 against eight points of `2 eta + 1`.
        const FITTED: f64 = 433.75;
        // The mask's codes, as Ucodes: the far end's own values 256 ... 2000.
        let codes: Vec<u8> = [256.0, 512.0, 768.0, 1024.0, 1280.0, 1536.0, 1792.0, 2000.0]
            .iter()
            .map(|&value| codegrid::nearest_code(value))
            .collect();

        // (1) The uniform table, at the fitted K and at one, is the design's own
        // set -- unchanged, and unchanged by `K`.
        let uniform = crate::v92::upchoice::pin_with(&design, 1, Trellis::Sixteen, 1e-9, LevelSet::Uniform)
            .expect("drn 1 on the odd lattice")
            .parameters;
        assert_eq!(uniform.levelset, LevelSet::Uniform);
        let set = uniform.sets[0].clone();
        assert_eq!(set, vec![1, 3, 5, 7, 9, 11, 13, 15]);
        for k in [1.0f64, FITTED, 0.5, 1e6] {
            assert_eq!(up_table(LevelSet::Uniform, &set, k), set, "K {k} reached the uniform table");
        }
        // And the same call in the code arm *is* moved by it, so the loop above
        // is a statement about the level set and not about `scaled` being the
        // identity on these points.
        let code = crate::v92::upchoice::pin_with(&design, 1, Trellis::Sixteen, 1e-9, LevelSet::Code)
            .expect("drn 1 on the grid")
            .parameters;
        assert_ne!(up_table(LevelSet::Code, &set, FITTED), set, "the code arm ignored K");

        // (2) The read path. A uniform call and a code call, the same CPu.
        let read = |levelset| {
            let parameters =
                crate::v92::upchoice::pin_with(&design, 1, Trellis::Sixteen, 1e-9, levelset)
                    .expect("drn 1")
                    .parameters;
            let mut modem = Modem::default();
            modem.parameters = Some(parameters);
            modem.up_sequence(CpFamily::Cpu(cpu_with_mask(&codes, false)));
            (modem.up_scale, modem.up_scale_source)
        };
        let (uniform_k, uniform_source) = read(LevelSet::Uniform);
        assert_eq!(uniform_k, 1.0, "the uniform arm took a fit");
        assert_eq!(uniform_source, "design (uniform)", "and its K has to say which arm left it at one");
        // The code arm fits the same mask and *says* it fitted, which is what
        // makes the assertion above a property of the level set and not of
        // `read_code_grid` having stopped working altogether.
        let (code_k, code_source) = read(LevelSet::Code);
        assert_eq!(code_source, "mask", "the code arm must still fit");
        // What the fit makes of the corpus mask against the *uniform* set is the
        // number the regression sliced at: eight points of `2 eta + 1` against
        // codes that run 256 to 2000 is a slope in the hundreds, and it is not
        // one. This is the value the uniform arm must refuse to look at.
        let values: Vec<u16> = codes.iter().map(|&code| codegrid::value(code)).collect();
        assert!(values.iter().all(|&value| value > 0), "the mask must decode to levels");
        let would_fit = codegrid::estimate_from_mask(&set, &values).expect("the mask fits something");
        assert!(would_fit > 100.0, "the mask no longer fits big: {would_fit}");
        assert_eq!(uniform_k, 1.0, "the uniform arm sliced at {would_fit}");
        // And the code arm's own scale is the design's, because its set is
        // already the far end's grid: a fit that lands on one there is the
        // grid's premise, not the lattice's 433.
        assert!((code_k - 1.0).abs() < 0.35, "the grid's own mask fits to {code_k}");
        // (3) The tables the replay reads: the uniform arm's is the design's
        // set whatever the fit says, the code arm's is `round(K a(eta))`.
        assert_eq!(up_table(LevelSet::Uniform, &set, would_fit), set);
        assert_ne!(up_table(LevelSet::Code, &code.sets[0], 1.0 + code_k), code.sets[0].clone());
    }

    /// **The data mode's table is the design's set on the odd lattice, point
    /// for point.** `docs/257` sec 2: `set_data_mode` used to put every level on
    /// the far end's code lattice, which is right for the code grid and wrong
    /// for `2 eta + 1`, and the snap is not a rounding -- it displaces each point
    /// by up to half a mu-law chord step.
    ///
    /// The assertion is on the slicer's own table, read back through
    /// `Receiver::data_levels`, and not on a helper's output: what broke the
    /// `loop-260` call was the table the decoder decided against, and a test that
    /// stops one call short of it would have passed on `54e9e1e`.
    #[test]
    fn the_data_mode_table_is_the_designs_set_on_the_odd_lattice() {
        let design = crate::v92::design::Design {
            filters: Filters { z2: vec![1.0], ..Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        let uniform =
            crate::v92::upchoice::pin_with(&design, 1, Trellis::Sixteen, 1e-9, LevelSet::Uniform)
                .expect("drn 1 on the odd lattice")
                .parameters;
        assert_eq!(uniform.levelset, LevelSet::Uniform);
        assert_eq!(uniform.sets[0], vec![1, 3, 5, 7, 9, 11, 13, 15]);
        let levels = data_mode_levels(&uniform, 1.0);
        // The eight points the far end will transmit, at CPd's own `G`: `G a(eta)`
        // and its negation, and nothing else. This is the call `docs/256` sec 1.2
        // made -- `levelset uniform`, `G` 0.1085, eight points.
        for eta in uniform.sets[0].iter().copied().map(f64::from) {
            let want = uniform.gain * eta;
            assert!(
                levels.iter().any(|&level| (level - want).abs() < 1e-12),
                "drn 1 point {} is not in the table the slicer decides against",
                eta,
            );
        }
        // The live path: the slicer holds what the design emitted, unchanged.
        let mut rx = Receiver::new();
        rx.set_data_mode(levels.clone());
        assert_eq!(rx.data_levels(), levels.as_slice(), "the slicer moved a point of the design's set");
        // The mechanism, so that removing the gate fails here and not on a bench:
        // snapping this set is not the identity, and what it moves is not small.
        let snapped = crate::v92::receiver::lattice_levels(&levels);
        let worst = levels
            .iter()
            .zip(&snapped)
            .map(|(design, snap)| (design - snap).abs() * crate::v92::receiver::LATTICE_LSB)
            .fold(0.0f64, f64::max);
        let spacing = uniform.gain * 2.0 * crate::v92::receiver::LATTICE_LSB;
        assert!(worst > 0.0, "the snap is the identity on this set; the gate is untested");
        assert!(worst > 0.03 * spacing, "{worst} LSB of a {spacing} LSB step is not the mechanism");
        // And the code grid's own table *is* put on the lattice, so the gate is a
        // gate and not a removal: every point of it is a code value.
        let code = crate::v92::upchoice::pin_with(&design, 1, Trellis::Sixteen, 1e-9, LevelSet::Code)
            .expect("drn 1 on the grid")
            .parameters;
        assert_eq!(code.levelset, LevelSet::Code);
        for level in data_mode_levels(&code, 1.0) {
            let moved = (crate::v92::receiver::on_lattice(level) - level).abs();
            assert!(moved < 1.0 / crate::v92::receiver::LATTICE_LSB, "{level} is off the code grid by {moved}");
        }
    }

    /// docs/131 case (a): a **compliant** far end -- `Su` 144T, its 24.5T
    /// `S-bar-u`, `Su` again -- is confirmed by the reversal detector, and the
    /// detector's report is what the timetable is checked against. No timetable
    /// event is raised, and the epsilon window opens on the `Su` **after** the
    /// reversal (9.5.1.1.9), wherever the detector put it.
    #[test]
    fn a_compliant_peer_confirms_the_timetable_with_no_event() {
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let mut script = Script::new(&Descriptor::none(), 0x7ffff, Some(cp));
        let mut modem = Modem::new(settings());
        let mut n = 0u64;
        // Past the reversal and into the run whose phase Jp is measured on,
        // but not so far that Jp has gone out and the window been cleared.
        drive(&mut modem, &mut script, &mut n, SU_START + 144 + 90);
        let notes = modem.clone().take_notes().join("\n");
        let line = phase3_line(&notes);
        assert!(line.contains("reversal seen at"), "{line}");
        assert!(!line.contains("reversal absent"), "a reversing peer raised an event: {line}");
        assert!(!line.contains("advanced on timetable"), "a timetable event fired: {line}");
        assert!(line.contains("advanced on the reversal"), "{line}");
        // The window is 9.5.1.1.9's: the Su after the S-bar-u the hunt found,
        // and not the schedule's 144T.
        let at = reversal_at(&notes);
        assert_eq!(modem.su_from, Some(at + SU_AFTER_S_BAR), "epsilon window moved: {line}");
        // And the call completes: Jp with the epsilon off that window, then
        // Phase 4.
        drive(&mut modem, &mut script, &mut n, 400 + 40 + 2400 + 1600);
        let notes = modem.clone().take_notes().join("\n");
        assert!(notes.contains("Jp with epsilon"), "no Jp off the window: {notes}");
        assert_eq!(phase3_line(&notes).matches("reversal seen at").count(), 1, "{notes}");
        assert_eq!(modem.stage, Stage::Phase4, "phase {} notes {notes}", modem.phase());
    }

    /// Where the far end's first `CPt` begins on [`Script`]: `Su` 144T, the
    /// 24.5T `S-bar-u`, `Su` again, the `S-bar-u` that answers `Jp`, the second
    /// `TRN1u` at 2400T, and then it.
    const CPT_START: u64 = SU_START + 144 + 24 + 400 + 40 + 2400;

    /// The far end's idle, at [`Script`]'s own level: the same sample every
    /// time, so mean = the sample and std 0.000, which is the shape `docs/294`
    /// §2 measured on the live line (there in the codec's own units, -1 LSB;
    /// the guard reads the shape, which is scale-free).
    const IDLE: f64 = -8000.0;

    /// **The three false decisions, pinned.** A reader trained on a real Phase 3
    /// that has read a real `CPt`, and then the far end's legal idle as what it
    /// is on this line -- a constant, delivered whole, mean -1.000 and std
    /// 0.000 -- must decide nothing off it: no `CPt`, no `E1u`, and no `Su`.
    ///
    /// This is the live sequence exactly (`docs/294` §2): `CPt drn 9` at 15.640,
    /// `E1u` at 15.960, both inside the far end's 3.98 s idle, and both of them
    /// ours. Before `docs/295` §1 the constant was audible (it has power), the
    /// equaliser sliced it to one repeated decision -- a stream of zeroes, which
    /// is the shape `E1u` is counted from -- and twelve of them in the `CPt`'s
    /// own next-repetition slot was `E1u`.
    #[test]
    fn the_far_end_s_constant_line_is_not_a_cpt_and_not_an_e1u_and_not_a_su() {
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let repetition = cp.to_bits_in(Layout::V92, UP_INTERVALS).len() as u64;
        let mut script = Script::new(&Descriptor::none(), 0x7ffff, Some(cp));
        let mut modem = Modem::new(settings());
        let mut n = 0u64;
        // A real `CPt`, read to the end of one repetition -- `cp_len` is set by
        // the sequence's own CRC, so this is a `CPt` and not a guess. `E1u` is
        // only decided out of a state a far end that never sent one cannot
        // reach, and the false `E1u` at 15.960 was decided out of exactly this
        // one.
        while modem.cp_len == 0 && n < CPT_START + 4 * repetition {
            modem.step(script.next(n) * 8000.0);
            n += 1;
        }
        assert!(modem.cp_len != 0, "no CPt was read, so nothing is pinned");
        // Then two hundred symbols of the idle before anything is asserted: the
        // step from a real `CPt` into a constant is one the equaliser's own
        // window spans, so for the length of that window the line still has
        // shape in it and deciding there is not what this pins. What is left
        // after it is the question, and it is the live one: `CPt drn 9` at
        // 15.640 and `E1u` at 15.960 were 0.32 s -- two thousand five hundred
        // and sixty symbols -- into the far end's idle, nowhere near a step.
        for _ in 0..200 {
            modem.step(IDLE);
        }
        let before = modem.clone().take_notes().join("\n");
        assert!(before.contains("V.92: CPt drn"), "no CPt was read, so nothing is pinned: {before}");
        // The idle, whole.
        for _ in 0..IDLE_SYMBOLS_PHASE3 {
            modem.step(IDLE);
        }
        let notes = modem.clone().take_notes().join("\n");
        let after = &notes[before.len()..];
        assert!(
            !after.contains("V.92: E1u"),
            "E1u was decided on the far end's constant: {after}"
        );
        assert!(
            !after.contains("V.92: CPt drn"),
            "a CPt was read off the far end's constant: {after}"
        );
        assert!(
            !after.contains("V.92: Su at"),
            "a Su was read off the far end's constant: {after}"
        );
        assert!(!matches!(modem.stage, Stage::RiBar), "the constant ended Phase 3: {after}");
    }

    /// The same 3.98 s of idle, in symbols at 8 kHz.
    const IDLE_SYMBOLS_PHASE3: u64 = 31_840;

    /// **And the polarity property, on more than `Ru`.** A whole Phase 3 line
    /// inverted -- every upstream level negated, `Ru`, `TRN1u`, `Ja`, `Su`, the
    /// second `TRN1u` and `CPt` -- is still read: the hunt's anchor is its own
    /// zero point (9.5.1.1.1), so the reversal is seen, `Su` is reported and the
    /// `CPt`'s CRC still checks out.
    ///
    /// This is `ru_and_its_reversal_are_found_whatever_the_polarity` widened
    /// from the reversal to the three decisions Phase 3 is made of, and it is
    /// what makes the constant-line refusal safe to ship: the new gate refuses
    /// a line with no shape, not a line with the other sign (`docs/295` §1).
    #[test]
    fn a_polarity_reversed_phase_three_line_is_still_read_to_its_cpt() {
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let repetition = cp.to_bits_in(Layout::V92, UP_INTERVALS).len() as u64;
        let mut script = Script::new(&Descriptor::none(), 0x7ffff, Some(cp));
        let mut modem = Modem::new(settings());
        let mut n = 0u64;
        for _ in 0..CPT_START + repetition + 40 {
            modem.step(-script.next(n) * 8000.0);
            n += 1;
        }
        let notes = modem.clone().take_notes().join("\n");
        assert!(notes.contains("V.92: Su at"), "no Su on a reversed line: {notes}");
        assert!(notes.contains("reversal seen at"), "no reversal on a reversed line: {notes}");
        assert!(notes.contains("V.92: CPt drn"), "no CPt on a reversed line: {notes}");
        assert!(notes.contains("V.92: E1u"), "no E1u on a reversed line: {notes}");
    }

    /// docs/131 case (b): the **HSF signature** -- one unmodulated tone from
    /// the start of `Su` to the point where it gives up, no `S-bar-u` anywhere
    /// -- is carried out of `AwaitSu` by the timetable itself, at `S0` + 144T +
    /// 24.5T + two periods, with exactly one event logged, and Jp's epsilon is
    /// still measured on the Su after where the reversal would have been.
    #[test]
    fn a_su_that_never_reverses_is_carried_by_the_timetable_and_logged_once() {
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let mut script = Script::without_reversal(&Descriptor::none(), 0x7ffff, Some(cp));
        let modem = run(&mut script, SU_START + 144 + 24 + 400 + 40 + 2400 + 1600);
        let notes = modem.clone().take_notes().join("\n");
        let line = phase3_line(&notes);
        assert!(line.contains("reversal absent"), "{line}");
        assert!(
            line.contains(&format!("advanced on timetable at S0+{SU_TIMETABLE}T")),
            "the timetable has to say where it landed: {line}"
        );
        // 144 + 24.5 + two periods, counted in symbols from the onset; the
        // count is the Recommendation's, not a tuned one.
        assert_eq!(SU_TIMETABLE, 144 + 24 + 2 * 6, "{SU_TIMETABLE}");
        // Evidence, reported and not consulted: a tone this clean is the best
        // case the coherence can read.
        assert!(line.contains("Su held "), "the evidence is not reported: {line}");
        assert!(line.contains("coherent at "), "{line}");
        // The epsilon is still measured, on the Su after the scheduled
        // reversal, and Jp goes out with it.
        assert!(notes.contains("Jp with epsilon"), "no Jp: {notes}");
        // It is the timetable and not an override that carried the call.
        assert!(!notes.contains("V92_SU_BLIND") && !notes.contains("V92_SU_TOLERANCE"), "{notes}");
    }

    /// docs/131 case (c): a far end that sends **no Su at all** is still
    /// 9.5.1.2.2's, unchanged -- the deadline is 5.1 s plus a round trip after
    /// the start of TRN1d, the call retrains there, and the timetable never
    /// runs because there is nothing to anchor it to.
    #[test]
    fn no_su_at_all_still_retrains_at_the_recommendation_s_deadline() {
        let mut settings = settings();
        // A round trip that shows up in the deadline rather than rounding out
        // of it: 5.25 s at 8 kHz is a whole number of symbols.
        settings.round_trip = 0.15;
        let habits = settings.habits;
        let mut script = Script::without_su(&Descriptor::none(), 0x7ffff);
        let mut modem = Modem::new(settings);
        let mut n = 0u64;
        // Up to the read of Ja, which is where the deadline is armed: one
        // symbol at a time, so that `now` is the symbol it was armed on.
        while modem.su_deadline.is_none() && n < SU_START {
            drive(&mut modem, &mut script, &mut n, 1);
        }
        let notes = modem.clone().take_notes().join("\n");
        assert!(notes.contains("Ja read"), "no Ja: {notes}");
        let deadline = modem.su_deadline.expect("a deadline from Ja");
        let trn1d_start =
            modem.now + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64;
        assert_eq!(
            deadline - trn1d_start,
            (habits.trn1d * FS) as u64 + ((SU_WITHIN + 0.15) * FS) as u64,
            "the deadline is not 5.1 s plus a round trip of TRN1d's start"
        );
        let trn1d_end = trn1d_start + (habits.trn1d * FS) as u64;
        assert_eq!(deadline - trn1d_end, ((SU_WITHIN + 0.15) * FS) as u64);
        // And it is where the call goes: no Su, no timetable, a retrain.
        let left = deadline - modem.now + 8;
        drive(&mut modem, &mut script, &mut n, left);
        let notes = modem.clone().take_notes().join("\n");
        assert!(notes.contains("no Su within 5.1 s of TRN1d"), "{notes}");
        assert!(!notes.contains("phase 3:"), "a timetable ran with no Su: {notes}");
        assert!(matches!(modem.status, Status::Failed(_)), "{notes}");
        assert!(habits.trn1d * FS >= TRN1D_MINIMUM as f64, "TRN1d is at least 2040T");
        let _ = trn1d_end;
    }

    /// docs/131 case (d): a **late** reversal is taken where it is. The
    /// detector runs first, re-anchors everything downstream on the symbol it
    /// actually found, the deviation from the schedule is recorded, and the
    /// epsilon window opens on the Su after *that* -- the measurement never
    /// moves to suit the schedule.
    ///
    /// How late is measured and not assumed: the hunt reports a reversal about a
    /// period after it happens on this script, and the timetable allows two
    /// periods for that, so 168T (24T late) is still the detector's and 170T is
    /// not. The second half of this test pins that boundary; there the timetable
    /// carries the stage, which is the right answer for a far end 26T off its
    /// own timetable.
    #[test]
    fn a_late_reversal_re_anchors_and_the_schedule_is_noted() {
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let mut script = Script::reversing_at(168, &Descriptor::none(), 0x7ffff, Some(cp));
        let mut modem = Modem::new(settings());
        let mut n = 0u64;
        drive(&mut modem, &mut script, &mut n, SU_START + 168 + 90);
        let notes = modem.clone().take_notes().join("\n");
        let line = phase3_line(&notes);
        assert!(line.contains("reversal seen at"), "{line}");
        assert!(!line.contains("reversal absent"), "the timetable pre-empted it: {line}");
        assert!(line.contains("off the schedule"), "no deviation noted: {line}");
        assert!(!line.contains("+0T off the schedule"), "deviation vanished: {line}");
        // Where it landed, and that the window followed it there.
        let at = reversal_at(&notes);
        assert!(at > SU_START + 144, "the script's reversal was not late: {line}");
        assert_eq!(modem.su_from, Some(at + SU_AFTER_S_BAR), "window moved: {line}");
        drive(&mut modem, &mut script, &mut n, 400 + 40 + 2400 + 1600);
        let notes = modem.clone().take_notes().join("\n");
        assert!(notes.contains("Jp with epsilon"), "no Jp after a late reversal: {notes}");
        assert_eq!(modem.stage, Stage::Phase4, "phase {} notes {notes}", modem.phase());

        // Past the margin the timetable is what carries the stage, and what the
        // far end was doing meanwhile is reported as evidence.
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        let mut script = Script::reversing_at(170, &Descriptor::none(), 0x7ffff, Some(cp));
        let modem = run(&mut script, SU_START + 170 + 24 + 400 + 40 + 2400 + 1600);
        let notes = modem.clone().take_notes().join("\n");
        let line = phase3_line(&notes);
        assert!(line.contains("reversal absent"), "170T was still detected: {line}");
        assert!(line.contains(&format!("advanced on timetable at S0+{SU_TIMETABLE}T")), "{line}");
        assert!(line.contains("Su held "), "the hold is not reported: {line}");
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
    // ---------------------------------------------------------------------
    // E2u's end and B1u's boundary
    // ---------------------------------------------------------------------

    /// A CPd-shaped parameter set at 24 000: drn 1, twelve moduli of eight
    /// (K = 36 bits a frame), one eight-point set and the gain 8.8.3 gives it.
    fn b1u_parameters() -> Parameters {
        Parameters {
            drn: 1,
            trellis: Trellis::Sixteen,
            extend_e2u: crate::v92::upchoice::EXTEND_E2U,
            gain: crate::v92::upchoice::gain_for(8),
            moduli: [8; crate::v92::UP_INTERVALS],
            // 8.8.3's own reference prefilter: one tap, so v(n) is u(n) and the
            // synthesised line is the constellation itself.
            filters: Filters { z2: vec![1.0], ..Filters::default() },
            // Eight positive points, which is what 6.4.2's k = 3 rule needs at
            // Mi = 8 (N = 16 >= 2 Mi) and what `gain_for(8)` is the gain of.
            sets: vec![vec![1, 3, 5, 7, 9, 11, 13, 15]],
            indices: [0; crate::v92::CONSTELLATION_FRAME],
            levelset: crate::v92::LevelSet::Uniform,
        }
    }

    /// B1u as the far end transmits it (8.7.1: forty-eight data frames of
    /// scrambled ones through 6.4's chain from zeroed memories), plus enough
    /// symbols past it for the last candidate's window to be whole.
    ///
    /// The levels are `G v(n)`, which is what a perfect line would carry and
    /// what the decoder divides by `G` again, so this is a line with no
    /// channel and no noise: every bit a candidate gets wrong here is a bit the
    /// candidate's own frame phase got wrong.
    fn b1u_line() -> Vec<f64> {
        let parameters = b1u_parameters();
        let mut encoder = UpEncoder::new(&parameters).expect("drn 1's parameters encode");
        let mut levels: Vec<f64> = encoder.b1u().iter().map(|symbol| symbol.out).collect();
        while levels.len() < B1U_SYMBOLS + E2U_SYMBOLS + 4 {
            levels.push(encoder.next_symbol(|| true).out);
        }
        levels
    }

    /// The bin instrument, on the line our own encoder builds: it reads zero,
    /// and it reads zero only at the origin the decoder was actually fed.
    ///
    /// `docs/285` sec 2. The decisions are taken by a real `Decoder` fed the
    /// far end's own B1u -- `UpEncoder::b1u()`, symbol for symbol, no channel
    /// and no noise -- so every pair is right by construction and anything the
    /// instrument reports is its own arithmetic. The candidate is the `back`
    /// the boundary search actually commits to on this line: twelve symbols,
    /// because B1u's forty-eight identical frames make the whole-frame twin
    /// read as well as the boundary and `choose_b1u_boundary` prefers the
    /// nearer of two. Pairing from `skip + 1` -- the origin the code used
    /// before this ticket, which drops the candidate's `back` -- reads
    /// **0.86** on these decisions; pairing from `skip + 1 - back`, which is
    /// the element `b1u_readings` actually fed first, reads **0**.
    #[test]
    fn a_bin_pairing_reads_zero_at_the_origin_its_decoder_was_fed() {
        let parameters = b1u_parameters();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        let levels = b1u_line();
        // The fire symbol is B1u's frame interval 0; the window starts one
        // symbol into B1u.
        let buffer: Vec<f64> = levels[1..B1U_SYMBOLS].iter().copied().chain(levels[1..2].iter().copied()).collect();
        let readings = b1u_readings(&parameters, 1_000, &buffer, skip, E2U_SYMBOLS);
        let winner = choose_b1u_boundary(&readings, false).expect("a boundary in the window");
        let back = readings[winner].back;
        assert_eq!(back, crate::v92::UP_INTERVALS, "the twin a data frame behind the run's end");
        // The same decoder `b1u_readings` built for that candidate, fed the
        // same window, drained the same way `decide_b1u_boundary` drains it.
        let mut decoder = b1u_decoders(&parameters, skip).0;
        for &level in &buffer[skip.saturating_sub(back)..=B1U_SYMBOLS - 1 - back] {
            decoder.feed(level);
        }
        let decisions = decoder.take_decisions_indexed();
        assert!(!decisions.is_empty(), "the window decided nothing");
        let at_the_origin = B1uBins::measure(&parameters, &decisions, skip + 1 - back, true);
        assert_eq!(at_the_origin.unpaired, 0, "every decided symbol found its partner");
        assert_eq!(at_the_origin.total_wrong, 0, "a perfect line is zero wrong at its own origin");
        assert_eq!(at_the_origin.weighted(), 0.0, "and its rate is zero");
        assert!(at_the_origin.trusted(), "the origin is decisively better than any displaced one");
        // The origin the code used before this ticket, on the same decisions.
        // This is the number the bench call printed, and it is the reason the
        // pairing's arithmetic is the finding and not the far end's line.
        let without_back = B1uBins::measure(&parameters, &decisions, skip + 1, true);
        assert_eq!(without_back.total, at_the_origin.total, "the same population");
        assert!(
            without_back.weighted() > 0.5,
            "dropping the candidate's back must not read as a measurement: {:.4}",
            without_back.weighted()
        );
        assert!(!without_back.trusted(), "and it must refuse to publish a rate");
    }

    /// The window `b1u_readings` is handed, on a line whose run ends
    /// `overrun` symbols after `B1u`'s frame interval 0.
    ///
    /// `levels[k]` is the symbol `end + 1 + k` (`collect_b1u`), and `B1u`'s
    /// symbol `s` is at `end + 1 + k` when `s = k + 1 + B`, so the window is
    /// `levels[1 + B .. 1 + B + B1U_SYMBOLS]`. `b1u_line` is long enough for
    /// `B` up to a whole data frame.
    fn b1u_window(overrun: usize) -> Vec<f64> {
        let levels = b1u_line();
        levels[1 + overrun..1 + overrun + B1U_SYMBOLS].to_vec()
    }

    /// The decisions a committed candidate's own decoder takes off a window.
    fn b1u_window_decisions(buffer: &[f64], back: usize) -> Vec<(u64, [i32; 4])> {
        let parameters = b1u_parameters();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        let mut decoder = b1u_decoders(&parameters, skip).0;
        for &level in &buffer[skip.saturating_sub(back)..=B1U_SYMBOLS - 1 - back] {
            decoder.feed(level);
        }
        decoder.take_decisions_indexed()
    }

    /// **The instrument is exact at one offset and at chance at every other,
    /// and the whole-frame offsets are outside the sweep that looks for a
    /// rival.**
    ///
    /// `docs/287` sec 2. The window is `B1u` itself with the run ending on
    /// `B1u`'s frame interval 0 (`B = 0`, the clause's anchor), the search
    /// commits the twin a data frame behind the run's end (12, the candidate
    /// whose own ones count is highest), and the same decisions are graded at
    /// every origin from `skip - 23` to `skip + 25`. The result is not a
    /// gradient: **one offset reads zero and all forty-eight others read
    /// 163-183 of 200**, which is 1 in 8 -- the size of 6.4.2's class alphabet
    /// at `Mi = 8`, i.e. no information at all. Two things follow, and both are
    /// asserted below rather than argued:
    ///
    /// * the only other offset that can ever read zero is a whole number of
    ///   data frames away, because 12 is a multiple of the twelve-symbol frame
    ///   and of the four-symbol trellis frame, so it does not move `modulus_at`
    ///   or `position_at` -- it only rewrites *which* sent frame is compared;
    /// * `nearest_rival` looks at `1 .. UP_INTERVALS`, i.e. one to eleven
    ///   symbols, and no offset in that range is better than chance. The
    ///   instrument's own self-consistency check therefore cannot see the one
    ///   error that produces 7 in 8, and `docs/287` sec 3 is the consequence.
    #[test]
    fn the_class_comparison_is_exact_at_one_offset_and_chance_at_every_other() {
        let parameters = b1u_parameters();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        let buffer = b1u_window(0);
        let readings = b1u_readings(&parameters, 1_000, &buffer, skip, E2U_SYMBOLS);
        let winner = choose_b1u_boundary(&readings, false).expect("a boundary in the window");
        let back = readings[winner].back;
        assert_eq!(back, crate::v92::UP_INTERVALS, "the twin a whole data frame back");
        let decisions = b1u_window_decisions(&buffer, back);
        assert_eq!(decisions.len() * 4, 200, "fifty trellis frames of decisions");
        // The offset the decoder was really fed at, from the three terms of
        // `first`: skip + 1 + B - back, with B = 0 on this line.
        let exact = skip + 1 - back;
        assert_eq!(B1uBins::measure(&parameters, &decisions, exact, true).total_wrong, 0);
        for delta in -24i64..=24 {
            let first = (exact as i64 + delta) as usize;
            let bins = B1uBins::measure(&parameters, &decisions, first, true);
            assert_eq!(bins.total, 200, "every offset pairs the same population");
            if delta == 0 {
                continue;
            }
            // One whole frame the other way is chance like everything else;
            // only `delta = 0` is exact.
            assert!(
                bins.total_wrong * 4 >= bins.total * 3,
                "an origin {} symbols out must read at chance, not {:.3}",
                delta,
                bins.weighted()
            );
        }
        // And the sweeps `nearest_rival` runs, which is where the 7-in-8 fault
        // lives and which never reaches a whole data frame.
        let mut nearest = u64::MAX;
        for shift in 1..crate::v92::UP_INTERVALS {
            for first in [exact.checked_sub(shift), Some(exact + shift)] {
                let Some(first) = first else { continue };
                nearest = nearest.min(B1uBins::measure(&parameters, &decisions, first, true).total_wrong);
            }
        }
        assert!(
            nearest * 4 >= 200 * 3,
            "no rival inside 1 .. 11 is better than chance ({nearest} of 200 wrong)"
        );
    }

    /// **8.7.1's content pins the frame phase and nothing else**: a whole data
    /// frame of displacement reads as ones as well as the boundary does, so the
    /// far end's E2u overrun `B` -- the third term of `first` -- is not
    /// observable from `B1u`.
    ///
    /// `docs/287` sec 3. Same line, two candidate boundaries: the run's end
    /// itself and the twin twelve symbols further back. Both read as ones
    /// (540 and 569 of 576), and both are accepted by the gate; a candidate one
    /// symbol out reads 285, which is chance. So the known content says
    /// `B = 0 (mod 12)` and cannot say `B = 0`, and any receiver that
    /// regenerates B1u itself can only place frame 0 relative to a phase.
    #[test]
    fn b1u_s_content_pins_the_frame_phase_and_not_the_frame_index() {
        let parameters = b1u_parameters();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        let buffer = b1u_window(0);
        let readings = b1u_readings(&parameters, 1_000, &buffer, skip, E2U_SYMBOLS);
        let at = |back: usize| {
            readings
                .iter()
                .find(|reading| reading.back == back)
                .map(|reading| (reading.best(), reads_as_b1u(reading)))
                .expect("every candidate in the set is read")
        };
        let (run_end, run_end_accepted) = at(0);
        let (twin, twin_accepted) = at(crate::v92::UP_INTERVALS);
        let (phase_out, phase_out_accepted) = at(1);
        assert!(run_end * 4 >= 576 * 3, "the run's end reads as B1u: {run_end} of 576");
        assert!(twin * 4 >= 576 * 3, "and so does a whole data frame back: {twin} of 576");
        assert!(
            phase_out * 4 < 576 * 3,
            "a symbol out of phase does not: {phase_out} of 576"
        );
        assert!(run_end_accepted || twin_accepted, "at least one twin is a boundary");
        assert!(!phase_out_accepted, "a wrong frame phase is never a boundary");
    }

    /// A displaced pairing is refused, not scored: the instrument has to fail
    /// on a wrong origin or it cannot certify a right one.
    ///
    /// `docs/285` sec 2. One symbol either way is a whole trellis position out
    /// and eleven is a data frame less one, so these are the two shapes a wrong
    /// origin takes. Both are in range and both pair every symbol -- which is
    /// why `unpaired` was zero on the call that read 0.875, and why the check
    /// cannot be a range check.
    #[test]
    fn a_displaced_pairing_is_refused_rather_than_scored() {
        let parameters = b1u_parameters();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        let mut encoder = UpEncoder::new(&parameters).expect("drn 1's parameters encode");
        let sent = encoder.b1u();
        let decisions: Vec<(u64, [i32; 4])> = (0..50u64)
            .map(|frame| {
                let mut etas = [0i32; 4];
                for (position, eta) in etas.iter_mut().enumerate() {
                    *eta = sent[skip + 1 + frame as usize * 4 + position].eta;
                }
                (frame, etas)
            })
            .collect();
        let right = B1uBins::measure(&parameters, &decisions, skip + 1, true);
        assert_eq!(right.total_wrong, 0);
        assert!(right.trusted(), "the right origin is the only one that reads zero");
        for displaced in [skip, skip + 2, skip + 1 + crate::v92::UP_INTERVALS - 1] {
            let bins = B1uBins::measure(&parameters, &decisions, displaced, true);
            assert_eq!(bins.unpaired, 0, "a displacement pairs everything: {displaced}");
            assert!(bins.total > 0, "and counts everything");
            assert!(
                !bins.trusted(),
                "a pairing {displaced} symbols out must be refused, not scored {:.4}",
                bins.weighted()
            );
        }
    }

    /// An alias must score zero: a stream that is numerically different and
    /// physically the same is not an error, and one that is physically
    /// different still is.
    ///
    /// `docs/285` sec 6, and it is the test that separates this ticket from
    /// `docs/285` sec 2's. That reproduction fed the pairing **our own
    /// encoder's etas**, so `eta == sent` held by construction and only a
    /// displacement could make it fail -- it was structurally blind to a
    /// quantity mismatch, which is why 0.860 came out of it and why the live
    /// 0.875 looked explained. This one carries the mismatch in: 6.4.2 gives
    /// the precoder an equivalence class and lets it pick the member
    /// (v92.txt:449-459), at `drn 1` every `Mi` is 8 over `LC = 8` points, so
    /// each class at k = 0, 1, 2 has exactly two members in `-8 ..= 7` and they
    /// differ by 8. The alias stream is the *other* member, symbol for symbol;
    /// it is numerically different on three symbols in four and carries the
    /// same `Ki`, hence the same bits. Zero class errors is the assertion, and
    /// the negative control -- the same stream moved one class along -- has to
    /// score wrong on the last symbol, or the first half is only re-asserting
    /// the perfect-input test.
    #[test]
    fn a_class_alias_scores_zero_and_a_wrong_class_does_not() {
        let parameters = b1u_parameters();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        let mut encoder = UpEncoder::new(&parameters).expect("drn 1's parameters encode");
        let sent = encoder.b1u();
        let points = parameters.sets[0].len() as i32;
        assert_eq!(points, 8, "drn 1's LC, so 6.4.2's range is -8 ..= 7");
        let decisions: Vec<(u64, [i32; 4])> = (0..50u64)
            .map(|frame| {
                let mut etas = [0i32; 4];
                for (position, eta) in etas.iter_mut().enumerate() {
                    *eta = sent[skip + 1 + frame as usize * 4 + position].eta;
                }
                (frame, etas)
            })
            .collect();
        let symbol_of = |frame: u64, position: usize| skip + 1 + frame as usize * 4 + position;
        // The other member of each class. At k = 3 the class steps by `2Mi` and
        // the set is one step wide, so it holds a single member and there is no
        // alias to express there -- which is what makes the three-in-four
        // count below a fact about 6.4.2 and not an assumption.
        let alias_of = |symbol: usize, eta: i32| -> i32 {
            let k = symbol % crate::v92::TRELLIS_FRAME;
            let modulus = i32::from(parameters.modulus(symbol % crate::v92::UP_INTERVALS));
            if k == crate::v92::TRELLIS_FRAME - 1 {
                eta
            } else if eta - modulus >= -points {
                eta - modulus
            } else {
                eta + modulus
            }
        };
        let aliased: Vec<(u64, [i32; 4])> = decisions
            .iter()
            .map(|(frame, etas)| {
                let mut out = [0i32; 4];
                for (position, eta) in out.iter_mut().enumerate() {
                    *eta = alias_of(symbol_of(*frame, position), etas[position]);
                }
                (*frame, out)
            })
            .collect();
        let numeric_wrong = aliased
            .iter()
            .flat_map(|(frame, etas)| {
                etas.iter()
                    .enumerate()
                    .map(move |(position, &eta)| (symbol_of(*frame, position), eta))
            })
            .filter(|&(symbol, eta)| eta != sent[symbol].eta)
            .count();
        let physical = B1uBins::measure(&parameters, &decisions, skip + 1, true);
        assert_eq!(physical.total_wrong, 0, "the unaliased stream is perfect");
        assert_eq!(physical.weighted(), 0.0);
        assert!(physical.trusted(), "and its origin is decisively better than any displaced one");
        let alias = B1uBins::measure(&parameters, &aliased, skip + 1, true);
        assert_eq!(alias.unpaired, 0, "every aliased symbol found its partner");
        assert_eq!(alias.total, physical.total, "the same population");
        assert!(
            numeric_wrong * 4 >= alias.total as usize * 3,
            "the alias stream is numerically different on three symbols in four: {numeric_wrong} of {}",
            alias.total
        );
        assert_eq!(
            alias.total_wrong, 0,
            "an alias is not an error: {} of {} numerically different, deltas {:?}",
            numeric_wrong, alias.total, alias.spread(parameters.drn)
        );
        assert_eq!(alias.weighted(), 0.0, "and it reads a rate of zero");
        // The negative control: one class along, on every symbol. At k = 3 the
        // class halves `eta`, so the step that moves a class there is two and
        // one everywhere else.
        let wrong_class: Vec<(u64, [i32; 4])> = decisions
            .iter()
            .map(|(frame, etas)| {
                let mut out = [0i32; 4];
                for (position, eta) in out.iter_mut().enumerate() {
                    let symbol = symbol_of(*frame, position);
                    let step = if symbol % crate::v92::TRELLIS_FRAME == crate::v92::TRELLIS_FRAME - 1 { 2 } else { 1 };
                    let sent_eta = etas[position];
                    *eta = if sent_eta + step <= points - 1 { sent_eta + step } else { sent_eta - step };
                }
                (*frame, out)
            })
            .collect();
        let wrong = B1uBins::measure(&parameters, &wrong_class, skip + 1, true);
        assert_eq!(wrong.unpaired, 0);
        assert_eq!(
            wrong.total_wrong, wrong.total,
            "a different class is a wrong symbol on every symbol: {:?}",
            wrong.spread(parameters.drn)
        );
        assert!(!wrong.trusted(), "and it is refused rather than published");
    }

    /// The reading of the boundary at `back`, and the gate it is held to.
    fn reading(levels: &[f64], end: u64, skip: usize, back: usize) -> B1uReading {
        let parameters = b1u_parameters();
        let readings = b1u_readings(&parameters, end, levels, skip, E2U_SYMBOLS);
        readings.into_iter().find(|reading| reading.back == back).expect("the candidate is in the set")
    }

    /// Which candidate the search would commit to, on the same levels.
    ///
    /// "Reads as B1u" is the rule's input here and is passed in: on a
    /// synthesised line the gate's *lead* between the polarities is not there
    /// (see [`choose_b1u_boundary_by`]), and the threshold is measured on the
    /// captures by `V92_B1U_GATE`. What is under test is the ordering.
    fn chosen_back(levels: &[f64], end: u64, skip: usize) -> usize {
        let parameters = b1u_parameters();
        let readings = b1u_readings(&parameters, end, levels, skip, E2U_SYMBOLS);
        // The gate's ratio, without its polarity lead: see the note above.
        let accept = |reading: &B1uReading| reading.best() * 8 >= reading.scored() * B1U_CONFIRM_EIGHTHS;
        let winner = choose_b1u_boundary_by(&readings, &accept, false).expect("a boundary in the window");
        readings[winner].back
    }

    /// B1u's 576 symbols, read from the run's end, are ones -- and nothing
    /// eleven symbols either side of it is.
    ///
    /// This is the property the search stands on: 8.7.1 makes the content
    /// known, and the twelve-symbol data frame plus the four-symbol trellis
    /// frame make exactly one of the thirteen candidate phases the far end's.
    #[test]
    fn one_candidate_phase_reads_b1u_as_ones() {
        let levels = b1u_line();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        // The run ends on B1u's own last E2u symbol: the fire symbol is B1u's
        // frame interval 0, and the buffer starts one symbol into B1u.
        let buffer: Vec<f64> = levels[1..=B1U_SYMBOLS].to_vec();
        assert_eq!(chosen_back(&buffer, 1_000, skip), 0, "the run ended where B1u begins");
        let at_the_boundary = reading(&buffer, 1_000, skip, 0);
        assert!(
            at_the_boundary.best() * 10 >= at_the_boundary.scored() * 9,
            "{}/{} ones at the boundary",
            at_the_boundary.best(),
            at_the_boundary.scored()
        );
        for back in 1..=E2U_SYMBOLS {
            let off = reading(&buffer, 1_000, skip, back);
            assert!(
                off.best() * 2 < off.scored() * 3,
                "a boundary {back} symbols out still read {}/{} ones",
                off.best(),
                off.scored()
            );
        }
    }

    /// The failing captures' shape: the run overruns E2u by two symbols, so the
    /// naive counter's answer reads chance and the boundary two symbols behind
    /// it reads ones.
    ///
    /// Measured on `/tmp/agr-v90-interop/152653-prime-a/agr-tx.raw`, where the
    /// data mode's constellation starts on 111 576 and the zero run ends on
    /// 111 577: the fire symbol 111 578 read 311 of 576 ones and 111 576 reads
    /// them all. This test has no capture in it -- a synthesised B1u with the
    /// same tail structure -- because the point is the *rule*, not that
    /// recording.
    #[test]
    fn a_run_that_overruns_e2u_is_read_from_the_boundary_behind_it() {
        let levels = b1u_line();
        let skip = B1U_ACQUIRE_FRAMES * crate::v92::UP_INTERVALS - 1;
        // B1u's frame interval 0 is levels[0]; the run ended two symbols later,
        // so the fire symbol is levels[2] and the buffer starts at levels[3].
        let buffer: Vec<f64> = levels[3..=B1U_SYMBOLS + 2].to_vec();
        assert_eq!(chosen_back(&buffer, 1_000, skip), 2, "the boundary is two symbols behind the run's end");
        let true_boundary = reading(&buffer, 1_000, skip, 2);
        let naive = reading(&buffer, 1_000, skip, 0);
        assert!(
            true_boundary.best() * 10 >= true_boundary.scored() * 9,
            "{}/{} ones at the boundary",
            true_boundary.best(),
            true_boundary.scored()
        );
        assert!(
            naive.best() < true_boundary.best(),
            "the run's end ({}/{}) is not worse than the boundary ({}/{})",
            naive.best(),
            naive.scored(),
            true_boundary.best(),
            true_boundary.scored()
        );
        // And the naive answer is not merely second: it fails the gate the
        // receiver holds a candidate to, which is the reason the calls this
        // test is modelled on dropped instead of connecting.
        assert!(
            naive.best() * 8 < naive.scored() * B1U_CONFIRM_EIGHTHS,
            "the run's end passed the confirm gate at {}/{} ones",
            naive.best(),
            naive.scored()
        );
    }
    /// The symbols the committed decoder is fed after the window are the ones
    /// between the end of B1u and the end of the window -- and none for the
    /// candidate at the run's end.
    ///
    /// This is the bug the first bench batch with the search found: a call that
    /// committed at `back = 2`, confirmed B1u at 576 of 576 and then delivered
    /// 9 820 bytes of nothing, because its data decoder was fed from
    /// `origin + 579` instead of `origin + 577`. Two symbols of a twelve-symbol
    /// data frame is every bit of it.
    #[test]
    fn the_data_mode_picks_up_where_b1u_ends() {
        let parameters = b1u_parameters();
        let line = b1u_line();
        let mut search = B1uSearch {
            end: 1_000,
            levels: Vec::new(),
            symbols: Vec::new(),
        };
        for index in 0..B1U_SYMBOLS {
            let symbol = Symbol {
                index: index as u64,
                raw: index as u64,
                value: line[index + 1],
                decided: None,
                line: line[index + 1],
                forward: 0.0,
            };
            search.symbols.push(symbol);
            search.levels.push(symbol.value);
        }
        assert!(b1u_backlog_symbols(&search, 0).is_empty(), "the run's end leaves no data behind it");
        let carried = b1u_backlog_symbols(&search, 2);
        assert_eq!(carried.len(), 2, "a two-symbol overrun carries two symbols of data");
        // Symbol `end + 1 + i` is entry `i`, and B1u's last symbol is entry
        // `B1U_SYMBOLS - 1 - back` for a candidate `back` behind the run's end.
        assert_eq!(carried[0].index, (B1U_SYMBOLS - 2) as u64);
        assert_eq!(carried[1].index, (B1U_SYMBOLS - 1) as u64);
    }
}

/// docs/158: the Phase 3 downlink against the five clauses the standard actually
/// puts on it, plus the one codified digital-impairment acknowledgement.
///
/// Every rule here is a property of the codeword stream this end puts on the
/// line, so every one is proved by driving [`Source`] -- the real downlink, not
/// a model of it -- and asserting the clause's own words against what came out.
///
/// * R1 — 8.4.5/V.90 (`v90.txt:1520-1525`), cited by 8.6.8/V.92: TRN1d is the
///   codeword whose Ucode is UINFO, signs from binary ones through the 5.3
///   scrambler zero-initialised, length a multiple of 6.
/// * R2 — Table 18/V.92 bits 25:31 (`v92.txt:1202-1204`): UINFO is the single
///   magnitude of that two-point train, and it "shall be greater than 66".
/// * R3 — 8.4.4/V.90 (`v90.txt:1509-1516`), cited by 8.6.7/V.92: Sd and S-bar-d
///   are the printed six-symbol patterns, six-periodic, first symbol in data
///   frame interval 0.
/// * R4 — 9.5.1.1.11/V.92 (`v92.txt:2554-2556`) with 8.4.1/V.90
///   (`v90.txt:1424-1465`): the DIL sent is the one the descriptor asked for --
///   N segments of `(Hc + 1) * 6`, REFc against the training Ucode by TP, SP's
///   signs, patterns restarted per segment, the whole sequence repeated.
/// * R5 — Table 21 and Table 22/V.92 (`v92.txt:1364-1428`): Jd and Jp at the bit
///   level, and their differential encoding, which 8.6.2 and 8.6.3 initialize
///   with the final symbol of TRN1d/Jd.
/// * R6 — `CPt` bit 128 (`v92.txt:1270-1275`, Table 23 `v92.txt:1544-1545`;
///   `v90.txt:1554-1556`) and INFO0d bits 38/39 (`v90.txt:1052-1054`).
#[cfg(test)]
mod conformance_158 {
    use super::*;
    use crate::v34::info::Info0d;

    fn settings_with(uinfo: u8) -> Settings {
        Settings {
            law: Law::Mu,
            uinfo,
            far_md: 0,
            round_trip: 0.0,
            wide: true,
            limits: FilterLimits::from_info1a(0, 0, 0),
            habits: Habits::PROMPT,
            probing: None,
        }
    }

    fn settings() -> Settings {
        settings_with(79)
    }

    /// Every level produced by `count` calls, labelled with the signal it
    /// belongs to.
    ///
    /// The label is `out()` *after* the call, and that is the correct one
    /// throughout: every arm that changes `out` changes it before producing the
    /// new signal's first level -- Sd rolls into S-bar-d and TRN1d into Jd
    /// through `continue`, and a pending change is taken and then fallen
    /// through -- so no level is ever produced for the signal that has just
    /// ended.
    fn labelled(source: &mut Source, count: usize) -> Vec<(Out, f64)> {
        (0..count)
            .map(|_| {
                let level = source.next();
                (source.out(), level)
            })
            .collect()
    }

    /// How many *sequences* of `want` went out: the stream is one codeword a
    /// symbol, and a CPd is four hundred and eighty of them.
    fn runs(stream: &[(Out, f64)], want: Out) -> usize {
        let mut count = 0;
        let mut in_run = false;
        for (out, _) in stream {
            if *out == want && !in_run {
                count += 1;
                in_run = true;
            } else if *out != want {
                in_run = false;
            }
        }
        count
    }

    fn only(stream: &[(Out, f64)], want: Out) -> Vec<f64> {
        stream.iter().filter(|(out, _)| *out == want).map(|(_, level)| *level).collect()
    }

    /// `1 + x^-18 + x^-23` (GPC, equation 7-1/V.34, which 5.3/V.90 names) driven
    /// by `input` from an all-zero register -- written out here rather than
    /// taken from [`Scrambler`], so that the rule is checked against the
    /// polynomial and not against the thing under test.
    fn gpc(input: &[bool]) -> Vec<bool> {
        fn tap(d: &[bool], k: i64) -> bool {
            if k < 0 { false } else { d[k as usize] }
        }
        let mut d = vec![false; input.len()];
        for n in 0..input.len() {
            let k = n as i64;
            d[n] = input[n] ^ tap(&d, k - 18) ^ tap(&d, k - 23);
        }
        d
    }

    /// R1 and R5's start: Sd, S-bar-d, TRN1d and the first two Jd repetitions,
    /// as one stream.
    ///
    /// `Habits::PROMPT` runs TRN1d for 0.3 s, 2400 symbols, so this is 432
    /// symbols of Sd plus 2400 of TRN1d plus 144 of Jd.
    fn opening(uinfo: u8) -> Vec<(Out, f64)> {
        let mut source = Source::new(&settings_with(uinfo));
        source.change(Out::Sd);
        labelled(&mut source, SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS + 2400 + 144)
    }

    /// R1 -- "TRN1d shall be an integer multiple of 6 symbols long" (8.4.5/V.90),
    /// and "the scrambler is initialized to zero prior to the transmission of
    /// TRN1d", its signs being "generated by applying binary ones to the input
    /// of the scrambler described in 5.3".
    #[test]
    fn r1_trn1d_is_whole_six_frames_of_zero_initialised_scrambled_ones() {
        let stream = opening(79);
        let trn = only(&stream, Out::Trn1d);
        assert!(!trn.is_empty(), "TRN1d never reached the line");
        assert_eq!(trn.len() % 6, 0, "8.4.5: TRN1d shall be an integer multiple of 6 symbols long, not {}", trn.len());
        assert!(trn.len() >= 2040, "9.5.1.1.4: TRN1d is at least 2040T, not {}", trn.len());

        // "a sequence of the PCM codeword whose Ucode is UINFO": one magnitude.
        let magnitude = ucode::level(Law::Mu, 79).abs();
        for (n, level) in trn.iter().enumerate() {
            assert!(
                (level.abs() - magnitude).abs() < 1e-12,
                "TRN1d symbol {n} is {} and not the codeword whose Ucode is UINFO ({magnitude})",
                level.abs()
            );
        }

        // The signs are binary ones through a zero-initialised GPC scrambler,
        // and TRN1d is not differentially encoded (8.4.5 has no such step;
        // 8.4.2's starts at Jd).
        let expected: Vec<bool> = gpc(&vec![true; trn.len()]);
        let signs: Vec<bool> = trn.iter().map(|level| *level > 0.0).collect();
        assert_eq!(signs, expected, "TRN1d's signs are not binary ones through a zeroed 5.3 scrambler");

        // The multiple of six is not an accident of a whole-frame habit: a
        // duration that is not a whole number of symbols is rounded up to the
        // next frame boundary, because 6 is the standard's unit and not a
        // convenience.
        let mut settings = settings();
        settings.habits.trn1d = 2399.0 / FS;
        let mut source = Source::new(&settings);
        source.change(Out::Sd);
        let stream = labelled(&mut source, SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS + 2401);
        assert_eq!(only(&stream, Out::Trn1d).len(), 2400, "TRN1d was not cut on a frame boundary");
    }

    /// R2 -- Table 18/V.92 bits 25:31: "UINFO: Ucode of the PCM codeword to be
    /// used by the digital modem for the 2-point train ... UINFO shall be
    /// greater than 66" (`v92.txt:1202-1204`).
    ///
    /// The train is the one magnitude the clause names, and this end's choice of
    /// UINFO -- the only side of the pair that *chooses* it -- is held above 66.
    #[test]
    fn r2_the_two_point_train_is_one_magnitude_and_uinfo_is_above_66() {
        for uinfo in [67u8, 79, 111] {
            let stream = opening(uinfo);
            let trn = only(&stream, Out::Trn1d);
            let mut magnitudes: Vec<u64> = trn.iter().map(|level| level.abs().to_bits()).collect();
            magnitudes.sort_unstable();
            magnitudes.dedup();
            assert_eq!(magnitudes.len(), 1, "UINFO {uinfo}: TRN1d used more than one magnitude");
            assert_eq!(
                f64::from_bits(magnitudes[0]),
                ucode::level(Law::Mu, uinfo).abs(),
                "UINFO {uinfo}: the train's magnitude is not that codeword"
            );
        }
        // "shall be greater than 66": what this end is allowed to name.
        assert!(crate::v34::info::UINFO_LOWEST > 66, "the lowest UINFO this end sends must exceed 66");
        let asked = crate::v34::info::Info1aPcmUp { uinfo: crate::v34::info::UINFO_LOWEST, ..Default::default() };
        assert!(asked.uinfo_is_sendable(), "and the bound itself is sendable");
        let low = crate::v34::info::Info1aPcmUp { uinfo: crate::v34::info::UINFO_LOWEST - 1, ..Default::default() };
        assert!(!low.uinfo_is_sendable(), "66 is not");
    }

    /// R3 -- 8.4.4/V.90: "Sd consists of 64 repetitions of the sequence
    /// {+W, +0, +W, -W, -0, -W} where W is defined to be the PCM codeword whose
    /// Ucode is 16 + UINFO and 0 is the PCM codeword with Ucode 0", S-bar-d is
    /// eight repetitions of the turned-over sequence, and "the first symbol of
    /// Sd is defined to be transmitted in data frame interval 0".
    #[test]
    fn r3_sd_and_s_bar_d_are_the_printed_six_from_frame_zero() {
        let w = ucode::level(Law::Mu, 16 + 79);
        let zero = ucode::level(Law::Mu, 0);
        let pattern = [w, zero, w, -w, -zero, -w];

        let mut source = Source::new(&settings());
        source.change(Out::Sd);
        let stream = labelled(&mut source, SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS);
        let sd = only(&stream, Out::Sd);
        let sd_bar = only(&stream, Out::SdBar);

        assert_eq!(sd.len(), 384, "8.4.4: 64 repetitions of six");
        assert_eq!(sd_bar.len(), 48, "8.4.4: 8 repetitions of six");
        // Frame interval 0 is Sd's first symbol: the counter the frame
        // boundaries are measured on started at zero with it, and 384 + 48 is a
        // whole number of frames, so TRN1d begins on a boundary too.
        assert_eq!(source.symbol, 432, "Sd did not start in data frame interval 0");

        for (n, level) in sd.iter().enumerate() {
            assert_eq!(*level, pattern[n % 6], "Sd symbol {n}: not the printed pattern, six-periodic");
        }
        for n in 0..48 {
            assert_eq!(sd_bar[n], -sd[n], "S-bar-d symbol {n}: not Sd turned over");
        }
    }

    /// R3's second half, 8.4.1/V.90: "Lc = (Hc + 1) * 6 symbols", so every
    /// DIL-segment boundary is a data frame boundary.
    #[test]
    fn r3_every_dil_segment_is_a_whole_number_of_frames() {
        for h in 0..=127u8 {
            let descriptor = Descriptor { h: [h; 8], ..Descriptor::none() };
            assert_eq!(descriptor.segment_length(0x00) % 6, 0, "Hc = {h}");
            assert_eq!(descriptor.segment_length(0x70) % 6, 0, "Hc = {h}");
        }
        let descriptor = asked_dil();
        let mut end = 0;
        for &code in &descriptor.ucodes {
            end += descriptor.segment_length(code);
            assert_eq!(end % 6, 0, "a segment boundary off the frame grid");
        }
    }

    /// A DIL descriptor of the widest shape 8.4.1 allows: one segment per Uchord,
    /// eight different Hc so eight different lengths, eight different REFc, and
    /// patterns whose lengths divide none of them evenly -- so a restart per
    /// segment and a per-segment sign/training phase are both visible.
    fn asked_dil() -> Descriptor {
        Descriptor {
            signs: vec![true, false, true, false, true],
            training: vec![true, false, false, true],
            h: [0, 1, 2, 3, 4, 5, 6, 7],
            refs: [0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78],
            ucodes: vec![0x01, 0x11, 0x21, 0x31, 0x41, 0x51, 0x61, 0x71],
        }
    }

    /// R4 -- 9.5.1.1.11/V.92: "After sending Jp', the digital modem shall send
    /// the DIL requested by the analogue modem", as 8.4.1/V.90 defines it.
    ///
    /// The check is that what `heard_ja` hands the source is exactly what the
    /// descriptor says, and that the source then puts it out symbol for symbol,
    /// whole sequence repeated, cut only on segment boundaries.
    #[test]
    fn r4_the_dil_sent_is_the_one_the_descriptor_asked_for() {
        let descriptor = asked_dil();
        let mut modem = Modem::new(settings());
        modem.heard_ja(descriptor.clone());

        // The descriptor's own reading, and the source's, are the same listing.
        let wanted: Vec<(u8, bool)> = descriptor.symbols().collect();
        assert_eq!(modem.source.dil, wanted, "the DIL the source holds is not the descriptor's");
        assert_eq!(wanted.len(), descriptor.len(), "one pass");
        assert_eq!(wanted.len(), (1 + 2 + 3 + 4 + 5 + 6 + 7 + 8) * 6, "8.4.1: N segments of (Hc + 1) * 6");

        let mut end = 0;
        let mut ends = Vec::new();
        for &code in &descriptor.ucodes {
            end += descriptor.segment_length(code);
            ends.push(end);
        }
        assert_eq!(modem.source.dil_ends, ends, "8.4.1: terminated on DIL-segment boundaries");

        // REFc against the training Ucode, by TP; SP's signs; both restarted at
        // each segment's first symbol according to LSB-first patterns.
        let mut at = 0;
        for (segment, &code) in descriptor.ucodes.iter().enumerate() {
            let chord = usize::from(code >> 4) & 7;
            let length = descriptor.segment_length(code) / 6;
            assert_eq!(length, segment + 1, "segment {segment} is not Hc + 1 frames");
            for n in 0..descriptor.segment_length(code) {
                let (ucode_out, positive) = modem.source.dil[at + n];
                let want = if descriptor.training[n % descriptor.training.len()] {
                    code
                } else {
                    descriptor.refs[chord]
                };
                assert_eq!(ucode_out, want, "segment {segment} symbol {n}: TP did not restart");
                assert_eq!(
                    positive,
                    descriptor.signs[n % descriptor.signs.len()],
                    "segment {segment} symbol {n}: SP did not restart"
                );
            }
            at += descriptor.segment_length(code);
        }

        // "The entire sequence, not just the last DIL-segment, is repeated."
        let mut source = Source::new(&settings());
        source.dil = wanted.clone();
        source.dil_ends = ends;
        source.change(Out::Dil);
        let length = wanted.len();
        let stream = labelled(&mut source, 3 * length);
        assert!(stream.iter().all(|(out, _)| *out == Out::Dil), "the DIL left the line");
        let levels: Vec<f64> = stream.iter().map(|(_, level)| *level).collect();
        assert_eq!(levels[..length], levels[length..2 * length], "the DIL did not repeat");
        assert_eq!(levels[length..2 * length], levels[2 * length..], "the DIL did not repeat");
        assert!(levels.iter().any(|level| level.abs() > 0.0), "the DIL was silent");
    }

    /// R5 -- Table 21/V.92 (`v92.txt:1364-1389`) and Table 22/V.92
    /// (`v92.txt:1406-1428`): the field layout of Jd and Jp, and the
    /// differential encoder 8.6.2 and 8.6.3 initialize "with the final symbol of
    /// the transmitted TRN1d" and "the transmitted Jd".
    #[test]
    fn r5_jd_and_jp_bits_and_the_differential_chain_from_trn1d() {
        let settings = settings();
        let jd = settings.jd_bits();
        assert_eq!(jd.len(), 72, "Jd is 72 bits, bit 0 first");
        assert!(jd[..17].iter().all(|&bit| bit), "Table 21 bits 0:16, frame sync");
        assert!(!jd[17], "bit 17, start bit");
        assert!(!jd[47], "bit 47: 0 = Jd");
        assert!(jd[68..].iter().all(|&bit| !bit), "bits 68:71, fill 0000");
        // Bits 41:46 are ITU-reserved zeros, and the rate mask is not all zeros.
        assert!(jd[41..47].iter().all(|&bit| !bit), "bits 41:46 reserved");
        assert!(jd[18..41].iter().any(|&bit| bit), "bits 18:40, the rate mask");

        let jp = Jp::default().to_bits();
        assert_eq!(jp.len(), 72);
        assert!(jp[..17].iter().all(|&bit| bit), "Table 22 bits 0:16, frame sync");
        assert!(!jp[17], "bit 17, start bit");
        assert!(jp[47], "bit 47: 1 = Jp");
        assert!(jp[35..47].iter().all(|&bit| !bit), "bits 35:46 reserved");
        assert!(!jp[50], "bit 50 reserved");
        assert!(jp[68..].iter().all(|&bit| !bit), "bits 68:71, fill 0000");
        assert_eq!(J::Jp(Jp::default()).to_bits(), jp, "bit 47 is what tells them apart");

        // The line: TRN1d's raw scrambler output, then 8.6.2's differential
        // encoder riding the same scrambler, initialized with TRN1d's final
        // symbol -- one continuous chain, not a reset at each signal.
        let stream = opening(79);
        let trn = only(&stream, Out::Trn1d);
        let jd_out = only(&stream, Out::Jd);
        assert_eq!(jd_out.len(), 144, "two whole Jd repetitions");
        let mut input = vec![true; trn.len()];
        input.extend_from_slice(&jd);
        input.extend_from_slice(&jd);
        let scrambled = gpc(&input);
        let mut expected: Vec<bool> = scrambled[..trn.len()].to_vec();
        let mut sign = *expected.last().expect("TRN1d is not empty");
        for bit in &scrambled[trn.len()..] {
            sign ^= *bit;
            expected.push(sign);
        }
        let signs: Vec<bool> = trn.iter().chain(jd_out.iter()).map(|level| *level > 0.0).collect();
        assert_eq!(signs, expected, "the TRN1d/Jd sign chain is not 8.4.5 + 8.6.2");
    }

    /// R6 -- `CPt` bit 128: "If the constellations at the digital modem's
    /// transmitter differ from those at the output to the codec's D/A converter,
    /// then bit 128 shall be set" (`v92.txt:1270-1275`). And INFO0d bits 38/39
    /// (`v90.txt:1052-1054`), the power measurement point and the companding law.
    #[test]
    fn r6_bit_128_is_clear_because_nothing_stands_between_us_and_the_codec() {
        // The digital modem's transmitter is the codec's D/A converter: R1/R2
        // above show TRN1d going out at exactly `ucode::level`, unshaped (8.6
        // /V.92: "signals transmitted by the digital modem during Phase 3 are
        // not spectrally shaped"), and no arm of `Source` scales a Phase 3
        // codeword. So the two constellations are the same one, and the only
        // legal setting of bit 128 is clear.
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        assert!(cp.codec.is_none(), "our CPt claims the codec's constellations differ from ours");
        let bits = cp.to_bits_in(Layout::V92, 1);
        assert!(!bits[128], "CPt bit 128 is set, and we have no second constellation to send with it");
        assert_eq!(cp.trn1d_ratio, 1 << 13, "bits 52:67: TRN1d at the transmitter over TRN1d at the codec, 1.0");

        // INFO0d bit 38: the power is measured at the codec's output (the
        // levels this end computes are the codewords the codec plays), and bit
        // 39 is 0 for the mu-law this end transmits.
        let ours = crate::v90::server::ours();
        assert!(ours.power_at_codec, "INFO0d bit 38: we transmit codewords, so the codec's output is where power is");
        assert!(!ours.a_law, "INFO0d bit 39: mu-law");
        let bits = ours.to_bits();
        let heard = Info0d::from_bits(&bits).expect("an INFO0d reads back");
        assert!(heard.power_at_codec && !heard.a_law);
        // Bit level: flipping bit 38 moves exactly one bit of the sequence, and
        // it is the one `from_bits` reads as bit 38 (`info[26]`, the twenty-
        // sixth bit of the information after the seventeen-bit sync and the
        // start bit of each block).
        let mut clear = crate::v90::server::ours();
        clear.power_at_codec = false;
        let flipped: Vec<usize> = (0..bits.len()).filter(|&at| bits[at] != clear.to_bits()[at]).collect();
        assert_eq!(flipped[0], 38, "Table 15 bit 38 is not where bits 38 sits");
        // The rest of the difference is that bit's CRC: Table 15's sequence is
        // four fill bits, the eight-bit sync, thirty bits of information and a
        // sixteen-bit CRC, so the CRC occupies 42 to 57.
        assert!(flipped[1..].iter().all(|at| (42..58).contains(at)), "{flipped:?}");
    }
    /// 9.5.1.1.3: "the digital modem **may wait for up to 500 ms** and shall
    /// then transmit signal Sd for 384T and signal S-d for 48T"
    /// (`v92.txt:2535-2536`). The wait is a setting, clamped into the clause's
    /// window and snapped to the frame grid; what follows it is untouched.
    #[test]
    fn sd_after_ja_is_clamped_into_the_clause_and_snapped_to_the_frame_grid() {
        // The window, from the clause's own two numbers: at most 500 ms, and
        // never negative.
        for asked in [-1.0, 0.0, 3.0, 6.0, 48.0, 54.0, 250.0, 499.0, 500.0, 501.0, 1000.0] {
            let habits = Habits { sd_after_ja: asked / 1000.0, ..Habits::LIVE };
            let frames = habits.sd_after_ja_frames();
            let ms = habits.sd_after_ja_symbols() as f64 * 1000.0 / FS;
            assert_eq!(
                habits.sd_after_ja_symbols(),
                frames * INTERVALS,
                "{asked} ms: the wait is not a whole number of frames"
            );
            assert!(
                (0.0..=SD_AFTER_JA_WITHIN * 1000.0).contains(&ms),
                "{asked} ms: the wait {ms} ms is outside 9.5.1.1.3's window"
            );
            assert!(
                frames <= Habits::sd_after_ja_frames_most(),
                "{asked} ms: more frames than the window holds"
            );
        }
        // The window's two ends, and the largest whole frame inside it.
        assert_eq!(Habits::sd_after_ja_frames_most(), 666);
        assert!((Habits::sd_after_ja_frames_most() as f64 * SD_FRAME_MS) <= 500.0);
        assert!((Habits::sd_after_ja_frames_most() as f64 + 1.0) * SD_FRAME_MS > 500.0);
        assert_eq!(Habits { sd_after_ja: -1.0, ..Habits::LIVE }.sd_after_ja_frames(), 0);
        assert_eq!(Habits { sd_after_ja: 0.0, ..Habits::LIVE }.sd_after_ja_frames(), 0);
        assert_eq!(Habits { sd_after_ja: 0.054, ..Habits::LIVE }.sd_after_ja_frames(), 72);
        assert_eq!(Habits { sd_after_ja: 1000.0, ..Habits::LIVE }.sd_after_ja_frames(), 666);
        // A request that is not on the grid is **snapped**, not accepted as
        // it stands and not rounded silently into something off-grid.
        for (asked, frames) in [(250.0, 333), (499.0, 665), (500.0, 666), (1.0, 1)] {
            let habits = Habits { sd_after_ja: asked / 1000.0, ..Habits::LIVE };
            assert_eq!(habits.sd_after_ja_frames(), frames, "{asked} ms");
            let ms = habits.sd_after_ja_symbols() as f64 / FS * 1000.0;
            assert_ne!(ms, asked, "{asked} ms is off the grid and was not snapped");
            assert_eq!(ms, frames as f64 * SD_FRAME_MS);
        }
        // And the default reproduces the behaviour every call had before this
        // existed: no wait at all.
        assert_eq!(Habits::LIVE.sd_after_ja_frames(), 0);
        assert_eq!(Habits::PROMPT.sd_after_ja_frames(), 0);
        assert_eq!(Habits::default().sd_after_ja_symbols(), 0);
    }

    /// The wait goes on the line **before** Sd and nowhere else: the source is
    /// silent for exactly the waiting frames, Sd is exactly 384T, S-bar-d
    /// exactly 48T, and TRN1d starts on the codeword immediately after it.
    ///
    /// And it is a *shift*: the same Ja read with a wait of W frames puts Sd's
    /// first symbol exactly `W * INTERVALS` later than with no wait, so what the
    /// wait moves is the placement and not the frame grid (the residue mod
    /// `INTERVALS` is identical, which is the whole point -- see `Source::next`).
    #[test]
    fn the_wait_delays_sd_by_whole_frames_and_leaves_every_length_alone() {
        let mut settings = settings();
        settings.habits = Habits { trn1d: 0.3, sd_after_ja: 0.054, ..Habits::LIVE };
        let wait = settings.habits.sd_after_ja_symbols();
        assert_eq!(wait, 432, "54 ms is 72 frames of six");

        // Ja read at symbol 2, i.e. four codewords into a frame: the residue a
        // real Ja ends on is not zero, and the wait must not move it.
        let mut with_wait = Modem::new(settings.clone());
        for _ in 0..2 {
            with_wait.source.next();
        }
        with_wait.heard_ja(asked_dil());
        let total = 2 * INTERVALS + wait + SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS + 6;
        let stream = labelled(&mut with_wait.source, total);
        let first_sd = stream
            .iter()
            .position(|(out, _)| *out == Out::Sd)
            .expect("Sd never reached the line");
        // The stream starts two codewords into a frame, so the frame boundary
        // is an absolute property: index + 2.
        let sd_at = first_sd + 2;
        assert_eq!(sd_at % INTERVALS, 0, "Sd did not start on a frame boundary");
        assert_eq!(
            &stream[..first_sd],
            &vec![(Out::Silence, 0.0); first_sd][..],
            "something other than silence went out before Sd"
        );
        assert_eq!(only(&stream, Out::Sd).len(), 384, "9.5.1.1.3: Sd is 384T");
        assert_eq!(only(&stream, Out::SdBar).len(), 48, "9.5.1.1.3: S-bar-d is 48T");
        assert_eq!(
            stream[first_sd + 432].0,
            Out::Trn1d,
            "TRN1d does not start on the codeword after S-bar-d"
        );

        // The control: the same Ja, no wait.
        let mut bare = settings.clone();
        bare.habits.sd_after_ja = 0.0;
        let mut no_wait = Modem::new(bare);
        for _ in 0..2 {
            no_wait.source.next();
        }
        no_wait.heard_ja(asked_dil());
        let stream = labelled(&mut no_wait.source, total);
        let first_sd_now = stream
            .iter()
            .position(|(out, _)| *out == Out::Sd)
            .expect("Sd never reached the line");
        assert_eq!(
            first_sd - first_sd_now,
            wait,
            "the wait did not move Sd by exactly its own whole frames"
        );
        assert_eq!(
            sd_at % INTERVALS,
            (first_sd_now + 2) % INTERVALS,
            "the frame residue moved"
        );
    }

    /// The 5.1 s deadline is what carries the whole of Phase 3's preamble from
    /// the sample Ja was read on, so 9.5.1.1.3's wait is one of its terms.
    #[test]
    fn the_su_deadline_carries_the_sd_wait_too() {
        let mut settings = settings();
        settings.habits = Habits { trn1d: 3.6, sd_after_ja: 0.5, ..Habits::LIVE };
        let habits = settings.habits;
        let wait = habits.sd_after_ja_symbols() as u64;
        assert!(wait > 0);
        let mut modem = Modem::new(settings);
        let ja_read = 8000u64;
        modem.now = ja_read;
        modem.heard_ja(asked_dil());
        let deadline = modem.su_deadline.expect("a deadline from Ja");
        assert_eq!(
            deadline,
            ja_read
                + wait
                + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64
                + (habits.trn1d * FS) as u64
                + (SU_WITHIN * FS) as u64,
            "the deadline does not carry 9.5.1.1.3's wait"
        );
        // Which leaves the 5.1 s where 9.5.1.2.2 puts it -- after TRN1d's own
        // length from TRN1d's start, exactly as it was before the wait existed,
        // so the wait moved the deadline by its own length and nothing else.
        let trn1d_start = ja_read
            + wait
            + (SD_FRAMES * INTERVALS + SD_BAR_FRAMES * INTERVALS) as u64;
        assert_eq!(
            deadline - trn1d_start,
            (habits.trn1d * FS) as u64 + (SU_WITHIN * FS) as u64
        );
    }

    // -----------------------------------------------------------------
    // 9.8 rate renegotiation: the responder (docs/214)
    // -----------------------------------------------------------------

    /// A data mode level stream: legal codewords of a V.92 downstream
    /// constellation, whose signs and magnitudes both wander. A six-symbol
    /// stretch of signs will match something by chance; a six-symbol stretch of
    /// signs *and* constant magnitude will not, which is what 8.5.5's "same
    /// structure used while transmitting 2 point TRN1u signal" is.
    fn data_levels(count: usize) -> Vec<f64> {
        let mut x: u64 = 0x9e37_79b9_7f4a_7c15;
        (0..count)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                // 1..=31, the spread of a mu-law data set, either sign.
                let magnitude = 1.0 + (x % 31) as f64;
                if x & 0x100 == 0 { magnitude } else { -magnitude }
            })
            .collect()
    }

    /// The six-symbol period of `Ru`, at a fixed magnitude.
    fn ru_periods(periods: usize, magnitude: f64, bar: bool) -> Vec<f64> {
        (0..periods)
            .flat_map(|_| {
                RU_PERIOD.iter().map(move |&positive| {
                    let positive = positive ^ bar;
                    if positive { magnitude } else { -magnitude }
                })
            })
            .collect()
    }

    // ---------------------------------------------------------------------
    // docs/216: the far end has left data mode -- the trigger and the re-entry
    // ---------------------------------------------------------------------

    /// A level table of the shape `begin_b1u` builds: `a(eta) = 2 eta + 1` over
    /// `points` positive points, at the gain 8.8.3 gives a sixteen-point set.
    fn data_table(points: usize) -> Vec<f64> {
        let gain = crate::v92::upchoice::gain_for(points);
        let mut levels: Vec<f64> = (0..points)
            .flat_map(|eta| {
                let level = gain * f64::from(2 * eta as u32 + 1);
                [level, -level]
            })
            .collect();
        levels.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        levels
    }

    /// The far end's generator pattern as `agr-tx.raw` carries it: a uniform
    /// ladder of 40 magnitudes over the whole scale, all of them above the
    /// constellation's loudest point by the table's own measure.
    fn generator_levels(count: usize) -> Vec<f64> {
        let mut x: u64 = 0x2545_f491_4f6c_dd1d;
        (0..count)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                // 360 ..= 22303, the magnitudes the far end's generator puts on
                // the line on the measured call.
                let magnitude = 360.0 + (x % 40) as f64 * (22303.0 - 360.0) / 39.0;
                if x & 0x100 == 0 { magnitude } else { -magnitude }
            })
            .collect()
    }

    /// Requirement 1 (`docs/216` §2): **the data mode of a call that does not
    /// fall back is unchanged.** The detector runs on every data mode symbol,
    /// and over a stream that is data and not a foreign pattern it must never
    /// fire -- not once, at any of a thousand symbols -- because a false fire
    /// puts `Ri` on the line in the middle of the far end's data and loses the
    /// call.
    #[test]
    fn no_data_loss_is_signalled_by_a_healthy_data_mode_run() {
        let table = data_table(8);
        let step = table[1] - table[0];
        assert!(step > 0.0);
        let mut detector = DataModeLoss::new(&table).expect("a table with a step");
        assert!(detector.threshold() > *table.last().unwrap(), "no margin over the loudest point");
        // Data mode: every symbol a point of the constellation, plus noise that
        // never carries it past the decision boundary of the point either side.
        let mut x: u64 = 0x9e37_79b9_7f4a_7c15;
        let mut fired = 0;
        for _ in 0..4000 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let point = table[(x as usize) % table.len()];
            let noise = (((x >> 20) & 0xFFFF) as f64 / 65_535.0 - 0.5) * step;
            if detector.feed(point + noise) {
                fired += 1;
            }
        }
        assert_eq!(fired, 0, "a healthy data mode fired the re-entry");
        assert!(!detector.fired());
    }

    /// `docs/216` §2: the trigger fires on a **sustained** upstream failure and
    /// not on one bad frame -- `[our choice]`'s whole content.
    #[test]
    fn the_data_loss_trigger_needs_a_sustained_failure_and_not_one_bad_frame() {
        let table = data_table(8);
        let step = table[1] - table[0];
        let loudest = *table.last().unwrap();
        let inside = { let _ = step; *table.last().unwrap() * 0.5 };
        assert!(inside < loudest);

        // One data frame of a foreign pattern, then data again: no fire.
        let mut detector = DataModeLoss::new(&table).expect("a table");
        let mut fired = false;
        for _ in 0..UP_INTERVALS {
            fired |= detector.feed(loudest * 2.0);
        }
        for _ in 0..(DATA_MODE_LOSS_FRAMES * UP_INTERVALS) {
            fired |= detector.feed(inside);
        }
        assert!(!fired, "one bad frame fired the re-entry");

        // Sustained: fire, and exactly once.
        let mut detector = DataModeLoss::new(&table).expect("a table");
        let mut fires = 0;
        let mut at = None;
        for (n, level) in generator_levels(4000).into_iter().enumerate() {
            if detector.feed(level) {
                fires += 1;
                at = Some(n);
            }
        }
        assert_eq!(fires, 1, "the trigger fired {} times", fires);
        let at = at.expect("the trigger never fired");
        assert!(
            at < DATA_MODE_LOSS_FRAMES * UP_INTERVALS + 40,
            "the trigger took {at} symbols, more than {} data frames",
            DATA_MODE_LOSS_FRAMES
        );
    }

    /// 9.5.1.1.12 with 9.5.2.1.10: `Ri` for 384T and `R-bar-i` for 24T, on the
    /// data frame grid, and then 9.6.1.1.1's `TRN2d`.
    ///
    /// The bar is V.90 8.6.4's "with the sign pattern - - - + + +"
    /// (`v90.txt:1835`) read off the source line, and not the `- - - + + -` an
    /// earlier draft had out of a PDF hyphenation artefact.
    #[test]
    fn ri_is_384t_and_r_bar_i_24t_on_a_data_frame_grid() {
        let mut settings = settings();
        settings.law = Law::Mu;
        let mut source = Source::new(&settings);
        source.change(Out::B1d);
        let mut emitted: Vec<f64> = Vec::new();
        // Seven symbols of data mode first, so that the opener has to wait for
        // the grid rather than starting where it was asked for.
        for _ in 0..7 {
            emitted.push(source.next());
        }
        assert_eq!(source.out(), Out::B1d);
        source.ri_opener();
        while source.out() != Out::Trn2d {
            emitted.push(source.next());
            assert!(emitted.len() < 1000, "Ri never ended");
        }
        let start = 12;
        // The last symbol pushed is TRN2d's first: the loop pushes until the
        // signal on the line has changed.
        assert_eq!(emitted.len(), start + R_FRAMES * INTERVALS + R_BAR_FRAMES * INTERVALS + 1);
        let ri = &emitted[start..start + R_FRAMES * INTERVALS];
        assert_eq!(ri.len(), 384);
        assert!(ri.iter().all(|v| (*v - ri[0]).abs() < 1e-9 || (*v + ri[0]).abs() < 1e-9));
        for (i, level) in ri.iter().enumerate() {
            assert_eq!(*level > 0.0, i % INTERVALS < 3, "Ri symbol {i} is not + + + - - -");
        }
        let bar = &emitted[start + R_FRAMES * INTERVALS..start + R_FRAMES * INTERVALS + R_BAR_FRAMES * INTERVALS];
        assert_eq!(bar.len(), 24, "R-bar-i is not 24T");
        assert!(bar.iter().all(|v| (*v - ri[0]).abs() < 1e-9 || (*v + ri[0]).abs() < 1e-9));
        for (i, level) in bar.iter().enumerate() {
            assert_eq!(*level > 0.0, i % INTERVALS >= 3, "R-bar-i symbol {i} is not - - - + + +");
        }
        assert_eq!(source.out(), Out::Trn2d);
    }

    /// 9.6.1.1.1: "The digital modem shall transmit TRN2d for at least 2040T",
    /// with 9.8.1.1.2's "up to 16008T" as the ceiling on this path.
    #[test]
    fn the_re_entry_trn2d_is_within_2040t_and_16008t() {
        let mut settings = settings();
        settings.law = Law::Mu;
        let mut source = Source::new(&settings);
        source.ri_opener();
        source.bound_trn2d(TRN2D_MOST);
        while source.out() != Out::Trn2d {
            source.next();
        }
        // The exchange asks for SUVd as soon as it can; the ceiling is what
        // bounds TRN2d then.
        source.send(Out::Suvd, vec![true; 24]);
        let mut symbols = 0;
        while source.out() == Out::Trn2d {
            source.next();
            symbols += 1;
            assert!(symbols <= TRN2D_MOST + INTERVALS, "TRN2d ran past 16008T");
        }
        assert!(symbols >= 2040, "TRN2d ran {symbols}T, under 2040T");
        assert!(symbols <= TRN2D_MOST);
    }
    /// Requirement 1: **a call with no renegotiation in it is unchanged.**
    ///
    /// The detector runs on every data mode symbol. Over a stream that is data
    /// and not `Ru` it must never fire -- not once, at any of a thousand
    /// symbols -- because a false fire puts `Rd` on the line in the middle of
    /// the far end's data and loses the call. This is the guard the whole
    /// ticket rests on.
    #[test]
    fn no_ru_in_data_mode_never_fires_the_responder() {
        let mut detector = RuDetector::new();
        let events: Vec<RuEvent> = data_levels(4000).into_iter().map(|l| detector.feed(l)).collect();
        assert!(
            events.iter().all(|e| *e == RuEvent::None),
            "a data mode stream fired the 9.8 responder: {:?}",
            events.iter().filter(|e| **e != RuEvent::None).count()
        );
        assert!(!detector.is_locked(), "the receiver was conditioned with no Ru");
    }

    /// 9.8.1.2.1: "After detecting `Ru`, the digital modem shall clamp circuit
    /// 104 to binary one and condition its receiver to detect the `Ru`-to-
    /// `R-bar-u` transition."
    ///
    /// `Ru` is 8.5.5's {+LU, +LU, +LU, -LU, -LU, -LU} repeated, and the
    /// detector needs [`RU_LOCK_PERIODS`] of them -- a number inside the 384T
    /// 9.8.2.1.1 gives the far end to spend.
    #[test]
    fn ru_is_detected_after_four_periods_and_then_the_receiver_waits_for_the_transition() {
        let mut detector = RuDetector::new();
        let mut events = Vec::new();
        // Three periods: not enough.
        for level in ru_periods(RU_LOCK_PERIODS - 1, 40.0, false) {
            events.push(detector.feed(level));
        }
        assert_eq!(events, vec![RuEvent::None; 6 * (RU_LOCK_PERIODS - 1)]);
        assert!(!detector.is_locked(), "locked one period early");
        // The fourth.
        for level in ru_periods(1, 40.0, false) {
            events.push(detector.feed(level));
        }
        assert_eq!(
            events.iter().filter(|e| **e == RuEvent::RuDetected).count(),
            1,
            "9.8.1.2.1 did not fire"
        );
        assert!(detector.is_locked(), "the receiver was not conditioned");
        // More Ru: still renegotiating, still waiting -- and no second clamp.
        for level in ru_periods(4, 40.0, false) {
            assert_eq!(detector.feed(level), RuEvent::None, "Ru fired twice");
        }
    }

    /// **What the far end actually puts on the line.** 8.5.5's
    /// {+LU, +LU, +LU, -LU, -LU, -LU} is a period-six wave; the Agere SV92PP's
    /// transmit path leaves one component of it, so the six symbols arrive as a
    /// 1 333.33 Hz tone whose *magnitudes* are three values and not one. The
    /// levels below are read off `runs-214/164250-sd108ab170-r214/agr-tx.raw`,
    /// records 192 090..192 470, which is the far end's own `Ru` (docs/224 §2):
    /// a 5 389-amplitude six-symbol wave at a constant phase, 46.5 ms long.
    ///
    /// This is the test that fails on the detector before `docs/224`: six
    /// magnitudes of 4 683 / 4 650 / 33 are 99 % apart, so the two-point test
    /// rejects every symbol and `Ru detected` is never printed -- which is
    /// exactly what the bench showed.
    #[test]
    fn the_far_ends_ru_is_a_period_six_wave_and_is_detected_as_one() {
        // One period as measured, oldest first. Six symbols, three magnitudes.
        let period = [-4683.0_f64, -33.0, 4650.0, 4683.0, 33.0, -4650.0];
        // The two-point reading of 8.5.5 cannot see this: the magnitudes are
        // 99 % apart, nearly three times RU_MAGNITUDE_SLACK.
        let loud = 4683.0;
        let quiet = 33.0;
        assert!((loud - quiet) / loud > 2.0 * RU_MAGNITUDE_SLACK);
        let mut detector = RuDetector::new();
        let mut events = Vec::new();
        // 9.8.2.1.1 gives the far end 384T of Ru; feed exactly that.
        for _ in 0..64 {
            for level in period {
                events.push(detector.feed(level));
            }
        }
        assert_eq!(
            events.iter().filter(|e| **e == RuEvent::RuDetected).count(),
            1,
            "9.8.1.2.1 did not fire on the far end's own Ru"
        );
        assert!(detector.is_locked(), "the receiver was not conditioned");
        // Where in the 384T it fires: after RU_TONE_SYMBOLS, not later -- the
        // 384T has 288 symbols left for R-bar-u and the frame boundary.
        let at = events.iter().position(|e| *e == RuEvent::RuDetected).unwrap();
        assert_eq!(at + 1, RU_TONE_SYMBOLS);
        // And no second fire inside the same Ru.
        assert_eq!(events.iter().filter(|e| **e == RuEvent::RuDetected).count(), 1);
    }

    /// The far end's `Ru` ends without an `R-bar-u` -- measured, phase constant
    /// to 0.3 degree over all 46.5 ms of it (docs/224 §2) -- so the transition
    /// 9.8.1.2.2 releases `Rd` on is the end of the wave. Back to data after it
    /// and the detector must say so, once.
    #[test]
    fn the_end_of_the_far_ends_ru_releases_rd_when_no_ru_bar_follows() {
        let period = [-4683.0_f64, -33.0, 4650.0, 4683.0, 33.0, -4650.0];
        let mut detector = RuDetector::new();
        let mut events = Vec::new();
        for _ in 0..64 {
            for level in period {
                events.push(detector.feed(level));
            }
        }
        assert!(detector.is_locked());
        // Data mode again: no six-symbol component, and the magnitudes wander.
        for level in data_levels(400) {
            events.push(detector.feed(level));
        }
        assert_eq!(
            events.iter().filter(|e| **e == RuEvent::RuToRuBar).count(),
            1,
            "the end of the far end's Ru did not release Rd"
        );
    }

    /// 9.8.1.2.2: "After detecting the `Ru`-to-`R-bar-u` transition, the digital
    /// modem shall transmit signal `Rd` ..."
    ///
    /// `R-bar-u` is the negation of 8.5.5's pattern and 9.8.2.1.1 gives it 24T,
    /// so the transition is one period of the inverted pattern -- and it is only
    /// a transition *after* `Ru`, which is what the conditioning is for.
    #[test]
    fn the_ru_to_ru_bar_transition_is_what_releases_rd_and_only_after_ru() {
        // Without Ru first, the bar cannot release Rd.
        //
        // It cannot even be *told apart* from Ru: 8.5.5's {+ + + - - -} read
        // three symbols on is {- - - + + +}, so the negation of the pattern is
        // the pattern itself started at another phase. A receiver watching
        // signs alone therefore sees a run of R-bar-u as a run of Ru -- which
        // is the whole reason 9.8.1.2.1 conditions the receiver *after*
        // detecting Ru rather than merely watching for a sequence. What must
        // not happen is a transition firing with nothing locked.
        let mut detector = RuDetector::new();
        let alone = ru_periods(8, 40.0, true);
        assert!(
            alone
                .into_iter()
                .all(|l| detector.feed(l) != RuEvent::RuToRuBar),
            "R-bar-u with no Ru locked released Rd"
        );
        // With Ru first, the first inverted period does it: 9.8.2.1.1 gives the
        // far end 384T + 24T, so 24T is all the transition has.
        let mut detector = RuDetector::new();
        for level in ru_periods(RU_LOCK_PERIODS, 40.0, false) {
            let _ = detector.feed(level);
        }
        assert!(detector.is_locked());
        let after: Vec<RuEvent> = ru_periods(1, 40.0, true).into_iter().map(|l| detector.feed(l)).collect();
        assert_eq!(
            after.last(),
            Some(&RuEvent::RuToRuBar),
            "the 24T of R-bar-u did not release Rd: {after:?}"
        );
    }

    /// A Source whose data mode constellation is CPd's and whose training
    /// constellation is CPt's, with the two deliberately disjoint so that a
    /// signal sent on the wrong one is visible in the levels.
    fn renegotiating_source() -> Source {
        let mut source = Source::new(&settings_with(79));
        // Twelve intervals, the data mode set a different codeword each one.
        let mut data = [0u8; DOWN_INTERVALS];
        for (i, slot) in data.iter_mut().enumerate() {
            *slot = 100 + i as u8;
        }
        // And training on different codewords again.
        let mut training = [0u8; DOWN_INTERVALS];
        for (i, slot) in training.iter_mut().enumerate() {
            *slot = 40 + i as u8;
        }
        source.arm_renegotiation(data, training);
        source
    }

    /// 9.8.1.1.1 and 9.8.1.2.2: "**The signal Rd shall begin on the boundary of
    /// a data frame.**"
    ///
    /// Asserted, not eyeballed -- `digital.rs:1781-1782` records a signal that
    /// started off-frame and rotated the grid the far end counts against, and
    /// the failure is silent on this side of the line.
    #[test]
    fn rd_begins_on_the_boundary_of_a_data_frame() {
        let mut source = renegotiating_source();
        // Land in the middle of data mode, three symbols into a frame.
        source.start(Out::B1d);
        source.symbol = 60; // ten frames exactly, then three more
        source.next();
        source.next();
        source.next();
        assert_eq!(source.symbol % INTERVALS as u64, 3 % INTERVALS as u64);
        // The far end's Ru-to-R-bar-u is read: Rd is asked for.
        source.change(Out::Rd);
        // Whatever is left of the frame goes out as data, and Rd starts on the
        // next boundary.
        let mut began_at = None;
        for _ in 0..64 {
            source.next();
            if source.out() == Out::Rd && began_at.is_none() {
                began_at = Some(source.symbol - 1);
            }
        }
        let at = began_at.expect("Rd never reached the line");
        assert_eq!(
            at % INTERVALS as u64,
            0,
            "9.8.1.2.2: Rd began {at} symbols in, {} into a data frame",
            at % INTERVALS as u64
        );
    }

    /// 9.8.1.1.1, 9.8.1.2.2: "transmit signal `Rd` for **384T** and then
    /// `R-bar-d` for **24T**", and V.90 8.6.4's "`R` consists of 4 repetitions
    /// of the 6-symbol sequence ... with the sign pattern - - - + + +"
    /// (`v90.txt:1835`).
    #[test]
    fn rd_is_384t_then_rd_bar_is_24t_with_the_four_repetition_structure() {
        let mut source = renegotiating_source();
        source.change(Out::Rd);
        let rd = labelled(&mut source, R_FRAMES * INTERVALS + R_BAR_FRAMES * INTERVALS + 12);
        let rd_len = rd.iter().filter(|(o, _)| *o == Out::Rd).count();
        let bar_len = rd.iter().filter(|(o, _)| *o == Out::RdBar).count();
        assert_eq!(rd_len, 384, "Rd is not 384T: {rd_len}");
        assert_eq!(bar_len, 24, "R-bar-d is not 24T: {bar_len}");
        // The bar's sign pattern, out of the clause: four repetitions of
        // - - - + + - with the left-most sign transmitted first.
        let bar: Vec<bool> = rd
            .iter()
            .filter(|(o, _)| *o == Out::RdBar)
            .map(|(_, level)| *level > 0.0)
            .collect();
        let want: Vec<bool> = (0..R_BAR_FRAMES)
            .flat_map(|_| [false, false, false, true, true, true])
            .collect();
        assert_eq!(bar, want, "R-bar-d is not 8.6.4's four repetitions");
        // And R's own: + + + - - - over the 384T.
        let r: Vec<bool> = rd
            .iter()
            .filter(|(o, _)| *o == Out::Rd)
            .take(INTERVALS)
            .map(|(_, level)| *level > 0.0)
            .collect();
        assert_eq!(r, [true, true, true, false, false, false], "Rd is not + + + - - -");
    }

    /// V.90 8.6.4: "**Rd** denotes signal R using the highest power PCM codeword
    /// from the **data mode** constellation of each data frame interval as
    /// passed in **CP**", while "**Rt** denotes signal R using the highest power
    /// PCM codeword from the **training** constellation of each data frame
    /// interval as passed in **CPt**".
    ///
    /// Two signals, two constellations, and 9.8.1.2.2 asks for one of them while
    /// 9.8.1.1.4 asks for the other. Sharing a hard-coded level between them is
    /// the mistake this test exists to catch.
    #[test]
    fn rd_is_the_data_mode_constellation_and_rt_is_the_training_one() {
        let mut source = renegotiating_source();
        source.change(Out::Rd);
        let rd: Vec<f64> = labelled(&mut source, R_FRAMES * INTERVALS)
            .into_iter()
            .filter(|(o, _)| *o == Out::Rd)
            .map(|(_, level)| level.abs())
            .collect();
        let mut source = renegotiating_source();
        source.change(Out::Rt);
        let rt: Vec<f64> = labelled(&mut source, R_FRAMES * INTERVALS)
            .into_iter()
            .filter(|(o, _)| *o == Out::Rt)
            .map(|(_, level)| level.abs())
            .collect();
        assert_eq!(rd.len(), 384);
        assert_eq!(rt.len(), 384);
        assert_ne!(rd[0], rt[0], "Rd and Rt were sent at the same level");
        // Twelve data frame intervals, twelve codewords: the pattern is a
        // twelve-symbol cycle even though the signs cycle every six.
        let distinct: std::collections::BTreeSet<u64> =
            rt.iter().take(DOWN_INTERVALS).map(|l| l.to_bits()).collect();
        assert_eq!(distinct.len(), DOWN_INTERVALS, "Rt did not follow the twelve intervals");
    }

    /// 9.8.1.2.3 -> 9.8.1.1.2: "The digital modem shall then proceed according
    /// to 9.8.1.1.2", which is "shall then transmit `TRN2d` for **up to
    /// 16008T** followed by `SUVd` sequences".
    ///
    /// "Up to" is a ceiling. What the bar is followed by is TRN2d, and how long
    /// this end holds TRN2d is inside the clause's bound.
    #[test]
    fn after_rd_bar_comes_trn2d_and_it_is_within_16008t() {
        let mut source = renegotiating_source();
        source.change(Out::Rd);
        let stream = labelled(&mut source, R_FRAMES * INTERVALS + R_BAR_FRAMES * INTERVALS + 40);
        let last_bar = stream.iter().rposition(|(o, _)| *o == Out::RdBar).unwrap();
        assert_eq!(
            stream[last_bar + 1].0,
            Out::Trn2d,
            "9.8.1.2.3/9.8.1.1.2: the bar was not followed by TRN2d"
        );
        // The clause's ceiling, and this end's own choice inside it.
        assert!(TRN2D_MOST == 16008, "9.8.1.1.2's ceiling was changed");
        assert!(
            TRN2D_LEAST <= TRN2D_MOST,
            "this end's TRN2d ({TRN2D_LEAST}) exceeds 9.8.1.1.2's 16008T ceiling"
        );
    }

    // -----------------------------------------------------------------
    // 9.8.1.1.2 to 9.6.1.1.2: the tail (docs/215)
    // -----------------------------------------------------------------

    /// One symbol as the receiver would hand it over.
    fn symbol_at(level: f64) -> Symbol {
        Symbol { index: 0, raw: 0, value: level, decided: None, line: level, forward: 0.0 }
    }

    /// **TRN2u's origin is the symbol the last `E1u` bit arrived on, not the
    /// newest sample.**
    ///
    /// The receiver is a centred fit: `equalise_and_slice` decides the symbol
    /// the samples are centred on and emits it `ahead` symbols later, with that
    /// symbol's own index in `Symbol::raw` (`count - ahead`). `cpt_bit` used to
    /// hand `begin_upstream` `rx.symbols()` -- the newest sample -- so the
    /// origin was `ahead - 1` symbols late: 47 at 96 forward taps, which is
    /// outside the +-27 the fit can absorb, and that is the `no phase 4 fit on
    /// TRN2u` at 16.240 on the HSF call (`docs/297` sections 1 and 2).
    ///
    /// Nothing here depends on the value of `ahead`: the symbol spells it out,
    /// and the assertion is that the origin follows `raw` and not `now`.
    #[test]
    fn the_trn2u_origin_is_the_symbol_the_last_e1u_bit_arrived_on() {
        let mut modem = Modem::default();
        // A CPt has been found, so the E1u window is open (`cp_len != 0`), and
        // the reader is in the stage 9.5.2.1.10/11 puts E1u in.
        modem.cpt = Some(Cp { drn: 16, constellations: Vec::new(), ..Cp::default() });
        modem.cp_len = 12;
        modem.cp_pos = 0;
        modem.e1u_zeroes = 0;
        modem.since_cpt = 0;
        modem.stage = Stage::Ri;
        let ahead = 48u64;
        let mut fired = None;
        for step in 0..24u64 {
            if modem.upstream.is_some() {
                break;
            }
            let raw = 1000 + step;
            modem.now = raw + ahead;
            let mut symbol = symbol_at(1.0);
            symbol.raw = raw;
            symbol.decided = Some(1.0);
            modem.symbol(symbol);
            if modem.upstream.is_some() {
                fired = Some(raw);
            }
        }
        let raw = fired.expect("twelve zeroes did not open the upstream reader");
        assert_eq!(
            modem.trn2u_from,
            raw + 1,
            "the origin is {} symbols from the symbol the E1u bit arrived on",
            modem.trn2u_from as i64 - raw as i64
        );
    }

    /// 9.6.1.2.1 does not wait for anything this end receives.
    ///
    /// "If the digital modem does not receive B1u within 20 s plus 6 round-trip
    /// delays from the end of INFO1a, the digital modem shall initiate a
    /// retrain" -- a bound on the *start-up*, not on the exchange. The arm this
    /// end had was `enter_data_mode`'s and `abandon_b1u`'s, and both need an
    /// upstream sequence to have been decoded, so a Phase 4 whose reader
    /// decodes nothing at all has no deadline: `docs/298` section 1's call sat
    /// at `SUVd out` from 16.560 until the harness's own connect timeout, with
    /// `9.6.1.2.1` never armed and the retrain it asks for never sent.
    #[test]
    fn phase_four_arms_96121_from_the_end_of_info1a() {
        let mut modem = Modem::default();
        modem.cpt = Some(Cp { drn: 16, constellations: Vec::new(), ..Cp::default() });
        modem.cp_len = 12;
        modem.cp_pos = 0;
        modem.e1u_zeroes = 0;
        modem.since_cpt = 0;
        modem.stage = Stage::Ri;
        modem.start_up_origin = 500;
        let ahead = 48u64;
        for step in 0..24u64 {
            if modem.upstream.is_some() {
                break;
            }
            let raw = 2000 + step;
            modem.now = raw + ahead;
            let mut symbol = symbol_at(1.0);
            symbol.raw = raw;
            symbol.decided = Some(1.0);
            modem.symbol(symbol);
        }
        assert!(modem.upstream.is_some(), "the upstream reader never started");
        assert!(
            modem.b1u_pending,
            "9.6.1.2.1's wait was not armed when Phase 4's reader opened: nothing the far \
             end sends can arm it, and a reader that decodes nothing leaves the call with \
             no bound at all"
        );
        assert_eq!(
            modem.b1u_deadline,
            Some(modem.start_up_by()),
            "9.6.1.2.1 is counted from the end of INFO1a, not from the arm"
        );
    }

    /// A CPd for drn 1, as `upchoice` would design it.
    fn reneg_cpd() -> Cpd {
        let design = crate::v92::design::Design {
            filters: crate::v92::Filters { z2: vec![1.0], ..crate::v92::Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        let parameters =
            crate::v92::upchoice::pin_with(&design, 1, Trellis::Sixteen, 1e-9, crate::v92::upchoice::LevelSet::Code)
                .expect("drn 1 is pinnable")
                .parameters;
        Cpd::from(&parameters)
    }

    /// A source, a tail and the exchange 9.8.1.1.2 hands 9.6.1.1.2 to: the
    /// things a renegotiation's tail passes between them.
    fn tail_rig() -> (Source, RenegTail, Exchange, Cpd) {
        let mut source = renegotiating_source();
        // 9.8.1.2.3 -> 9.8.1.1.2: the cap the tail's own TRN2d must respect.
        source.bound_trn2d(TRN2D_MOST);
        let cpd = reneg_cpd();
        let (tail, exchange) = RenegTail::new(0.0, FS);
        (source, tail, exchange, cpd)
    }

    /// Drive the tail and the source together, symbol by symbol, with `peer`
    /// called on every symbol so a test can deliver an `SUVu` at the instant it
    /// means to.
    fn drive_tail(
        source: &mut Source,
        tail: &mut RenegTail,
        exchange: &mut Exchange,
        cpd: &Cpd,
        symbols: usize,
        peer: &mut dyn FnMut(u64, &mut Exchange),
    ) -> Vec<(Out, f64)> {
        let mut stream = Vec::with_capacity(symbols);
        for now in 0..symbols as u64 {
            tail.push(source, exchange, Some(cpd), now);
            peer(now, exchange);
            let level = source.next();
            stream.push((source.out(), level));
        }
        stream
    }

    /// 9.8.1.1.2: "The digital modem shall then transmit `TRN2d` for **up to
    /// 16008T** followed by `SUVd` sequences."
    ///
    /// "Up to" is a ceiling and the countdown is what makes it one: `docs/214`
    /// §6 gap 1 was that nothing set a pending change after TRN2d, so the arm's
    /// exit condition never fired and TRN2d ran on for ever -- no `SUVd`, no
    /// `CPd`, nothing for the far end to answer.
    #[test]
    fn trn2d_is_cut_at_16008t_and_suvd_follows() {
        let (mut source, mut tail, mut exchange, cpd) = tail_rig();
        // 9.8.1.2.2's Rd and R-bar-d first, on the source's own arms.
        source.change(Out::Rd);
        let stream = drive_tail(
            &mut source,
            &mut tail,
            &mut exchange,
            &cpd,
            R_FRAMES * INTERVALS + R_BAR_SYMBOLS + TRN2D_MOST + 60,
            &mut |_, _| {},
        );
        let trn2d = stream.iter().filter(|(out, _)| *out == Out::Trn2d).count();
        assert_eq!(trn2d, 16008, "9.8.1.1.2: TRN2d is not cut at 16008T");
        let last_trn2d = stream.iter().rposition(|(out, _)| *out == Out::Trn2d).unwrap();
        assert_eq!(
            stream[last_trn2d + 1].0,
            Out::Suvd,
            "9.8.1.1.2: TRN2d was not followed by SUVd"
        );
        // And the cut is on the data frame grid the SUVd after it must start on.
        assert_eq!(
            (last_trn2d + 1) % INTERVALS,
            0,
            "the SUVd after TRN2d began off the data frame boundary"
        );
    }

    /// 9.8.1.1.2 with 9.6.1.1.2: with bit 32 clear in both directions -- "the
    /// digital modem shall proceed according to 9.6.1.1.2 unless bit 32 is set
    /// in either `SUVd` or `SUVu`" -- the far end's `SUVu` releases **one**
    /// `CPd`.
    #[test]
    fn with_bit_32_clear_in_both_directions_one_cpd_follows_the_suvu() {
        let (mut source, mut tail, mut exchange, cpd) = tail_rig();
        source.change(Out::Rd);
        // The far end's SUVu arrives 200 symbols into the SUVd that follows
        // TRN2d, with bit 32 clear and the acknowledge bit clear (it has not
        // received a CPd yet: 9.8.2.1.3).
        let mut delivered = false;
        let stream = drive_tail(
            &mut source,
            &mut tail,
            &mut exchange,
            &cpd,
            R_FRAMES * INTERVALS + R_BAR_SYMBOLS + TRN2D_MOST + 1200,
            &mut |_, exchange| {
                if !delivered && exchange.sending().is_some() {
                    delivered = true;
                    exchange.peer_suv(PeerSuv { ack: false, silence: false, wait_for_cp: false }, 0);
                }
            },
        );
        assert!(delivered, "no SUVd went out to answer");
        let cpds = runs(&stream, Out::Cpd);
        assert_eq!(cpds, 1, "9.6.1.1.2: `a single CPd`, and {cpds} went out");
        // And it went out after the SUVd that answered the SUVu, not before.
        let first_cpd = stream.iter().position(|(out, _)| *out == Out::Cpd).unwrap();
        assert!(
            stream[..first_cpd].iter().any(|(out, _)| *out == Out::Suvd),
            "the CPd went out before any SUVd"
        );
        assert!(tail.stage() != RenegStage::Silence, "bit 32 clear took the silence branch");
    }

    /// 9.8.1.1.3: "The digital modem shall then transmit `SUVd` sequences with
    /// **bit 33 set**. After receiving an `SUVu` sequence with bit 33 set or
    /// `E2u`, the digital modem shall complete sending the current `SUVd` and
    /// then transmit `Ed` followed by silence ... It shall **retain data frame
    /// alignment** during this period of silence."
    ///
    /// This is the branch 9.8.1.1.2 opens when bit 32 is set in *either*
    /// direction, and here it is the far end's `SUVu` that sets it -- the one
    /// place the two directions interact.
    #[test]
    fn suvu_bit_32_set_takes_9_8_1_1_3_s_suvd_bit_33_then_ed_then_silence() {
        let (mut source, mut tail, mut exchange, cpd) = tail_rig();
        source.change(Out::Rd);
        let total = R_FRAMES * INTERVALS + R_BAR_SYMBOLS + TRN2D_MOST + 2400;
        let mut asked = false;
        let mut acked = false;
        // What the SUVd sequences on the line actually carry: bit 32 and bit
        // 33 read back out of the sequence itself (`Suvd::from_bits`), not out
        // of the intent.
        let mut carried: Vec<(bool, bool)> = Vec::new();
        let mut stream = Vec::with_capacity(total);
        for now in 0..total as u64 {
            if let Some(Out::Suvd) = tail.push(&mut source, &mut exchange, Some(&cpd), now) {
                // What the sequence this just asked for carries, read back out
                // of it (`Suvd::from_bits`) rather than out of the intent.
                let sent = Suvd::from_bits(&source.last).expect("an SUVd was asked for but not built");
                if !carried.contains(&(sent.silence, sent.ack)) {
                    carried.push((sent.silence, sent.ack));
                }
            }
            // 9.8.2.1.3: SUVu with bit 32 set asks for the silent period.
            if !asked && exchange.sending().is_some() {
                asked = true;
                exchange.peer_suv(PeerSuv { ack: false, silence: true, wait_for_cp: false }, 0);
            }
            // Then "an SUVu sequence with bit 33 set", which is 9.8.1.1.3's
            // other trigger.
            if asked && !acked && exchange.peer_silence() {
                acked = true;
                exchange.peer_suv(PeerSuv { ack: true, silence: true, wait_for_cp: false }, 1);
            }
            let level = source.next();
            stream.push((source.out(), level));
        }
        assert!(
            carried.contains(&(true, true)),
            "9.8.1.1.3: no SUVd went out with bit 32 and bit 33 both set: {carried:?}"
        );
        // SUVd with bit 33 set: the sequence in the SUVd after TRN2d that
        // follows the far end's request.
        let first_suvd = stream.iter().position(|(out, _)| *out == Out::Suvd).unwrap();
        let first_ed = stream.iter().position(|(out, _)| *out == Out::Ed).expect("no Ed: 9.8.1.1.3");
        assert!(first_ed > first_suvd, "the Ed came before any SUVd'");
        // "complete sending the current SUVd and then transmit Ed": the Ed
        // begins where a whole SUVd has run out, which is a data frame multiple.
        assert_eq!(
            first_ed % INTERVALS,
            0,
            "the Ed began {} symbols into a data frame, not after a whole SUVd",
            first_ed % INTERVALS
        );
        // "followed by silence", Ucode 0 (9.8.1.1.3's own definition of it).
        let after: Vec<&(Out, f64)> = stream[first_ed..].iter().collect();
        let silent = after.iter().position(|(out, _)| *out == Out::Silence);
        let silent = silent.unwrap_or_else(|| panic!("no silence after the Ed: {after:?}"));
        assert!(
            stream[first_ed + silent..first_ed + silent + 12]
                .iter()
                .all(|(out, level)| *out != Out::Silence || *level == 0.0),
            "9.8.1.1.3: the silence is not Ucode 0"
        );
        // "It shall retain data frame alignment during this period of silence":
        // the silence begins on a frame boundary and runs whole frames, because
        // it occupies data frame intervals like every other signal
        // (`Source::next`) and the Ed before it ended on a boundary.
        let silence_start = first_ed + silent;
        assert_eq!(
            silence_start % INTERVALS,
            0,
            "the silence began {} symbols into a data frame",
            silence_start % INTERVALS
        );
        let run = stream[silence_start..].iter().take_while(|(out, _)| *out == Out::Silence).count();
        assert!(run >= INTERVALS, "the silence was shorter than a data frame: {run}");
        assert_eq!(run % INTERVALS, 0, "the silence did not occupy whole data frames: {run}");
        assert_eq!(tail.stage(), RenegStage::Silence, "the tail did not reach the silence");
    }

    /// 9.8.1.1.4: "If bit 32 of `SUVu` was set, the digital modem shall wait to
    /// receive `SUVu` with bit 32 clear. After receiving `SUVu` with bit 32
    /// clear, the digital modem shall transmit `Rt` for 384T followed by
    /// `R-bar-t` for 24T and `SUVd`. The digital modem shall then proceed
    /// according to 9.6.1.1.2."
    ///
    /// `R-bar-t` is V.90 8.6.4's "4 repetitions of the 6-symbol sequence ...
    /// with the sign pattern – – – + + +" (`v90.txt:1835`), read off that line.
    #[test]
    fn suvu_bit_32_cleared_after_the_silence_releases_rt_then_r_bar_t_then_suvd() {
        let (mut source, mut tail, mut exchange, cpd) = tail_rig();
        source.change(Out::Rd);
        let mut phase = 0;
        let mut acked_at = None;
        let stream = drive_tail(
            &mut source,
            &mut tail,
            &mut exchange,
            &cpd,
            R_FRAMES * INTERVALS
                + R_BAR_SYMBOLS
                + TRN2D_MOST
                + 2400
                + R_FRAMES * INTERVALS
                + R_BAR_SYMBOLS
                + 600,
            &mut |now, exchange| {
                if phase == 0 && exchange.sending().is_some() {
                    phase = 1;
                    exchange.peer_suv(PeerSuv { ack: false, silence: true, wait_for_cp: false }, 0);
                } else if phase == 1 && exchange.peer_silence() {
                    phase = 2;
                    acked_at = Some(now);
                    exchange.peer_suv(PeerSuv { ack: true, silence: true, wait_for_cp: false }, 1);
                } else if phase == 2 && now >= acked_at.unwrap_or(u64::MAX) + 2 * INTERVALS as u64 {
                    // 9.8.1.1.4's "SUVu with bit 32 clear", after the Ed and
                    // into the silence -- and then the plain SUVu sequences of
                    // Figures 16 and 17, which are what release the CPd.
                    phase = 3;
                    exchange.peer_suv(PeerSuv { ack: false, silence: false, wait_for_cp: false }, 2);
                } else if phase == 3 {
                    exchange.peer_suv(PeerSuv { ack: false, silence: false, wait_for_cp: false }, 3);
                }
            },
        );
        assert!(phase == 3, "the far end never sent the SUVu with bit 32 clear");
        let rt = stream.iter().filter(|(out, _)| *out == Out::Rt).count();
        let rt_bar = stream.iter().filter(|(out, _)| *out == Out::RtBar).count();
        assert_eq!(rt, 384, "9.8.1.1.4: Rt is not 384T: {rt}");
        assert_eq!(rt_bar, 24, "9.8.1.1.4: R-bar-t is not 24T: {rt_bar}");
        // The bar's sign pattern, from `v90.txt:1835` and not from memory.
        let bar: Vec<bool> = stream
            .iter()
            .filter(|(out, _)| *out == Out::RtBar)
            .map(|(_, level)| *level > 0.0)
            .collect();
        let want: Vec<bool> = (0..R_BAR_FRAMES).flat_map(|_| [false, false, false, true, true, true]).collect();
        assert_eq!(bar, want, "R-bar-t is not 8.6.4's four repetitions of – – – + + +");
        // And the SUVd after the bar, then 9.6.1.1.2's one CPd.
        let last_bar = stream.iter().rposition(|(out, _)| *out == Out::RtBar).unwrap();
        assert!(
            stream[last_bar + 1..].iter().any(|(out, _)| *out == Out::Suvd),
            "9.8.1.1.4: the bar was not followed by SUVd"
        );
        assert_eq!(runs(&stream, Out::Cpd), 1, "9.6.1.1.2: not exactly one CPd after 9.8.1.1.4");
    }

    /// `docs/213` §6's missing feed: the far end's `SUVu` reader is the Phase 3
    /// `up_finder`, and it must be fed **only** once a renegotiation is in
    /// progress.
    ///
    /// Two halves, and the second is the one the whole ticket rests on. A call
    /// with no `Ru` in it must reach the data path byte for byte as it did: the
    /// feed is gated on the tail, the tail is only built by `ru_ended`, and
    /// `ru_ended` is only reached from [`RuEvent::RuToRuBar`] -- which
    /// `no_ru_in_data_mode_never_fires_the_responder` pins at zero firings over
    /// 4000 data mode symbols.
    #[test]
    fn the_suvu_feed_is_armed_by_a_renegotiation_and_by_nothing_else() {
        let mut modem = Modem::default();
        // The data mode path: the detector runs on every symbol (it is a side
        // channel), and four thousand of them never arm the finder.
        for level in data_levels(4000) {
            modem.b1u_symbol(symbol_at(level));
        }
        assert!(modem.reneg.is_none(), "a renegotiation's tail was built with no Ru");
        assert!(modem.reneg_trn2u_from.is_none(), "the SUVu reader was armed with no Ru");
        assert!(modem.up_finder.is_none(), "the SUVu finder was armed in data mode");
        assert!(modem.upstream_data.is_none(), "a decoder was invented for a call with no data mode");

        // And with Ru followed by R-bar-u the same feed is armed, which is the
        // other side of the gate.
        for level in ru_periods(RU_LOCK_PERIODS, 40.0, false) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(1, 40.0, true) {
            modem.b1u_symbol(symbol_at(level));
        }
        assert!(modem.reneg.is_some(), "9.8.1.1.2's tail was not built on R-bar-u");
        assert_eq!(
            modem.reneg_trn2u_from,
            Some(modem.now + TRN2U_AFTER_RU_BAR),
            "the far end's TRN2u 24T after R-bar-u begins is not the fiducial"
        );
        // Out of data mode, and the reader is built *on* TRN2u's first symbol
        // (8.7.6's scrambler reset), not before it.
        assert!(modem.upstream.is_none());
        assert!(modem.up_finder.is_none());
        assert_eq!(TRN2U_AFTER_RU_BAR, (R_BAR_SYMBOLS - RU_BAR_DEVIATIONS) as u64);
    }

    /// 9.8.1.2.1: a rate renegotiation takes the data mode down, so the pump is
    /// **not** connected while it runs.
    ///
    /// This is the spurious CONNECT (docs/227 §3, docs/230 §1). `begin_data`
    /// latches `Status::Connected` at the first data mode and `ru_ended` used
    /// to leave it set, so `Pump::status` reported `Progress::Connected` for
    /// the whole of 9.8. The modem above built a V.42 stack over a line that
    /// had stopped carrying B1u, its detection phase found no ODP and gave up,
    /// and the CONNECT it had been holding was released -- for a call whose
    /// line had only ever reached `SUVd`. It goes out on the bench as
    /// `CONNECT 24000` followed by `0 octets`, which is a lie that masks every
    /// failure underneath it.
    ///
    /// Fails on `e3d1114`: there the status is still `Connected` here.
    #[test]
    fn nine_eight_takes_the_digital_modem_out_of_the_connected_state() {
        // A first data mode, the one whose CONNECT the modem above is holding
        // while V.42's detection phase runs.
        let mask = |points: std::ops::RangeInclusive<u8>| {
            points.fold(0u128, |m, u| m | 1 << u)
        };
        let cpu = Cp {
            data_mode: true,
            drn: 18,
            intervals: [0, 1, 0, 1, 0, 1],
            constellations: vec![mask(3..=95), mask(3..=88)],
            ..Cp::default()
        };
        let mut modem = Modem::default();
        modem.cpu = Some(cpu);
        modem.begin_data();
        assert!(modem.downstream.is_some(), "no B1d mapping was built from that CPu");
        assert!(
            matches!(modem.status(), Status::Connected { .. }),
            "no first data mode: nothing to be un-latched from"
        );

        for level in ru_periods(RU_LOCK_PERIODS, 40.0, false) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(1, 40.0, true) {
            modem.b1u_symbol(symbol_at(level));
        }
        assert!(modem.reneg.is_some(), "9.8.1.1.2's tail was not built on R-bar-u");
        assert_eq!(
            modem.status(),
            Status::Phase4,
            "9.8.1.2.1: the data mode was torn down, so the pump must not report \
             `Connected` through the renegotiation -- the modem above releases \
             the held CONNECT on it"
        );
    }


    /// 9.8.1.2.3 -> 9.8.1.1.2 -> 9.6.1.1.2 -> 9.6.1.1.4's `Ed` -> 9.6.1.1.5's
    /// `B1d`: a renegotiation the far end starts with `Ru` ends in data mode
    /// again.
    ///
    /// Two halves, and the first is the one the bench found dead (docs/227).
    /// `ru_ended` drops **both** upstream readers -- `upstream` and
    /// `upstream_data` -- so that `reneg_symbol` can read the far end's
    /// `TRN2u`/`SUVu`, but `symbol()` gated the feed on those two readers
    /// alone. The upstream was therefore silent for the whole of 9.8: no
    /// `SUVu` was decoded, `Exchange::peer_suv` was never called, the tail sat
    /// at `RenegStage::Suvd` repeating `SUVd`, nothing ever answered the far
    /// end's `CPu`, and this end never reached its own data mode -- while the
    /// far end, having heard the `Rd` and the `SUVd`, threw `0x96212` ten
    /// seconds later.
    #[test]
    fn nine_eight_one_two_three_to_nine_six_one_one_two_then_ed_then_b1d() {
        let mut modem = Modem::default();
        // Into data mode, then the far end's Ru and the end of it (9.8.1.2.1).
        for level in data_levels(4000) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(RU_LOCK_PERIODS, 40.0, false) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(1, 40.0, true) {
            modem.b1u_symbol(symbol_at(level));
        }
        assert!(modem.reneg.is_some(), "9.8.1.1.2's tail was not built on R-bar-u");
        let fiducial = modem.reneg_trn2u_from.expect("9.8.2.1.1's fiducial for the far end's TRN2u");
        assert!(modem.upstream.is_none(), "a reader was built before the far end's TRN2u");

        // 9.8.1.2.3 hands 9.8.1.1.2 `TRN2d`; from the fiducial on, the far
        // end's symbols have to reach `reneg_symbol`, which builds the reader
        // on TRN2u's first symbol (8.7.6's scrambler reset). This is the line
        // that was dead: `symbol()` never called `symbol_upstream` during 9.8.
        modem.now = fiducial;
        modem.symbol(symbol_at(1.0));
        assert!(
            modem.upstream.is_some() && modem.up_finder.is_some(),
            "9.8.1.1.2: the far end's TRN2u/SUVu feed is not armed -- `symbol()` \
             did not route the symbol to `reneg_symbol`"
        );

        // The exchange itself, from 9.6.1.1.2 to 9.6.1.1.5: one CPd after the
        // far end's SUVu, Ed after the far end's CPu and its acknowledged
        // SUVu, and B1d after the Ed.
        modem.cpd = Some(reneg_cpd());
        let mut stream: Vec<Out> = Vec::new();
        let mut suv_seen = 0;
        for _ in 0..(R_FRAMES * INTERVALS + R_BAR_SYMBOLS + TRN2D_MOST + 6000) {
            modem.reneg_out();
            let now = modem.now;
            modem.now += 1;
            if let Some(exchange) = modem.exchange.as_mut() {
                match suv_seen {
                    // 9.8.2.1.3: the far end's first SUVu carries no
                    // acknowledge -- it has not received our CPd yet.
                    0 if modem.source.out() == Out::Suvd => {
                        suv_seen = 1;
                        exchange.peer_suv(PeerSuv { ack: false, silence: false, wait_for_cp: false }, now);
                    }
                    // 9.6.2.1.2: its CPu answers a CPd.
                    1 if stream.contains(&Out::Cpd) => {
                        suv_seen = 2;
                        exchange.peer_cp(PeerCp { ack: false }, now);
                    }
                    // 9.6.1.1.4: it acknowledges once it has our CPd, and the
                    // Ed follows the acknowledged SUVd.
                    2 => {
                        suv_seen = 3;
                        exchange.peer_suv(PeerSuv { ack: true, silence: false, wait_for_cp: false }, now);
                    }
                    _ => {}
                }
            }
            let _ = modem.source.next();
            stream.push(modem.source.out());
            if modem.reneg.is_none() {
                // 9.6.1.1.5: `B1d` is asked for at the Ed's boundary and is on
                // the line from the source's next symbol, so run a few more.
                for _ in 0..(2 * INTERVALS) {
                    let _ = modem.source.next();
                    stream.push(modem.source.out());
                }
                break;
            }
        }
        // `runs` counts runs and not symbols: a CPd is 480 symbols.
        let runs: Vec<&Out> = stream.windows(1).map(|w| &w[0]).collect();
        let first_suv = runs.iter().position(|o| **o == Out::Suvd).expect("no SUVd went out");
        let cpd = runs.iter().position(|o| **o == Out::Cpd).expect("9.6.1.1.2: no CPd went out");
        let ed = runs.iter().position(|o| **o == Out::Ed).expect("9.6.1.1.4: no Ed went out");
        let b1d = runs.iter().position(|o| **o == Out::B1d).expect("9.6.1.1.5: no B1d went out");
        assert!(first_suv < cpd, "the CPd went out before any SUVd");
        assert!(cpd < ed, "the Ed went out before the CPd");
        assert!(ed < b1d, "the B1d went out before the Ed");
        assert!(modem.reneg.is_none(), "the tail did not finish at 9.6.1.1.5");
        assert!(modem.exchange.is_none(), "the exchange outlived the renegotiation");
    }

    /// 9.6.1.1.5 after 9.8: **a renegotiation leaves the pump connected**, and
    /// leaves it connected through the same door Phase 4 uses.
    ///
    /// This is the mirror of
    /// [`Self::nine_eight_takes_the_digital_modem_out_of_the_connected_state`],
    /// and the pair exists because the two halves were fixed a ticket apart.
    /// 9.8.1.2.1 takes the data mode down (`ru_ended` latches `Status::Phase4`),
    /// the modem above reads that as `Progress::Retraining`, withdraws the
    /// `announce`+`ec` it was holding (`lib.rs:1966`) -- and nothing in 9.8 put
    /// any of it back. Measured (`runs-237/204437-b1u171-1`): the renegotiation
    /// completed at 36.540, `E2u` at 36.560, `B1u confirmed` 576/576 ones at
    /// 36.620, and the far end logged `Fallback initiated! LAPM timeout`
    /// 1.008 s into its data mode because no `LAPM` frame ever went downstream
    /// again: the pump reported `Phase4` (-> `Progress::Retraining`) for the
    /// rest of the call, so the modem above never built a stack and V.42's
    /// clocks stayed held at zero (`lib.rs:1924`).
    ///
    /// Fails on `150ad56`: there the status is still `Phase4` and `B1d`'s
    /// countdown is still zero, because `finish_renegotiation` re-armed the
    /// `B1u` wait by hand and never re-ran `begin_data`'s body.
    #[test]
    fn nine_eight_leaves_the_digital_modem_connected_and_lapm_able() {
        // A first data mode with a CPu, so both rates are real and the second
        // `B1d` has an encoder to be rebuilt from.
        let mask = |points: std::ops::RangeInclusive<u8>| {
            points.fold(0u128, |m, u| m | 1 << u)
        };
        let cpu = Cp {
            data_mode: true,
            drn: 18,
            intervals: [0, 1, 0, 1, 0, 1],
            constellations: vec![mask(3..=95), mask(3..=88)],
            ..Cp::default()
        };
        // CPd's own design, so the upstream half of the latch has a rate to
        // report and 9.6.1.1.6's arm has something to arm on -- `begin_data`
        // and `enter_data_mode` both read `parameters` for it.
        let design = crate::v92::design::Design {
            filters: crate::v92::Filters { z2: vec![1.0], ..crate::v92::Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        let parameters = crate::v92::upchoice::pin_with(
            &design,
            1,
            Trellis::Sixteen,
            1e-9,
            crate::v92::upchoice::LevelSet::Code,
        )
        .expect("drn 1 is pinnable")
        .parameters;
        let mut modem = Modem::default();
        modem.cpu = Some(cpu);
        modem.parameters = Some(parameters);
        modem.begin_data();
        assert!(
            matches!(modem.status(), Status::Connected { .. }),
            "no first data mode: nothing to be un-latched from"
        );
        // What a fresh 9.6.1.1.5 leaves `B1d`'s countdown at, so that the
        // re-latch is held to starting one rather than to a number.
        let armed = modem.b1d_left;
        assert!(armed > 0, "9.6.1.1.5: the first B1d armed no countdown at all");

        // 9.8.1.2.1: Ru, then the transition that releases Rd.
        for level in ru_periods(RU_LOCK_PERIODS, 40.0, false) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(1, 40.0, true) {
            modem.b1u_symbol(symbol_at(level));
        }
        assert!(modem.reneg.is_some(), "9.8.1.1.2's tail was not built on R-bar-u");
        assert_eq!(
            modem.status(),
            Status::Phase4,
            "9.8.1.2.1: the data mode was torn down, so the pump must not report \
             `Connected` while the renegotiation runs"
        );

        // 9.8.1.2.3 -> 9.8.1.1.2 -> 9.6.1.1.2 -> Ed, exactly as the test above
        // drives it: the far end's SUVu, its CPu, and its acknowledged SUVu.
        let fiducial = modem.reneg_trn2u_from.expect("9.8.2.1.1's fiducial for the far end's TRN2u");
        modem.now = fiducial;
        modem.symbol(symbol_at(1.0));
        modem.cpd = Some(reneg_cpd());
        let mut stream: Vec<Out> = Vec::new();
        let mut suv_seen = 0;
        for _ in 0..(R_FRAMES * INTERVALS + R_BAR_SYMBOLS + TRN2D_MOST + 6000) {
            modem.reneg_out();
            let now = modem.now;
            modem.now += 1;
            if let Some(exchange) = modem.exchange.as_mut() {
                match suv_seen {
                    0 if modem.source.out() == Out::Suvd => {
                        suv_seen = 1;
                        exchange.peer_suv(PeerSuv { ack: false, silence: false, wait_for_cp: false }, now);
                    }
                    1 if stream.contains(&Out::Cpd) => {
                        suv_seen = 2;
                        exchange.peer_cp(PeerCp { ack: false }, now);
                    }
                    2 => {
                        suv_seen = 3;
                        exchange.peer_suv(PeerSuv { ack: true, silence: false, wait_for_cp: false }, now);
                    }
                    _ => {}
                }
            }
            let _ = modem.source.next();
            stream.push(modem.source.out());
            if modem.reneg.is_none() {
                for _ in 0..(2 * INTERVALS) {
                    let _ = modem.source.next();
                    stream.push(modem.source.out());
                }
                break;
            }
        }
        assert!(modem.reneg.is_none(), "the tail did not finish at 9.6.1.1.5");
        assert!(
            stream.contains(&Out::B1d),
            "9.6.1.1.5: no B1d went out after the renegotiation's Ed"
        );

        // **The defect.** 9.6.1.1.5-6 is data mode again, so the pump says so:
        // `v90::startup`'s `pcm::Status::Connected` arm (`startup.rs:534`) turns
        // this into `Progress::Connected`, and that is what makes the modem
        // above build its V.42/LAPM stack (`lib.rs:2011-2141`) and run V.42's
        // clocks (`lib.rs:1924`) instead of holding them.
        let Status::Connected { downstream, upstream } = modem.status() else {
            panic!(
                "9.6.1.1.5: after a renegotiation the pump must report `Connected` \
                 again -- it reports {:?}, so no LAPM frame goes downstream and the \
                 far end's T401 expires 1.008 s into its data mode",
                modem.status()
            );
        };
        assert!(downstream > 0, "the re-latch did not rebuild B1d's encoder from CPu");
        assert_eq!(upstream, modem.parameters.as_ref().map_or(0, Parameters::rate));
        // 8.6.1: the second `B1d` is a `B1d` -- a fresh scrambler and 48 data
        // frames of scrambled ones the far end counts down before its data.
        assert_eq!(
            modem.b1d_left, armed,
            "9.6.1.1.5: the renegotiation's B1d was not re-armed -- a fresh one \
             starts at {armed} and this one is at {}",
            modem.b1d_left
        );
        // 9.6.1.1.6: and the E2u that follows every 9.6.1.1.5 is waited for.
        assert!(modem.awaiting_e2u, "the renegotiation's E2u is not being waited for");
        assert!(modem.b1u_pending, "9.6.1.1.6's B1u arm is not armed after the renegotiation");
        assert!(
            modem.b1u_deadline.is_some_and(|d| d > modem.now),
            "9.6.1.1.6's B1u deadline was not re-armed by the re-latch: {:?} at {}",
            modem.b1u_deadline, modem.now
        );
    }

    /// 8.7.2 with 8.7.6: the renegotiation's `E2u` is read by the descrambler
    /// that has run **since `TRN2u`'s first symbol**, not by a new one at `Ed`.
    ///
    /// 8.7.2: "`E2u` is a data frame of **scrambled**, differentially encoded
    /// zeroes. It is transmitted using the corresponding `TRN2u` modulation
    /// during Training and **Rate Renegotiation**." 8.7.6: "The scrambler shall
    /// be reset at the beginning of `TRN2u`." So its zeroes are only zeroes
    /// through the state `TRN2u`'s reset began, carried through every sequence
    /// between it and `E2u` -- and `Ed` is not a place a reader may start,
    /// because nothing on the far end's side restarts there.
    ///
    /// Fails on `73d5ad1`: `finish_renegotiation` built a fresh `Trn2uReader` at
    /// `Ed`, restarting the GPA descrambler mid-stream, so the renegotiation's
    /// `E2u` read as white data, no data frame's worth of zeroes ever formed,
    /// and the call ended on our own line "no E2u that reads as B1u within 2 s
    /// of Ed" (`runs-236c/203200-f171-1`, 38.540) with the far end's
    /// `6009521a` after it. The initial Phase 4 never exposed it: its reader is
    /// built once, at `TRN2u`'s first symbol (`begin_upstream`), and nothing
    /// between there and its `E2u` rebuilds it.
    #[test]
    fn a_renegotiations_e2u_is_read_by_the_descrambler_that_started_at_trn2u() {
        use crate::v92::up_signals::Trn2uSender;
        let mut modem = Modem::default();
        // A first data mode, and the far end's Ru and R-bar-u (9.8.1.2.1).
        for level in data_levels(4000) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(RU_LOCK_PERIODS, 40.0, false) {
            modem.b1u_symbol(symbol_at(level));
        }
        for level in ru_periods(1, 40.0, true) {
            modem.b1u_symbol(symbol_at(level));
        }
        assert!(modem.reneg.is_some(), "9.8.1.1.2's tail was not built on R-bar-u");
        // 9.8.2.1.1's fiducial: the far end's renegotiation TRN2u begins here,
        // and 8.7.6's scrambler reset is on its first symbol.
        let fiducial = modem.reneg_trn2u_from.expect("9.8.2.1.1's fiducial for the far end's TRN2u");
        let mut peer = Trn2uSender::seeded(Trn2uSize::Four, false);
        modem.now = fiducial;
        let first = peer.next_symbol();
        modem.symbol(symbol_at(first));
        assert!(modem.upstream.is_some(), "no reader on the far end's TRN2u");
        // The renegotiation's exchange, 9.6.1.1.2 to 9.6.1.1.4's Ed, with the
        // far end's line present the whole way: its TRN2u idles on ones, which
        // carry no start bit and so no sequence the finder can mistake.
        modem.cpd = Some(reneg_cpd());
        let design = crate::v92::design::Design {
            filters: crate::v92::Filters { z2: vec![1.0], ..crate::v92::Filters::default() },
            residual: 0.0,
            scale: 1.0,
            limited: false,
            delay: 0,
        };
        modem.parameters = Some(
            crate::v92::upchoice::pin_with(
                &design,
                1,
                Trellis::Sixteen,
                1e-9,
                crate::v92::upchoice::LevelSet::Code,
            )
            .expect("drn 1 is pinnable")
            .parameters,
        );
        let mut suv_seen = 0;
        let mut stream: Vec<Out> = Vec::new();
        for _ in 0..(R_FRAMES * INTERVALS + R_BAR_SYMBOLS + TRN2D_MOST + 6000) {
            modem.reneg_out();
            let now = modem.now;
            modem.now += 1;
            if let Some(exchange) = modem.exchange.as_mut() {
                match suv_seen {
                    0 if modem.source.out() == Out::Suvd => {
                        suv_seen = 1;
                        exchange.peer_suv(PeerSuv { ack: false, silence: false, wait_for_cp: false }, now);
                    }
                    1 if stream.contains(&Out::Cpd) => {
                        suv_seen = 2;
                        exchange.peer_cp(PeerCp { ack: false }, now);
                    }
                    2 => {
                        suv_seen = 3;
                        exchange.peer_suv(PeerSuv { ack: true, silence: false, wait_for_cp: false }, now);
                    }
                    _ => {}
                }
            }
            let level = peer.next_symbol();
            modem.symbol(symbol_at(level));
            let _ = modem.source.next();
            stream.push(modem.source.out());
            if modem.reneg.is_none() {
                break;
            }
        }
        assert!(modem.reneg.is_none(), "the renegotiation never reached 9.6.1.1.4's Ed");
        assert!(modem.awaiting_e2u, "Ed did not arm the search for E2u");
        // 8.7.2: E2u is a data frame of scrambled zeroes -- twelve symbols, two
        // bits each on TRN2u's four points -- and 8.7.1's B1u, which reads as
        // ones, is where the run ends.
        let frame = crate::v92::UP_INTERVALS;
        peer.push(&vec![false; 2 * frame]);
        peer.push(&vec![true; 2 * frame]);
        for _ in 0..(3 * frame) {
            modem.now += 1;
            let level = peer.next_symbol();
            modem.symbol(symbol_at(level));
        }
        assert!(
            !modem.awaiting_e2u,
            "8.7.2/8.7.6: the renegotiation's E2u was not read as E2u -- the descrambler that \
             recovers its zeroes is not the one that has run since TRN2u's first symbol"
        );
        assert!(modem.b1u_search.is_some(), "9.6.1.1.6: E2u did not condition the receiver for B1u");
    }

    /// 9.8.1.1.3: "The digital modem shall generate silence by sending PCM
    /// codewords with magnitudes represented by **Ucode 0**. It shall **retain
    /// data frame alignment** during this period of silence."
    ///
    /// The alignment is the load-bearing half: silence that does not advance
    /// the frame count leaves the next signal off the grid.
    #[test]
    fn silence_is_ucode_zero_and_it_keeps_the_data_frame_grid() {
        let mut source = renegotiating_source();
        source.start(Out::B1d);
        source.symbol = 12 * 5; // on a frame boundary
        source.change(Out::Silence);
        let stream = labelled(&mut source, 30);
        let silent = stream.iter().filter(|(o, _)| *o == Out::Silence).count();
        assert!(silent >= 6, "no silence to measure: {stream:?}");
        assert!(
            stream.iter().all(|(o, level)| *o != Out::Silence || *level == 0.0),
            "9.8.1.1.3: silence is not Ucode 0"
        );
        // The grid moved by exactly the codewords that went out.
        assert_eq!(source.symbol % INTERVALS as u64, 0, "the frame grid was rotated by silence");
    }

}

/// docs/218: V.90 8.6.4's three level sources, kept apart.
///
/// `Rd`, `Ri` and `Rt` take their levels from three different places -- the
/// data mode constellation as passed in CP, the single codeword U_INFO, and the
/// training constellation as passed in CPt -- and the defect this module pins
/// was one message's numbers reaching the wrong one of them: `Rd` went out at
/// Ucode 2, linear 16, out of this end's own CPd.
#[cfg(test)]
mod conformance_218 {
    use super::*;


// ---------------------------------------------------------------------
// docs/218: 8.6.4's three level sources, kept apart
// ---------------------------------------------------------------------

/// Every level `count` symbols of the source produce, after asking for
/// `out` (`Source::arm_renegotiation` first where 9.8's codewords matter).
pub(super) fn r_stream(source: &mut Source, out: Out, count: usize) -> Vec<f64> {
    source.change(out);
    (0..count).map(|_| source.next()).collect()
}


/// Settings at a named U_INFO, which is what `Ri` is emitted at.
pub(super) fn settings_uinfo(uinfo: u8) -> Settings {
    Settings {
        law: Law::Mu,
        uinfo,
        far_md: 0,
        round_trip: 0.0,
        wide: true,
        limits: FilterLimits::from_info1a(0, 0, 0),
        habits: Habits::PROMPT,
        probing: None,
    }
}

/// A CPt of the shape the analogue modem sends in Phase 3: `points` Ucodes in
/// every interval's one constellation, at the rate `drn` names.
pub(super) fn cpt_of(drn: u8, points: std::ops::RangeInclusive<u8>) -> Cp {
    let mask = points.fold(0u128, |m, u| m | 1 << u);
    Cp {
        data_mode: false,
        drn,
        redundancy: crate::v90::sign::Redundancy::None,
        trn1d_ratio: 1 << 13,
        intervals: [0; INTERVALS],
        constellations: vec![mask],
        ..Cp::default()
    }
}

/// A CPu as a far end sends one in data mode: `sets` constellations, each a
/// dense run of Ucodes, and the interval index naming which of them each of
/// the six constellation frames uses. `drn 18` is 38 bits a frame, which is
/// what these masks carry.
pub(super) fn cpu_of(sets: &[std::ops::RangeInclusive<u8>], intervals: [u8; INTERVALS]) -> Cp {
    Cp {
        data_mode: true,
        drn: 18,
        intervals,
        constellations: sets
            .iter()
            .map(|r| (*r.start()..=*r.end()).fold(0u128, |mask, u| mask | 1 << u))
            .collect(),
        ..Cp::default()
    }
}

/// The modem in data mode on that CPu: `B1d`'s own encoder is built, so
/// `renegotiation_points` reads the constellation on the line and not a
/// description of one.
pub(super) fn in_data_mode(cpu: Cp) -> Modem {
    let mut modem = Modem::default();
    modem.cpu = Some(cpu);
    modem.begin_data();
    assert!(modem.downstream.is_some(), "no B1d mapping was built from that CPu");
    modem
}

/// **`Rd`** — V.90 8.6.4 (`v90.txt:1841-1842`): "**Rd** denotes signal R
/// using the highest power PCM codeword from the **data mode** constellation
/// of each data frame interval as passed in **CP**".
///
/// The CP that carries the digital modem's data mode constellations is the
/// CPu (`v92.txt:1474`, and 8.8.4's `Rf` reads its own level out of it by
/// name, `v92.txt:1961-1962`), and `begin_data` builds `B1d`'s encoder out
/// of exactly that. So the test drives a known CPu through the modem and
/// compares `Rd` against the level table the encoder is already using.
///
/// This is the test the -66 dBFS `Rd` of `runs-216/170104` would have
/// failed: it was Ucode 2, linear 16, because the level came out of this
/// end's own CPd -- the **upstream** constellation, whose points are
/// `V92_LEVELSET` level indices -- instead of the downstream one.
#[test]
fn rd_is_the_highest_power_codeword_of_the_data_mode_constellation() {
    let intervals = [0u8, 1, 0, 1, 0, 1];
    let cpu = cpu_of(&[3..=95, 3..=88], intervals);
    let mut modem = in_data_mode(cpu.clone());
    let (data, _) = modem.renegotiation_points();

    // Interval by interval: the loudest Ucode of the constellation that
    // interval transmits on, read off the CPu's own masks ...
    let expected: Vec<u8> = (0..DOWN_INTERVALS)
        .map(|i| *cpu.points(i % INTERVALS).last().expect("a mask with points"))
        .collect();
    assert_eq!(data.to_vec(), expected, "Rd is not the data mode constellation's loudest point");
    // ... and off the level table the encoder on the line is sliced from.
    let table: Vec<u8> = (0..DOWN_INTERVALS)
        .map(|i| {
            modem
                .downstream
                .as_ref()
                .expect("B1d's encoder")
                .mapping()
                .sets[i % INTERVALS]
                .points()
                .iter()
                .copied()
                .max()
                .expect("a set with points")
        })
        .collect();
    assert_eq!(data.to_vec(), table, "Rd does not agree with the encoder B1d goes out on");

    // And it reaches the line at that codeword's level: 95 is Table 1's
    // 7932, not the 16 that went out at 24.998 s in `runs-216/170104`.
    let law = modem.settings.law;
    assert_eq!(ucode::linear(law, 95), 7932, "Ucode 95 is not the codeword measured on the line");
    let training = modem.renegotiation_points().1;
    modem.source.arm_renegotiation(data, training);
    let levels = r_stream(&mut modem.source, Out::Rd, R_FRAMES * INTERVALS);
    for (n, level) in levels.iter().enumerate() {
        let want = ucode::level(law, data[n % DOWN_INTERVALS]);
        assert!((level.abs() - want).abs() < 1e-12, "Rd symbol {n} is {level}, not {want}");
    }
    let loudest = levels.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!((loudest - ucode::level(law, 95)).abs() < 1e-12, "Rd's loudest symbol is {loudest}");
    assert!((loudest - ucode::level(law, 2)).abs() > 1e-6, "Rd is still the CPd reading's Ucode 2");
}

/// **`Ri`** — V.90 8.6.4 (`v90.txt:1844`): "**Ri** denotes signal R using
/// the single PCM codeword whose Ucode is U_INFO for all data frame
/// intervals". A codeword, not a constellation: arming 9.8 with the loudest
/// data mode points must not move it.
#[test]
fn ri_is_the_u_info_codeword_in_every_interval() {
    let law = Law::Mu;
    // U_INFO 78 is what this bench's far end put in its INFO1a: Table 1's
    // linear 3772, the magnitude `Ri` was measured at in
    // `runs-216/170104` and `runs-216/165935`.
    assert_eq!(ucode::linear(law, 78), 3772);
    let mut source = Source::new(&settings_uinfo(78));
    source.change(Out::B1d);
    let mut emitted: Vec<f64> = (0..7).map(|_| source.next()).collect();
    assert_eq!(source.out(), Out::B1d);
    // 9.8's codewords are armed and are **not** what Ri is made of.
    source.arm_renegotiation([95; DOWN_INTERVALS], [88; DOWN_INTERVALS]);
    source.ri_opener();
    while source.out() != Out::Trn2d {
        emitted.push(source.next());
        assert!(emitted.len() < 1000, "Ri never ended");
    }
    let start = 12;
    let ri = &emitted[start..start + R_FRAMES * INTERVALS];
    assert_eq!(ri.len(), 384);
    let want = ucode::level(law, 78);
    for (i, level) in ri.iter().enumerate() {
        assert!((level.abs() - want).abs() < 1e-12, "Ri symbol {i} is {level}, not U_INFO");
        assert_eq!(*level > 0.0, i % INTERVALS < 3, "Ri symbol {i} is not + + + - - -");
    }
    let bar = &emitted[start + R_FRAMES * INTERVALS..start + R_FRAMES * INTERVALS + R_BAR_FRAMES * INTERVALS];
    assert_eq!(bar.len(), 24, "R-bar-i is not 24T");
    for level in bar {
        assert!((level.abs() - want).abs() < 1e-12, "R-bar-i is {level}, not U_INFO");
    }
    // Three sources, three levels: U_INFO here is neither Rd's nor Rt's.
    assert!((want - ucode::level(law, 95)).abs() > 1e-6);
    assert!((want - ucode::level(law, 88)).abs() > 1e-6);
}

/// **`Rt`** — V.90 8.6.4 (`v90.txt:1847-1848`): "**Rt** denotes signal R
/// using the highest power PCM codeword from the **training** constellation
/// of each data frame interval as passed in **CPt**". CPt and CPu are not
/// the same constellation and this end sends both, so the two arrays must
/// differ whenever the messages do.
#[test]
fn rt_is_the_largest_codeword_of_the_training_constellation_not_the_data_mode_one() {
    let law = Law::Mu;
    let mut modem = in_data_mode(cpu_of(&[3..=95, 3..=88], [0, 1, 0, 1, 0, 1]));
    assert!(modem.source.set_cpt(&cpt_of(16, 72..=88)), "CPt of 17 points at drn 16 was refused");
    let (data, training) = modem.renegotiation_points();
    // CPt's loudest point in every interval, and CPu's in its own.
    assert_eq!(training, [88u8; DOWN_INTERVALS], "Rt is not CPt's loudest codeword");
    assert_eq!(data, [95, 88, 95, 88, 95, 88, 95, 88, 95, 88, 95, 88], "Rd moved with CPt");
    assert_ne!(data, training, "Rd and Rt came out the same array");
    // 88 is Table 1's 6140: the level the Phase 4 modulation actually went
    // out at in `runs-216/170104` after the opener, and **not** the data
    // mode's 7932.
    assert_eq!(ucode::linear(law, 88), 6140);

    let mut source = modem.source;
    source.arm_renegotiation(data, training);
    let levels = r_stream(&mut source, Out::Rt, R_FRAMES * INTERVALS);
    for (n, level) in levels.iter().enumerate() {
        let want = ucode::level(law, training[n % DOWN_INTERVALS]);
        assert!((level.abs() - want).abs() < 1e-12, "Rt symbol {n} is {level}, not {want}");
    }
    let loudest = levels.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!((loudest - ucode::level(law, 88)).abs() < 1e-12, "Rt's loudest symbol is {loudest}");
}
}

#[cfg(test)]
mod conformance_222 {
    use super::*;

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

    /// One level a symbol, labelled with the signal it belongs to.
    fn labelled(source: &mut Source, count: usize) -> Vec<(Out, f64)> {
        (0..count)
            .map(|_| {
                let level = source.next();
                (source.out(), level)
            })
            .collect()
    }

// ---------------------------------------------------------------------------
// docs/222: 9.9.1.2, the fast parameter exchange
// ---------------------------------------------------------------------------

    fn cpt(drn: u8, points: std::ops::RangeInclusive<u8>) -> Cp {
        let mask = points.fold(0u128, |m, u| m | 1 << u);
        Cp {
            data_mode: false,
            drn,
            redundancy: crate::v90::sign::Redundancy::None,
            trn1d_ratio: 1 << 13,
            intervals: [0; INTERVALS],
            constellations: vec![mask],
            ..Cp::default()
        }
    }

    /// A CPu over one contiguous set of codewords -- the data mode's own
    /// constellation, which is what `Rf` and a fast parameter exchange's `SUVd`
    /// both have to be built from.
    fn cpu_over(points: std::ops::RangeInclusive<u8>) -> Cp {
        let mask = points.fold(0u128, |m, u| m | 1 << u);
        Cp {
            data_mode: true,
            drn: 3,
            intervals: [0; INTERVALS],
            constellations: vec![mask],
            ..Cp::default()
        }
    }


/// [quoted] 8.8.5 (`v92.txt:1963-1966`): `SUVd` is sent on "the corresponding
/// TRN2d modulation" during Training and Rate Renegotiation, but "**During Fast
/// Parameter Exchange it is transmitted using the preceding data mode
/// modulation**". Fails on the code before `Source::begin_fpe`: `SUVd` went out
/// on CPt's constellation whatever procedure was in flight.
#[test]
fn s_8_8_5_suvd_during_a_fast_parameter_exchange_leaves_on_the_data_mode_modulation() {
    let law = Law::Mu;
    // CPt: the training constellation, a Phase 4 coder.
    // CPt: a *quieter* training constellation, so that a sequence on CPt's
    // modulation and one on CPu's cannot be confused.
    let training = cpt(3, 64..=71);
    // CPu: the data mode constellation B1d is going out on.
    let data = cpu_over(88..=95);
    let bits: Vec<bool> = (0..64).map(|i| i % 3 == 0).collect();

    let loudest = |source: &mut Source| -> f64 {
        source.send(Out::Suvd, bits.clone());
        let stream = labelled(source, 96);
        stream
            .iter()
            .filter(|(out, _)| *out == Out::Suvd)
            .fold(0.0f64, |m, (_, level)| m.max(level.abs()))
    };

    // The data mode's loudest codeword, straight out of CPu's own mapping: the
    // same resolution `renegotiation_points` makes for `Rf`.
    let data_loudest = {
        let mapping = CodeMapping::from_cp(&data).expect("CPu mapping");
        mapping
            .sets
            .iter()
            .flat_map(|set| set.points().iter().copied())
            .map(|ucode| ucode::level(law, ucode).abs())
            .fold(0.0f64, f64::max)
    };

    // Without a fast parameter exchange, CPt is the corresponding modulation.
    let mut phase4 = Source::new(&settings());
    assert!(phase4.set_cpt(&training), "CPt");
    let without = loudest(&mut phase4);
    let training_loudest = {
        let mapping = CodeMapping::from_cp(&training).expect("CPt mapping");
        mapping
            .sets
            .iter()
            .flat_map(|set| set.points().iter().copied())
            .map(|ucode| ucode::level(law, ucode).abs())
            .fold(0.0f64, f64::max)
    };
    assert!(
        (without - training_loudest).abs() < 1e-9 && (without - data_loudest).abs() > 1e-9,
        "SUVd off the FPE path is CPt's {training_loudest}, not the data mode's {data_loudest}: {without}"
    );

    // On it, the preceding data mode modulation is -- and 9.9.1.2.3's resets.
    let mut fpe = Source::new(&settings());
    assert!(fpe.set_cpt(&training), "CPt");
    assert!(fpe.begin_fpe(&data), "CPu mapping");
    let during = loudest(&mut fpe);
    assert!(
        (during - data_loudest).abs() < 1e-9,
        "8.8.5: SUVd during a fast parameter exchange is the data mode modulation, \
         whose loudest is {data_loudest}, and it went out at {during}"
    );
    assert_ne!(
        during,
        ucode::level(law, 88),
        "SUVd during a fast parameter exchange went out on CPt's constellation -- 8.8.5"
    );
    fpe.end_fpe();
}

/// 9.9.1.2.1/.2's patterns, **generated from Table 25/26's `K_i` rows**.
#[test]
fn s_9_9_1_2_1_the_rm_patterns_are_tables_25_and_26() {
    // Table 25 (`v92.txt:1630-1647`): `K_i = M_i - 1` for i in {0,1,4,5,8,9},
    // `K_i = 0` for i in {2,3,6,7,10}, u11 = 0.
    let table25 = [
        true, true, false, false, true, true, false, false, true, true, false, false,
    ];
    assert_eq!(RM_K_LOUD, table25, "Table 25 is {{max, max, min, min}} three times");
    // Table 26 (`v92.txt:1654-1667`): `K_i = 0` for i in {0,1,4,5,8,9} and
    // `K_i = M_i - 1` for i in {2,3,6,7,10,11}.
    assert_eq!(RM_BAR_K_LOUD, table25.map(|loud| !loud), "Table 26 is Table 25 inverted");
}

/// 9.9.1.2.1 and 9.9.1.2.2's two detections, with their latency in **symbols**.
#[test]
fn s_9_9_1_2_1_rm_is_detected_from_table_25_and_the_transition_from_table_26() {
    // The ends of the constellation the far end's CPu declared, as levels.
    let levels = [1000.0, 2000.0, 3000.0, 9000.0, 10_000.0];
    let mut detector = RmDetector::new(&levels).expect("a table to measure against");
    let loud = 10_000.0;
    let quiet = 1000.0;
    let mut stream: Vec<f64> = Vec::new();
    // 384T of RM is Table 25 repeated 32 times; 24T of R-bar-M is Table 26
    // repeated twice. Eight repetitions here is enough for both the detection
    // and the transition, and the lengths that matter are the detector's.
    for _ in 0..8 {
        stream.extend(RM_K_LOUD.iter().map(|l| if *l { loud } else { quiet }));
    }
    for _ in 0..2 {
        stream.extend(RM_BAR_K_LOUD.iter().map(|l| if *l { loud } else { quiet }));
    }
    let transition = 8 * RM_K_LOUD.len();
    let mut seen: Vec<(usize, RmEvent)> = Vec::new();
    for (i, level) in stream.iter().enumerate() {
        // The far end's signs are its own; 8.8.4's pattern is not RM's, so a
        // negative sign must not matter to a magnitude detector.
        let level = if i % 2 == 0 { *level } else { -*level };
        match detector.feed(level) {
            RmEvent::None => {}
            event => seen.push((i, event)),
        }
    }
    assert_eq!(
        seen,
        vec![
            (RM_SYMBOLS - 1, RmEvent::RmDetected),
            (transition + RM_BAR_SYMBOLS - 1, RmEvent::RmToRmBar)
        ],
        "RM detected {} symbols in, RM-to-R-bar-M {} symbols after the transition",
        RM_SYMBOLS,
        RM_BAR_SYMBOLS
    );
}

/// The guard: a healthy data mode carries no `RM` and the detector stays quiet.
#[test]
fn no_rm_in_a_healthy_data_mode_fires_the_9_9_detector() {
    let levels = [1000.0, 2000.0, 3000.0, 9000.0, 10_000.0];
    let mut detector = RmDetector::new(&levels).expect("a table");
    // A data mode's symbols, everywhere in the constellation including both
    // ends, on a plain LCG: no RM, and nothing here may ask for Rf.
    let mut state = 0x2545_f491_4f6c_dd1du64;
    for _ in 0..20_000 {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let pick = (state >> 33) as usize % levels.len();
        let level = levels[pick] * if (state >> 20) & 1 == 0 { 1.0 } else { -1.0 };
        assert_eq!(detector.feed(level), RmEvent::None, "a healthy data mode fired 9.9");
    }
}

/// [quoted] 9.9.1.2.2: `Rf` 384T and `R-bar-f` 24T, the sign pattern read off
/// `v92.txt:1957-1961`, the codeword off `CPu`'s loudest, all on the grid.
#[test]
fn s_9_9_1_2_2_rf_is_384t_and_r_bar_f_is_24t_on_a_data_frame_grid() {
    let mut source = Source::new(&settings());
    // `docs/218`'s resolution: one codeword an interval, the loudest of the
    // data mode constellation CPu passed. Distinct values here so that the
    // interval each symbol came from is checkable.
    let data: [u8; DOWN_INTERVALS] = std::array::from_fn(|i| 60 + i as u8);
    let training = [79u8; DOWN_INTERVALS];
    source.arm_renegotiation(data, training);
    source.change(Out::Rf);
    let stream = labelled(&mut source, R_FRAMES * INTERVALS + R_BAR_FRAMES * INTERVALS + 12);

    let rf: Vec<f64> = stream
        .iter()
        .filter(|(out, _)| *out == Out::Rf)
        .map(|(_, level)| *level)
        .collect();
    let bar: Vec<f64> = stream
        .iter()
        .filter(|(out, _)| *out == Out::RfBar)
        .map(|(_, level)| *level)
        .collect();
    assert_eq!(rf.len(), R_FRAMES * INTERVALS, "Rf is 384T");
    assert_eq!(bar.len(), R_BAR_FRAMES * INTERVALS, "R-bar-f is 24T");
    // "Rf shall begin on the boundary of a data frame" (9.9.1.2.2).
    assert_eq!(stream[0].0, Out::Rf, "Rf did not begin where the source was asked for it");
    // 8.8.4's sign pattern, from the source line: `+ + - - + + - - + + - -`
    // with "the left-most sign ... transmitted first", and R-bar-f's
    // `- - + + - - + + - - + +` -- the same codewords, inverted.
    let signs = |levels: &[f64]| -> Vec<bool> { levels.iter().map(|l| *l > 0.0).collect() };
    let rf_signs = signs(&rf);
    let bar_signs = signs(&bar);
    let rf_12 = [
        true, true, false, false, true, true, false, false, true, true, false, false,
    ];
    assert_eq!(&rf_signs[..12], &rf_12, "Rf's twelve-symbol sign pattern");
    assert_eq!(
        &bar_signs[..12],
        &rf_12.map(|s| !s),
        "R-bar-f is 2 repetitions of the 12-symbol inverted pattern"
    );
    // The pattern is period four and it never slips: all 384T of Rf.
    for (i, sign) in rf_signs.iter().enumerate() {
        assert_eq!(*sign, rf_12[i % 12], "Rf symbol {i} left 8.8.4's pattern");
    }
    // And the codewords are the ones CPu named, one an interval, unchanged by
    // the bar: "the highest power PCM codeword from the data mode constellation
    // of each data frame interval as passed in CPu" (8.8.4).
    for (i, level) in rf.iter().enumerate() {
        let interval = i % DOWN_INTERVALS;
        let expected = ucode::level(Law::Mu, data[interval]).abs();
        assert!(
            (level.abs() - expected).abs() < 1e-9,
            "Rf symbol {i} is {} and interval {interval}'s loudest is {expected}",
            level.abs()
        );
    }
}

/// 9.9.1.2.3: the tail is `SUVd` and 9.6.1.1.2 -- **no `TRN2d` on this path**.
#[test]
fn s_9_9_1_2_3_the_tail_begins_at_suvd_with_no_trn2d_before_it() {
    let (mut tail, mut exchange) = RenegTail::from_suvd(0.0, FS);
    assert_eq!(tail.stage(), RenegStage::Suvd, "9.9.1.2.3 begins at SUVd");
    let mut source = Source::new(&settings());
    let data = [90u8; DOWN_INTERVALS];
    source.arm_renegotiation(data, data);
    // The SUVd 9.9.1.2.3 asks for goes out with bit 32 clear: no silent period
    // is entered, so the exchange never asks for Ed-and-silence here.
    let asked = tail.push(&mut source, &mut exchange, None, 0);
    assert_eq!(asked, Some(Out::Suvd), "9.9.1.2.3's first sequence is SUVd");
    assert!(!exchange.silence_requested(), "9.9.1.2.3 sets bit 32 clear");
    assert!(!exchange.peer_asked_silence());
}}

/// `docs/221`: the differential harness and the property tests behind it.
///
/// The claim being tested is `v92.txt:339-341`'s -- "The data signalling
/// rates, symbol rate, scrambler and encoder for the digital modem shall be
/// the same as those given in clause 5/V.90" -- plus its ten single-sentence
/// imports. It is a claim about a signal **repertoire**, and it is false about
/// the signal **stream**, so the diffable set is closed by construction:
/// [`DIFFABLE`] names every signal the harness diffs, and every signal it does
/// not is named in [`EXCLUSIONS`] with the clause line that excludes it. There
/// is no unexplained skip.
///
/// The V.90 side of each diff is the V.90 **transmitter itself**, through
/// `crate::v90::digital::probe` (`v90/digital.rs`, test-only), not a
/// re-derivation of it: a re-derivation would agree with a V.92 bug that the
/// V.90 transmitter does not have.
///
/// **The oracle's limits, stated where they are used.** The V.90 path is a
/// valid oracle for codeword *ordering* (`v90.txt:654-656`, "label 0 ...
/// largest") and for the level table (`ucode::level`, Table 1). It is **not**
/// an oracle for constellation *legality*: neither path enforces V.90's Table
/// 15 average-power ceiling (`v90.txt:1649-1671`) anywhere, and V.92 has no
/// such clause at all, so no test here may conclude from "the V.90 path did
/// the same thing" that a constellation we were handed is legal to send.
#[cfg(test)]
mod conformance_221 {
    use super::conformance_218::{cpt_of, cpu_of as cpu_218, in_data_mode, r_stream, settings_uinfo};
    use super::*;
    use crate::v90::digital::probe::Tx as V90Tx;

    /// The signals this harness diffs, and the clause each is imported by.
    ///
    /// One row per signal: `(name, clause)`. A signal in this table is
    /// asserted equal, symbol for symbol, between the two transmitters.
    pub(super) const DIFFABLE: [(&str, &str); 8] = [
        ("Sd", "v92.txt:1445-1446 -> v90.txt:1509-1516 (8.4.4)"),
        ("Trn1d", "v92.txt:1448-1449 -> v90.txt:1520-1525 (8.4.5)"),
        ("Dil", "v92.txt:1349-1350 -> v90.txt:1424-1465 (8.4.1)"),
        ("Ri", "v92.txt:1436-1437 -> v90.txt:1844 (8.6.4)"),
        ("Rd", "v92.txt:1956 -> v90.txt:1841-1842 (8.6.4)"),
        ("Rt", "v92.txt:1956 -> v90.txt:1847-1848 (8.6.4)"),
        ("Trn2d", "v92.txt:2000-2001 -> v90.txt:1853-1859 (8.6.5)"),
        ("Ed", "v92.txt:1758-1762 -> v90.txt:1741-1744 (8.6.2), FPE excluded (E7)"),
    ];

    /// The closed exception set: every signal the diff must **not** contain,
    /// and the clause line that puts it out. A reader who wonders why `Jd` or
    /// `SCR` is missing from [`DIFFABLE`] finds the answer here rather than a
    /// gap.
    pub(super) const EXCLUSIONS: [(&str, &str, &str); 9] = [
        ("E1 Jd", "v92.txt:1371-1372 vs v90.txt:1492-1494", "bits 47/48 redefined: Jd/Jp identifier and a reserved bit, against constellation size and rate-renegotiation"),
        ("E2 Jd-prime", "v90.txt:1503-1507", "deleted from V.92 8.6; V.90 always sends it after Jd"),
        ("E3 Jp/Jp-prime", "v92.txt:1391-1435", "no V.90 downstream counterpart exists"),
        ("E4 SCR", "v92.txt:1436-1444", "V.92-only downstream signal; V.90's SCR is the analogue modem's (v90.txt:1411-1415)"),
        // E5 is the one exception whose signal *does* have a transmit path in
        // this tree (`Out::Rf`/`Out::RfBar`): it stays excluded from the diff
        // because V.90 has no `Rf` (v92.txt:1957) to diff it against, and it is
        // pinned instead by
        // `c_8_8_4_rf_is_the_loudest_codeword_of_the_data_mode_constellation_as_passed_in_cpu`.
        ("E5 Rf", "v92.txt:1957-1962", "V.92-only: 8.8.4 defines it, V.90 has no Rf at all, so there is no V.90 stream to diff; pinned by c_8_8_4_rf_* instead"),

        ("E6 CPd/SUVd", "v92.txt:1763-1843, 1964-1998", "V.92-only; they replace V.90's MP (v90.txt:1746-1830)"),
        ("E7 Ed transmitted during Fast Parameter Exchange", "v92.txt:1758-1762", "FPE puts Ed on the preceding data mode modulation; the diff covers the training framing only"),
        ("E8 Ri/R-bar-i placement and Ucode 0 silence", "v92.txt:2557-2562, 2971-2975", "Ri is at the end of Phase 3 and the silence is V.92-only; Ri's *symbols* are diffed, its *position* cannot be"),
        ("E9 power ceiling", "v92.txt 0 hits vs v90.txt:1649-1671", "no downstream average-power limit in V.92; not a signal, and not enforced by our V.90 path either"),
    ];

    /// The table is closed and every entry says why: diffed implies not
    /// excluded, excluded implies a clause line and a reason, and no signal is
    /// in both.
    #[test]
    fn the_diffable_set_and_the_exception_set_are_closed_and_disjoint() {
        for (name, clause) in DIFFABLE {
            assert!(clause.contains("v90.txt:"), "{name} is diffed without naming the V.90 clause");
            assert!(
                !EXCLUSIONS.iter().any(|(e, _, _)| {
                    // The tag is "En signal": the signal is what follows the
                    // number, and nothing shorter than it, so that "Ed
                    // transmitted during Fast Parameter Exchange" does not
                    // collide with the diffed "Ed".
                    e.split_once(' ').map(|(_, rest)| rest).unwrap_or(e) == name
                }),
                "{name} is both diffed and excluded"
            );
        }
        for (tag, clause, why) in EXCLUSIONS {
            assert!(
                clause.contains("v92.txt") || clause.contains("v90.txt"),
                "{tag} is excluded without a clause line"
            );
            assert!(!why.trim().is_empty(), "{tag} is excluded without a reason");
        }
        // The nine of docs/219 §2, and no more: an exclusion added here to
        // hide a disagreement must be a clause change, not an edit.
        assert_eq!(EXCLUSIONS.len(), 9);
        assert_eq!(DIFFABLE.len(), 8);
    }

    const TRN1D_SYMBOLS: usize = 96;
    const SD_SYMBOLS: usize = (SD_FRAMES + SD_BAR_FRAMES) * INTERVALS;
    const B1D_SYMBOLS: usize = B1D_FRAMES * INTERVALS;

    fn settings() -> Settings {
        settings_uinfo(78)
    }

    /// The two transmitters, started at the same place.
    fn pair() -> (Source, V90Tx) {
        let s = settings();
        (Source::new(&s), V90Tx::new(s.law, s.uinfo, s.habits.trn1d))
    }

    /// **D4** `Sd` and `S-bar-d`, `v92.txt:1445-1446` -> `v90.txt:1509-1516`.
    #[test]
    fn d4_sd_and_sd_bar_are_the_same_stream_on_both_transmitters() {
        let (mut v92, mut v90) = pair();
        v92.change(Out::Sd);
        assert!(v90.change("Sd"));
        let a = (0..SD_SYMBOLS).map(|_| v92.next()).collect::<Vec<f64>>();
        let b = v90.symbols(SD_SYMBOLS);
        assert_eq!(a.len(), b.len());
        assert_eq!(a, b, "8.4.4's Sd/S-bar-d differ between the two transmitters");
    }

    /// **D5** `TRN1d`, `v92.txt:1448-1449` -> `v90.txt:1520-1525`.
    #[test]
    fn d5_trn1d_is_the_same_stream_on_both_transmitters() {
        let (mut v92, mut v90) = pair();
        v92.change(Out::Trn1d);
        assert!(v90.change("Trn1d"));
        let a = (0..TRN1D_SYMBOLS).map(|_| v92.next()).collect::<Vec<f64>>();
        let b = v90.symbols(TRN1D_SYMBOLS);
        assert_eq!(a, b, "8.4.5's TRN1d differs between the two transmitters");
    }

    /// **D6** `Ri` and `R-bar-i`, `v92.txt:1436-1437` -> `v90.txt:1844`.
    ///
    /// The *symbols*, not the placement: V.92 moves `Ri` to the end of Phase 3
    /// (`v92.txt:2557-2562`), which is E8 and is not diffable.
    #[test]
    fn d6_ri_and_ri_bar_are_the_same_stream_on_both_transmitters() {
        let (mut v92, mut v90) = pair();
        v92.change(Out::Ri);
        assert!(v90.change("Ri"));
        let ri: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| v92.next()).collect();
        assert_eq!(ri, v90.symbols(R_FRAMES * INTERVALS), "8.6.4's Ri differs");
        let (mut v92, mut v90) = pair();
        v92.change(Out::RiBar);
        assert!(v90.change("RiBar"));
        let bar: Vec<f64> = (0..R_BAR_FRAMES * INTERVALS).map(|_| v92.next()).collect();
        assert_eq!(bar, v90.symbols(R_BAR_FRAMES * INTERVALS), "8.6.4's R-bar-i differs");
    }

    /// **D7** `Rd`, `v92.txt:1956` -> `v90.txt:1841-1842`.
    ///
    /// `Rt` is in [`DIFFABLE`] too but the V.90 transmitter has no `Out::Rt`
    /// (V.90 sends `Rt` only as 9.4.1.4's `R` in Phase 3, which this test
    /// harness does not drive), so `Rt` is diffed against the V.90 mapping --
    /// `Mapping::from_cp(CPt)`, whose loudest point it must be -- in
    /// [`c_8_6_4_rt_is_the_loudest_codeword_of_the_training_constellation`].
    #[test]
    fn d7_rd_is_the_same_stream_on_both_transmitters() {
        for (sets, intervals) in [
            (vec![3..=95u8, 3..=88u8], [0u8, 1, 0, 1, 0, 1]),
            (vec![40..=95u8, 41..=95, 42..=95], [0, 1, 2, 0, 1, 2]),
        ] {
            let cpu = cpu_218(&sets, intervals);
            let mut modem = in_data_mode(cpu.clone());
            let (data, training) = modem.renegotiation_points();
            modem.source.arm_renegotiation(data, training);
            modem.source.change(Out::Rd);
            let a: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| modem.source.next()).collect();

            let s = settings();
            let mut v90 = V90Tx::new(s.law, s.uinfo, s.habits.trn1d);
            assert!(v90.set_cpu(&cpu));
            assert!(v90.change("Rd"));
            let b = v90.symbols(R_FRAMES * INTERVALS);
            assert_eq!(a, b, "8.6.4's Rd differs from the V.90 transmitter's: {sets:?}");
        }
    }

    /// **D1** `B1d`, `v92.txt:1755-1756` -> `v90.txt:1731-1739`.
    ///
    /// The V.90 transmitter maps `B1d`'s 48 data frames itself; this end takes
    /// octets that the mapper has already chosen (3.6). So the diff hands the
    /// V.92 side the V.90 side's own codewords and demands the same samples and
    /// the same length, which is where a sign convention error (`from_octet`'s
    /// inverse read as "positive") shows up on the first symbol.
    #[test]
    fn d1_b1d_is_the_same_stream_on_both_transmitters() {
        for (sets, intervals) in [
            (vec![3..=95u8, 3..=88u8], [0u8, 1, 0, 1, 0, 1]),
            (vec![40..=95u8, 41..=95, 42..=95], [0, 1, 2, 0, 1, 2]),
        ] {
            let cpu = cpu_218(&sets, intervals);
            let s = settings();
            let mut v90 = V90Tx::new(s.law, s.uinfo, s.habits.trn1d);
            assert!(v90.set_cpu(&cpu));
            assert!(v90.change("B1d"));
            let want = v90.symbols(B1D_SYMBOLS);
            assert_eq!(want.len(), 48 * INTERVALS, "8.6.1: B1d is 48 data frames");

            let mut v92 = Source::new(&s);
            v92.change(Out::B1d);
            for level in &want {
                let (u, negative) = ucode::nearest(s.law, (level * 32768.0).round() as i32);
                v92.codes.push_back(ucode::octet(s.law, u, negative));
            }
            let got: Vec<f64> = (0..B1D_SYMBOLS).map(|_| v92.next()).collect();
            assert_eq!(got, want, "8.6.1's B1d differs between the two transmitters");
        }
    }

    /// **D2** `TRN2d`, `v92.txt:2000-2001` -> `v90.txt:1853-1859`.
    #[test]
    fn d2_trn2d_is_the_same_stream_on_both_transmitters() {
        let cpt = cpt_of(16, 72..=88);
        let s = settings();
        let mut v92 = Source::new(&s);
        assert!(v92.set_cpt(&cpt));
        v92.change(Out::Trn2d);
        let a: Vec<f64> = (0..60).map(|_| v92.next()).collect();

        let mut v90 = V90Tx::new(s.law, s.uinfo, s.habits.trn1d);
        assert!(v90.set_cpt(&cpt));
        assert!(v90.change("Trn2d"));
        let b = v90.symbols(60);
        assert_eq!(a, b, "8.6.5's TRN2d differs between the two transmitters");
    }

    /// **D3** `Ed` in the training/renegotiation framing, `v92.txt:1758-1762`
    /// -> `v90.txt:1741-1744`. E7 excludes the FPE framing.
    #[test]
    fn d3_ed_is_the_same_stream_on_both_transmitters_outside_fpe() {
        let cpt = cpt_of(16, 72..=88);
        let s = settings();
        let d = CodeMapping::from_cp(&cpt).expect("a CPt that fits its rate").frame_bits();
        let mut v92 = Source::new(&s);
        assert!(v92.set_cpt(&cpt));
        v92.change(Out::Trn2d);
        let _: Vec<f64> = (0..60).map(|_| v92.next()).collect();
        v92.last = vec![false; 2 * d];
        v92.change(Out::Ed);
        let a: Vec<f64> = (0..2 * INTERVALS).map(|_| v92.next()).collect();

        let mut v90 = V90Tx::new(s.law, s.uinfo, s.habits.trn1d);
        assert!(v90.set_cpt(&cpt));
        assert!(v90.change("Trn2d"));
        let _ = v90.symbols(60);
        assert!(v90.change("Ed"));
        let b = v90.symbols(2 * INTERVALS);
        assert_eq!(a, b, "8.6.2's Ed differs between the two transmitters");
    }

    /// **D9** `DIL`, `v92.txt:1349-1350` -> `v90.txt:1424-1465`.
    ///
    /// The descriptor's symbols are built once, by `crate::v90::sequences::
    /// Descriptor::symbols` -- the same function `v90/digital.rs:1062` fills
    /// the V.90 transmitter with -- so the diff is that the V.92 source puts
    /// exactly those (Ucode, sign) pairs on the line and ends the pass where
    /// the descriptor says a segment ends.
    #[test]
    fn d9_dil_is_the_descriptors_own_symbols() {
        let mut descriptor = Descriptor::none();
        descriptor.signs = vec![true, false, true, true, false, false];
        descriptor.training = vec![false; 6];
        descriptor.h = [0, 2, 0, 0, 0, 0, 0, 0];
        descriptor.refs = [0, 60, 0, 0, 0, 0, 0, 0];
        descriptor.ucodes = vec![0, 32, 16, 0, 0, 0, 0, 0, 0];
        let want: Vec<f64> = descriptor
            .symbols()
            .map(|(u, positive)| ucode::level(Law::Mu, u) * if positive { 1.0 } else { -1.0 })
            .collect();
        assert!(!want.is_empty(), "8.4.1: this descriptor has no DIL at all");
        let mut source = Source::new(&settings());
        source.dil = descriptor.symbols().collect();
        source.change(Out::Dil);
        let got: Vec<f64> = (0..want.len()).map(|_| source.next()).collect();
        assert_eq!(got, want, "8.4.1's DIL symbols are not the descriptor's");
    }

    /// **D8** the `drn` table, `v92.txt:1492-1495` = `v90.txt:1580-1582`:
    /// "drn between 0 and 22 ... Data signalling rate = (drn + 20) x 8000/6 in
    /// CP(u) and (drn + 8) x 8000/6 in CPt". One table, two callers.
    #[test]
    fn d8_the_drn_table_is_v90s_unchanged() {
        for drn in 1..=22u8 {
            let want = (u32::from(drn) + 20) * 8000 / 6;
            assert_eq!(crate::v90::sequences::data_rate(drn), Some(want), "drn {drn}");
            assert_eq!(crate::v90::sequences::data_bits(drn), usize::from(drn) + 20);
        }
        assert_eq!(crate::v90::sequences::data_rate(0), None, "drn 0 is cleardown");
        assert_eq!(crate::v90::sequences::data_rate(1), Some(28_000));
        assert_eq!(crate::v90::sequences::data_rate(22), Some(56_000));
    }
}

/// `docs/221` §2: the property tests, which catch the class whether or not a
/// signal is diffable.
///
/// Every one of them compares against **the level table the code itself
/// builds** for the constellation in force, driven by a known input, so that an
/// index read as a codeword and a `.last()` on a loudest-first list are both
/// mistakes a test makes red rather than mistakes a reader has to notice.
#[cfg(test)]
mod conformance_221_properties {
    use super::conformance_218::{cpu_of, cpt_of, in_data_mode, settings_uinfo};
    use super::*;

    /// A CPt of six *different* constellations, one per interval: the shape
    /// that makes "a point legal in all six" a real intersection.
    fn cpt_six(drn: u8) -> Cp {
        let sets: Vec<u128> = (0..INTERVALS)
            .map(|i| (u32::from(50 - i as u8)..=95u32).fold(0u128, |m, u| m | 1 << u))
            .collect();
        Cp {
            data_mode: false,
            drn,
            redundancy: crate::v90::sign::Redundancy::None,
            trn1d_ratio: 1 << 13,
            intervals: [0, 1, 2, 3, 4, 5],
            constellations: sets,
            ..Cp::default()
        }
    }

    /// 5.4.4/V.90 (`v90.txt:654-656`): the mapper's "label 0 ... largest".
    /// This is the one fact every level test below stands on, so it is asserted
    /// once, over every mapping this end constructs, rather than assumed.
    #[test]
    fn c_5_4_4_label_zero_is_the_largest_pcm_code() {
        let law = Law::Mu;
        let cps = [
            cpu_of(&[3..=95, 3..=88], [0, 1, 0, 1, 0, 1]),
            cpu_of(&[40..=95, 41..=95, 42..=95], [0, 1, 2, 0, 1, 2]),
            cpt_of(16, 72..=88),
            cpt_six(16),
        ];
        for cp in &cps {
            let mapping = CodeMapping::from_cp(cp).expect("a CP that fits its own rate");
            for (i, set) in mapping.sets.iter().enumerate() {
                let points = set.points();
                let first = *points.first().expect("a set with points");
                let loudest = points.iter().copied().max_by_key(|&u| ucode::linear(law, u)).unwrap();
                assert_eq!(first, loudest, "interval {i}: label 0 is not the largest code");
                let quietest = points.iter().copied().min_by_key(|&u| ucode::linear(law, u)).unwrap();
                if points.len() > 1 {
                    assert_ne!(*points.last().unwrap(), first, "interval {i}: first and last agree");
                    assert_eq!(quietest, *points.last().unwrap(), "interval {i}: last is not quietest");
                }
            }
        }
    }

    /// 8.6.4/V.90 (`v90.txt:1841-1842`): `Rd` is "the highest power PCM codeword
    /// from the data mode constellation of each data frame interval", and
    /// `docs/218`'s defect was a **level index** from this end's own CPd read as
    /// one of those codewords -- Ucode 2, linear 16, -66 dBFS.
    ///
    /// Driven by two CPu shapes so the property holds for the table the code
    /// builds and not for one vector, and the regression half asserts the
    /// emitted levels are points of that table at all.
    #[test]
    fn c_8_6_4_rd_is_the_loudest_codeword_of_the_data_mode_constellation() {
        let law = Law::Mu;
        for (sets, intervals) in [
            (vec![3..=95u8, 3..=88u8], [0u8, 1, 0, 1, 0, 1]),
            (vec![40..=95u8, 41..=95, 42..=95], [0, 1, 2, 0, 1, 2]),
        ] {
            let cpu = cpu_of(&sets, intervals);
            let mut modem = in_data_mode(cpu.clone());
            let (data, training) = modem.renegotiation_points();
            let mapping = modem.downstream.as_ref().expect("B1d's encoder").mapping().clone();
            for i in 0..DOWN_INTERVALS {
                let set = &mapping.sets[i % INTERVALS];
                let loudest = *set.points().first().expect("a set with points");
                assert_eq!(data[i], loudest, "interval {i}: Rd is not the loudest codeword");
                assert!(
                    set.points().contains(&data[i]),
                    "interval {i}: Rd's codeword is not a point of the constellation in force"
                );
            }
            // **Regression, docs/218**: a CPd *level index* (15 at LC 8) read as
            // a codeword gives Ucode 15/2, whose level is nowhere near.
            let wrong = ucode::level(law, 2);
            modem.source.arm_renegotiation(data, training);
            modem.source.change(Out::Rd);
            let levels: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| modem.source.next()).collect();
            assert!(
                !levels.iter().any(|l| (l.abs() - wrong).abs() < 1e-12),
                "Rd went out at the CPd level index again (docs/218)"
            );
        }
    }

    /// 8.6.4/V.90 (`v90.txt:1847-1848`): `Rt` is the same rule against the
    /// **training** constellation, CPt. **Regression, docs/218**: `.last()` on
    /// `Constellation::points()` -- which is sorted loudest first -- took the
    /// *quietest* codeword.
    #[test]
    fn c_8_6_4_rt_is_the_loudest_codeword_of_the_training_constellation() {
        for cpt in [cpt_of(16, 72..=88), cpt_six(16)] {
            let mut modem = in_data_mode(cpu_of(&[3..=95, 3..=88], [0, 1, 0, 1, 0, 1]));
            assert!(modem.source.set_cpt(&cpt));
            let (data, training) = modem.renegotiation_points();
            let mapping = modem.source.phase4.as_ref().expect("CPt's coder").coder.mapping().clone();
            for i in 0..DOWN_INTERVALS {
                let set = &mapping.sets[i % INTERVALS];
                let points = set.points();
                assert_eq!(training[i], points[0], "interval {i}: Rt is not the loudest codeword");
                if points.len() > 1 {
                    assert_ne!(training[i], *points.last().unwrap(), "interval {i}: Rt is the quietest");
                }
            }
            assert_ne!(data, training, "Rd and Rt came out the same array");
            modem.source.arm_renegotiation(data, training);
            modem.source.change(Out::Rt);
            let levels: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| modem.source.next()).collect();
            for (n, level) in levels.iter().enumerate() {
                let want = ucode::level(Law::Mu, training[n % DOWN_INTERVALS]);
                assert!((level.abs() - want).abs() < 1e-12, "Rt symbol {n} is {level}, not {want}");
            }
        }
    }

    /// 8.6.4/V.90 (`v90.txt:1844`): `Ri` is "the single PCM codeword whose Ucode
    /// is U_INFO for all data frame intervals". Pinned to U_INFO 78 = 3772, the
    /// magnitude measured on the line, and kept apart from both R arrays.
    #[test]
    fn c_8_6_4_ri_is_the_u_info_codeword_in_every_interval() {
        let law = Law::Mu;
        assert_eq!(ucode::linear(law, 78), 3772, "U_INFO 78 is not Table 1's 3772");
        let mut source = Source::new(&settings_uinfo(78));
        // 9.8's own codewords are armed and are not what Ri is made of.
        source.arm_renegotiation([95; DOWN_INTERVALS], [88; DOWN_INTERVALS]);
        source.change(Out::Ri);
        let levels: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| source.next()).collect();
        for (i, level) in levels.iter().enumerate() {
            let want = ucode::level(law, 78);
            assert!((level.abs() - want).abs() < 1e-12, "Ri symbol {i} is {level}, not U_INFO");
            assert_eq!(*level > 0.0, i % INTERVALS < 3, "Ri symbol {i} is not + + + - - -");
            assert!((level.abs() - ucode::level(law, 95)).abs() > 1e-9);
            assert!((level.abs() - ucode::level(law, 88)).abs() > 1e-9);
        }
    }

    /// 8.4.4/V.90 (`v90.txt:1509-1516`): `Sd` is 64 data frames of
    /// `{+W, +0, +W, -W, -0, -W}` with `W` the Ucode `16 + UINFO`, and
    /// `S-bar-d` is the same magnitudes turned over.
    ///
    /// `W` comes from the clause literally and not from a constellation -- that
    /// is what the clause says -- so what is *checked* here is that it is never
    /// quietly moved onto one, and *recorded* is whether it happens to be a
    /// point of the constellation in force. A `W` outside every constellation
    /// is a compliant `Sd` and an unreadable one; requiring membership would
    /// make a compliant transmitter fail.
    #[test]
    fn c_8_4_4_sd_is_w_and_zero() {
        let law = Law::Mu;
        for uinfo in [66u8, 70, 78, 85] {
            let mut source = Source::new(&settings_uinfo(uinfo));
            source.change(Out::Sd);
            let w = ucode::level(law, 16 + uinfo);
            let z = 0.0;
            let pattern = [w, z, w, -w, -z, -w];
            let sd: Vec<f64> = (0..SD_FRAMES * INTERVALS).map(|_| source.next()).collect();
            for (i, level) in sd.iter().enumerate() {
                let want = pattern[i % INTERVALS];
                assert!((level - want).abs() < 1e-12, "Sd symbol {i} at U_INFO {uinfo} is {level}");
            }
            let bar: Vec<f64> = (0..SD_BAR_FRAMES * INTERVALS).map(|_| source.next()).collect();
            for (i, level) in bar.iter().enumerate() {
                let want = -pattern[i % INTERVALS];
                assert!((level - want).abs() < 1e-12, "S-bar-d symbol {i} at U_INFO {uinfo} is {level}");
            }
            // W is `16 + UINFO` and never a point chosen for convenience.
            assert_eq!(ucode::linear(law, 16 + uinfo) as f64 / 32768.0, w);
            // Membership, recorded rather than required: `Sd` is Phase 3, so
            // the constellation in force is CPt's.
            let cpt = cpt_of(16, 40..=95);
            let in_cpt = cpt.points(0).contains(&(16 + uinfo));
            assert_eq!(
                in_cpt,
                (40..=95).contains(&(16 + uinfo)),
                "W's membership of CPt disagrees with the mask, U_INFO {uinfo}"
            );
        }
    }

    /// 8.6.1/V.90 (`v90.txt:1731-1739`): `B1d`'s codewords are the ones the
    /// data mode encoder produces. Every symbol of a `B1d` run must be a point
    /// of the constellation in force **for its interval**.
    ///
    /// This is the property that makes a U_INFO fallback in data mode
    /// impossible to write: U_INFO 78 is not in the CPu used here.
    #[test]
    fn c_8_6_1_no_b1d_symbol_outside_the_constellation_in_force() {
        // U_INFO 78 is deliberately outside every interval here: that is what
        // makes a U_INFO symbol in data mode visible rather than plausible.
        let cpu = Cp {
            data_mode: true,
            drn: 3,
            intervals: [0, 1, 2, 0, 1, 2],
            constellations: [79..=95u8, 80..=95, 81..=95]
                .iter()
                .map(|r| r.clone().fold(0u128, |m, u| m | 1 << u))
                .collect(),
            ..Cp::default()
        };
        assert!(!cpu.points(0).contains(&78), "this CPu contains U_INFO, so it cannot catch the hole");
        let mut modem = in_data_mode(cpu);
        let mapping = modem.downstream.as_ref().expect("B1d's encoder").mapping().clone();
        modem.source.change(Out::B1d);
        // A quarter of a data frame queued and nothing else: the queue runs dry
        // and the arm has to hold a codeword rather than invent a level.
        let octet = ucode::octet(Law::Mu, 95, false);
        modem.source.codes.push_back(octet);
        for n in 0..B1D_FRAMES * INTERVALS {
            let level = modem.source.next();
            let interval = (modem.source.symbol as usize - 1) % INTERVALS;
            let set = &mapping.sets[interval];
            let legal = set
                .points()
                .iter()
                .any(|&u| (ucode::level(Law::Mu, u) - level.abs()).abs() < 1e-12);
            assert!(legal, "B1d symbol {n} is {level}, which is not a point of interval {interval}");
        }
    }

    /// **`docs/221`'s live latent bug, `digital.rs:2425`.** `B1d`'s
    /// queue-empty arm emitted `self.uinfo` while its own comment said "the last
    /// codeword again": a -18 dB hole in the middle of data mode. It must hold
    /// the last codeword.
    #[test]
    fn regression_docs_221_b1d_queue_empty_holds_the_last_codeword() {
        let law = Law::Mu;
        let mut source = Source::new(&settings_uinfo(78));
        source.change(Out::B1d);
        source.codes.push_back(ucode::octet(law, 95, false));
        source.codes.push_back(ucode::octet(law, 60, true));
        let first = source.next();
        let second = source.next();
        assert!((first.abs() - ucode::level(law, 95)).abs() < 1e-12);
        assert!(second < 0.0, "the queued sign was lost");
        assert!((second.abs() - ucode::level(law, 60)).abs() < 1e-12);
        // The queue is dry from here: every further symbol repeats the last
        // codeword, sign and all, and is never U_INFO.
        for n in 0..12 {
            let level = source.next();
            assert!(
                (level - second).abs() < 1e-12,
                "symbol {n} after the queue ran dry is {level}, not the last codeword {second}"
            );
            assert!((level.abs() - ucode::level(law, 78)).abs() > 1e-9, "the queue fell back to U_INFO");
        }
    }

    /// 8.6.5/V.90 (`v90.txt:1853-1859`): `TRN2d`'s codeword comes from CPt.
    /// `trn2d_point` used to hand-derive "the loudest point common to all six
    /// intervals"; it now asks `Mapping::common_loudest`, the owner of the
    /// table, and this asserts the two agree on a CPt whose six sets differ.
    #[test]
    fn c_8_6_5_trn2d_point_is_the_tables_loudest_common_point() {
        let cpt = cpt_six(16);
        let mut source = Source::new(&settings_uinfo(78));
        assert!(source.set_cpt(&cpt));
        let mapping = source.phase4.as_ref().expect("CPt's coder").coder.mapping().clone();
        // Independently: the loudest Ucode by `ucode::linear` present in every
        // one of the six constellations.
        let common: Vec<u8> = (0..=95u8)
            .filter(|u| (0..INTERVALS).all(|i| mapping.sets[i].points().contains(u)))
            .collect();
        let loudest = *common
            .iter()
            .max_by_key(|&&u| ucode::linear(Law::Mu, u))
            .expect("these six constellations share a point");
        assert_eq!(source.trn2d_point(), loudest, "TRN2d's codeword is not the table's loudest common");
        assert_eq!(mapping.common_loudest(), Some(loudest));
        assert_ne!(loudest, 78, "the CPt of this test contains U_INFO, so the branch is untested");
        // And it is a legal symbol in all six intervals, which is the whole
        // reason the two-point fallback exists.
        for i in 0..INTERVALS {
            assert!(mapping.sets[i].points().contains(&loudest), "interval {i} cannot send it");
        }
    }

    /// 9.8.1.1.4 / 8.6.4/V.90: `Rd` and `Rt` are "already resolved" when the
    /// renegotiation starts -- `arm_renegotiation` (`2066`) is called at
    /// `5780`/`5837` -- so a CPu or CPt arriving *during* the renegotiation
    /// must not move them. Nothing tested this before `docs/221`.
    #[test]
    fn c_9_8_1_1_4_the_armed_r_arrays_are_frozen_against_later_messages() {
        let law = Law::Mu;
        let mut modem = in_data_mode(cpu_of(&[3..=95, 3..=88], [0, 1, 0, 1, 0, 1]));
        assert!(modem.source.set_cpt(&cpt_of(16, 72..=88)));
        let (data, training) = modem.renegotiation_points();
        modem.source.arm_renegotiation(data, training);
        // A second CPt and a second CPu, both narrower, arrive mid-signal.
        assert!(modem.source.set_cpt(&cpt_of(16, 40..=60)));
        modem.cpu = Some(cpu_of(&[70..=90u8, 71..=91u8], [0, 1, 0, 1, 0, 1]));
        modem.begin_data();
        modem.source.change(Out::Rd);
        let levels: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| modem.source.next()).collect();
        for (n, level) in levels.iter().enumerate() {
            let want = ucode::level(law, data[n % DOWN_INTERVALS]);
            assert!((level.abs() - want).abs() < 1e-12, "Rd symbol {n} moved when a message arrived");
        }
        modem.source.change(Out::Rt);
        let levels: Vec<f64> = (0..R_FRAMES * INTERVALS).map(|_| modem.source.next()).collect();
        for (n, level) in levels.iter().enumerate() {
            let want = ucode::level(law, training[n % DOWN_INTERVALS]);
            assert!((level.abs() - want).abs() < 1e-12, "Rt symbol {n} moved when a message arrived");
        }
    }

    /// 8.6.4/V.90: `Rd`/`Rt` with no constellation armed has no defined signal,
    /// and `docs/221` made the silent U_INFO fallback loud (`digital.rs:2149`).
    #[test]
    fn c_8_6_4_rd_without_arming_is_a_loud_failure() {
        if !cfg!(debug_assertions) {
            return;
        }
        let mut source = Source::new(&settings_uinfo(78));
        source.change(Out::Rd);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| source.next()));
        assert!(result.is_err(), "a codeword meaning nothing went out quietly");
    }

    /// 8.8.4 (`v92.txt:1957-1962`) [quoted]: "*Rf is transmitted by ... repeating
    /// the 12-symbol sequence containing the PCM codewords with the sign pattern
    /// `+ + - - + + - - + + - -` where the left-most sign is transmitted
    /// first*", "*R f consists of 2 repetitions of the 12-symbol sequence
    /// containing the same PCM codewords with the sign pattern
    /// `- - + + - - + + - - + +`*", and "*The PCM codewords used are the
    /// highest power PCM codeword from the data mode constellation of each data
    /// frame interval as passed in CPu*".
    ///
    /// `docs/221` could not write this test: `Rf` had no transmit path in that
    /// tree, which is why `docs/221` left `Rf` to "whoever implements it". It
    /// now has one (`Out::Rf`/`Out::RfBar`,
    /// `digital.rs`, and the arm that emits it), so the exclusion keeps it out
    /// of the **diff** -- V.90 has no `Rf` to diff it against, `v92.txt:1957` --
    /// and this property test pins it against the three mistakes this end can
    /// still make: a codeword that is not the loudest of the CPu in force, a
    /// pattern read off an em dash instead of the line, or a bar that repeats
    /// rather than inverts.
    ///
    /// The codewords are read through `renegotiation_points`, i.e. from the
    /// mapping `B1d`'s encoder is sliced from, not from a vector this test
    /// made up: two CPu shapes, so the property is the table's.
    #[test]
    fn c_8_8_4_rf_is_the_loudest_codeword_of_the_data_mode_constellation_as_passed_in_cpu() {
        let law = Law::Mu;
        // [quoted] v92.txt:1957-1958 and 1958-1960, left-most sign first:
        // `+ + - - + + - - + + - -`, and R-bar-f's `- - + + - - + + - - + +`.
        let rf_12 = [true, true, false, false, true, true, false, false, true, true, false, false];
        let bar_12 = rf_12.map(|sign| !sign);
        for (sets, intervals) in [
            (vec![3..=95u8, 3..=88u8], [0u8, 1, 0, 1, 0, 1]),
            (vec![40..=95u8, 41..=95, 42..=95], [0, 1, 2, 0, 1, 2]),
        ] {
            let mut modem = in_data_mode(cpu_of(&sets, intervals));
            let (data, training) = modem.renegotiation_points();
            // "the highest power PCM codeword ... as passed in CPu": one an
            // interval, resolved off the constellation mask the CPu carried.
            let mapping = modem.downstream.as_ref().expect("B1d's encoder").mapping().clone();
            for i in 0..DOWN_INTERVALS {
                let set = &mapping.sets[i % INTERVALS];
                let loudest = *set.points().first().expect("a set with points");
                assert_eq!(data[i], loudest, "interval {i}: Rf's codeword is not CPu's loudest");
                assert!(set.points().contains(&data[i]), "interval {i}: Rf's codeword is not in CPu");
            }
            modem.source.arm_renegotiation(data, training);
            modem.source.change(Out::Rf);
            let mut rf = Vec::new();
            let mut bar = Vec::new();
            for _ in 0..(R_FRAMES + R_BAR_FRAMES) * INTERVALS {
                let level = modem.source.next();
                match modem.source.out() {
                    Out::Rf => rf.push(level),
                    Out::RfBar => bar.push(level),
                    other => panic!("Rf ran into {other:?} before R-bar-f was over"),
                }
            }
            assert_eq!(rf.len(), R_FRAMES * INTERVALS, "Rf is 384T (9.9.1.2.2)");
            assert_eq!(bar.len(), R_BAR_FRAMES * INTERVALS, "R-bar-f is 24T (9.9.1.2.2)");
            for (i, level) in rf.iter().enumerate() {
                let want = ucode::level(law, data[i % DOWN_INTERVALS]);
                assert!((level.abs() - want).abs() < 1e-12, "Rf symbol {i} is not CPu interval {}'s loudest codeword", i % DOWN_INTERVALS);
                assert_eq!(*level > 0.0, rf_12[i % 12], "Rf symbol {i} left 8.8.4's sign pattern");
            }
            for (i, level) in bar.iter().enumerate() {
                let want = ucode::level(law, data[i % DOWN_INTERVALS]);
                assert!((level.abs() - want).abs() < 1e-12, "R-bar-f symbol {i} is not the same PCM codewords");
                assert_eq!(
                    *level > 0.0,
                    bar_12[i % 12],
                    "R-bar-f symbol {i} is not 2 repetitions of the inverted 12"
                );
            }
        }
    }
}
