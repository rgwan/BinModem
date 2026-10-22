//! The digital modem's upstream PCM receiver: hunting Ru and Su and their
//! reversals (8.5.5, 8.5.6), training on TRN1u (8.5.7) and TRN2u (8.7.6),
//! holding the 12-symbol frame alignment of 9.5.1.1.10, and handing out
//! equalised symbols.
//!
//! V92-22. One sample per symbol arrives from the central office's A/D: the
//! analogue modem is required to put its symbols on the A/D instants (8.6.3),
//! so there is nothing here to interpolate and no fractionally spaced
//! equaliser to build. What is left is the job V.90's digital receiver has on
//! the other side of the call, at T spacing:
//!
//! ```text
//! hunt Ru/Su -> train on TRN1u (least squares) -> slice -> 12-symbol framing
//! ```
//!
//! Two of those are worth saying out loud here.
//!
//! **The hunt looks at shape, not at polarity.** Ru is three symbols up and
//! three down; Su is `+a, 0, +a, -a, 0, -a`. A line inverts either, and Su's
//! zeros are real zeros, so what tells them apart is the zero every third
//! symbol -- which no channel that has not smeared a whole symbol can invent.
//! Both are found by correlating against themselves half a period apart,
//! which is what a receiver wants anyway because the reversal it is looking
//! for is exactly where that correlation changes sign.
//!
//! **Training is least squares against a sequence known from its first
//! symbol.** TRN1u is "binary ones applied to a zeroed GPA scrambler" (6.3,
//! 8.5.7), so this side can generate it exactly and never has to decide
//! anything while it trains. The spike (V92-26) measured what that is worth:
//! a fit a few tenths of a milli-level out costs 26 bits a call in bursts, and
//! one three orders better costs none. So the fit is long, and it is tried
//! again on a later stretch if the first is no good -- a burst at the start of
//! TRN1u must cost a retry, not the call.

use std::collections::VecDeque;

use dsp::Complex;

use super::up_signals::{self, RU_PERIOD};
use crate::v32::Mode;

use super::UP_INTERVALS;

/// The mean square a window needs before any of its shape means anything.
///
/// Only enough that a window of digital silence -- which is exactly zeros on a
/// digital network -- is not correlated at all: the shape test that follows is
/// normalised, so the level it is applied at cannot change the answer. This
/// used to be 0.05, which is a mean square of 0.22 in the levels' own units and
/// was taken from the same synthetic bursts the rest of this module was proved
/// on. A real analogue modem on a real line reaches the far end at about a
/// quarter of full scale -- 0.06 of mean square on a good day -- so the hunt
/// never fired at all and Phase 3 waited for a Ru that was on the line.
const AUDIBLE: f64 = 1e-4;

/// Ru's shape over one period: "+++---" (8.5.5).
const RU_SHAPE: [f64; RU_PERIOD] = [1.0, 1.0, 1.0, -1.0, -1.0, -1.0];

/// Su's, with its zeros: "+0+-0-" (8.5.6). The level is left out because the
/// correlation is normalised, and a real zero is a zero at any level.
const SU_SHAPE: [f64; RU_PERIOD] = [1.0, 0.0, 1.0, -1.0, 0.0, -1.0];

/// Su as a transmitter that band-limits it to its fundamental leaves it.
///
/// The two zeroes in [`SU_SHAPE`] are what tells Su from Ru, and they do not
/// survive a real transmit filter: an Agere SV92PP on an 8 kHz line sends its
/// Su as the same six-symbol period with those slots filled in, which is very
/// nearly a sine at 8000/6 Hz. Measured off that modem's own audio, one period
/// of it correlates 0.911 with Ru's shape and 0.644 with Su's -- so a hunt with
/// one Su template calls the analogue modem's Su a **Ru**, and the digital
/// modem waits out its Su deadline with Su on the line, which is exactly what
/// happened before this template existed.
///
/// Against this one the same window correlates 0.99, and Ru's square wave only
/// 0.58, so the pair separates the two again. Su is therefore a bank of two
/// shapes: what 8.5.6 prints, and what a filtered one looks like.
const SU_FILTERED: [f64; RU_PERIOD] = [0.0, 0.866, 0.866, 0.0, -0.866, -0.866];

/// How well a window has to match a shape before it is that sequence. Through
/// a line that has not smeared a whole symbol the match is better than 0.99;
/// 0.85 leaves room for one that has smeared a little.
const MATCHED: f64 = 0.90;

/// How many samples of history the receiver keeps: enough for the training
/// fit, the epsilon estimator and a channel estimate to all look at the same
/// stretch.
/// Samples of line the receiver keeps. It has to cover the whole of TRN1u
/// that the training fit uses plus the wait before the fit is taken: at
/// 8000 samples a second this is a second of line, and TRN1u's 2040T minimum
/// is a quarter of it.
pub const HISTORY: usize = 8192;

/// Feed-forward taps the training fit takes. The spike's measurement -- 48
/// taps on 4800 symbols is a million times better than 32 on 2400 -- is what
/// this number is; TRN1u is 2040 symbols at least (8.5.6), so the fit has
/// forty times as many equations as unknowns.
pub const TRAIN_TAPS: usize = 48;

/// Symbols of TRN1u the fit is taken over. TRN1u is at least 2040T (8.5.6)
/// and the fit must not reach past its end into Ja.
const TRAIN_WINDOW: usize = 1200;

/// How many stretches of TRN1u the training will try before giving up.
pub const TRAIN_TRIES: usize = 3;

/// The signal to noise a successful training has to reach.
///
/// Twenty was the figure the synthetic bursts reached, where the line is the
/// taps and nothing else. On a real one -- a codec's 8-bit words, a network's
/// own resampling and 48 forward taps with no decision feedback -- the same
/// fit measures about sixteen, and sixteen is plenty for the two-level
/// decisions Phase 3 is made of: Ja's bits are read at a tenth of the error
/// rate of the moduli Phase 4 will choose.
pub const TRAINED_SNR_DB: f64 = 12.0;

/// The first good match of Ru or Su, kept as the polarity and phase every
/// later period is compared against.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Anchor {
    template: [f64; RU_PERIOD],
    kind: Hunt,
    phase: u64,
    /// Periods the shape has matched in a row. Nothing is reported until
    /// there are [`HELD_PERIODS`] of them: a six-sample window of noise
    /// matches a six-sample shape by chance often enough that a single match
    /// is not a sequence, and a hunt that acts on one fires on a phase 2's
    /// worth of tones.
    held: u32,
    /// Periods that matched neither the shape nor its inverse. The period in
    /// which one sequence turns into the other straddles the boundary and
    /// matches nothing, so a single miss is expected there and must not cost
    /// the anchor: dropping it is what lost the Ru-to-R-bar-u reversal that
    /// Phase 3 is waiting for.
    stray: u32,
}

/// Periods a shape must match in a row before the hunt calls it a signal.
///
/// Ru is 384T and Su 144T, so sixty-four and twenty-four periods of it are on
/// the line. Two was not enough: a six-sample window of a phase 2's tones
/// matches a six-sample shape by chance several times a second, and a hunt that
/// fired on those had the modem chasing Ru through every tone in the call.
/// Eight periods is six milliseconds, which costs a real Ru nothing.
const HELD_PERIODS: u32 = 8;

/// The normalised correlation of a window against a template: +1 identical,
/// -1 inverted, 0 unrelated.
fn correlate(window: &[f64], template: &[f64; RU_PERIOD]) -> f64 {
    let mut together = 0.0;
    let mut window_power = 0.0;
    let mut template_power = 0.0;
    for (i, &x) in window.iter().enumerate() {
        let t = template[i % RU_PERIOD];
        together += x * t;
        window_power += x * x;
        template_power += t * t;
    }
    together / (window_power * template_power).sqrt().max(1e-30)
}

/// What the hunt is currently hearing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Hunt {
    /// No sequence yet, or the last one has been accounted for.
    Searching,
    /// Ru, as sent, held long enough to be called.
    Ru,
    /// And turned over, which is the reversal 9.5.1.1.1 wants.
    RuTurned,
    Su,
    SuTurned,
}

/// What the receiver has to report, in the event-queue shape the rest of this
/// code base uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Heard {
    /// Ru, found at this symbol.
    Ru { at: u64 },
    /// Ru has become R-bar-u at about this symbol (9.5.1.1.1).
    RuReversal { at: u64 },
    /// Su, found at this symbol.
    Su { at: u64 },
    /// And S-bar-u (9.5.1.1.6).
    SuReversal { at: u64 },
    /// Trained, with what the fit left.
    Trained { snr_db: f64 },
    /// One symbol, equalised and sliced.
    Symbol(Symbol),
    /// The decisions went bad all at once, and the loops are holding still.
    Lost,
    /// And they are good again.
    Found,
}

/// One symbol after the equaliser.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Symbol {
    /// Symbols since the first sample this receiver was given, with the frame
    /// origin subtracted: [`Self::interval`] comes from it.
    pub index: u64,
    /// The same count with no origin taken off.
    pub raw: u64,
    /// The equaliser's output, in the levels' own units.
    pub value: f64,
    /// What the slicer decided, if it decided.
    pub decided: Option<f64>,
}

impl Symbol {
    /// Which data frame interval it was sent in (9.5.1.1.10 starts the count
    /// at the first symbol of the second TRN1u).
    #[must_use]
    pub fn interval(&self) -> usize {
        (self.index % UP_INTERVALS as u64) as usize
    }
}

/// What the slicer is deciding between, which changes three times in a call:
/// two points through TRN1u and E1u, four or eight through TRN2u (8.7.6), and
/// the full set once CPd has said what it is.
#[derive(Debug, Clone, PartialEq)]
pub enum Slicer {
    /// `+-LU`.
    TwoPoint,
    /// The 4-point TRN2u: `+-LU, +-3LU` on a spacing of two.
    FourPoint,
    /// The 8-point TRN2u.
    EightPoint,
    /// The levels CPd's constellation set holds, in ascending order.
    Levels(Vec<f64>),
}

impl Slicer {
    /// The nearest level to `value`, and how far from it that was.
    fn decide(&self, value: f64) -> Option<(f64, f64)> {
        match self {
            Self::TwoPoint => Some((if value >= 0.0 { 1.0 } else { -1.0 }, (value.abs() - 1.0).abs())),
            Self::FourPoint => {
                let levels = [-3.0, -1.0, 1.0, 3.0];
                nearest(&levels, value)
            }
            Self::EightPoint => {
                let levels = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
                nearest(&levels, value)
            }
            Self::Levels(levels) => nearest(levels, value),
        }
    }
}

/// The nearest of `levels` to `value`, with the distance.
fn nearest(levels: &[f64], value: f64) -> Option<(f64, f64)> {
    levels
        .iter()
        .map(|&level| (level, (value - level).abs()))
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// The digital modem's upstream receiver.
#[derive(Debug, Clone)]
pub struct Receiver {
    /// The polynomial the training reference is generated with: `Answer`
    /// (GPA) for what an analogue modem sends, `Call` (GPC) for a digital
    /// modem's own downstream training (6.3/V.92).
    reference_mode: Mode,
    /// Every sample, for training, the epsilon estimator and the channel
    /// estimate. The oldest fall off the front.
    history: VecDeque<f64>,
    events: VecDeque<Heard>,
    /// The last `RU_PERIOD` samples' second moment, and the cross moment half
    /// a period back: what tells Ru from Su from silence.
    recent: VecDeque<f64>,
    products: VecDeque<(f64, f64)>,
    /// Which shape this end is waiting for, when the call's own order says so.
    ///
    /// Ru and Su are one six-symbol period each and differ only in that Su has
    /// two symbols at zero (8.5.5, 8.5.6). Nothing tells them apart once a
    /// transmitter has band-limited them: an Agere SV92PP's Su and its Ru both
    /// arrive as a 1333 Hz waveform that matches *Ru*'s template at 0.911 and
    /// *Su*'s at 0.644 -- the same figures for both -- so a receiver choosing
    /// by shape alone cannot see the analogue modem's Su at all, and reports its
    /// Ru as Su.
    ///
    /// The Recommendation's timetable does know: Ru opens Phase 3 and is the
    /// only one of the two before TRN1u, Su answers Jd and is the only one of
    /// the two after it, and Ri's "+ + + - - -" comes after CPt and nothing
    /// else does. So an end that knows which it must be hearing next can say
    /// so, and the hunt then looks for that shape only. Left as `None`, the
    /// hunt goes back to choosing on shape, which is what it wants when the
    /// timetable cannot say.
    expected: Option<Hunt>,
    /// Where the hunt has got to: what it has been hearing, and for how long.
    state: Hunt,
    /// What the first good match looked like: the polarity everything after
    /// it is measured against, and the phase it was taken at.
    anchor: Option<Anchor>,
    /// Training.
    taps: Vec<f64>,
    trained: bool,
    snr_db: f64,
    /// Where the fit was taken, so that a retry can move on.
    trained_at: usize,
    /// The 12-symbol count's origin (9.5.1.1.10), and the symbol count.
    origin: Option<u64>,
    count: u64,
    slicer: Slicer,
    /// Decisions recently made, for the error watch.
    wrong: u32,
    right: u32,
    lost: bool,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver {
    #[must_use]
    pub fn new() -> Self {
        Self {
            reference_mode: Mode::Answer,
            history: VecDeque::with_capacity(HISTORY),
            events: VecDeque::new(),
            recent: VecDeque::new(),
            products: VecDeque::new(),
            expected: None,
            state: Hunt::Searching,
            anchor: None,
            taps: Vec::new(),
            trained: false,
            snr_db: 0.0,
            trained_at: 0,
            origin: None,
            count: 0,
            slicer: Slicer::TwoPoint,
            wrong: 0,
            right: 0,
            lost: false,
        }
    }

    /// One symbol, T spaced, in the levels' own units.
    pub fn feed(&mut self, level: f64) {
        self.history.push_back(level);
        if self.history.len() > HISTORY {
            self.history.pop_front();
            self.trained_at = self.trained_at.saturating_sub(1);
        }
        let _found = self.recognise(level);
        if self.trained {
            self.equalise_and_slice(level);
        }
        self.count += 1;
    }

    /// What the receiver has to say, one event at a time.
    pub fn heard(&mut self) -> Option<Heard> {
        self.events.pop_front()
    }

    /// The symbols fed so far.
    #[must_use]
    pub fn symbols(&self) -> u64 {
        self.count
    }

    /// Put the hunt back to the start, as a cleardown or a lost line needs.
    pub fn restart_hunt(&mut self) -> &mut Self {
        self.recent.clear();
        self.products.clear();
        self
    }

    /// Look for Ru only: what Phase 3's opening is, and what Ri looks like
    /// after CPt (8.6.5 with 8.5.5).
    pub fn expect_ru(&mut self) -> &mut Self {
        self.expect(Hunt::Ru)
    }

    /// Look for Su only: what answers Jd, and what Sd is shaped like.
    pub fn expect_su(&mut self) -> &mut Self {
        self.expect(Hunt::Su)
    }

    /// Go back to choosing between the two on shape alone.
    pub fn expect_either(&mut self) -> &mut Self {
        self.expected = None;
        self.anchor = None;
        self.state = Hunt::Searching;
        self
    }

    fn expect(&mut self, shape: Hunt) -> &mut Self {
        if self.expected != Some(shape) {
            // A window held against the other shape is not a window of this
            // one: start again rather than let the anchor's kind stand.
            self.anchor = None;
            self.state = Hunt::Searching;
        }
        self.expected = Some(shape);
        self
    }

    /// The signal the last training reached, in decibels.
    #[must_use]
    pub fn snr_db(&self) -> f64 {
        self.snr_db
    }

    /// Whether there is an equaliser behind the slicer.
    #[must_use]
    pub fn is_trained(&self) -> bool {
        self.trained
    }

    /// What the slicer is deciding between, and where the fit put it.
    /// Which polynomial the training reference uses: `Answer` for an
    /// analogue modem's TRN1u, `Call` for a digital modem's TRN1d.
    pub fn set_reference_mode(&mut self, mode: Mode) -> &mut Self {
        self.reference_mode = mode;
        self
    }

    pub fn set_slicer(&mut self, slicer: Slicer) {
        self.slicer = slicer;
    }

    /// 9.5.1.1.10: "the first symbol of the second TRN1u shall be symbol 0 of
    /// the 12-symbol data frame". The count starts here.
    pub fn set_frame_origin(&mut self, at: u64) {
        self.origin = Some(at);
    }

    /// The symbol's own index with the origin taken off.
    #[must_use]
    pub fn interval(&self) -> usize {
        let index = self.origin.map_or(self.count, |origin| self.count.saturating_sub(origin));
        (index % UP_INTERVALS as u64) as usize
    }

    /// Whether the error watch has declared the line lost.
    #[must_use]
    pub fn is_lost(&self) -> bool {
        self.lost
    }

    /// The equaliser's feed-forward taps, for the channel estimate.
    #[must_use]
    pub fn taps(&self) -> &[f64] {
        &self.taps
    }

    /// A stretch of history, for the epsilon estimator: `count` symbols
    /// ending `back` symbols before the newest.
    #[must_use]
    pub fn history_since(&self, from: u64) -> Vec<f64> {
        let from = from as usize;
        let len = self.history.len();
        if from >= len {
            return Vec::new();
        }
        self.history.iter().skip(from).copied().collect()
    }

    /// How many samples of history the receiver holds.
    #[must_use]
    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// Recognise Ru, Su and their reversals from the shape of the last window.
    ///
    /// The window is one period of six, which is what both sequences repeat
    /// (8.5.5, 8.5.6), and the test is a normalised correlation against each
    /// sequence's own shape at each of the six phases it could be caught in.
    /// The shape is what tells Ru from Su -- Su has a symbol in three at zero
    /// and Ru has none.
    ///
    /// A reversal is a *change*, and that is the only way it can be seen. Ru
    /// and R-bar-u are one the other's image half a period along: a single
    /// period of either correlates with both shapes equally well, and with
    /// itself the same way whether it has been inverted or not, because
    /// inverting a sequence that is already anti-correlated with itself half a
    /// period later changes nothing. So the first good match becomes the
    /// anchor -- its polarity is this end's zero, since nothing on the line
    /// says which way up it is -- and every later period is compared with
    /// that anchor. When the comparison turns over, the sequence has turned
    /// over (9.5.1.1.1), and the anchor moves with it.
    fn recognise(&mut self, level: f64) -> bool {
        self.recent.push_back(level);
        if self.recent.len() > RU_PERIOD {
            self.recent.pop_front();
        }
        if self.recent.len() < RU_PERIOD {
            return false;
        }
        let window: Vec<f64> = self.recent.iter().copied().collect();
        let power: f64 = window.iter().map(|x| x * x).sum();
        if !power.is_finite() || power <= AUDIBLE {
            self.anchor = None;
            self.state = Hunt::Searching;
            return false;
        }
        let at = self.count;
        if let Some(anchor) = self.anchor.as_mut() {
            // One comparison per period, at the phase the anchor was taken at,
            // so that the two windows are the same part of the sequence.
            if at % RU_PERIOD as u64 != anchor.phase {
                return false;
            }
            let share = correlate(&window, &anchor.template);
            if share > MATCHED {
                // Still the same signal. The first period it is held for is
                // when it is reported as having been found.
                anchor.held += 1;
                if anchor.held == HELD_PERIODS {
                    match anchor.kind {
                        Hunt::Ru => self.events.push_back(Heard::Ru { at }),
                        Hunt::Su => self.events.push_back(Heard::Su { at }),
                        _ => {}
                    }
                }
                return false;
            }
            if share > -MATCHED {
                // Neither. One of these is the period the sequence turned over
                // in; two in a row means something else is being sent now.
                if anchor.stray < 1 {
                    anchor.stray += 1;
                    return false;
                }
                self.anchor = None;
                self.state = Hunt::Searching;
                return false;
            }
            anchor.stray = 0;
            // Turned over. The anchor moves with it so that a second reversal
            // is seen as one, and a turn of something that was never held is
            // a coincidence rather than a reversal.
            if anchor.held >= HELD_PERIODS {
                let turned = match anchor.kind {
                    Hunt::Ru => Some(Heard::RuReversal { at }),
                    Hunt::Su => Some(Heard::SuReversal { at }),
                    _ => None,
                };
                if let Some(turned) = turned {
                    anchor.kind = match anchor.kind {
                        Hunt::Ru => Hunt::RuTurned,
                        Hunt::Su => Hunt::SuTurned,
                        other => other,
                    };
                    anchor.template = window.try_into().unwrap_or([0.0; RU_PERIOD]);
                    anchor.held = 0;
                    self.state = anchor.kind;
                    self.events.push_back(turned);
                    return true;
                }
            }
            self.anchor = None;
            self.state = Hunt::Searching;
            return false;
        }
        // No anchor: look for the shape at any phase, among those this end is
        // waiting for.
        let mut best = (0.0f64, Hunt::Searching);
        for phase in 0..RU_PERIOD {
            for (shape, kind) in [
                (RU_SHAPE, Hunt::Ru),
                (SU_SHAPE, Hunt::Su),
                (SU_FILTERED, Hunt::Su),
            ] {
                if let Some(expected) = self.expected
                    && kind != expected
                {
                    continue;
                }
                let mut template = [0.0; RU_PERIOD];
                for (i, slot) in template.iter_mut().enumerate() {
                    *slot = shape[(i + phase) % RU_PERIOD];
                }
                let size = correlate(&window, &template).abs();
                if size > best.0 {
                    best = (size, kind);
                }
            }
        }
        if best.0 < MATCHED || best.1 == Hunt::Searching {
            self.state = Hunt::Searching;
            return false;
        }
        self.anchor = Some(Anchor {
            template: window.try_into().unwrap_or([0.0; RU_PERIOD]),
            kind: best.1,
            phase: at % RU_PERIOD as u64,
            held: 1,
            stray: 0,
        });
        self.state = best.1;
        true
    }

    /// Equalise one symbol and slice it.
    fn equalise_and_slice(&mut self, level: f64) {
        let taps = &self.taps;
        let len = self.history.len();
        if taps.len() > len {
            return;
        }
        let mut value = 0.0;
        for (k, &tap) in taps.iter().enumerate() {
            value += tap * self.history[len - 1 - k];
        }
        let index = self.origin.map_or(self.count, |origin| self.count.saturating_sub(origin));
        let decided = self.slicer.decide(value);
        if let Some((_, error)) = decided {
            if error > 0.5 {
                self.wrong += 1;
            } else {
                self.right += 1;
            }
            if self.wrong > 24 && self.wrong > 4 * self.right && !self.lost {
                self.lost = true;
                self.events.push_back(Heard::Lost);
            } else if self.lost && self.wrong == 0 {
                self.lost = false;
                self.events.push_back(Heard::Found);
            }
            if self.right + self.wrong > 128 {
                self.right /= 2;
                self.wrong /= 2;
            }
        }
        self.events.push_back(Heard::Symbol(Symbol {
            index,
            raw: self.count,
            value,
            decided: decided.map(|(level, _)| level),
        }));
        let _ = level;
    }

    /// Train on the TRN1u that begins at history index `from`.
    ///
    /// Least squares against the sequence TRN1u actually is: GPA from a zeroed
    /// register, fed ones (6.3, 8.5.7). A coarse alignment search covers the
    /// symbol the caller may have been out by, and a fit that comes back worse
    /// than [`TRAINED_SNR_DB`] is retried on a later stretch, because a burst
    /// at the start of TRN1u must cost a retry and not the call.
    pub fn train_on_trn1u(&mut self, from: u64) -> bool {
        let from = from as usize;
        for attempt in 0..TRAIN_TRIES {
            // Each retry moves on by a quarter of what is left, so that three
            // tries use three stretches of a long TRN1u.
            let skip = if attempt == 0 {
                0
            } else {
                self.history.len().saturating_sub(from) / 4 * attempt
            };
            let start = from + skip;
            if !self.fit(start) {
                continue;
            }
            if self.snr_db >= TRAINED_SNR_DB {
                self.trained = true;
                self.trained_at = start;
                self.events.push_back(Heard::Trained { snr_db: self.snr_db });
                return true;
            }
        }
        self.trained = false;
        false
    }

    /// Train on the TRN1u that begins within `search` symbols of `from`.
    ///
    /// The hunt reports where a reversal was to within a period, not to the
    /// symbol, and TRN1u's first symbol starts 24T after it: on a live line
    /// the anchor is a few symbols out, and a fit against a reference shifted
    /// by even two symbols is a fit against the wrong signal. So every
    /// alignment in the window is fitted and the best one wins.
    pub fn train_near(&mut self, from: u64, search: u64) -> bool {
        let base = from as usize;
        let lo = base.saturating_sub(search as usize);
        let hi = base + search as usize;
        let mut best: Option<(f64, usize)> = None;
        for start in lo..=hi {
            if !self.fit(start) {
                continue;
            }
            if std::env::var_os("V92_TRACE").is_some() && self.snr_db > 6.0 {
                eprintln!("V92TRACE: candid {start} snr {:.1}", self.snr_db);
            }
            if best.is_none_or(|(snr, _)| self.snr_db > snr) {
                best = Some((self.snr_db, start));
            }
        }
        if std::env::var_os("V92_TRACE").is_some() {
            eprintln!("V92TRACE: train_near(from {from} search {search}) best {best:?}");
        }
        let Some((snr, start)) = best else { return false };
        if snr < TRAINED_SNR_DB {
            self.trained = false;
            return false;
        }
        // Leave the winner's taps in place rather than the last ones tried.
        if !self.fit(start) {
            return false;
        }
        self.trained = true;
        self.trained_at = start;
        self.snr_db = snr;
        self.events.push_back(Heard::Trained { snr_db: snr });
        true
    }

    /// One least-squares fit of the equaliser against the known sequence
    /// starting at `from`.
    fn fit(&mut self, from: usize) -> bool {
        // A fixed length, not everything the history holds: by the time the
        // fit is taken the history reaches past the end of TRN1u into Ja and
        // whatever follows it, and a reference that goes on pretending to be
        // TRN1u past that point is fitted against the wrong signal for as
        // much of the window as it overruns. 1200 of TRN1u's 2040 minimum is
        // twenty-five equations per tap.
        let end = (from + TRAIN_WINDOW).min(self.history.len());
        let Some(reference) = reference(from, end, self.reference_mode) else {
            return false;
        };
        let received: Vec<f64> = self.history.iter().copied().collect();
        let taps = TRAIN_TAPS.min(reference.len() / 8).max(1);
        if received.len() < from + reference.len() {
            return false;
        }
        // The filter this solves for is the equaliser, not the channel: what
        // goes in is what the line brought and what comes out is the known
        // sequence, so `reference = sum(taps * received)`. Solving it the
        // other way round -- `received = sum(taps * reference)` -- gives the
        // channel's own impulse response, and running that over the line is
        // the channel applied twice, which is why the equaliser's output was
        // four hundredths of the level it was reading.
        let mut a = vec![Complex::ZERO; taps * taps];
        let mut b = vec![Complex::ZERO; taps];
        let length = reference.len();
        for i in 0..taps {
            for j in 0..taps {
                let mut sum = 0.0;
                for n in taps - 1..length {
                    sum += received[from + n - i] * received[from + n - j];
                }
                a[i * taps + j] = Complex::new(sum, 0.0);
            }
            let mut sum = 0.0;
            for n in taps - 1..length {
                sum += received[from + n - i] * reference[n];
            }
            b[i] = Complex::new(sum, 0.0);
        }
        let trace: f64 = (0..taps).map(|i| a[i * taps + i].re).sum();
        let ridge = 1e-9 * trace / taps as f64;
        for i in 0..taps {
            a[i * taps + i].re += ridge;
        }
        let Some(solved) = dsp::complex::solve_hermitian(&a, &b) else {
            return false;
        };
        self.taps = solved.iter().map(|value| value.re).collect();
        // What the fit leaves is the noise it was made in, measured where the
        // equaliser will be used: its output against the sequence it should
        // be producing.
        let mut signal = 0.0;
        let mut noise = 0.0;
        for n in taps - 1..length {
            let mut model = 0.0;
            for (k, &tap) in self.taps.iter().enumerate() {
                model += tap * received[from + n - k];
            }
            let error = reference[n] - model;
            noise += error * error;
            signal += reference[n] * reference[n];
        }
        let count = length.saturating_sub(taps - 1).max(1) as f64;
        let signal = signal / count;
        let noise = noise / count;
        self.snr_db = if noise > 0.0 { 10.0 * (signal / noise).log10() } else { f64::INFINITY };
        self.snr_db.is_finite() && self.snr_db > 0.0
    }
}

/// TRN1u's symbols from `from` onwards, as many as the history holds.
fn reference(from: usize, length: usize, mode: Mode) -> Option<Vec<f64>> {
    if length <= from + TRAIN_TAPS {
        return None;
    }
    // "The polynomial goes with the role, not with who dialled": the analogue
    // modem's TRN1u is GPA and the digital modem's TRN1d is GPC (6.3/V.92).
    // Reading the other end's training with the wrong one is reading a
    // different sequence.
    let mut trn = up_signals::Trn1u::with_mode(mode);
    let mut out = Vec::with_capacity(length - from);
    for _ in 0..(length - from) {
        out.push(trn.next_symbol());
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::up_signals::{ru, su};

    /// Ru, then R-bar-u, is found without knowing which way up the line is.
    #[test]
    fn ru_and_its_reversal_are_found_whatever_the_polarity() {
        for inverted in [false, true] {
            let mut rx = Receiver::new();
            let mut found = Vec::new();
            for n in 0..(RU_SYMBOLS_LOCAL + 64) {
                let bar = n >= RU_SYMBOLS_LOCAL;
                let level = ru(n as u64, bar) * if inverted { -1.0 } else { 1.0 };
                rx.feed(level);
                while let Some(event) = rx.heard() {
                    if matches!(event, Heard::Ru { .. } | Heard::RuReversal { .. }) {
                        found.push((event, n));
                    }
                }
            }
            assert!(found.iter().any(|(e, _)| matches!(e, Heard::Ru { .. })), "no Ru, inverted {inverted}");
            let reversal = found.iter().find(|(e, _)| matches!(e, Heard::RuReversal { .. }));
            let (_, at) = reversal.unwrap_or_else(|| panic!("no reversal, inverted {inverted}"));
            assert!(*at >= RU_SYMBOLS_LOCAL && *at <= RU_SYMBOLS_LOCAL + RU_PERIOD, "reversal at {at}");
        }
    }

    const RU_SYMBOLS_LOCAL: usize = 384;

    /// Su is found through the zeros Ru does not have, and its reversal too.
    #[test]
    fn su_is_found_through_its_zeros_and_its_reversal() {
        let mut rx = Receiver::new();
        let mut found = Vec::new();
        let first = 144;
        for n in 0..(first + 64) {
            let bar = n >= first;
            rx.feed(su(n as u64, bar));
            while let Some(event) = rx.heard() {
                if matches!(event, Heard::Su { .. } | Heard::SuReversal { .. }) {
                    found.push((event, n));
                }
            }
        }
        assert!(found.iter().any(|(e, _)| matches!(e, Heard::Su { .. })), "no Su: {found:?}");
        // And a reversal, within a period of where Su turned over.
        assert!(
            found.iter().any(|(e, _)| matches!(e, Heard::SuReversal { .. })),
            "no Su reversal: {found:?}"
        );
    }

    /// Su as a real analogue modem's transmit filter leaves it -- the same
    /// six-symbol period with the zeros filled in, so nearly a sine -- is Su
    /// and not Ru (see [`SU_FILTERED`]). The window an Agere SV92PP sends
    /// correlates 0.911 with Ru's shape and 0.90 is the gate, so before the
    /// second template this burst was reported as Ru and the digital modem
    /// waited out its deadline with Su on the line.
    #[test]
    fn a_band_limited_su_is_still_su() {
        let mut rx = Receiver::new();
        let mut found = Vec::new();
        // Level and phase as measured off the Agere engine's own audio.
        let levels = [-0.071, 0.026, 0.097, 0.071, -0.026, -0.097];
        for n in 0..400 {
            let bar = n >= 144;
            let sign = if bar { -1.0 } else { 1.0 };
            rx.feed(sign * levels[n % RU_PERIOD]);
            while let Some(event) = rx.heard() {
                if !matches!(event, Heard::Symbol(_)) {
                    found.push((event, n));
                }
            }
        }
        assert!(
            found.iter().any(|(e, _)| matches!(e, Heard::Su { .. })),
            "a filtered Su read as something else: {found:?}"
        );
        assert!(!found.iter().any(|(e, _)| matches!(e, Heard::Ru { .. })), "filtered Su read as Ru: {found:?}");
        // And its turn is still a reversal, which is what Jp is measured from.
        assert!(found.iter().any(|(e, _)| matches!(e, Heard::SuReversal { .. })), "{found:?}");
    }

    /// The timetable is worth more than the shape when a transmitter has
    /// band-limited both sequences into nearly the same waveform: asked for Su,
    /// the hunt reports Su and never Ru; asked for Ru, the reverse.
    #[test]
    fn what_the_timetable_asked_for_is_what_is_reported() {
        // One period of the waveform an Agere SV92PP puts on the line: Ru's
        // shape through a filter that fills Su's zeroes, or Su's own -- the
        // two are the same figures, which is the point.
        let levels = [-0.071, 0.026, 0.097, 0.071, -0.026, -0.097];
        for (asked_ru, expect_su) in [(false, true), (true, false)] {
            let mut rx = Receiver::new();
            if asked_ru {
                rx.expect_ru();
            } else {
                rx.expect_su();
            }
            let mut heard = Vec::new();
            for n in 0..400 {
                rx.feed(levels[n % RU_PERIOD]);
                while let Some(event) = rx.heard() {
                    if !matches!(event, Heard::Symbol(_)) {
                        heard.push(event);
                    }
                }
            }
            let ru = heard.iter().any(|e| matches!(e, Heard::Ru { .. }));
            let su = heard.iter().any(|e| matches!(e, Heard::Su { .. }));
            assert_eq!(su, expect_su, "asked for Ru {asked_ru}: {heard:?}");
            assert_eq!(ru, !expect_su, "asked for Ru {asked_ru}: {heard:?}");
        }
    }

    /// The training fit, over a channel with noise: TRN1u is known, so this is
    /// least squares and nothing else. 25 dB is the plan's gate at 1e-4 of
    /// noise.
    #[test]
    fn trn1u_trains_the_receiver_over_a_channel_with_noise() {
        let mut rx = Receiver::new();
        let channel = [1.0, -0.45, 0.12];
        let mut state = 0x1234_5678u64;
        let mut noise = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state as i64 as f64) / (i64::MAX as f64) * 0.01
        };
        let mut line = VecDeque::new();
        let mut trn = up_signals::Trn1u::new();
        for _ in 0..2000 {
            let symbol = trn.next_symbol();
            line.push_front(symbol);
            let mut seen = 0.0;
            for (k, &tap) in channel.iter().enumerate() {
                seen += tap * line.get(k).copied().unwrap_or(0.0);
            }
            rx.feed(seen + noise());
        }
        while rx.heard().is_some() {}
        assert!(rx.train_on_trn1u(0), "the fit failed");
        assert!(rx.snr_db() >= 25.0, "trained at {:.1} dB", rx.snr_db());
        assert!(rx.is_trained());
        // And two-point decisions on more of the same are right.
        rx.set_slicer(Slicer::TwoPoint);
        let mut wrong = 0;
        let mut total = 0;
        for _ in 0..400 {
            let symbol = trn.next_symbol();
            line.push_front(symbol);
            let mut seen = 0.0;
            for (k, &tap) in channel.iter().enumerate() {
                seen += tap * line.get(k).copied().unwrap_or(0.0);
            }
            rx.feed(seen + noise());
            while let Some(event) = rx.heard() {
                if let Heard::Symbol(symbol) = event
                    && let Some(decided) = symbol.decided
                {
                    total += 1;
                    if (decided - symbol_at(symbol.value)).abs() > 0.5 {
                        wrong += 1;
                    }
                }
            }
        }
        assert!(total > 100, "only {total} symbols decided");
        assert_eq!(wrong, 0, "{wrong} of {total} two-point decisions wrong");
    }

    /// What a two-point decision should have been, from the value's sign.
    fn symbol_at(value: f64) -> f64 {
        if value >= 0.0 { 1.0 } else { -1.0 }
    }

    /// Silence and noise are neither Ru nor Su nor anything to train on.
    #[test]
    fn silence_and_noise_do_not_train() {
        let mut rx = Receiver::new();
        let mut state = 0xfeed_faceu64;
        for _ in 0..4000 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let level = (state as i64 as f64) / (i64::MAX as f64) * 0.02;
            rx.feed(level);
        }
        let mut events = Vec::new();
        while let Some(event) = rx.heard() {
            events.push(event);
        }
        assert!(
            !events.iter().any(|event| matches!(event, Heard::Ru { .. } | Heard::Su { .. })),
            "noise was read as a sequence: {events:?}"
        );
        assert!(!rx.train_on_trn1u(0), "noise trained");
        assert!(!rx.is_trained());
        // Silence is not Ru either.
        let mut rx = Receiver::new();
        for _ in 0..2000 {
            rx.feed(0.0);
        }
        let mut events = Vec::new();
        while let Some(event) = rx.heard() {
            events.push(event);
        }
        assert!(!events.iter().any(|event| matches!(event, Heard::Ru { .. } | Heard::Su { .. })));
    }

    /// 9.5.1.1.10: the twelve-symbol count starts where it is told, and runs
    /// modulo twelve from there.
    #[test]
    fn the_twelve_symbol_count_starts_where_it_is_told() {
        let mut rx = Receiver::new();
        for _ in 0..100 {
            rx.feed(0.5);
        }
        assert_eq!(rx.interval(), 100 % UP_INTERVALS, "with no origin the count is the index");
        rx.set_frame_origin(40);
        assert_eq!(rx.interval(), 0, "the origin is interval 0");
        for _ in 0..5 {
            rx.feed(0.5);
        }
        assert_eq!(rx.interval(), 5);
        for _ in 0..7 {
            rx.feed(0.5);
        }
        assert_eq!(rx.interval(), 0, "twelve symbols is a whole frame");
        // And the history is there for the epsilon estimator.
        assert!(rx.history_len() >= TRAIN_TAPS);
        let _ = rx.history_since(90);
    }
}


