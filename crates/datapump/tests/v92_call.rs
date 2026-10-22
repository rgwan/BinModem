//! A V.92 call: the digital modem of [`datapump::v92::digital`] against an
//! analogue end built out of the signals in `v92::up_signals`.
//!
//! The analogue modem's own state machine (`v92::analogue`, V92-36) is not
//! built yet, so this file drives its side of Phase 3 from a script: Ru and
//! its bar, TRN1u, Ja repeated until the digital end answers with Sd, then
//! silence while TRN1d and Jd go the other way, then Su, the bar after Jp, the
//! TRN1u that covers the DIL, CPt and E1u. What it reads off the line it reads
//! with the same receiver the digital modem reads Ja with.
//!
//! What it proves is the half that is written: the digital modem reads Ja,
//! sends Sd, S-bar-d, TRN1d and Jd, measures Su, answers with Jp, raises
//! circuit 107, reads CPt and reaches Phase 4. Phase 4's exchange and the PCM
//! upstream carrying bytes need `v92::analogue` on the other side (V92-36,
//! V92-39, V92-41).

use std::collections::VecDeque;

use datapump::v90::sequences::{Cp, Descriptor, Layout};
use datapump::v90::ucode::Law;
use datapump::v92::digital::{self, Status};
use datapump::v92::receiver::{Heard, Receiver};
use datapump::v92::sequences::{J, Jp};
use datapump::v32::Mode;
use datapump::v92::up_signals::{ru, su, Trn1u, TwoPointReader, TwoPointSender};

/// The digital modem's own rate: one codeword per 125 microseconds.
const FS: f64 = 8000.0;

fn settings() -> digital::Settings {
    digital::Settings {
        law: Law::Mu,
        uinfo: 79,
        far_md: 0,
        round_trip: 0.0,
        wide: true,
        habits: digital::Habits::PROMPT,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Leading,
    Trn1u,
    Ja,
    /// The digital end's S-bar-d came: silent while TRN1d and Jd go out.
    TurnedRound,
    Su,
    AfterJp,
    Cpt,
}

/// The analogue end, as far as Phase 3 needs it.
struct Script {
    step: Step,
    count: u64,
    trn: Trn1u,
    /// Ja's and CPt's bits, a repetition at a time.
    sender: TwoPointSender,
    sender_bits: Vec<bool>,
    /// The line the digital end is sending on.
    rx: Receiver,
    reader: TwoPointReader,
    window: VecDeque<bool>,
    /// TRN1u symbols left in the step that covers the DIL.
    trn_left: u64,
    /// CPt frames left before E1u.
    cpt_left: u64,
    /// Where TRN1d began, in the receiver's own symbol count.
    trn_from: Option<u64>,
    trained: bool,
    notes: Vec<String>,
}

impl Script {
    fn new() -> Self {
        Self {
            step: Step::Leading,
            count: 0,
            trn: Trn1u::new(),
            sender: TwoPointSender::after(Trn1u::new()),
            sender_bits: Vec::new(),
            // The digital end's TRN1d is the digital modem's own sequence,
            // which is GPC, not the GPA an analogue modem sends.
            rx: {
                let mut rx = Receiver::new();
                rx.set_reference_mode(Mode::Call);
                rx
            },
            // Jd and Jp are the digital modem's, so their scrambler is GPC.
            reader: TwoPointReader::differential_with(Mode::Call),
            window: VecDeque::new(),
            trn_left: 0,
            cpt_left: 0,
            trn_from: None,
            trained: false,
            notes: Vec::new(),
        }
    }

    fn take_notes(&mut self) -> Vec<String> {
        std::mem::take(&mut self.notes)
    }

    /// One network sample in (what the digital end sent), one out.
    fn step(&mut self, from_digital: f64) -> f64 {
        self.rx.feed(from_digital);
        while let Some(heard) = self.rx.heard() {
            self.heard(heard);
        }
        // A4: the equaliser trains on the first 2040T of TRN1d. The fit is
        // indexed against the receiver's history, which has slid past the
        // reversal by the time it is taken.
        if let Some(from) = self.trn_from
            && !self.trained
            && self.rx.symbols() >= from + 700
        {
            let base = self.rx.symbols() - self.rx.history_len() as u64;
            if self.rx.train_near(from.saturating_sub(base), 64) {
                self.trained = true;
                self.reader = TwoPointReader::differential_with(Mode::Call);
                self.window.clear();
                self.notes.push(format!("script: TRN1d trained at {:.1} dB", self.rx.snr_db()));
            } else {
                // The fit is a hundred and thirty least squares over forty
                // eight taps: it is not something to ask every sample. Moving
                // the anchor on is what makes the retry cheap and what the
                // digital end does with its own.
                self.trn_from = Some(from + 600);
            }
        }
        let level = self.next();
        self.count += 1;
        level
    }

    fn heard(&mut self, heard: Heard) {
        match heard {
            // Sd is Su's shape the other way up (V.90 8.4.4), so its reversal
            // is the digital end's Sd-to-S-bar-d turn (V.92 A3).
            Heard::Su { at } => self.notes.push(format!("script: Su at {at} (step {:?})", self.step)),
            Heard::Ru { at } => self.notes.push(format!("script: Ru at {at} (step {:?})", self.step)),
            Heard::SuReversal { at } if self.step == Step::Ja => {
                self.notes.push(format!("script: SuReversal at {at}"));
                self.step = Step::TurnedRound;
                self.count = 0;
                // S-bar-d is 48T long and TRN1d follows it (V.90 8.4.4).
                self.trn_from = Some(at + 48);
            }
            Heard::Symbol(symbol) => {
                if !matches!(self.step, Step::TurnedRound | Step::Su) || !self.trained {
                    return;
                }
                let Some(bit) = self.reader.feed(symbol.value) else {
                    return;
                };
                self.window.push_back(bit);
                while self.window.len() > 72 {
                    self.window.pop_front();
                }
                if self.window.len() == 72
                    && let Some(j) = J::from_bits(&self.window.iter().copied().collect::<Vec<_>>())
                {
                    self.heard_frame(j);
                }
            }
            _ => {}
        }
    }

    fn heard_frame(&mut self, j: J) {
        match (self.step, j) {
            (Step::TurnedRound, J::Jd(jd)) => {
                self.notes.push(format!("Jd read, lookahead {}", jd.lookahead));
                self.step = Step::Su;
                self.count = 0;
            }
            (Step::Su, J::Jp(jp)) => {
                self.notes.push(format!("Jp read, epsilon {}", jp.epsilon));
                self.step = Step::AfterJp;
                self.count = 0;
                self.trn_left = 2100;
            }
            _ => {}
        }
    }

    fn next(&mut self) -> f64 {
        let n = self.count;
        match self.step {
            Step::Leading => {
                let lead = (0.070 * FS) as u64;
                if n < lead {
                    0.0
                } else if n < lead + 384 {
                    ru(n - lead, false)
                } else if n < lead + 408 {
                    ru(n - lead, true)
                } else {
                    self.step = Step::Trn1u;
                    self.count = 0;
                    self.trn.next_symbol()
                }
            }
            Step::Trn1u => {
                if n < 2400 {
                    self.trn.next_symbol()
                } else {
                    self.step = Step::Ja;
                    self.count = 0;
                    self.ja_level()
                }
            }
            Step::Ja => self.ja_level(),
            Step::TurnedRound => 0.0,
            Step::Su => {
                if n < 144 {
                    su(n, false)
                } else if n < 168 {
                    su(n, true)
                } else {
                    su(n, false)
                }
            }
            Step::AfterJp => {
                if n < 24 {
                    su(n, true)
                } else if self.trn_left > 0 {
                    self.trn_left -= 1;
                    self.trn.next_symbol()
                } else {
                    self.step = Step::Cpt;
                    self.count = 0;
                    self.cpt_level()
                }
            }
            Step::Cpt => {
                if self.cpt_left == 0 {
                    self.cpt_left = 600;
                    let mut sender = TwoPointSender::after(Trn1u::new());
                    sender.preamble();
                    let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
                    sender.push(&cp.to_bits_in(Layout::V92, 1));
                    self.sender = sender;
                }
                self.cpt_left -= 1;
                self.cpt_level()
            }
        }
    }

    fn ja_level(&mut self) -> f64 {
        if let Some(level) = self.sender.next_symbol() {
            return level;
        }
        // Ja is Table 20's descriptor and the upstream rate mask, behind the
        // 24-one preamble (8.5.4) -- not a Jp, which is what the digital end
        // sends back.
        let mut sender = TwoPointSender::after(Trn1u::new());
        sender.preamble();
        sender.push(&Descriptor::none().to_bits_in(Some(0x7ffff)));
        self.sender = sender;
        self.sender.next_symbol().unwrap_or(0.0)
    }

    fn cpt_level(&mut self) -> f64 {
        if let Some(level) = self.sender.next_symbol() {
            return level;
        }
        let mut sender = TwoPointSender::after(Trn1u::new());
        sender.preamble();
        let cp = Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() };
        sender.push(&cp.to_bits_in(Layout::V92, 1));
        self.sender = sender;
        self.sender.next_symbol().unwrap_or(0.0)
    }
}

/// Phase 3 runs to its end: Ja read, Sd and TRN1d and Jd sent, Su heard, Jp
/// answered, circuit 107 on, CPt read, Phase 4 entered.
#[test]
fn phase_3_completes_and_the_digital_end_reaches_phase_4() {
    let mut modem = digital::Modem::new(settings());
    let mut script = Script::new();
    let mut notes = Vec::new();
    let mut spoke = false;

    // One tick: the script's previous sample goes to the digital end, what
    // the digital end sends comes back to the script. That one-sample delay is
    // the line, and it is what keeps the two of them causally apart.
    let mut up = 0.0;
    for _ in 0..60_000u64 {
        let down = modem.step(up);
        spoke |= down.abs() > 0.0;
        up = script.step(down);
        notes.append(&mut modem.take_notes());
        if modem.status() == Status::Phase4 {
            break;
        }
    }

    let script_notes = script.take_notes();
    eprintln!("--- digital ---\n{notes:#?}\n--- analogue ---\n{script_notes:#?}");
    assert_eq!(modem.status(), Status::Phase4, "phase: {} notes: {notes:?}", modem.phase());
    assert!(modem.descriptor().is_some(), "no Ja: {notes:?}");
    assert!(modem.cpt().is_some(), "no CPt: {notes:?}");
    assert!(modem.circuit107(), "circuit 107 never came on: {notes:?}");
    assert!(spoke, "the digital end never spoke");
}
