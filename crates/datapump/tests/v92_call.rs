//! A V.92 call: the digital modem of [`datapump::v92::digital`] against the
//! analogue modem of [`datapump::v92::analogue`], one line sample at a time.
//!
//! What it proves is Phase 3 end to end. The analogue end sends Ru and its bar,
//! TRN1u and Ja, goes silent, has its equaliser trained by TRN1d, reads Jd, sends
//! Su, reads Jp and raises circuit 107, sends the second TRN1u while the DIL
//! arrives, then CPt until Ri and E1u; the digital end reads Ja, sends Sd,
//! S-bar-d, TRN1d and Jd, measures Su, answers with Jp, reads CPt, sends Ri, Ri
//! gone round and reaches Phase 4.
//!
//! Phase 4's exchange and the PCM upstream carrying bytes are V92-35/V92-39/
//! V92-41: what is here stops at E1u on one side and TRN2d on the other.

use datapump::v90::sequences::{Cp, Descriptor};
use datapump::v90::ucode::Law;
use datapump::v92::analogue;
use datapump::v92::digital;

/// Both ends' own rate: one symbol per 125 microseconds.
const DEFAULT_STEPS: u64 = 60_000;

fn digital_settings(far_md: u8) -> digital::Settings {
    digital::Settings {
        law: Law::Mu,
        uinfo: 79,
        far_md,
        round_trip: 0.0,
        wide: true,
        habits: digital::Habits::PROMPT,
    }
}

/// A CPt offering 40 000 bit/s, which is what an analogue end on a clean line
/// asks for.
fn cpt() -> Cp {
    Cp { drn: 16, constellations: vec![(1u128 << 40) - 1], ..Cp::default() }
}

fn analogue_settings(md_length: u8) -> analogue::Settings {
    analogue::Settings {
        md_length,
        ..analogue::Settings::new(Descriptor::none(), 0x7ffff, cpt())
    }
}

/// The two modems on one line, with one sample of delay between them.
struct Call {
    digital: digital::Modem,
    analogue: analogue::Modem,
    /// What the analogue sent last, which is what the digital end hears now.
    up: f64,
    digital_notes: Vec<String>,
    analogue_notes: Vec<String>,
}

impl Call {
    fn new(digital: digital::Settings, analogue: analogue::Settings) -> Self {
        Self {
            digital: digital::Modem::new(digital),
            analogue: analogue::Modem::new(analogue),
            up: 0.0,
            digital_notes: Vec::new(),
            analogue_notes: Vec::new(),
        }
    }

    /// One tick: the analogue's previous symbol goes to the digital end, what the
    /// digital end sends comes back. That one-sample delay is the line, and it is
    /// what keeps the two of them causally apart.
    fn tick(&mut self) {
        let down = self.digital.step(self.up);
        self.up = self.analogue.step(down);
        self.digital_notes.append(&mut self.digital.take_notes());
        self.analogue_notes.append(&mut self.analogue.take_notes());
    }

    /// Run until `done`, and say whether it ever was.
    fn run_until(&mut self, done: impl Fn(&Self) -> bool, steps: u64) -> bool {
        for _ in 0..steps {
            if done(self) {
                return true;
            }
            self.tick();
        }
        done(self)
    }
}

/// Phase 3 runs to its end: Ja read, Sd and TRN1d and Jd sent, Su heard, Jp
/// answered, circuit 107 on, CPt read on one side and E1u sent on the other.
#[test]
fn phase_3_completes_and_the_digital_end_reaches_phase_4() {
    let mut call = Call::new(digital_settings(0), analogue_settings(0));
    let reached = call.run_until(
        |call| {
            call.digital.status() == digital::Status::Phase4
                && call.analogue.status() == analogue::Status::Phase3Done
        },
        DEFAULT_STEPS,
    );
    eprintln!(
        "--- digital ---\n{:#?}\n--- analogue ---\n{:#?}",
        call.digital_notes, call.analogue_notes
    );
    assert!(reached, "digital {:?}, analogue {:?}", call.digital.phase(), call.analogue.phase());

    // The digital end's half.
    assert!(call.digital.descriptor().is_some(), "no Ja: {:?}", call.digital_notes);
    assert!(call.digital.cpt().is_some(), "no CPt: {:?}", call.digital_notes);
    assert!(call.digital.circuit107(), "circuit 107 never came on: {:?}", call.digital_notes);
    assert_eq!(call.digital.cpt().map(|cp| cp.drn), Some(cpt().drn), "the CPt this end sent");

    // The analogue end's half.
    assert!(call.analogue.far_jd().is_some(), "no Jd: {:?}", call.analogue_notes);
    assert!(call.analogue.jp().is_some(), "no Jp: {:?}", call.analogue_notes);
    assert!(call.analogue.circuit107(), "circuit 107 never came on: {:?}", call.analogue_notes);
    // 9.5.2.1.7-8: half a symbol at the S-bar-u after Su, then the epsilon Jp
    // carried at the one after Jp. That epsilon is the digital end's measurement
    // of Su (9.5.1.1.6-7): whatever the phase turned out to be, both ends have
    // to be talking about the same number, which is what this holds together.
    let delays = call.analogue.delays();
    assert_eq!(delays.first(), Some(&0.5), "the half symbol the S-bar-u gives");
    let epsilon = call.analogue.epsilon().expect("Jp carried an epsilon");
    assert_eq!(delays.get(1), Some(&epsilon), "the analogue applied what Jp said");
    assert!((0.0..1.0).contains(&epsilon), "a fraction of a symbol: {epsilon}");
}

/// The same call with an MD in it: this end announces 3 x 276 = 828 symbols in
/// INFO1a, sends it between the two Ru reversals, and the digital end waits
/// exactly that long before it hunts Ru again (9.5.1.1.1).
#[test]
fn phase_3_completes_with_md() {
    let mut call = Call::new(digital_settings(3), analogue_settings(3));
    let reached = call.run_until(
        |call| {
            call.digital.status() == digital::Status::Phase4
                && call.analogue.status() == analogue::Status::Phase3Done
        },
        DEFAULT_STEPS,
    );
    eprintln!(
        "--- digital ---\n{:#?}\n--- analogue ---\n{:#?}",
        call.digital_notes, call.analogue_notes
    );
    assert!(reached, "digital {:?}, analogue {:?}", call.digital.phase(), call.analogue.phase());

    assert!(
        call.analogue_notes.iter().any(|note| note.contains("MD of 828 symbols")),
        "no MD was sent: {:?}",
        call.analogue_notes
    );
    assert!(
        call.digital_notes.iter().any(|note| note.contains("MD")),
        "the digital end never waited out an MD: {:?}",
        call.digital_notes
    );
    assert!(call.digital.cpt().is_some(), "no CPt: {:?}", call.digital_notes);
    assert!(call.analogue.jp().is_some(), "no Jp: {:?}", call.analogue_notes);
}

/// A digital end that never sends Sd leaves Ja unanswered, and AR1's 1500 ms --
/// counted from the start of Ja and with no round-trip term -- is what notices.
#[test]
fn a_digital_end_that_never_sends_sd_fails_analogue() {
    let mut modem = analogue::Modem::new(analogue_settings(0));
    let mut ja_began = None;
    let mut failed = false;
    for _ in 0..40_000u64 {
        // Nothing on the line at all: no Sd, no S-bar-d, no TRN1d.
        modem.step(0.0);
        if modem.phase() == "V.92 phase 3: Ja" && ja_began.is_none() {
            ja_began = Some(modem.elapsed());
        }
        if let analogue::Status::Failed(why) = modem.status() {
            let began = ja_began.expect("Ja went out");
            assert_eq!(why, "no Sd reversal within 1500 ms of Ja");
            assert!(
                modem.elapsed() >= began + 12_000,
                "AR1 is 1500 ms: {} symbols after Ja began",
                modem.elapsed() - began
            );
            assert!(modem.elapsed() <= began + 12_200, "and it is a deadline, not a mood");
            failed = true;
            break;
        }
    }
    let notes = modem.take_notes();
    assert!(failed, "no failure in 40 000 symbols: {} notes {notes:?}", modem.phase());
    assert!(notes.iter().any(|note| note.contains("no Sd reversal")), "{notes:?}");
}
