//! Dialling a V.90 server: the modem as a terminal drives it, and a server
//! built from the pieces a digital modem is made of -- V.8's answer, V.90's
//! digital start-up and V.42 -- at the far end of a simulated network.

use datapump::v8 as v8line;
use datapump::v34::info::{Info0, Info0d};
use datapump::v90::network::Network;
use datapump::v90::startup::{Digital, Status};
use datapump::v90::ucode::Law;
use ec::stack::Stack;
use ec::{Params, Role as EcRole};
use modem::{Ended, Modem, State};
use v8::{Access, CallFunction, Modulation, Modulations, Pcm, PcmRole};

const FS: f64 = 16_000.0;
const NETWORK_FS: f64 = 8000.0;

/// A V.90 server on the network's side of the call.
struct Server {
    v8: Option<v8line::Modem>,
    startup: Option<Digital>,
    ec: Option<Stack>,
    received: Vec<u8>,
    ticks: u64,
}

impl Server {
    fn new() -> Self {
        let v8 = v8line::Modem::new(
            v8line::Role::Answering,
            CallFunction::Data,
            Modulations::of(&[Modulation::V34Duplex, Modulation::V32bis]),
            NETWORK_FS,
        )
        .offering_lapm()
        .offering_pcm_on(Pcm { digital: true, ..Pcm::default() }, Access { digital: true, ..Access::default() });
        Self { v8: Some(v8), startup: None, ec: None, received: Vec::new(), ticks: 0 }
    }

    fn info0d() -> Info0d {
        Info0d {
            v34: Info0 { constellation_1664: true, ..Info0::default() },
            nominal_power: 4,
            max_power: 23,
            power_at_codec: true,
            a_law: false,
            upstream_3429: false,
        }
    }

    fn step(&mut self, input: f64) -> f64 {
        self.ticks += 1;
        if let Some(v8) = self.v8.as_mut() {
            // V.8 at the network's rate, at a sensible level.
            let out = 0.3 * v8.step(input);
            match v8.status() {
                v8line::Status::Negotiating => {}
                v8line::Status::Agreed(_) => {
                    assert_eq!(v8.pcm_role(), Some(PcmRole::Digital), "V.8 did not settle on V.90");
                    self.v8 = None;
                    self.startup = Some(Digital::new(Self::info0d(), 0.0));
                }
                other => panic!("V.8 came to {other:?}"),
            }
            return out;
        }
        let startup = self.startup.as_mut().expect("V.8 is over");
        let out = startup.step(input);
        if let Status::Connected { transmit, receive } = startup.status() {
            let stack = self.ec.get_or_insert_with(|| {
                let params = Params { t401_ms: ec::lapm::t401_for_line(transmit.min(receive), 60), ..Params::default() };
                Stack::new(EcRole::Answerer, params).over_a_round_trip(60)
            });
            for bit in startup.take_bits() {
                stack.feed_bit(bit);
            }
            while startup.pending_bits() < 256 {
                let bits: Vec<bool> = (0..64).map(|_| stack.next_bit()).collect();
                startup.send_bits(&bits);
            }
            if self.ticks.is_multiple_of(8) {
                stack.tick(1);
            }
            self.received.extend(stack.take_received());
        }
        out
    }
}

struct Call {
    net: Network,
    caller: Modem,
    server: Server,
    up: Vec<f64>,
    said: Vec<u8>,
}

impl Call {
    fn new() -> Self {
        Self {
            net: Network::new(Law::Mu, FS).with_delay(0.015, FS).with_noise(1e-5),
            caller: Modem::new(FS),
            server: Server::new(),
            up: Vec::new(),
            said: Vec::new(),
        }
    }

    fn type_at(&mut self, line: &str) {
        for b in line.bytes() {
            self.caller.feed_dte(b);
        }
        self.caller.feed_dte(b'\r');
    }

    fn run(&mut self, seconds: f64) {
        for _ in 0..(seconds * NETWORK_FS) as usize {
            let to_server = self.net.up(&self.up);
            self.up.clear();
            let from_server = self.server.step(to_server);
            for x in self.net.down(from_server) {
                self.up.push(self.caller.step(x));
                self.said.extend(self.caller.take_dte());
            }
        }
    }

    fn saw(&self) -> String {
        String::from_utf8_lossy(&self.said).into_owned()
    }

    /// The same as [`Self::run`], taking the caller's line notes after every
    /// sample, as the window does, each with how far into the call it came.
    fn run_noting(&mut self, seconds: f64, notes: &mut Vec<(f64, String)>) {
        for _ in 0..(seconds * NETWORK_FS) as usize {
            let to_server = self.net.up(&self.up);
            self.up.clear();
            let from_server = self.server.step(to_server);
            for x in self.net.down(from_server) {
                self.up.push(self.caller.step(x));
                self.said.extend(self.caller.take_dte());
                for text in self.caller.take_line_notes() {
                    notes.push((self.caller.call_seconds(), text));
                }
            }
        }
    }
}

#[test]
fn dialling_a_v90_server_connects_at_pcm_rates_and_carries_text() {
    let mut call = Call::new();
    call.type_at("AT+MS=V90");
    call.run(0.05);
    assert!(call.saw().contains("OK"), "{:?}", call.saw());
    call.type_at("ATD5551234");
    call.run(20.0);
    println!("terminal saw {:?}, standard {}, phase {}", call.saw(), call.caller.standard(), call.caller.line_phase());
    assert_eq!(call.caller.state(), State::Data, "not online: {:?}", call.saw());
    assert_eq!(call.caller.standard(), "V.90");
    let down = call.caller.rate().expect("no rate");
    let up = call.caller.transmit_rate().expect("no sending rate");
    assert!(down >= 48_000, "downstream {down}");
    assert!((24_000..=33_600).contains(&up), "upstream {up}");
    assert!(call.saw().contains(&format!("CONNECT {down}")), "{:?}", call.saw());

    // Text from the terminal reaches the server, through V.42 over V.90.
    call.run(3.0);
    for b in b"hello over fifty-six thousand" {
        call.caller.feed_dte(*b);
    }
    call.run(3.0);
    let got = String::from_utf8_lossy(&call.server.received).into_owned();
    assert!(got.contains("hello over fifty-six thousand"), "the server got {got:?}");
    // And back down.
    if let Some(stack) = call.server.ec.as_mut() {
        stack.send(b"and back again");
    }
    call.run(3.0);
    assert!(call.saw().contains("and back again"), "the terminal saw {:?}", call.saw());

    // A server that retrains (9.5.1) takes the call back through phase 2 and
    // up again, and V.42 carries on over it.
    assert!(call.server.startup.as_mut().unwrap().retrain());
    call.run(0.5);
    assert!(call.caller.retraining(), "the caller never saw the retrain");
    call.run(15.0);
    assert!(!call.caller.retraining(), "still retraining, phase {}", call.caller.line_phase());
    assert_eq!(call.caller.state(), State::Data, "{:?}", call.saw());
    assert_eq!(call.caller.standard(), "V.90");
    for b in b"after the retrain" {
        call.caller.feed_dte(*b);
    }
    call.run(3.0);
    let got = String::from_utf8_lossy(&call.server.received).into_owned();
    assert!(got.contains("after the retrain"), "the server got {got:?}");
}

/// Phase 4 reaches the transcript as it happens, a line for each different
/// thing and not one for every repetition of it: the CPt and CP this end
/// sends, with the rate and drn, K, each interval's size and field, Sr, the
/// look-ahead and the acknowledge bit (Table 14); the MP and MP' the server
/// sends, with the type, the upstream drn, the acknowledge bit and whether it
/// precodes (Table 16); E going up; and Ed and B1d coming down (8.6.1,
/// 8.6.2). In the order 9.4.2 has them: CP after CPt (9.4.2.2), CP' after MP
/// (9.4.2.3), E after CP' and after MP' or Ed (9.4.2.4), and B1d after Ed
/// (9.4.2.6). MP' says the server has "received CP from far end", any CP,
/// so it can come before this end's CP' as well as after.
#[test]
fn the_phase_4_exchange_reaches_the_transcript_a_line_for_each_step() {
    let mut call = Call::new();
    call.type_at("AT+MS=V90");
    call.run(0.05);
    call.type_at("ATD5551234");
    let mut notes = Vec::new();
    call.run_noting(20.0, &mut notes);
    for (at, text) in &notes {
        println!("{at:8.3}  {text}");
    }
    assert_eq!(call.caller.state(), State::Data, "not online: {:?}", call.saw());
    let only = |start: &str| {
        let found: Vec<&(f64, String)> = notes.iter().filter(|n| n.1.starts_with(start)).collect();
        assert_eq!(found.len(), 1, "{start:?} told {} times", found.len());
        found[0].clone()
    };
    let (cpt, cp, mp, cp_ack, mp_ack, e, ed, b1d) = (
        only("sent CPt:"),
        only("sent CP:"),
        only("found MP:"),
        only("sent CP':"),
        only("found MP':"),
        only("sent E:"),
        only("found Ed:"),
        only("found B1d:"),
    );
    assert!(cpt.0 < cp.0 && mp.0 < cp_ack.0 && mp.0 < mp_ack.0, "{notes:#?}");
    assert!(cp_ack.0 < e.0 && mp_ack.0.min(ed.0) < e.0 && ed.0 < b1d.0, "{notes:#?}");
    // Each says what it is asked to say.
    for (line, words) in [
        (&cpt.1, &["bit/s (drn", "K ", "sizes [", "on fields [0, 1, 2, 3, 4, 5]", "Sr ", "look-ahead ", "acknowledge 0"][..]),
        (&cp.1, &["bit/s (drn", "K ", "sizes [", "on fields [0, 1, 2, 3, 4, 5]", "Sr ", "look-ahead ", "acknowledge 0"]),
        (&cp_ack.1, &["acknowledge 1"]),
        (&mp.1, &["type ", "upstream at most", "(drn ", "acknowledge 0", "precoding "]),
        (&mp_ack.1, &["acknowledge 1"]),
    ] {
        for word in words {
            assert!(line.contains(word), "{line:?} does not say {word:?}");
        }
    }
    // And the rate CP asked for is the rate the terminal was told.
    let down = call.caller.rate().expect("no rate");
    assert!(cp.1.contains(&format!("CP: {down} bit/s")), "{} against {down}", cp.1);
    assert!(notes.len() <= 10, "{} lines for one start-up", notes.len());
}

/// Both ends of a V.90 call are these modems: one dials, the other answers
/// as the digital modem through a softphone.
struct Pair {
    net: Network,
    caller: Modem,
    host: Modem,
    up: Vec<f64>,
    /// The softphone at the answering end: G.711 decoded and resampled to
    /// the host's rate; the host's samples resampled to 48 kHz, delayed by a
    /// few samples, and down to the encoder's 8 kHz -- or not resampled at
    /// all, but taken one in two.
    decoded: dsp::Resampler,
    raised: dsp::Resampler,
    delay: std::collections::VecDeque<f64>,
    lowered: dsp::Resampler,
    to_encoder: std::collections::VecDeque<f64>,
    /// No sample rate conversion on the way to the encoder at all: every
    /// other sample of the host's, from the one this says.
    straight: Option<usize>,
    count: usize,
    caller_said: Vec<u8>,
    host_said: Vec<u8>,
}

impl Pair {
    fn new(net: Network, delay: usize) -> Self {
        Self {
            net,
            caller: Modem::new(FS),
            host: Modem::new(FS),
            up: Vec::new(),
            decoded: dsp::Resampler::new(NETWORK_FS, FS),
            raised: dsp::Resampler::new(FS, 48_000.0),
            delay: std::collections::VecDeque::from(vec![0.0; delay]),
            lowered: dsp::Resampler::new(48_000.0, NETWORK_FS),
            to_encoder: std::collections::VecDeque::new(),
            straight: None,
            count: 0,
            caller_said: Vec::new(),
            host_said: Vec::new(),
        }
    }

    fn type_at(modem: &mut Modem, line: &str) {
        for b in line.bytes() {
            modem.feed_dte(b);
        }
        modem.feed_dte(b'\r');
    }

    fn run(&mut self, seconds: f64) {
        let (mut host_in, mut high, mut low) = (Vec::new(), Vec::new(), Vec::new());
        for _ in 0..(seconds * NETWORK_FS) as usize {
            let from_caller = self.net.up(&self.up);
            self.up.clear();
            host_in.clear();
            self.decoded.process(from_caller, &mut host_in);
            for &x in &host_in {
                let y = self.host.step(x);
                self.host_said.extend(self.host.take_dte());
                self.count += 1;
                if let Some(phase) = self.straight {
                    if self.count % 2 == phase {
                        self.to_encoder.push_back(y);
                    }
                    continue;
                }
                high.clear();
                self.raised.process(y, &mut high);
                for &h in &high {
                    self.delay.push_back(h);
                    let Some(late) = self.delay.pop_front() else { continue };
                    low.clear();
                    self.lowered.process(late, &mut low);
                    self.to_encoder.extend(low.iter().copied());
                }
            }
            let level = self.to_encoder.pop_front().unwrap_or(0.0);
            for x in self.net.down(level) {
                self.up.push(self.caller.step(x));
                self.caller_said.extend(self.caller.take_dte());
            }
        }
    }
}

/// Two of these, one dialling and one answering with `AT+MS=V90`: the
/// answering one is V.90's digital modem, behind a softphone that hands every
/// other sample it is given to its G.711 encoder. Which of its samples that is
/// depends on when the call began. Taking the codewords, the call comes up at
/// PCM rates; taking the samples between them, the DIL shows nothing V.90
/// could use and the call comes up as V.34 instead. Text crosses both ways
/// either way.
#[test]
fn two_of_these_connect_at_pcm_rates_when_the_codewords_reach_the_encoder() {
    let mut rates = Vec::new();
    for phase in [0, 1] {
        let mut pair = Pair::new(Network::new(Law::Mu, FS).with_delay(0.015, FS).with_noise(1e-5), 0);
        pair.straight = Some(phase);
        Pair::type_at(&mut pair.host, "AT+MS=V90");
        Pair::type_at(&mut pair.caller, "AT+MS=V90");
        pair.run(0.05);
        Pair::type_at(&mut pair.host, "ATA");
        Pair::type_at(&mut pair.caller, "ATD5551234");
        for _ in 0..40 {
            pair.run(1.0);
            if pair.caller.state() == State::Data && pair.host.state() == State::Data {
                break;
            }
        }
        let saw = String::from_utf8_lossy(&pair.caller_said).into_owned();
        println!("phase {phase}: {} at {:?} down, {:?} up: {saw:?}", pair.caller.standard(), pair.caller.rate(), pair.caller.transmit_rate());
        assert_eq!(pair.caller.state(), State::Data, "phase {phase}: {saw:?}");
        assert_eq!(pair.host.state(), State::Data, "phase {phase}");
        assert_eq!(pair.caller.standard(), pair.host.standard());
        rates.push((pair.caller.standard(), pair.caller.rate().unwrap_or(0)));
        // Each says which half of V.90 the other offered in V.8.
        let modulations = |m: &Modem| m.distant().into_iter().find(|r| r.0 == "modulations").map(|r| r.1).unwrap_or_default();
        assert!(modulations(&pair.host).starts_with("V.90 analogue, V.34"), "{}", modulations(&pair.host));
        assert!(modulations(&pair.caller).starts_with("V.90 digital, V.34"), "{}", modulations(&pair.caller));
        // And the server's scope is sized for the hundreds of points coming
        // up, which reach well past one.
        if pair.host.standard() == "V.90" {
            assert!(pair.host.constellation_peak() > 1.2, "peak {}", pair.host.constellation_peak());
        }

        pair.run(2.0);
        for b in b"from the caller" {
            pair.caller.feed_dte(*b);
        }
        for b in b"from the server" {
            pair.host.feed_dte(*b);
        }
        pair.run(3.0);
        let host_saw = String::from_utf8_lossy(&pair.host_said).into_owned();
        assert!(host_saw.contains("from the caller"), "phase {phase}: the server saw {host_saw:?}");
        let caller_saw = String::from_utf8_lossy(&pair.caller_said).into_owned();
        assert!(caller_saw.contains("from the server"), "phase {phase}: the caller saw {caller_saw:?}");
        // Compressed, and said so. Both panels used to say "none offered"
        // here while both ends ran V.44, which is where the README's "not
        // offered correctly" came from.
        for (who, m) in [("caller", &pair.caller), ("server", &pair.host)] {
            assert_eq!(m.compression_name(), Some("V.44"), "phase {phase}: the {who}");
            let running = m.distant().into_iter().find(|r| r.0 == "compression").map(|r| r.1).unwrap_or_default();
            assert!(running.starts_with("V.44, "), "phase {phase}: the {who}'s panel said {running:?}");
        }
    }
    rates.sort_by_key(|r| r.1);
    assert_eq!(rates[0], ("V.34", 33_600), "{rates:?}");
    assert_eq!(rates[1].0, "V.90", "{rates:?}");
    assert!(rates[1].1 >= 48_000, "{rates:?}");
}

/// Bring a pair up in V.90 data mode with V.42 running, the way every test
/// below needs it.
fn connected_pair() -> Pair {
    let mut pair = Pair::new(Network::new(Law::Mu, FS).with_delay(0.015, FS).with_noise(1e-5), 0);
    pair.straight = Some(0);
    Pair::type_at(&mut pair.host, "AT+MS=V90");
    Pair::type_at(&mut pair.caller, "AT+MS=V90");
    pair.run(0.05);
    Pair::type_at(&mut pair.host, "ATA");
    Pair::type_at(&mut pair.caller, "ATD5551234");
    for _ in 0..40 {
        pair.run(1.0);
        if pair.caller.state() == State::Data && pair.host.state() == State::Data {
            break;
        }
    }
    assert_eq!((pair.caller.state(), pair.host.state()), (State::Data, State::Data));
    // Which of V.90 and V.34 does not matter here: what these tests are about
    // is what happens to a connection V.42 is carrying, and V.42 is either.
    pair.run(1.0);
    pair.caller_said.clear();
    pair.host_said.clear();
    pair
}

/// Run until `done`, in the smallest steps the harness has, and say how long
/// it took and when each side noticed the call was over.
fn run_until(pair: &mut Pair, limit: f64, done: impl Fn(&Pair) -> bool) -> f64 {
    let mut waited = 0.0;
    while !done(pair) && waited < limit {
        pair.run(0.05);
        waited += 0.05;
    }
    waited
}

/// V.42 8.7.2, from the terminal's side: a hang-up on a connection V.42 is
/// carrying is a DISC the far end answers before the carrier goes.
///
/// The order is the whole point. A carrier that stops between two samples is
/// what a cut line looks like, and a far end that loses one reports the call
/// as a carrier loss -- the one outcome a hang-up is not. So: the terminal is
/// answered at once, the line stays up until the far end has acknowledged the
/// DISC, and the far end ends the call as a peer clear rather than as a fault.
#[test]
fn a_hang_up_sends_a_disc_before_the_carrier_goes() {
    let mut pair = connected_pair();
    assert_eq!(pair.caller.compression_name(), Some("V.44"), "V.42 was not up");

    pair.caller.hang_up();
    pair.caller_said.extend(pair.caller.take_dte());
    // The terminal has been answered and the carrier is still there: nothing
    // has been said to the far end yet.
    let said = String::from_utf8_lossy(&pair.caller_said).into_owned();
    assert!(said.contains("OK"), "the terminal was not answered at once: {said:?}");
    assert_eq!(pair.caller.state(), State::Data, "the carrier went before anything was said");

    let mut cleared_at = None;
    let mut waited = 0.0;
    while pair.caller.state() != State::Command && waited < 10.0 {
        pair.run(0.05);
        waited += 0.05;
        if cleared_at.is_none() && pair.host.ended_because() == Some(Ended::PeerCleared) {
            cleared_at = Some(waited);
        }
    }
    let said = String::from_utf8_lossy(&pair.caller_said).into_owned();
    let notes = pair.caller.take_line_notes().join(" | ");
    let what = format!(
        "after {waited} s the caller is {:?} and said {said:?}, notes {notes:?}, the far end's excuse {:?}",
        pair.caller.state(),
        pair.host.ended_because()
    );
    assert_eq!(pair.caller.state(), State::Command, "{what}");
    // The far end ended on the announcement, which is only possible if the
    // DISC arrived while its carrier was still up -- i.e. before the carrier
    // went.
    assert_eq!(pair.host.ended_because(), Some(Ended::PeerCleared), "{what}");
    assert!(cleared_at.is_some_and(|at| at < waited), "{what}");
    let far_said = String::from_utf8_lossy(&pair.host_said).into_owned();
    assert!(far_said.contains("NO CARRIER"), "{what}: the far end said {far_said:?}");
    // Every DISC that was announced, and how it ended, is in one line.
    assert!(notes.contains("hang-up: DISC"), "{what}");
    // The far end's own record is the proof the DISC arrived while its
    // carrier was up: only a DISC that it read could have ended its call as a
    // peer clear, and it did that before this end's carrier went.
    assert!(pair.host.ended_because() == Some(Ended::PeerCleared), "{what}");
}

/// The same from the other end: the far end's DISC is a *peer* clear, so the
/// terminal gets NO CARRIER and the record says the peer cleared the call --
/// not that the carrier failed.
#[test]
fn a_disc_from_the_far_end_ends_the_call_as_a_peer_clear() {
    let mut pair = connected_pair();
    pair.host.hang_up();
    let waited = run_until(&mut pair, 10.0, |p| p.caller.state() == State::Command);
    // The UA this end owed the far end is on the line behind the carrier
    // going, so the far end needs a moment to read its own answer.
    pair.run(0.25);
    let said = String::from_utf8_lossy(&pair.caller_said).into_owned();
    let notes = pair.caller.take_line_notes().join(" | ");
    let what = format!(
        "after {waited} s the caller is {:?} and said {said:?}, notes {notes:?}, excuse {:?}",
        pair.caller.state(),
        pair.caller.ended_because()
    );
    assert_eq!(pair.caller.state(), State::Command, "{what}");
    assert_eq!(pair.caller.ended_because(), Some(Ended::PeerCleared), "{what}");
    assert!(said.contains("NO CARRIER"), "{what}");
    // And it is recorded as a peer clear on this side, not as a fault.
    assert!(notes.contains("peer DISC"), "{what}");
}

/// A call V.42 is not carrying has nothing to announce itself with, and the
/// line goes at once exactly as it always did -- said so in the log line, so
/// that a batch of calls stays classifiable.
#[test]
fn a_hang_up_without_v42_drops_the_line_at_once() {
    let mut pair = Pair::new(Network::new(Law::Mu, FS).with_delay(0.015, FS).with_noise(1e-5), 0);
    pair.straight = Some(0);
    // V.250 Table 20's `<orig_rqst>` of 0: no error control to negotiate, so
    // no V.42 here to say goodbye with.
    Pair::type_at(&mut pair.host, "AT+MS=V90");
    Pair::type_at(&mut pair.caller, "AT+MS=V90");
    Pair::type_at(&mut pair.caller, "AT+ES=0");
    pair.run(0.05);
    Pair::type_at(&mut pair.host, "ATA");
    Pair::type_at(&mut pair.caller, "ATD5551234");
    let mut waited = 0.0;
    while (pair.caller.state() != State::Data || pair.host.state() != State::Data) && waited < 40.0 {
        pair.run(1.0);
        waited += 1.0;
    }
    assert_eq!((pair.caller.state(), pair.host.state()), (State::Data, State::Data));
    pair.run(1.0);
    pair.caller_said.clear();
    pair.caller.hang_up();
    pair.caller_said.extend(pair.caller.take_dte());
    let said = String::from_utf8_lossy(&pair.caller_said).into_owned();
    let notes = pair.caller.take_line_notes().join(" | ");
    let what = format!("the caller is {:?} and said {said:?}, notes {notes:?}", pair.caller.state());
    // No wait at all: the carrier is down on the same call.
    assert_eq!(pair.caller.state(), State::Command, "{what}");
    assert!(said.contains("OK"), "{what}");
    assert!(notes.contains("not connected: no DISC"), "{what}");
    // And nothing was said on the line before it went.
    assert!(notes.contains("0 DISC sent"), "{what}");
}

/// Two of these in V.90 data mode, then one puts the line down: the other
/// says NO CARRIER and goes back to command state by itself, a few seconds
/// later. It used to stay in data mode, sending PCM or V.34 at a far end that
/// had gone, until somebody stopped it.
#[test]
fn when_one_end_hangs_up_the_other_notices() {
    for host_hangs_up in [true, false] {
        let mut pair = Pair::new(Network::new(Law::Mu, FS).with_delay(0.015, FS).with_noise(1e-5), 0);
        pair.straight = Some(0);
        Pair::type_at(&mut pair.host, "AT+MS=V90");
        Pair::type_at(&mut pair.caller, "AT+MS=V90");
        pair.run(0.05);
        Pair::type_at(&mut pair.host, "ATA");
        Pair::type_at(&mut pair.caller, "ATD5551234");
        for _ in 0..40 {
            pair.run(1.0);
            if pair.caller.state() == State::Data && pair.host.state() == State::Data {
                break;
            }
        }
        assert_eq!((pair.caller.state(), pair.host.state()), (State::Data, State::Data));
        assert_eq!(pair.caller.standard(), "V.90");
        pair.run(2.0);
        pair.caller_said.clear();
        pair.host_said.clear();
        if host_hangs_up {
            pair.host.hang_up();
        } else {
            pair.caller.hang_up();
        }
        let mut waited = 0.0;
        let left = |pair: &Pair| if host_hangs_up { pair.caller.state() } else { pair.host.state() };
        while left(&pair) != State::Command && waited < 10.0 {
            pair.run(0.25);
            waited += 0.25;
        }
        let said = String::from_utf8_lossy(if host_hangs_up { &pair.caller_said } else { &pair.host_said }).into_owned();
        println!("host hangs up {host_hangs_up}: the other end noticed after {waited} s and said {said:?}");
        assert_eq!(left(&pair), State::Command, "host hangs up {host_hangs_up}: still {:?} after {waited} s", left(&pair));
        assert!(waited <= 4.0, "host hangs up {host_hangs_up}: {waited} s");
        assert!(said.contains("NO CARRIER"), "host hangs up {host_hangs_up}: {said:?}");
    }
}

/// V.42 8.7.2 through a retrain: a call put down while the line is being
/// rebuilt still announces itself with a DISC.
///
/// The DISC cannot go out until phase 4 has given the transmitter a bit
/// stream back, so the release waits for that instead of concluding on the
/// first sample. It used to conclude at once -- "DISC timeout after 0 retries;
/// 0 DISC sent" -- and bring the carrier down with the frame still queued,
/// which left the far end with a carrier that stopped: the one outcome the
/// DISC exists to prevent, reported as a timeout that never happened.
#[test]
fn a_hang_up_during_a_retrain_still_sends_the_disc() {
    let mut pair = connected_pair();
    assert_eq!(pair.caller.compression_name(), Some("V.44"), "V.42 was not up");
    // The far end retrains (V.90 9.5.1), and the terminal is put down in the
    // middle of it: the data mode is gone, so nothing on the line can carry a
    // DISC at that instant.
    pair.caller.ask_for_retrain();
    pair.run(0.4);
    assert!(
        pair.caller.retraining(),
        "the caller never saw the retrain: phase {}",
        pair.caller.line_phase()
    );
    pair.caller.hang_up();
    pair.caller_said.extend(pair.caller.take_dte());
    let said = String::from_utf8_lossy(&pair.caller_said).into_owned();
    assert!(said.contains("OK"), "the terminal was not answered at once: {said:?}");
    // The carrier is still up: the DISC is queued and waiting for the line.
    assert_ne!(pair.caller.state(), State::Command, "the carrier went with the line down");

    let waited = run_until(&mut pair, 40.0, |p| p.caller.state() == State::Command);
    // One UA's worth of line after the end, so the far end has read its own
    // answer before it is asked what it made of the call.
    pair.run(0.25);
    let notes = pair.caller.take_line_notes().join(" | ");
    let what = format!(
        "after {waited:.2} s the caller is {:?}, notes {notes:?}, the far end's excuse {:?}",
        pair.caller.state(),
        pair.host.ended_because()
    );
    assert_eq!(pair.caller.state(), State::Command, "{what}");
    // A DISC went out: at least one, and never the report of a goodbye that
    // was never attempted.
    assert!(notes.contains("hang-up: DISC"), "{what}");
    assert!(!notes.contains("0 DISC sent"), "{what}");
    assert!(!notes.contains("0 retries"), "{what}");
    // Which is the whole point: the far end read a goodbye, not a carrier
    // that stopped. A far end left without one reports a fault -- V.42 has no
    // other way to tell a hang-up from a cut line.
    assert_eq!(pair.host.ended_because(), Some(Ended::PeerCleared), "{what}");
}
