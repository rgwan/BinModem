//! Ask the Su hunt what it sees on a capture.
//!
//! Phase 3 ends on a transition: `Su` for 144T, `S-bar-u` for 24.5T, then `Su`
//! until `Jp` (9.5.2.1.6-8), and the digital modem sends `Jp` when it has seen
//! the reversal (9.5.1.1.7-9). When a far end's `Su` is found and then its
//! deadline expires anyway, there are exactly three things it can have been:
//!
//! 1. the reversal was never on the line,
//! 2. it was on the line and under [`TURNED_MATCHED`](datapump::v92::receiver),
//! 3. it was on the line and looked like something else (a step this end did
//!    not expect, so `step` was the wrong number).
//!
//! The three have the same log line and different fixes, and the numbers that
//! tell them apart are inside [`Receiver::recognise`] and are not printed
//! anywhere. This example prints them, on a capture rather than on a call, so
//! that asking the question costs nothing: a trace inside the Su window is a
//! trace in the critical window, and on this bench one of those takes the far
//! end with it.
//!
//! ```text
//! cargo run --release --example v92_su_probe -- FILE [OPTIONS]
//!   --from N        first sample to feed (default 0)
//!   --to N          last sample to feed (default end of file)
//!   --expect su|ru|either   what the timetable says is next (default su)
//!   --step F        how far the far end steps at the reversal (default 0.5)
//!   --every N       print a line every N samples (default 0: summary only)
//!   --train AT      fit TRN1u at absolute sample AT and report what the line
//!                   let the fit have, and what it did not
//!   --search N      how far either side of AT to try that fit (default 8)
//!   --align AT      look for TRN1u's start within N of absolute sample AT
//!   --dump PATH     write that fit, for the error spectrum (`Receiver::dump_fit`)
//! ```

use std::env;
use std::fs;

use datapump::v92::epsilon;
use datapump::v92::receiver::{Heard, Receiver};

/// The gate a reversal has to clear, and the gate a shape has to clear at
/// acquisition -- repeated here so that a captured trace says which one a
/// number failed rather than leaving it to the reader.
const TURNED_GATE: f64 = 0.75;
const MATCHED: f64 = 0.90;

fn value<T: std::str::FromStr>(args: &[String], flag: &str) -> Option<T> {
    let at = args.iter().position(|a| a == flag)?;
    args.get(at + 1)?.parse().ok()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(path) = args.first().filter(|a| !a.starts_with("--")) else {
        eprintln!("usage: v92_su_probe FILE [--from N] [--to N] [--step F] ...");
        return Ok(());
    };
    let bytes = fs::read(path)?;
    let line: Vec<f64> = bytes
        .chunks_exact(2)
        .map(|c| f64::from(i16::from_le_bytes([c[0], c[1]])) / 32768.0)
        .collect();
    let from: usize = value(&args, "--from").unwrap_or(0);
    let to: usize = value(&args, "--to").unwrap_or(line.len());
    let step: f64 = value(&args, "--step").unwrap_or(0.5);
    let every: usize = value(&args, "--every").unwrap_or(0);
    let train: Option<usize> = value(&args, "--train");
    let search: u64 = value(&args, "--search").unwrap_or(8);
    let align: Option<usize> = value(&args, "--align");
    let dump: Option<String> = {
        let at = args.iter().position(|a| a == "--dump");
        at.and_then(|at| args.get(at + 1).cloned())
    };

    println!(
        "{path}: {} samples ({:.3} s at 8 kHz), feeding {from}..{to} ({:.3}..{:.3} s)",
        line.len(),
        line.len() as f64 / 8000.0,
        from as f64 / 8000.0,
        to as f64 / 8000.0
    );

    let mut rx = Receiver::new();
    match args.iter().position(|a| a == "--expect") {
        Some(at) if args.get(at + 1).map(String::as_str) == Some("ru") => {
            rx.expect_ru();
        }
        Some(at) if args.get(at + 1).map(String::as_str) == Some("either") => {
            rx.expect_either();
        }
        _ => {
            rx.expect_su();
        }
    }
    rx.expect_step(step);

    // The two numbers `recognise` decides on, and the epsilon the run is
    // asking for, kept over every period the hunt looked at.
    let (mut worst_turned, mut worst_turned_at) = (f64::NEG_INFINITY, 0usize);
    let (mut best_kept, mut worst_kept, mut over_gate) = (f64::NEG_INFINITY, f64::INFINITY, 0usize);
    let mut periods = 0usize;
    let mut events = 0usize;
    let mut recoveries = 0usize;

    for i in from..to.min(line.len()) {
        rx.feed(line[i]);
        while let Some(heard) = rx.heard() {
            events += 1;
            let what = match heard {
                Heard::Su { at } => format!("Su at {at}"),
                Heard::SuReversal { at } => format!("S-bar-u at {at}"),
                Heard::Ru { at } => format!("Ru at {at}"),
                Heard::RuReversal { at } => format!("R-bar-u at {at}"),
                Heard::Trained { snr_db } => format!("trained at {snr_db:.1} dB"),
                Heard::Lost => "lost".to_owned(),
                Heard::Found => "found".to_owned(),
                Heard::Symbol(_) => continue,
            };
            println!("  sample {i} t={:.4}  {what}", i as f64 / 8000.0);
        }
        // The hunt compares one period in six, at the anchor's own phase, so
        // these are the numbers on the samples it actually looks at.
        let Some((kind, phase, held, stray, step)) = rx.hunt_state() else {
            recoveries += 1;
            continue;
        };
        // `recognise` runs inside `feed`, before the count moves, and its gate
        // is `count % 6 == anchor.phase` with the anchor's phase taken against
        // that same count -- so the periods it compares are the ones where the
        // count *before* this sample lands on it.
        let now = rx.symbols();
        if (now - 1) % 6 != phase {
            continue;
        }
        // The same six samples `recognise` correlates, and the same six: the
        // window it keeps, not the whole history (`history_since` returns
        // everything from an index onward).
        let window = rx.history_range(now.saturating_sub(6), now);
        if window.len() < 6 {
            continue;
        }
        let Some((kept, turned)) = rx.hunt_shares(&window) else { continue };
        periods += 1;
        best_kept = best_kept.max(kept);
        worst_kept = worst_kept.min(kept);
        if turned > worst_turned {
            worst_turned = turned;
            worst_turned_at = i;
        }
        if turned > TURNED_GATE {
            over_gate += 1;
        }
        if every > 0 && periods % every == 0 {
            let lag = epsilon::lag(&window).map_or("-".to_owned(), |lag| format!("{lag:.4}"));
            println!(
                "  sample {i} t={:.4}  {kind} phase {phase} held {held} stray {stray} \
                 step {step:.3}  kept {kept:+.3}  turned {turned:+.3}  lag {lag}",
                i as f64 / 8000.0
            );
        }
    }

    println!(
        "\n  {periods} periods compared, {events} events over the window, \
         {recoveries} samples with no anchor"
    );
    println!(
        "  kept share: {worst_kept:+.3} .. {best_kept:+.3} (gate {MATCHED:.2} to hold, \
         {TURNED_GATE:.2} to turn)"
    );
    println!(
        "  turned share: best {worst_turned:+.3} at sample {worst_turned_at} (t={:.4}), \
         {over_gate} periods over {TURNED_GATE:.2}",
        worst_turned_at as f64 / 8000.0
    );

    if let Some(absolute) = align {
        // Where TRN1u actually starts, by correlating against the sequence
        // rather than by trusting what the call was told: a fit at the wrong
        // alignment reads zero whatever the line is doing.
        let base = rx.history_base();
        let counted = (absolute as u64).saturating_sub(from as u64);
        if counted < base {
            println!("\n  align: sample {absolute} has fallen out of history");
            return Ok(());
        }
        let at = counted - base;
        match rx.align_trn1u(at, search, 512) {
            Some((start, quality)) => println!(
                "\n  TRN1u starts at absolute sample {} (count-relative {start}, quality {quality:.3})",
                start as u64 - base + from as u64
            ),
            None => println!("\n  no TRN1u near {absolute}"),
        }
    }

    if let Some(absolute) = train {
        // `--train` is an absolute sample index, which is what a capture is
        // indexed by; the receiver's history slides, so its own indices are not.
        let base = rx.history_base();
        let counted = (absolute as u64).saturating_sub(from as u64);
        let at = counted.saturating_sub(base) as usize;
        if counted < base {
            println!("\n  TRN1u fit: sample {absolute} has fallen out of history (base {base})");
            return Ok(());
        }
        let took = rx.train_near(at as u64, search);
        println!(
            "\n  TRN1u fit at {at}: {} {:.2} dB, {} feed-forward taps, {} symbols fed",
            if took { "took" } else { "refused" },
            rx.snr_db(),
            rx.taps().len(),
            rx.symbols()
        );
        if let Some(path) = dump {
            rx.dump_fit(std::path::Path::new(&path), at, 512);
            println!("  fit written to {path}");
        }
    }
    Ok(())
}
