//! Where the second TRN1u begins, and what the refit reads there.
//!
//! The refit that follows Jp (9.5.2.1.9) is a least-squares fit of the line
//! against the sequence this end generates, so it is a fit against *the right
//! sequence at the right symbol* or it is nothing: an alignment one symbol out
//! is a fit against a different stream, and on the bench that reads 6.4 dB
//! where the same fit at the right alignment reads 13.6. The alignment is
//! therefore the whole of the question, and this example answers it on a
//! capture rather than on a call -- a probe in the refit's own window would be a
//! probe in the critical window (`docs/93`).
//!
//! ```text
//! cargo run --release --example v92_refit_probe -- FILE [OPTIONS]
//!   --from N --to N      samples of the capture to feed (default all)
//!   --repeat A B [--step S]
//!                        the six-symbol repetition measure from absolute
//!                        sample A to B, one line each S samples
//!   --ends A B           where `su_ends` puts TRN1u's start, searched from A
//!                        to B
//!   --fit A B [--every S]
//!                        what the training fit reads with TRN1u's first symbol
//!                        at each absolute sample in A..B, every S
//! ```

use std::env;
use std::fs;

use datapump::v92::receiver::{LadderStep, Receiver};

fn values<T: std::str::FromStr>(args: &[String], flag: &str) -> Option<[T; 2]> {
    let at = args.iter().position(|a| a == flag)?;
    Some([args.get(at + 1)?.parse().ok()?, args.get(at + 2)?.parse().ok()?])
}

fn value<T: std::str::FromStr>(args: &[String], flag: &str) -> Option<T> {
    let at = args.iter().position(|a| a == flag)?;
    args.get(at + 1)?.parse().ok()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(path) = args.first().filter(|a| !a.starts_with("--")) else {
        eprintln!("usage: v92_refit_probe FILE [--from N] [--to N] ...");
        return Ok(());
    };
    let bytes = fs::read(path)?;
    let line: Vec<f64> = bytes
        .chunks_exact(2)
        .map(|c| f64::from(i16::from_le_bytes([c[0], c[1]])) / 32768.0)
        .collect();
    let from: usize = value(&args, "--from").unwrap_or(0);
    let to: usize = value(&args, "--to").unwrap_or(line.len());
    println!(
        "{path}: {} samples ({:.3} s at 8 kHz), feeding {from}..{to} ({:.3}..{:.3} s)",
        line.len(),
        line.len() as f64 / 8000.0,
        from as f64 / 8000.0,
        to as f64 / 8000.0
    );

    let mut rx = Receiver::new();
    for &sample in &line[from..to.min(line.len())] {
        rx.feed(sample);
    }
    // The capture is indexed in samples and the receiver in symbols it was fed:
    // with the whole file fed they are the same index, which is what makes the
    // numbers below comparable with a bench log's own symbol counts.
    let base = rx.history_base();
    println!(
        "  history: base {base}, {} samples, symbols fed {}",
        rx.history_len(),
        rx.symbols()
    );
    let absolute = |at: u64| at + from as u64 - base;

    if let Some([a, b]) = values::<u64>(&args, "--repeat") {
        let step: u64 = value(&args, "--step").unwrap_or(1);
        let window: usize = value(&args, "--window").unwrap_or(24);
        println!("\n  six-symbol repetition, window {window}:");
        let mut at = a.max(base + from as u64);
        while at <= b {
            match rx.repetition(at, window) {
                Some(r) => println!("  {at} t={:.4} repetition {r:+.3}", at as f64 / 8000.0),
                None => println!("  {at} t={:.4} no measurement", at as f64 / 8000.0),
            }
            at += step;
        }
    }

    if let Some([a, _b]) = values::<u64>(&args, "--ends") {
        let window: usize = value(&args, "--window").unwrap_or(24);
        let hold: usize = value(&args, "--hold").unwrap_or(48);
        println!("\n  su_ends looking back {a} (window {window}, hold {hold}):");
        match rx.su_ends(a, window, hold) {
            Some(at) => println!("  the six-symbol upstream ends at absolute {at} (t={:.4})", at as f64 / 8000.0),
            None => println!("  still six-periodic: no second TRN1u in that stretch"),
        }
    }

    if let Some([a, b]) = values::<u64>(&args, "--fit") {
        let step: u64 = value(&args, "--every").unwrap_or(1);
        let window: usize = value(&args, "--window").unwrap_or(600);
        println!("\n  training fit with TRN1u's first symbol at each alignment:");
        let mut best = (f64::NEG_INFINITY, 0u64);
        let mut at = a;
        while at <= b {
            let index = at.saturating_sub(base);
            if rx.train_verified(index, window) {
                let snr = rx.snr_db();
                if snr > best.0 {
                    best = (snr, at);
                }
                println!("  {at} t={:.4} fit {snr:.2} dB", at as f64 / 8000.0);
            } else {
                println!("  {at} t={:.4} no fit", at as f64 / 8000.0);
            }
            at += step;
        }
        println!("\n  best alignment {b0} at {b1:.2} dB", b0 = best.1, b1 = best.0);
    }
    if let Some([a, b]) = values::<u64>(&args, "--profile") {
        // What the correlation says at every whole-symbol start in a range,
        // against what the fit reads there: the two numbers the refit's search
        // is made of, side by side.
        let every: u64 = value(&args, "--every").unwrap_or(1);
        let window: usize = value(&args, "--window").unwrap_or(512);
        let fit_window: usize = value(&args, "--fit-window").unwrap_or(1200);
        println!("\n  profile, correlation (window {window}) and fit (window {fit_window}):");
        let mut at = a;
        while at <= b {
            let index = at.saturating_sub(base);
            let quality = rx.align_trn1u(index, 0, window).map(|(_, q)| q);
            // The same window with no equaliser at all: one tap, a gain. That
            // is the correlation the search is *meant* to be reading, and the
            // two numbers together say whether the profile is measuring the
            // sequence or the search's own normalisation.
            let plain = rx.score_of(&[1.0], &[], index as usize, 0, window);
            let fit = rx.train_verified(index, fit_window).then(|| rx.snr_db());
            println!(
                "  {at} t={:.4} quality {:.3} one-tap {} fit {}",
                at as f64 / 8000.0,
                quality.unwrap_or(f64::NAN),
                plain.map_or("-".to_owned(), |db| format!("{db:.2} dB")),
                fit.map_or("-".to_owned(), |db| format!("{db:.2} dB"))
            );
            at += every;
        }
    }

    if let Some([a, b]) = values::<u64>(&args, "--locator") {
        // A cheap fit's profile: which alignment does a short section like,
        // against which one the full section likes?
        let every: u64 = value(&args, "--every").unwrap_or(4);
        let taps: usize = value(&args, "--taps").unwrap_or(32);
        let short: usize = value(&args, "--window").unwrap_or(192);
        println!("\n  locator, {taps} taps over {short} samples, with the full fit beside it:");
        let mut at = a;
        while at <= b {
            let cheap = rx.fit_alignment_with(at, short, taps);
            let full = rx.fit_alignment(at, 1200);
            println!(
                "  {at} t={:.4} {taps}tap {} full {}",
                at as f64 / 8000.0,
                cheap.map_or("-".to_owned(), |db| format!("{db:.2}")),
                full.map_or("-".to_owned(), |db| format!("{db:.2}"))
            );
            at += every;
        }
    }

    if let Some(onset) = value::<u64>(&args, "--sweep") {
        // The measured walk the refit now makes: from where the six-symbol
        // upstream ended, coarse to fine, choosing the alignment the fit likes.
        let search: u64 = value(&args, "--search").unwrap_or(128);
        let step: u64 = value(&args, "--every").unwrap_or(8);
        let fit_window: usize = value(&args, "--fit-window").unwrap_or(1200);
        println!("\n  ladder from {onset} up {search} by {step}, fit window {fit_window}:");
        // One rung per call, as the sample pump takes it.
        let mut rungs = 0;
        let mut done = None;
        loop {
            rungs += 1;
            match rx.ladder_step(onset, search, step, fit_window) {
                LadderStep::Continue => continue,
                LadderStep::Done(at, fit) => done = Some((at, fit)),
                LadderStep::Empty => {}
            }
            break;
        }
        println!("  {rungs} rungs");
        match done {
            Some((at, fit)) => {
                println!("  TRN1u's first symbol at {at} (t={:.4}), fit {fit:.2} dB", at as f64 / 8000.0);
                let taken = rx.train_verified(at.saturating_sub(base), fit_window);
                println!("  taken {taken} at {:.2} dB", rx.snr_db());
            }
            None => println!("  no alignment fitted"),
        }
    }

    if let Some(a) = value::<u64>(&args, "--alignment") {
        // The production sequence, exactly: `refit_from`'s own estimate, the
        // correlation search the digital modem runs around it, the fit that
        // chooses between the correlation's shortlist, and the fit the caller
        // then takes at the winner.
        let index = a.saturating_sub(base);
        let window: usize = value(&args, "--window").unwrap_or(512);
        let fit_window: usize = value(&args, "--fit-window").unwrap_or(1200);
        let search: u64 = value(&args, "--search").unwrap_or(192);
        println!("\n  alignment from {a} (index {index}) +/-{search}, window {window}:");
        match rx.align_trn1u(index, search, window) {
            Some((start, quality)) => {
                println!("  correlation peaks: starts at {start} quality {quality:.3}")
            }
            None => println!("  correlation: nothing"),
        }
        match rx.align_trn1u_by_fit(index, search, window, 0.5, fit_window) {
            Some((start, fit)) => {
                println!("  chosen by fit: {start} at {fit:.2} dB");
                let taken = rx.train_verified(start, fit_window);
                println!("  the fit the caller takes: {taken} at {:.2} dB, {} taps", rx.snr_db(), rx.taps().len());
            }
            None => println!("  no alignment at all"),
        }
    }

    let _ = absolute(0);
    Ok(())
}
