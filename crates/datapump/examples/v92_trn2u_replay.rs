//! Replay a captured 8 kHz upstream through `Receiver::train_on_trn2u` and say
//! what the four-point fit measures, for every origin and every window end.
//!
//! The question this answers is one the bench cannot: the harness prints one
//! line per call, `no phase 4 fit on TRN2u`, and the same line is printed by a
//! window that is in the wrong place, an origin that is a few hundred symbols
//! off, and a fit that simply measured less than the design it replaced. On a
//! capture every one of those can be varied while the line stays the same.
//!
//! What it does, in the code's own order (`digital.rs:4968-4990`,
//! `digital.rs:5032`):
//!
//! ```text
//! for each window end E:      feed the capture up to E
//!     for each candidate origin O:
//!         receiver trained only on the capture up to E, cloned
//!         train_on_trn2u(TRAIN_WINDOW, O)   -- the fit, exactly as the modem calls it
//!         report (snr, took?)
//! ```
//!
//! A fresh `Receiver` has `snr_db` 0, so `train_on_trn2u`'s own gate
//! (`found.snr() > before.snr() - 3.0`, `receiver.rs:3752`) accepts any fit that
//! `fit_at` completed: what this prints is the fit and not the gate on top of
//! it. The origin the modem used and the window it read are named on the
//! command line so the capture's numbers and the log's line up.
//!
//! ```text
//! cargo run --release -p datapump --example v92_trn2u_replay -- \
//!     --raw /tmp/hsf-v90-interop/<run>/rx8.raw --end 129920 \
//!     --origin 127520 --sweep 2000 --step 40
//! ```

use std::env;
use std::fs;

use datapump::v92::receiver::{Receiver, TRAIN_WINDOW};

fn value<T: std::str::FromStr>(args: &[String], flag: &str) -> Option<T> {
    let at = args.iter().position(|a| a == flag)?;
    args.get(at + 1)?.parse().ok()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(path) = args.first().filter(|a| !a.starts_with("--")) else {
        eprintln!(
            "usage: v92_trn2u_replay FILE --end N [--origin N] [--sweep N] [--step N] [--ends a,b,c]"
        );
        return Ok(());
    };
    let bytes = fs::read(path)?;
    let line: Vec<f64> = bytes
        .chunks_exact(2)
        .map(|c| f64::from(i16::from_le_bytes([c[0], c[1]])))
        .collect();
    let end: usize = value(&args, "--end").unwrap_or(line.len());
    let origin: u64 = value(&args, "--origin").unwrap_or(end as u64 - 2400);
    let sweep: u64 = value(&args, "--sweep").unwrap_or(0);
    let step: u64 = value(&args, "--step").unwrap_or(1).max(1);
    let ends: Vec<usize> = {
        let at = args.iter().position(|a| a == "--ends");
        match at.and_then(|at| args.get(at + 1)) {
            Some(list) => list.split(',').filter_map(|s| s.trim().parse().ok()).collect(),
            None => vec![end],
        }
    };

    println!(
        "{path}: {} samples ({:.3} s at 8 kHz); origin {origin} ({:.3} s); window {} symbols",
        line.len(),
        line.len() as f64 / 8000.0,
        origin as f64 / 8000.0,
        TRAIN_WINDOW
    );

    // Feed once, clone from the one history: every candidate sees the same line.
    let stop = *ends.iter().max().unwrap_or(&end).min(&line.len());
    let mut rx = Receiver::new();
    let mut trained = 0usize;
    for &sample in line.iter().take(stop) {
        rx.feed(sample);
        while rx.heard().is_some() {
            trained += 1;
        }
    }
    println!("fed to {stop} ({:.3} s), {trained} heard event(s)", stop as f64 / 8000.0);

    let low = if sweep == 0 { origin } else { origin.saturating_sub(sweep) };
    let high = if sweep == 0 { origin } else { origin + sweep };
    let mut best = (f64::NEG_INFINITY, 0u64, false);
    for o in (low..=high).step_by(step as usize) {
        let mut probe = rx.clone();
        // `train_on_trn2u` leaves the fit in `phase_four`; `install_phase_four`
        // is what puts it in the receiver and what `snr_db` then reports --
        // the same two calls `digital.rs:5032` makes.
        let took = probe.train_on_trn2u(TRAIN_WINDOW, o) && probe.install_phase_four();
        let snr = probe.snr_db();
        let window_start = stop as u64 - TRAIN_WINDOW as u64;
        println!(
            "  origin {o:>7} ({:+.0} vs the modem's) window [{window_start}, {stop}]  {} {snr:>7.2} dB",
            o as i64 - origin as i64,
            if took { "fit  " } else { "no fit" }
        );
        if took && snr > best.0 {
            best = (snr, o, took);
        }
    }
    if sweep != 0 {
        println!(
            "best: origin {} ({} from the modem's) at {:.2} dB",
            best.1,
            best.1 as i64 - origin as i64,
            best.0
        );
    }
    Ok(())
}
