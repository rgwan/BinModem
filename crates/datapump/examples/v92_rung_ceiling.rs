//! What the rung ceiling makes of a level stream: the offline half of the rate
//! rule.
//!
//! `rung`'s unit tests hold the two recordings the ceiling was built against
//! (`crates/datapump/tests/data/*.score.levels`, the `score` run of each). This
//! example is the same measurement on a stream taken today, so the rule can be
//! pointed at a fresh capture without editing a test:
//!
//! ```text
//! cargo run --release --example v92_rung_ceiling -- \
//!     /tmp/fe/ber/c24-4-off3-none.levels:8 \
//!     /dev/shm/q/b32-mi16-2.levels:16
//! ```
//!
//! Each argument is `PATH:LC`: the level stream the decoder is fed through B1u
//! (the one `V92_B1U_LEVELS` names, one `now level mode` line per upstream
//! symbol), and the number of positive points in the constellation set the run
//! was made with. `--recorded DRN` names the rung the far end was pinned to and
//! is printed beside the ceiling; it is not an input to it.
//!
//! What it prints per stream: the residual in index steps on the rung's own
//! grid, the same residual on the LC = 8 grid the ceiling is quoted in, the
//! symbol error probability that implies, the highest rung permitted and the
//! Ja mask with the ceiling applied. Nothing here sends anything to a bench.

use datapump::v92::rung::{Rung, apply_ceiling, ceiling_rung, lc_for, sigma_from_stream};
use datapump::v92::{UP_RATES, ja_mask_drns};

fn main() -> std::process::ExitCode {
    let mut recorded: Option<u8> = None;
    let mut streams: Vec<(String, usize)> = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--recorded" {
            let Some(drn) = args.next().and_then(|value| value.parse::<u8>().ok()) else {
                eprintln!("--recorded wants a drn");
                return std::process::ExitCode::FAILURE;
            };
            if !(1..=UP_RATES as u8).contains(&drn) {
                eprintln!("drn {drn} is not on the ladder");
                return std::process::ExitCode::FAILURE;
            }
            recorded = Some(drn);
            continue;
        }
        let Some((path, lc)) = arg.rsplit_once(':') else {
            eprintln!("{arg}: expected PATH:LC");
            return std::process::ExitCode::FAILURE;
        };
        let Some(lc) = lc.parse::<usize>().ok().filter(|lc| lc.is_power_of_two() && *lc > 0) else {
            eprintln!("{arg}: LC must be a power of two, the number of positive points in the set");
            return std::process::ExitCode::FAILURE;
        };
        streams.push((path.to_string(), lc));
    }
    if streams.is_empty() {
        eprintln!("usage: v92_rung_ceiling [--recorded DRN] PATH:LC [PATH:LC ...]");
        return std::process::ExitCode::FAILURE;
    }

    let mut failed = false;
    for (path, lc) in streams {
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(error) => {
                eprintln!("{path}: {error}");
                failed = true;
                continue;
            }
        };
        let Some(measured) = sigma_from_stream(&text, lc) else {
            eprintln!("{path}: no `score` run in this stream, so there is nothing to measure");
            failed = true;
            continue;
        };
        let ceiling = ceiling_rung(measured);
        let mask = apply_ceiling(0, measured);
        let at = |drn: u8| {
            let rung = Rung::of(drn).expect("a rung");
            (rung, measured.error(rung.lc))
        };
        println!("{} (LC {lc})", path);
        println!(
            "  residual {:.4} index steps on its own grid, {:.4} on the LC = 8 grid, bound {:?}",
            measured.at(lc),
            measured.steps(),
            measured.bound()
        );
        if let Some(drn) = recorded {
            let (rung, error) = at(drn);
            println!(
                "  recorded at drn {drn} ({} bit/s, K {}, LC {}): symbol error {error:.3e}{}",
                rung.rate,
                rung.k,
                rung.lc,
                if drn <= ceiling.unwrap_or(0) { ", permitted" } else { ", refused" }
            );
        }
        match ceiling {
            Some(drn) => {
                let rung = Rung::of(drn).expect("a rung");
                println!(
                    "  ceiling: drn {drn}, {} bit/s, symbol error {:.3e}; mask {mask:#b} = drns {:?}",
                    rung.rate,
                    measured.error(rung.lc),
                    ja_mask_drns(mask)
                );
            }
            None => println!("  ceiling: none -- even the slowest rung is more than this line carries"),
        }
    }
    if failed { std::process::ExitCode::FAILURE } else { std::process::ExitCode::SUCCESS }
}
