//! Cost-weight autotuner: fits the shared cost function's soft weights to
//! expert human trajectories with maximum-entropy IRL (DriveIRL-style; see
//! src/tuning/).
//!
//! Usage:
//!   tune --dir PATH [--dir PATH]... [--iters N] [--l2 X]
//!
//! Scenarios must carry an `expert` trajectory (nuPlan logs exported with
//! tools/export_nuplan_scenarios.py include one); scenarios without a usable
//! expert are skipped. Prints the learned weights and the `WEIGHTS` line to
//! paste into src/planning/cost.rs. Collision and off-road remain infinite
//! cost by fiat — only the soft weights are learned.

use nanoplan::{scenarios, tuning};

fn main() {
    let mut dirs: Vec<String> = vec![];
    let mut opts = tuning::Options::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = |name: &str| {
            args.next()
                .unwrap_or_else(|| panic!("{name} needs a value"))
        };
        match arg.as_str() {
            "--dir" => dirs.push(value("--dir")),
            "--iters" => opts.iters = value("--iters").parse().expect("--iters: not a number"),
            "--l2" => opts.l2 = value("--l2").parse().expect("--l2: not a number"),
            other => panic!("unknown argument {other}"),
        }
    }
    assert!(
        !dirs.is_empty(),
        "usage: tune --dir PATH [--dir PATH]... [--iters N] [--l2 X]"
    );

    let mut all = vec![];
    for dir in &dirs {
        let loaded = scenarios::load_dir(std::path::Path::new(dir))
            .unwrap_or_else(|e| panic!("loading {dir}: {e}"));
        eprintln!("loaded {} scenarios from {dir}", loaded.len());
        all.extend(loaded);
    }
    print!("{}", tuning::tune(&all, &opts).report());
}
