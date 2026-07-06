//! Batch runner: every planner over a large set of scenarios, CSV to stdout.
//!
//! Usage:
//!   batch [--count N] [--seed S] [--dir PATH]...
//!
//! Scenarios come from the synthetic generator (`--count`, default 20) and/or
//! directories of scenario JSON files (`--dir`), e.g. converted from
//! CommonRoad scenarios with tools/export_commonroad_scenarios.py or
//! exported from nuPlan logs with tools/export_nuplan_scenarios.py. A
//! per-planner mean-score summary goes to stderr.

use nanoplan::metrics::METRICS;
use nanoplan::{PlannerKind, scenarios, simulate};

const DURATION_S: f64 = 20.0;
const DT: f64 = 0.1;

fn main() {
    let mut count = 20;
    let mut seed = 42;
    let mut dirs: Vec<String> = vec![];
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = |name: &str| {
            args.next()
                .unwrap_or_else(|| panic!("{name} needs a value"))
        };
        match arg.as_str() {
            "--count" => count = value("--count").parse().expect("--count: not a number"),
            "--seed" => seed = value("--seed").parse().expect("--seed: not a number"),
            "--dir" => dirs.push(value("--dir")),
            other => panic!("unknown argument {other}"),
        }
    }

    let mut scenarios = scenarios::synthetic_batch(count, seed);
    for dir in &dirs {
        let loaded = scenarios::load_dir(std::path::Path::new(dir))
            .unwrap_or_else(|e| panic!("loading {dir}: {e}"));
        eprintln!("loaded {} scenarios from {dir}", loaded.len());
        scenarios.extend(loaded);
    }

    println!(
        "scenario,planner,score,{}",
        METRICS.map(|m| m.label.replace(' ', "_")).join(",")
    );
    let mut totals = vec![0.0; PlannerKind::ALL.len()];
    for sc in &scenarios {
        for (k, kind) in PlannerKind::ALL.into_iter().enumerate() {
            let m = simulate(sc, kind, DURATION_S, DT).metrics;
            totals[k] += m.score;
            println!(
                "{},{},{:.4},{}",
                sc.name,
                kind.name().replace(' ', "_"),
                m.score,
                m.aggregate.map(|v| format!("{v:.4}")).join(",")
            );
        }
    }
    eprintln!("\nmean score over {} scenarios:", scenarios.len());
    for (k, kind) in PlannerKind::ALL.into_iter().enumerate() {
        eprintln!(
            "  {:22} {:.4}",
            kind.name(),
            totals[k] / scenarios.len().max(1) as f64
        );
    }
}
