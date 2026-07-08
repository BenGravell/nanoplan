//! Planner latency profiler: every planner over a batch of scenarios, CSV to stdout.
//!
//! Usage:
//!   profile_latency [--count N] [--seed S] [--dir PATH]...
//!
//! Scenarios come from the same sources as the batch runner: the synthetic
//! generator (`--count`, default 20) and/or directories of scenario JSON files
//! (`--dir`). Latency comes from the simulator's existing per-rollout
//! `LatencyStats`; this tool only aggregates it across the batch.

use nanoplan::planning::LatencyStats;
use nanoplan::{PlannerKind, Scenario, scenarios, simulate};

const DURATION_S: f64 = 20.0;
const DT: f64 = 0.1;

#[derive(Clone, Copy, Default)]
struct Aggregate {
    calls: usize,
    total_ms: f64,
    max_ms: f64,
}

impl Aggregate {
    fn mean_ms(&self) -> f64 {
        self.total_ms / self.calls.max(1) as f64
    }
}

#[derive(Clone, Default)]
struct PlannerSummary {
    seams: Vec<(&'static str, Aggregate)>,
}

impl PlannerSummary {
    fn absorb(&mut self, stats: &LatencyStats) {
        for seam in &stats.seams {
            match self.seams.iter_mut().find(|(name, _)| *name == seam.name) {
                Some((_, sum)) => {
                    sum.calls += seam.calls;
                    sum.total_ms += seam.total_ms;
                    sum.max_ms = sum.max_ms.max(seam.max_ms);
                }
                None => self.seams.push((
                    seam.name,
                    Aggregate {
                        calls: seam.calls,
                        total_ms: seam.total_ms,
                        max_ms: seam.max_ms,
                    },
                )),
            }
        }
    }

    fn seam(&self, name: &str) -> Option<&Aggregate> {
        self.seams.iter().find(|(n, _)| *n == name).map(|(_, a)| a)
    }
}

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

    let scenarios = load_scenarios(count, seed, &dirs);
    if scenarios.is_empty() {
        eprintln!("no scenarios to profile");
        return;
    }

    let mut summaries = vec![PlannerSummary::default(); PlannerKind::ALL.len()];
    for sc in &scenarios {
        for (i, kind) in PlannerKind::ALL.into_iter().enumerate() {
            summaries[i].absorb(&simulate(sc, kind, DURATION_S, DT).latency);
        }
    }

    println!("planner,seam,calls,total_ms,mean_ms,max_ms");
    for (i, kind) in PlannerKind::ALL.into_iter().enumerate() {
        for (name, agg) in &summaries[i].seams {
            println!(
                "{},{},{},{:.3},{:.3},{:.3}",
                kind.name().replace(' ', "_"),
                name,
                agg.calls,
                agg.total_ms,
                agg.mean_ms(),
                agg.max_ms
            );
        }
    }

    eprintln!(
        "\nmean total latency over {} scenarios ({:.1}s each):",
        scenarios.len(),
        DURATION_S
    );
    for (i, kind) in PlannerKind::ALL.into_iter().enumerate() {
        if let Some(total) = summaries[i].seam("total") {
            eprintln!("  {:22} {:8.3} ms/plan", kind.name(), total.mean_ms());
        }
    }
}

fn load_scenarios(count: usize, seed: u64, dirs: &[String]) -> Vec<Scenario> {
    let mut out = scenarios::synthetic_batch(count, seed);
    for dir in dirs {
        let loaded = scenarios::load_dir(std::path::Path::new(dir))
            .unwrap_or_else(|e| panic!("loading {dir}: {e}"));
        eprintln!("loaded {} scenarios from {dir}", loaded.len());
        out.extend(loaded);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use nanoplan::planning::SeamStats;

    #[test]
    fn summary_merges_matching_seams() {
        let mut summary = PlannerSummary::default();
        summary.absorb(&LatencyStats {
            seams: vec![SeamStats {
                name: "total",
                calls: 2,
                total_ms: 10.0,
                max_ms: 6.0,
            }],
        });
        summary.absorb(&LatencyStats {
            seams: vec![SeamStats {
                name: "total",
                calls: 3,
                total_ms: 12.0,
                max_ms: 5.0,
            }],
        });

        let total = summary.seam("total").unwrap();
        assert_eq!(total.calls, 5);
        assert!((total.total_ms - 22.0).abs() < 1e-12);
        assert!((total.max_ms - 6.0).abs() < 1e-12);
        assert!((total.mean_ms() - 4.4).abs() < 1e-12);
    }
}
