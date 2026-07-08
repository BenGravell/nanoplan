//! Planner latency profiler: every planner over a batch of scenarios, CSV to stdout.
//!
//! Usage:
//!   profile_latency [--mode scenarios|world] [--count N] [--seed S] [--dir PATH]...
//!
//! The default `scenarios` mode uses the same sources as the batch runner:
//! the synthetic generator (`--count`, default 20) and/or directories of
//! scenario JSON files (`--dir`). `world` mode runs the procedural
//! `LiveWorld` loop offline with the viewer's traffic defaults. Latency comes
//! from the existing planner `LatencyStats`; this tool only aggregates it.

use std::io::Write;

use nanoplan::planning::{Latency, LatencyStats};
use nanoplan::world::LiveWorld;
use nanoplan::{PlannerKind, Scenario, scenarios, simulate};

const DURATION_S: f64 = 20.0;
const DT: f64 = 0.1;
const WORLD_SEED: u64 = 1;
const WORLD_MAX_ACTORS: usize = 64;
const WORLD_GOAL_OFFSET: [f64; 2] = [300.0, 60.0];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Scenarios,
    World,
}

struct Args {
    mode: Mode,
    count: usize,
    seed: u64,
    world_seed: u64,
    dirs: Vec<String>,
    duration_s: f64,
    max_actors: usize,
}

impl Default for Args {
    fn default() -> Self {
        Args {
            mode: Mode::Scenarios,
            count: 20,
            seed: 42,
            world_seed: WORLD_SEED,
            dirs: vec![],
            duration_s: DURATION_S,
            max_actors: WORLD_MAX_ACTORS,
        }
    }
}

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
    let args = parse_args();
    let Some((summaries, label)) = run(&args) else {
        return;
    };

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

    std::io::stdout().flush().unwrap();
    eprintln!("\nmean total latency {label}:");
    for (i, kind) in PlannerKind::ALL.into_iter().enumerate() {
        if let Some(total) = summaries[i].seam("total") {
            eprintln!("  {:22} {:8.3} ms/plan", kind.name(), total.mean_ms());
        }
    }
}

fn parse_args() -> Args {
    let mut out = Args::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = |name: &str| {
            args.next()
                .unwrap_or_else(|| panic!("{name} needs a value"))
        };
        match arg.as_str() {
            "--mode" => {
                out.mode = match value("--mode").as_str() {
                    "scenario" | "scenarios" => Mode::Scenarios,
                    "world" => Mode::World,
                    other => panic!("unknown --mode {other}"),
                };
            }
            "--count" => out.count = value("--count").parse().expect("--count: not a number"),
            "--seed" => out.seed = value("--seed").parse().expect("--seed: not a number"),
            "--world-seed" => {
                out.world_seed = value("--world-seed")
                    .parse()
                    .expect("--world-seed: not a number");
            }
            "--dir" => out.dirs.push(value("--dir")),
            "--duration" => {
                out.duration_s = value("--duration")
                    .parse()
                    .expect("--duration: not a number");
            }
            "--max-actors" => {
                out.max_actors = value("--max-actors")
                    .parse()
                    .expect("--max-actors: not a number");
            }
            other => panic!("unknown argument {other}"),
        }
    }
    out
}

fn run(args: &Args) -> Option<(Vec<PlannerSummary>, String)> {
    match args.mode {
        Mode::Scenarios => run_scenarios(args),
        Mode::World => Some(run_world(args)),
    }
}

fn run_scenarios(args: &Args) -> Option<(Vec<PlannerSummary>, String)> {
    let scenarios = load_scenarios(args.count, args.seed, &args.dirs);
    if scenarios.is_empty() {
        eprintln!("no scenarios to profile");
        return None;
    }

    let mut summaries = vec![PlannerSummary::default(); PlannerKind::ALL.len()];
    for sc in &scenarios {
        for (i, kind) in PlannerKind::ALL.into_iter().enumerate() {
            summaries[i].absorb(&simulate(sc, kind, args.duration_s, DT).latency);
        }
    }

    Some((
        summaries,
        format!(
            "over {} scenarios ({:.1}s each)",
            scenarios.len(),
            args.duration_s
        ),
    ))
}

fn run_world(args: &Args) -> (Vec<PlannerSummary>, String) {
    let ticks = (args.duration_s / DT) as usize;
    let mut summaries = vec![PlannerSummary::default(); PlannerKind::ALL.len()];
    for (i, kind) in PlannerKind::ALL.into_iter().enumerate() {
        let mut world = LiveWorld::new(args.world_seed, kind, args.max_actors, DT);
        let recorder = Latency::default();
        let mut stats = LatencyStats::default();
        for _ in 0..ticks {
            if world.goal.is_none() {
                set_live_goal(&mut world);
            }
            world.tick_recording_latency(&recorder);
            stats.absorb(recorder.take());
        }
        summaries[i].absorb(&stats);
    }

    (
        summaries,
        format!(
            "over procedural world {:.1}s (seed {}, max actors {})",
            args.duration_s, args.world_seed, args.max_actors
        ),
    )
}

fn set_live_goal(world: &mut LiveWorld) {
    world.set_goal([
        world.ego.x + WORLD_GOAL_OFFSET[0],
        world.ego.y + WORLD_GOAL_OFFSET[1],
    ]);
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
