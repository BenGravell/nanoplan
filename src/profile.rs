//! Application scenario runner and profiler.

use web_time::Instant;

use crate::planning::{Latency, LatencyStats, PlannerKind};
use crate::track::{TRACK_CATALOG, TRACK_PRESETS};
use crate::world::LiveWorld;

const PLANNERS: [(&str, PlannerKind); 12] = [
    ("straight", PlannerKind::Straight),
    ("basic", PlannerKind::Basic),
    ("bezier-toppra", PlannerKind::BezierToppra),
    ("lattice", PlannerKind::Lattice),
    ("pi2-ddp", PlannerKind::Pi2Ddp),
    ("rrt-star", PlannerKind::RrtStar),
    ("predictive-sampling", PlannerKind::PredictiveSampling),
    ("cem", PlannerKind::Cem),
    ("mppi", PlannerKind::Mppi),
    ("rrt", PlannerKind::Rrt),
    ("ilqr", PlannerKind::Ilqr),
    ("treetop", PlannerKind::Treetop),
];

#[derive(Debug)]
pub struct SeamProfile {
    pub name: &'static str,
    pub calls: usize,
    pub mean_ms: f64,
    pub max_ms: f64,
    pub mean_clocks: f64,
    pub max_clocks: u64,
    pub total_clocks: u64,
}

#[derive(Debug)]
pub struct LapProfile {
    pub planner: &'static str,
    pub track: String,
    pub requested_laps: f64,
    pub completed_laps: f64,
    pub completed: bool,
    pub collision_count: usize,
    pub ticks: usize,
    pub simulated_seconds: f64,
    pub wall_ms: f64,
    pub seams: Vec<SeamProfile>,
}

pub fn downloaded_track_ids() -> impl Iterator<Item = &'static str> {
    TRACK_CATALOG.iter().map(|track| track.id)
}

pub fn planner_ids() -> impl Iterator<Item = &'static str> {
    PLANNERS.iter().map(|(id, _)| *id)
}

fn planner_kind(name: &str) -> Option<PlannerKind> {
    let normalized = name.to_ascii_lowercase().replace(['_', ' '], "-");
    PLANNERS
        .iter()
        .find_map(|(id, kind)| (*id == normalized).then_some(*kind))
        .or(match normalized.as_str() {
            "bezier" => Some(PlannerKind::BezierToppra),
            "frenet-lattice" => Some(PlannerKind::Lattice),
            "pi2ddp" => Some(PlannerKind::Pi2Ddp),
            "rrt*" => Some(PlannerKind::RrtStar),
            "ps" => Some(PlannerKind::PredictiveSampling),
            _ => None,
        })
}

fn track_index(name: &str) -> Result<(usize, String), String> {
    match name.to_ascii_lowercase().replace(['_', ' '], "-").as_str() {
        "generated" => Err("generated track is not available for profiling".into()),
        "large" | "test-track-large" => Ok((1, TRACK_PRESETS[0].name.into())),
        "small" | "test-track-small" => Ok((2, TRACK_PRESETS[1].name.into())),
        id => {
            crate::track::loader::load()?;
            TRACK_CATALOG
                .iter()
                .position(|track| track.id.replace('_', "-") == id)
                .map(|i| (TRACK_PRESETS.len() + i + 1, TRACK_CATALOG[i].name.into()))
                .ok_or_else(|| format!("unknown track {name:?}"))
        }
    }
}

pub fn run(planner: &str, track: &str, laps: f64) -> Result<LapProfile, String> {
    if !laps.is_finite() || laps <= 0.0 {
        return Err(format!("laps must be finite and positive, got {laps}"));
    }
    let planner_kind =
        planner_kind(planner).ok_or_else(|| format!("unknown planner {planner:?}"))?;
    let (track_index, track_name) = track_index(track)?;
    let mut world = LiveWorld::with_track(track_index, 1, planner_kind, 0, 0.1);
    let lap_length = world
        .track
        .lap_length()
        .ok_or_else(|| format!("track {track_name:?} has no lap length"))?;
    let target_progress = laps * lap_length;
    let max_ticks = (5_000.0 * laps).ceil().max(500.0) as usize;
    let recorder = Latency::default();
    let mut latency = LatencyStats::default();
    let started = Instant::now();
    let mut ticks = 0;

    while world.track_progress < target_progress && ticks < max_ticks {
        world.tick_recording_latency(&recorder);
        latency.absorb(recorder.take());
        ticks += 1;
    }

    Ok(LapProfile {
        planner: planner_kind.name(),
        track: track_name,
        requested_laps: laps,
        completed_laps: world.track_progress / lap_length,
        completed: world.track_progress >= target_progress,
        collision_count: world.ego_collision_count,
        ticks,
        simulated_seconds: ticks as f64 * world.dt(),
        wall_ms: started.elapsed().as_secs_f64() * 1e3,
        seams: latency
            .seams
            .into_iter()
            .map(|seam| SeamProfile {
                name: seam.name,
                calls: seam.calls,
                mean_ms: seam.mean_ms(),
                max_ms: seam.max_ms,
                mean_clocks: seam.mean_clocks(),
                max_clocks: seam.max_clocks,
                total_clocks: seam.total_clocks,
            })
            .collect(),
    })
}
