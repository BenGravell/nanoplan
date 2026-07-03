//! The rollout cache and the chunked async simulation job — lets the viewer
//! avoid blocking the UI thread on an expensive planner like PI²-DDP.

use std::collections::HashMap;

use bevy::prelude::*;
use nanoplan::{IncrementalSim, PlannerKind, Rollout};
use web_time::Instant;

/// Finished closed-loop simulations, keyed by scenario + planner so
/// re-selecting a combo we've already simulated is instant.
#[derive(Resource, Default)]
pub(crate) struct RolloutCache(pub HashMap<(usize, PlannerKind), Rollout>);

/// A simulation in progress, time-sliced across frames so an expensive
/// planner (PI²-DDP) never blocks the UI thread — see `IncrementalSim`.
///
/// `IncrementalSim` holds a `Box<dyn Planner>` and an interior-mutable
/// latency recorder, neither of which are `Sync`, so this is a `NonSend`
/// resource rather than a regular one.
#[derive(Default)]
pub(crate) struct ActiveJob(pub Option<((usize, PlannerKind), IncrementalSim)>);

/// Per-frame wall-clock budget for stepping the active job.
const FRAME_BUDGET_MS: u64 = 8;

/// Advance the in-flight simulation (if any) by one frame's time budget,
/// so an expensive planner never blocks the UI thread. Once it finishes,
/// the result moves into the cache and the job slot frees up.
pub(crate) fn step_active_job(mut job: NonSendMut<ActiveJob>, mut cache: ResMut<RolloutCache>) {
    let Some((_, sim)) = &mut job.0 else { return };
    sim.step_until(Instant::now() + std::time::Duration::from_millis(FRAME_BUDGET_MS));
    if sim.is_done() {
        let (key, sim) = job.0.take().unwrap();
        cache.0.insert(key, sim.finish());
    }
}
