//! Planner introspection diagnostics: auxiliary geometry a planner
//! considered while computing its plan, for visualizing *how* it decided
//! rather than just what it decided.
//!
//! Optional and per-call, exactly like [`Latency`](super::Latency) — a
//! planner records into the [`Diagnostics`] recorder reachable from the
//! planning [`Context`](super::Context) only when one is present, so the
//! closed-loop simulation loop (which never asks for diagnostics) pays
//! nothing for this. What gets recorded is planner-specific: the Frenet
//! lattice records its sampled (station, lateral) grid and the DP's
//! candidate edges; PI²-DDP records its sampled rollouts.

use crate::simulation::Position;

/// Recorded introspection geometry from one `plan()` call.
#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Default)]
pub(crate) struct DiagnosticsData {
    /// Standalone sample points, e.g. lattice grid nodes or PI²-DDP rollout
    /// states.
    pub(crate) points: Vec<Position>,
    /// Polylines, e.g. lattice DP edges or PI²-DDP sampled rollouts.
    pub(crate) trajectories: Vec<Vec<Position>>,
    /// Seconds from the planning start, matching each trajectory point.
    pub(crate) trajectory_times: Vec<Vec<f64>>,
}

/// Per-call recorder. Interior mutability so it can sit behind the shared
/// [`Context`](super::Context) reference planners already receive.
#[derive(Default)]
pub(crate) struct Diagnostics {
    data: std::cell::RefCell<DiagnosticsData>,
}

impl Diagnostics {
    pub(crate) fn record_point(&self, p: Position) {
        self.data.borrow_mut().points.push(p);
    }

    /// Record a rollout starting at the ego state, sampled at the planning timestep.
    pub(crate) fn record_trajectory(&self, traj: Vec<Position>) {
        let times = (0..traj.len()).map(|tick| tick as f64 * super::PLANNING_DT_S).collect();
        self.record_timed_trajectory(traj, times);
    }

    pub(crate) fn record_timed_trajectory(&self, traj: Vec<Position>, times: Vec<f64>) {
        assert_eq!(traj.len(), times.len());
        let mut data = self.data.borrow_mut();
        data.trajectories.push(traj);
        data.trajectory_times.push(times);
    }

    /// Drain the data recorded since the last take.
    pub(crate) fn take(&self) -> DiagnosticsData {
        self.data.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recorder_collects_and_drains() {
        let diag = Diagnostics::default();
        diag.record_point(Position::new(1.0, 2.0));
        diag.record_trajectory(vec![Position::new(0.0, 0.0), Position::new(1.0, 1.0)]);
        let data = diag.take();
        assert_eq!(data.points, vec![Position::new(1.0, 2.0)]);
        assert_eq!(
            data.trajectories,
            vec![vec![Position::new(0.0, 0.0), Position::new(1.0, 1.0)]]
        );
        assert_eq!(data.trajectory_times, vec![vec![0.0, super::super::PLANNING_DT_S]]);
        assert!(diag.take().trajectory_times.is_empty());
        assert!(diag.take().points.is_empty());
        assert!(diag.take().trajectories.is_empty());
    }
}
