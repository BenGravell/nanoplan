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

/// Recorded introspection geometry from one `plan()` call.
#[derive(Debug, Clone, Default)]
pub struct DiagnosticsData {
    /// Standalone sample points, e.g. lattice grid nodes or PI²-DDP rollout
    /// states.
    pub points: Vec<[f64; 2]>,
    /// Polylines, e.g. lattice DP edges or PI²-DDP sampled rollouts.
    pub trajectories: Vec<Vec<[f64; 2]>>,
}

/// Per-call recorder. Interior mutability so it can sit behind the shared
/// [`Context`](super::Context) reference planners already receive.
#[derive(Default)]
pub struct Diagnostics {
    data: std::cell::RefCell<DiagnosticsData>,
}

impl Diagnostics {
    pub fn record_point(&self, p: [f64; 2]) {
        self.data.borrow_mut().points.push(p);
    }

    pub fn record_trajectory(&self, traj: Vec<[f64; 2]>) {
        self.data.borrow_mut().trajectories.push(traj);
    }

    /// Drain the data recorded since the last take.
    pub fn take(&self) -> DiagnosticsData {
        self.data.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recorder_collects_and_drains() {
        let diag = Diagnostics::default();
        diag.record_point([1.0, 2.0]);
        diag.record_trajectory(vec![[0.0, 0.0], [1.0, 1.0]]);
        let data = diag.take();
        assert_eq!(data.points, vec![[1.0, 2.0]]);
        assert_eq!(data.trajectories, vec![vec![[0.0, 0.0], [1.0, 1.0]]]);
        assert!(diag.take().points.is_empty());
        assert!(diag.take().trajectories.is_empty());
    }
}
