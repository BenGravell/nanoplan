//! Planner latency diagnostics.
//!
//! A seam is a named timed span inside a `plan()` call, recorded through the
//! [`Latency`] recorder reachable from the planning [`Context`](super::Context).
//!
//! Standardized seam names — use these wherever the phase exists, so
//! planners stay comparable:
//! - `total`: the whole `plan()` call (recorded by the simulator, not the planner)
//! - `route`: turning the centerline into the planner's road representation
//! - `optimize`: computing the trajectory/decision
//! - `extract`: converting the internal solution into controls
//! - `cost`: evaluating the shared trajectory-cost function
//!   ([`super::constraints::HardConstraints::point_cost`]) at one sample.
//!   Every planner that samples and compares candidate trajectories (the
//!   lattice, PI²-DDP, RRT*) times its calls into
//!   `constraints::HardConstraints` under this name, so the viewer's latency
//!   table can compare "time spent pricing candidates" across implementations.
//!
//! Planners add their own seams for phases only they have (e.g. PI²-DDP's
//! `rollouts`). Seams may nest — they are
//! independent named spans, not a partition of `total`. A seam recorded
//! several times within one `plan()` call is summed for that call.
//!
//! Open-world profiling also records `world_*` seams around live-world setup
//! work that feeds the planner: window maintenance, traffic stepping,
//! goal-distance update, and route-aware actor culling.

use web_time::Instant;

/// Per-call span recorder. Interior mutability so it can sit behind the
/// shared [`Context`](super::Context) reference planners already receive.
#[derive(Default)]
pub(crate) struct Latency {
    spans: std::cell::RefCell<Vec<(&'static str, f64)>>,
}

impl Latency {
    /// Time `f` and record it under `name` (milliseconds).
    pub(crate) fn time<T>(&self, name: &'static str, f: impl FnOnce() -> T) -> T {
        let t0 = Instant::now();
        let out = f();
        self.spans
            .borrow_mut()
            .push((name, t0.elapsed().as_secs_f64() * 1e3));
        out
    }

    /// Drain the spans recorded since the last take.
    pub(crate) fn take(&self) -> Vec<(&'static str, f64)> {
        self.spans.take()
    }
}

/// One seam's statistics over a rollout. `calls` counts the `plan()` calls
/// in which the seam appeared; repeated recordings within one call are
/// summed before folding in.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SeamStats {
    pub(crate) name: &'static str,
    pub(crate) calls: usize,
    pub(crate) total_ms: f64,
    pub(crate) max_ms: f64,
}

impl SeamStats {
    pub(crate) fn mean_ms(&self) -> f64 {
        self.total_ms / self.calls.max(1) as f64
    }
}

/// Latency statistics for a whole rollout, seams in order of first appearance.
#[derive(Debug, Clone, Default)]
pub(crate) struct LatencyStats {
    pub(crate) seams: Vec<SeamStats>,
}

impl LatencyStats {
    /// Fold in the spans of one `plan()` call.
    pub(crate) fn absorb(&mut self, spans: Vec<(&'static str, f64)>) {
        // sum repeated seams within this call, preserving first-seen order
        let mut per_call: Vec<(&'static str, f64)> = Vec::new();
        for (name, ms) in spans {
            match per_call.iter_mut().find(|(n, _)| *n == name) {
                Some((_, sum)) => *sum += ms,
                None => per_call.push((name, ms)),
            }
        }
        for (name, ms) in per_call {
            match self.seams.iter_mut().find(|s| s.name == name) {
                Some(s) => {
                    s.calls += 1;
                    s.total_ms += ms;
                    s.max_ms = s.max_ms.max(ms);
                }
                None => self.seams.push(SeamStats {
                    name,
                    calls: 1,
                    total_ms: ms,
                    max_ms: ms,
                }),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seams_sum_within_a_call_and_track_max_across_calls() {
        let mut stats = LatencyStats::default();
        stats.absorb(vec![("rollouts", 1.0), ("rollouts", 2.0), ("total", 5.0)]);
        stats.absorb(vec![("total", 3.0)]);
        let rollouts = &stats.seams[0];
        assert_eq!((rollouts.name, rollouts.calls), ("rollouts", 1));
        assert!((rollouts.total_ms - 3.0).abs() < 1e-12);
        let total = &stats.seams[1];
        assert_eq!(total.calls, 2);
        assert!((total.mean_ms() - 4.0).abs() < 1e-12);
        assert!((total.max_ms - 5.0).abs() < 1e-12);
    }

    #[test]
    fn recorder_times_and_drains() {
        let lat = Latency::default();
        let v = lat.time("work", || 42);
        assert_eq!(v, 42);
        let spans = lat.take();
        assert_eq!(spans.len(), 1);
        assert_eq!(spans[0].0, "work");
        assert!(spans[0].1 >= 0.0);
        assert!(lat.take().is_empty());
    }
}
