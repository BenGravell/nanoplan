//! Runtime latency diagnostics.
//!
//! A seam is a named timed span. Planner internals record through the
//! [`Latency`] recorder reachable from the planning [`Context`](super::Context);
//! the live world and viewer use the same recorder around their own work.
//!
//! Standardized seam names — use these wherever the phase exists, so
//! planners stay comparable:
//! - `planner.total`: the whole `plan()` call (recorded by the world, not the planner)
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
//! `rollouts`). Seams may nest — they are independent named spans, not a
//! partition of `planner.total`. A seam recorded
//! several times within one `plan()` call is summed for that call.
//!
//! Live profiling uses namespaced seams such as `simulation.actors`,
//! `simulation.preview`, `visualization.roads`, and
//! `visualization.ego_carpet`. Seams recorded several times in one rendered
//! frame (because the fixed-step simulation catches up) are summed.
//!
//! Each span carries two measurements:
//! - wall-clock milliseconds, useful for profiling a particular build/machine;
//! - logical `clocks`, deterministic work units advanced at domain boundaries.
//!   Clocks are hardware independent and therefore suitable for regression
//!   assertions. Nested seams see the same work units, just as they see the
//!   same elapsed wall time.

use web_time::Instant;

#[derive(Debug, Clone, Copy)]
pub(crate) struct Span {
    pub(crate) name: &'static str,
    pub(crate) milliseconds: f64,
    pub(crate) clocks: u64,
}

/// Per-call span recorder. Interior mutability so it can sit behind the
/// shared [`Context`](super::Context) reference planners already receive.
#[derive(Default)]
pub(crate) struct Latency {
    spans: std::cell::RefCell<Vec<Span>>,
    clocks: std::cell::Cell<u64>,
}

impl Latency {
    /// Time `f` and record wall milliseconds plus logical work clocks.
    pub(crate) fn time<T>(&self, name: &'static str, f: impl FnOnce() -> T) -> T {
        let t0 = Instant::now();
        let c0 = self.clocks.get();
        let out = f();
        self.record_span(Span {
            name,
            milliseconds: t0.elapsed().as_secs_f64() * 1e3,
            clocks: self.clocks.get() - c0,
        });
        out
    }

    /// Advance the deterministic work clock.
    pub(crate) fn work(&self, clocks: u64) {
        self.clocks.set(self.clocks.get() + clocks);
    }

    /// Record an already measured wall span and its deterministic work.
    pub(crate) fn record(&self, name: &'static str, milliseconds: f64, clocks: u64) {
        self.record_span(Span {
            name,
            milliseconds,
            clocks,
        });
    }

    fn record_span(&self, span: Span) {
        self.spans.borrow_mut().push(span);
    }

    /// Drain the spans recorded since the last take.
    pub(crate) fn take(&self) -> Vec<Span> {
        self.clocks.set(0);
        self.spans.take()
    }
}

/// One seam's statistics over a run. `calls` counts the samples (normally
/// rendered frames) in which the seam appeared; repeated recordings within
/// one sample are summed before folding in.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SeamStats {
    pub(crate) name: &'static str,
    pub(crate) calls: usize,
    pub(crate) total_ms: f64,
    pub(crate) max_ms: f64,
    pub(crate) total_clocks: u64,
    pub(crate) max_clocks: u64,
}

impl SeamStats {
    pub(crate) fn mean_ms(&self) -> f64 {
        self.total_ms / self.calls.max(1) as f64
    }

    pub(crate) fn mean_clocks(&self) -> f64 {
        self.total_clocks as f64 / self.calls.max(1) as f64
    }
}

/// Latency statistics for a whole run, seams in order of first appearance.
#[derive(Debug, Clone, Default)]
pub(crate) struct LatencyStats {
    pub(crate) seams: Vec<SeamStats>,
}

impl LatencyStats {
    /// Fold in the spans of one sample.
    pub(crate) fn absorb(&mut self, spans: Vec<Span>) {
        // Sum repeated seams within this sample, preserving first-seen order.
        let mut per_call: Vec<Span> = Vec::new();
        for span in spans {
            match per_call
                .iter_mut()
                .find(|candidate| candidate.name == span.name)
            {
                Some(sum) => {
                    sum.milliseconds += span.milliseconds;
                    sum.clocks += span.clocks;
                }
                None => per_call.push(span),
            }
        }
        for span in per_call {
            match self.seams.iter_mut().find(|s| s.name == span.name) {
                Some(s) => {
                    s.calls += 1;
                    s.total_ms += span.milliseconds;
                    s.max_ms = s.max_ms.max(span.milliseconds);
                    s.total_clocks += span.clocks;
                    s.max_clocks = s.max_clocks.max(span.clocks);
                }
                None => self.seams.push(SeamStats {
                    name: span.name,
                    calls: 1,
                    total_ms: span.milliseconds,
                    max_ms: span.milliseconds,
                    total_clocks: span.clocks,
                    max_clocks: span.clocks,
                }),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seams_sum_wall_time_and_stable_clocks() {
        let mut stats = LatencyStats::default();
        stats.absorb(vec![
            Span {
                name: "rollouts",
                milliseconds: 1.0,
                clocks: 2,
            },
            Span {
                name: "rollouts",
                milliseconds: 2.0,
                clocks: 3,
            },
            Span {
                name: "total",
                milliseconds: 5.0,
                clocks: 8,
            },
        ]);
        stats.absorb(vec![Span {
            name: "total",
            milliseconds: 3.0,
            clocks: 4,
        }]);
        let rollouts = &stats.seams[0];
        assert_eq!((rollouts.name, rollouts.calls), ("rollouts", 1));
        assert!((rollouts.total_ms - 3.0).abs() < 1e-12);
        assert_eq!((rollouts.total_clocks, rollouts.max_clocks), (5, 5));
        let total = &stats.seams[1];
        assert_eq!(total.calls, 2);
        assert!((total.mean_ms() - 4.0).abs() < 1e-12);
        assert!((total.max_ms - 5.0).abs() < 1e-12);
        assert_eq!(total.mean_clocks(), 6.0);
        assert_eq!(total.max_clocks, 8);
    }

    #[test]
    fn recorder_captures_nested_hardware_independent_work() {
        let lat = Latency::default();
        let v = lat.time("total", || {
            lat.work(2);
            lat.time("inner", || lat.work(3));
            42
        });
        assert_eq!(v, 42);
        let spans = lat.take();
        assert_eq!(spans.len(), 2);
        assert_eq!((spans[0].name, spans[0].clocks), ("inner", 3));
        assert_eq!((spans[1].name, spans[1].clocks), ("total", 5));
        assert!(spans.iter().all(|span| span.milliseconds >= 0.0));
        assert!(lat.take().is_empty());
    }
}
