//! The kinematic vehicle model and the closed-loop simulator.

use serde::{Deserialize, Serialize};
use web_time::Instant;

use crate::metrics::{self, Metrics};
use crate::planning::{Context, Latency, LatencyStats, Planner, PlannerKind};
use crate::scenarios::Scenario;

/// Ego state: position, yaw, and speed.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct State {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub yaw: f64,
    #[serde(default)]
    pub speed: f64,
}

/// Control action: longitudinal acceleration and path curvature.
/// The default (all zeros) drives straight ahead at constant speed.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Control {
    #[serde(default)]
    pub accel: f64,
    #[serde(default)]
    pub curvature: f64,
}

/// Advance the kinematic model by one Euler step of length `dt`.
pub fn step(s: State, u: Control, dt: f64) -> State {
    State {
        x: s.x + s.speed * s.yaw.cos() * dt,
        y: s.y + s.speed * s.yaw.sin() * dt,
        yaw: s.yaw + s.speed * u.curvature * dt,
        speed: s.speed + u.accel * dt,
    }
}

/// Ego vehicle simulator.
pub struct Simulator {
    pub state: State,
    pub dt: f64,
}

impl Simulator {
    /// Replan from the current state, apply the first planned control,
    /// and advance one tick. Returns the new state.
    /// An empty plan coasts (zero control).
    pub fn tick(&mut self, planner: &mut dyn Planner, ctx: &Context) -> State {
        let u = ctx
            .time("total", || planner.plan(self.state, ctx))
            .first()
            .copied()
            .unwrap_or_default();
        self.state = step(self.state, u, self.dt);
        self.state
    }
}

/// A finished closed-loop simulation: ego and actor states at every tick,
/// plus the metrics of the rollout.
pub struct Rollout {
    pub ego: Vec<State>,
    pub actors: Vec<Vec<State>>,
    pub metrics: Metrics,
    /// Planner latency seams aggregated over the rollout.
    pub latency: LatencyStats,
}

/// Run a planner closed-loop through a scenario, all at once.
///
/// For an expensive planner (PI²-DDP can take seconds over a full rollout —
/// see [`IncrementalSim`]) this blocks the calling thread until every tick
/// is done. Fine for tests and the batch runner; the viewer uses
/// `IncrementalSim` instead so it doesn't freeze while this runs.
pub fn simulate(sc: &Scenario, kind: PlannerKind, duration_s: f64, dt: f64) -> Rollout {
    IncrementalSim::start(sc, kind, duration_s, dt).finish()
}

/// A `simulate()` run split into resumable chunks, so a caller with a frame
/// budget (a GUI) can advance it a little at a time instead of blocking
/// until the whole rollout is done.
///
/// This is deliberately not multithreaded: the viewer targets wasm as well
/// as desktop, and wasm has no portable way to run a planner on another
/// thread without extra tooling (`wasm-bindgen-rayon`, `SharedArrayBuffer`,
/// a special build). Time-slicing across frames works identically on both
/// targets with no platform-specific code.
pub struct IncrementalSim {
    actors: Vec<Vec<State>>,
    centerline: Vec<[f64; 2]>,
    target_speed: f64,
    dt: f64,
    sim: Simulator,
    planner: Box<dyn Planner>,
    recorder: Latency,
    latency: LatencyStats,
    ego: Vec<State>,
    steps_total: usize,
}

impl IncrementalSim {
    pub fn start(sc: &Scenario, kind: PlannerKind, duration_s: f64, dt: f64) -> Self {
        let steps_total = (duration_s / dt) as usize;
        IncrementalSim {
            actors: sc.actors.iter().map(|a| a.trace(steps_total, dt)).collect(),
            centerline: sc.centerline.clone(),
            target_speed: sc.target_speed,
            dt,
            sim: Simulator { state: sc.ego, dt },
            planner: kind.build(),
            recorder: Latency::default(),
            latency: LatencyStats::default(),
            ego: vec![sc.ego],
            steps_total,
        }
    }

    pub fn is_done(&self) -> bool {
        self.ego.len() > self.steps_total
    }

    /// Fraction of ticks completed, for a progress bar.
    pub fn progress(&self) -> f32 {
        (self.ego.len() - 1) as f32 / self.steps_total.max(1) as f32
    }

    fn tick_once(&mut self) {
        let i = self.ego.len() - 1;
        let current: Vec<State> = self.actors.iter().map(|t| t[i]).collect();
        let ctx = Context {
            centerline: &self.centerline,
            actors: &current,
            target_speed: self.target_speed,
            dt: self.dt,
            horizon: 1,
            latency: Some(&self.recorder),
        };
        let state = self.sim.tick(self.planner.as_mut(), &ctx);
        self.latency.absorb(self.recorder.take());
        self.ego.push(state);
    }

    /// Run ticks until `deadline` (wall clock) or completion, whichever
    /// comes first.
    pub fn step_until(&mut self, deadline: Instant) {
        while !self.is_done() && Instant::now() < deadline {
            self.tick_once();
        }
    }

    /// Run any remaining ticks synchronously and compute the final
    /// `Rollout`. Cheap (returns immediately) if already done.
    pub fn finish(mut self) -> Rollout {
        while !self.is_done() {
            self.tick_once();
        }
        let metrics = metrics::evaluate(
            &self.ego,
            &self.actors,
            &self.centerline,
            self.target_speed,
            self.dt,
        );
        Rollout {
            ego: self.ego,
            actors: self.actors,
            metrics,
            latency: self.latency,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulate_collects_latency_seams() {
        let sc = &crate::scenarios::synthetic_batch(1, 5)[0]; // a lead scenario
        let r = simulate(sc, PlannerKind::Lattice, 2.0, 0.1);
        let names: Vec<_> = r.latency.seams.iter().map(|s| s.name).collect();
        // standardized seams plus the lattice's custom one
        for expected in ["total", "route", "optimize", "extract", "edge_costs"] {
            assert!(names.contains(&expected), "missing seam {expected}");
        }
        let total = r.latency.seams.iter().find(|s| s.name == "total").unwrap();
        assert_eq!(total.calls, 20); // one per plan() call
        assert!(total.max_ms >= total.mean_ms());
    }

    #[test]
    fn incremental_sim_matches_simulate_and_reports_progress() {
        let sc = &crate::scenarios::synthetic_batch(1, 5)[0];
        let expected = simulate(sc, PlannerKind::Lattice, 2.0, 0.1);

        let mut job = IncrementalSim::start(sc, PlannerKind::Lattice, 2.0, 0.1);
        assert_eq!(job.progress(), 0.0);
        assert!(!job.is_done());
        // a deadline already in the past advances zero ticks
        job.step_until(web_time::Instant::now());
        assert_eq!(job.progress(), 0.0);
        // a generous deadline runs it to completion in one call
        job.step_until(web_time::Instant::now() + std::time::Duration::from_secs(3600));
        assert!(job.is_done());
        assert_eq!(job.progress(), 1.0);

        let r = job.finish();
        assert_eq!(r.ego, expected.ego);
        assert_eq!(r.metrics.score, expected.metrics.score);
    }

    #[test]
    fn drives_straight() {
        let s0 = State {
            speed: 1.0,
            ..Default::default()
        };
        let s1 = step(s0, Control::default(), 0.1);
        assert_eq!(
            s1,
            State {
                x: 0.1,
                speed: 1.0,
                ..Default::default()
            }
        );
    }

    #[test]
    fn turns_left_with_positive_curvature() {
        let s0 = State {
            speed: 1.0,
            ..Default::default()
        };
        let u = Control {
            accel: 0.0,
            curvature: 0.1,
        };
        let s1 = step(s0, u, 0.1);
        assert!(s1.yaw > 0.0);
    }
}
