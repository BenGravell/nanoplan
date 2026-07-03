//! The kinematic vehicle model and the closed-loop simulator.

use serde::{Deserialize, Serialize};

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

/// Run a planner closed-loop through a scenario.
pub fn simulate(sc: &Scenario, kind: PlannerKind, duration_s: f64, dt: f64) -> Rollout {
    let steps = (duration_s / dt) as usize;
    let actors: Vec<Vec<State>> = sc.actors.iter().map(|a| a.trace(steps, dt)).collect();
    let mut sim = Simulator { state: sc.ego, dt };
    let mut planner = kind.build();
    let recorder = Latency::default();
    let mut latency = LatencyStats::default();
    let mut ego = vec![sc.ego];
    ego.extend((0..steps).map(|i| {
        let current: Vec<State> = actors.iter().map(|t| t[i]).collect();
        let ctx = Context {
            centerline: &sc.centerline,
            actors: &current,
            target_speed: sc.target_speed,
            dt,
            horizon: 1,
            latency: Some(&recorder),
        };
        let state = sim.tick(planner.as_mut(), &ctx);
        latency.absorb(recorder.take());
        state
    }));
    let metrics = metrics::evaluate(&ego, &actors, &sc.centerline, sc.target_speed, dt);
    Rollout {
        ego,
        actors,
        metrics,
        latency,
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
