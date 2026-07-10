//! The kinematic vehicle model and the closed-loop simulator.

use web_time::Instant;

use crate::metrics::{self, Metrics};
use crate::planning::{Context, Latency, LatencyStats, Planner, PlannerKind};
use crate::scenarios::{Path, Road, Scenario};

mod collision;
mod integration;
pub(crate) mod physics;
mod state_control;

pub(crate) use crate::barrier::collide_with_road_barriers;
pub use crate::barrier::{BARRIER_RESTITUTION, Barrier, collide_with_barriers, road_side_barriers};
#[cfg(test)]
use crate::vehicle::MAX_ABS_CURVATURE_RATE;
pub use crate::vehicle::{
    AIR_DENSITY_KG_M3, DRAG_AREA_M2, EGO_MASS_KG, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL,
    MAX_LON_ACCEL, MIN_LON_ACCEL, ROLLING_RESISTANCE_COEFF,
};
pub(crate) use collision::{collide_with_actors, collide_with_car_actors};
pub(crate) use integration::CommandLimiter;
pub use state_control::{Control, Pose, Position, State};

/// Ego vehicle simulator.
pub struct Simulator {
    pub state: State,
    pub dt: f64,
    limiter: CommandLimiter,
}

impl Simulator {
    pub fn new(state: State, dt: f64) -> Self {
        Simulator {
            state,
            dt,
            limiter: CommandLimiter::new(),
        }
    }

    /// Replan from the current state, advance one tick through the shared
    /// forward model, and return the new state. An empty plan coasts straight.
    pub fn tick(&mut self, planner: &mut dyn Planner, ctx: &Context) -> State {
        let u = ctx
            .time("total", || planner.plan(self.state, ctx))
            .first()
            .copied()
            .unwrap_or_default();
        let prev = self.state;
        let next =
            collide_with_road_barriers(prev, self.limiter.step(self.state, u, self.dt), ctx.road);
        let next = collide_with_car_actors(next, ctx.actors.iter().copied());
        self.state = collide_with_road_barriers(prev, next, ctx.road);
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
    road: Road,
    /// The route as a path, for tapering the target speed into its end.
    route: Path,
    /// The scenario's own target speed, restored for scoring after each tick's
    /// goal taper.
    base_target_speed: f64,
    sim: Simulator,
    planner: Box<dyn Planner>,
    recorder: Latency,
    latency: LatencyStats,
    ego: Vec<State>,
    steps_total: usize,
}

/// Comfortable deceleration the target speed is tapered by into the route end,
/// so the ego arrives stopped at the goal instead of sailing off the end of
/// its reference — where the degenerate past-the-end geometry otherwise
/// provokes a wild spin. Matches the open world's own goal taper.
const GOAL_DECEL_MS2: f64 = 1.5;

impl IncrementalSim {
    pub fn start(sc: &Scenario, kind: PlannerKind, duration_s: f64, dt: f64) -> Self {
        let steps_total = (duration_s / dt) as usize;
        let road = sc.road(dt);
        let route = Path::new(&road.centerline);
        let base_target_speed = road.target_speed;
        IncrementalSim {
            actors: sc.actors.iter().map(|a| a.trace(steps_total, dt)).collect(),
            road,
            route,
            base_target_speed,
            sim: Simulator::new(sc.ego, dt),
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
        // taper the target speed the planner sees into a comfortable stop at
        // the route end, so it arrives and holds the goal pose instead of
        // driving off the end of its reference and spinning. Scoring keeps the
        // scenario's own speed limit (`base_target_speed`, restored in
        // `finish`); this only shapes planning near the goal, and where the
        // route outlasts the horizon (the ego never nears its end) it never
        // binds.
        let ego = self.sim.state;
        let remaining = self.route.length() - self.route.project(ego.position()).0;
        self.road.target_speed = self
            .base_target_speed
            .min((2.0 * GOAL_DECEL_MS2 * remaining.max(0.0)).sqrt());
        let ctx = Context {
            road: &self.road,
            actors: &current,
            horizon: 1,
            latency: Some(&self.recorder),
            diagnostics: None,
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
        // score against the scenario's own speed limit, not the last tick's
        // goal-tapered value
        self.road.target_speed = self.base_target_speed;
        let metrics = metrics::evaluate(&self.ego, &self.actors, &self.road);
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
        // standardized seams, including the shared cost function's "cost"
        for expected in ["total", "route", "optimize", "extract", "cost"] {
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
    fn tapers_to_a_stop_at_the_route_end() {
        use crate::scenarios::{MapData, Scenario};
        // a route much shorter than 20 s of travel: without the goal taper the
        // ego reaches the end at speed and drives off it; with it, it arrives
        // stopped and holds the end pose.
        let sc = Scenario {
            name: "short-route".into(),
            ego: State {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
                speed: 10.0,
                ..Default::default()
            },
            actors: vec![],
            centerline: vec![[-5.0, 0.0], [60.0, 0.0]],
            target_speed: 10.0,
            map: MapData::new(5.5),
            expert: vec![],
        };
        let r = simulate(&sc, PlannerKind::BezierIdm, 20.0, 0.1);
        let end = r.ego.last().unwrap();
        assert!(end.x > 54.0 && end.x < 64.0, "ended at x {}", end.x);
        assert!(end.speed < 1.0, "never stopped, speed {}", end.speed);
        // and it stayed on its road the whole way (no spin off the end)
        assert_eq!(r.metrics.aggregate[1], 1.0, "left the drivable area");
    }

    #[test]
    fn a_wild_plan_cannot_spin_the_car() {
        // a planner slamming the wheel every tick at speed: the simulator
        // holds curvature rate and lateral accel to plant limits regardless.
        let mut sim = Simulator::new(
            State {
                speed: 8.0,
                ..Default::default()
            },
            0.1,
        );
        for k in 0..200 {
            let prev_applied = sim.limiter.applied;
            let u = Control {
                acceleration: 0.0,
                curvature: if k % 2 == 0 { 5.0 } else { -5.0 },
            };
            let prev_yaw = sim.state.yaw;
            sim.state = sim.limiter.step(sim.state, u, sim.dt);
            let yaw_rate = crate::math::wrap_angle(sim.state.yaw - prev_yaw) / sim.dt;
            let lat_accel = yaw_rate * sim.state.speed;
            let dk = (sim.limiter.applied.curvature - prev_applied.curvature).abs();
            assert!(
                dk <= MAX_ABS_CURVATURE_RATE * sim.dt + 1e-9,
                "steer step {dk}"
            );
            assert!(
                lat_accel.abs() <= MAX_ABS_LAT_ACCEL + 1e-6,
                "lat accel {lat_accel}"
            );
        }
    }
}
