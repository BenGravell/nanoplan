//! The kinematic vehicle model and the closed-loop simulator.

use serde::{Deserialize, Serialize};
use web_time::Instant;

use crate::metrics::{self, Metrics};
use crate::planning::{Context, Latency, LatencyStats, Planner, PlannerKind};
use crate::scenarios::{Path, Road, Scenario};

/// Vehicle state: pose, speed, and actuator positions.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct State {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub yaw: f64,
    #[serde(default)]
    pub speed: f64,
    #[serde(default)]
    pub accel: f64,
    #[serde(default)]
    pub curvature: f64,
}

/// Control action: longitudinal jerk and curvature rate.
/// The default holds the current actuator positions.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Control {
    #[serde(default)]
    pub jerk: f64,
    #[serde(default)]
    pub curvature_rate: f64,
}

/// Advance the kinematic model by one Euler step of length `dt`, enforcing
/// control bounds (jerk / curvature rate) and state bounds (acceleration /
/// curvature / lateral acceleration) in the same place.
pub fn step(s: State, u: Control, dt: f64) -> State {
    let s = clamp_state(s);
    let u = clamp_control(u);
    let accel = (s.accel + u.jerk * dt).clamp(MIN_LON_ACCEL, MAX_LON_ACCEL);
    let curvature = clamp_curvature(s.curvature + u.curvature_rate * dt, s.speed);
    State {
        x: s.x + s.speed * s.yaw.cos() * dt,
        y: s.y + s.speed * s.yaw.sin() * dt,
        yaw: s.yaw + s.speed * curvature * dt,
        speed: s.speed + accel * dt,
        accel,
        curvature,
    }
}

// The plant's actuation limits are the physical **capability** of a typical
// dry-road passenger car — what it *can* do, not what is comfortable. Comfort
// is a separate concern the `comfort` metric and the planners' cost express;
// the simulator only models physics, so these bounds sit at grip/traction/
// actuator capability, well outside the comfort envelope. Values are tuned to
// published passenger-car test data (skidpad, braking, 0–100 km/h, and
// steering-geometry figures) and cited per constant below.

/// Strongest forward acceleration, m/s². Traction/engine limited: a typical
/// passenger car reaches 100 km/h (27.8 m/s) in ~7–11 s — a ~2.5–4 m/s²
/// average — with a higher launch peak; 4.0 (~0.41 g) is a representative
/// peak for a brisk passenger car (cf. Bosch Automotive Handbook 0–100 km/h
/// figures). Real accel falls with speed; a single peak cap is a conservative
/// simplification.
pub const MAX_LON_ACCEL: f64 = 4.0;
/// Hardest braking deceleration, m/s². Tyre-grip limited on dry asphalt with
/// ABS: consumer/industry 100–0 km/h tests stop in ~34–38 m, i.e. ~10–11
/// m/s² (~1.0–1.1 g); −9.0 (~0.9 g) is a conservative dry-road capability,
/// consistent with the lateral grip limit (friction ellipse).
pub const MIN_LON_ACCEL: f64 = -9.0;
/// Longitudinal jerk capability, m/s³: how fast the powertrain/brakes can
/// change force. Emergency brake onset builds ~0.9 g in ~0.3–0.5 s (~20–30
/// m/s³); 20 is a conservative actuator-rate capability, far above the ~4
/// m/s³ the comfort metric calls smooth.
pub const MAX_ABS_LON_JERK: f64 = 20.0;

/// Tightest steer the plant will execute — a ~5 m turning radius
/// (κ = 1/R = 0.2 /m), matching a compact passenger car's minimum turning
/// circle. Also the absolute curvature the judo samplers clamp to. Only binds
/// at low speed; above it the lateral-grip cap is tighter.
pub const MAX_ABS_CURVATURE: f64 = 0.2;
/// Lateral-acceleration (tyre-grip) limit, m/s². Passenger-car skidpad tests
/// sustain ~0.85–0.95 g on dry asphalt; 9.0 (~0.9 g) is a representative
/// grip capability. The curvature a car can actually hold at a given speed is
/// `MAX_ABS_LAT_ACCEL / speed²`, so this tightens the steer as speed rises
/// (no hairpins at highway speed).
pub const MAX_ABS_LAT_ACCEL: f64 = 9.0;

/// Fastest the plant can change curvature, in 1/(m·s): the steering-rate
/// (steering-wheel rate) limit. A fast hand spins the wheel ~500–700 °/s;
/// through a ~15:1 steering ratio and a ~2.7 m wheelbase that is a curvature
/// rate of roughly 0.2–0.4 /(m·s) at small angles. At 0.4 the whole steering
/// range (`±MAX_ABS_CURVATURE`) takes about a second to traverse — a quick
/// emergency steer, not an instant one.
///
/// This is the steering analogue of the longitudinal jerk limit: curvature
/// rate is the action, while curvature itself is actuator state.
pub const MAX_ABS_CURVATURE_RATE: f64 = 0.4;

/// State curvature bound for a given speed: the tighter of steering geometry
/// and lateral grip.
pub fn curvature_limit(speed: f64) -> f64 {
    let v2 = speed * speed;
    if v2 > 1e-6 {
        MAX_ABS_CURVATURE.min(MAX_ABS_LAT_ACCEL / v2)
    } else {
        MAX_ABS_CURVATURE
    }
}

fn clamp_curvature(curvature: f64, speed: f64) -> f64 {
    let limit = curvature_limit(speed);
    curvature.clamp(-limit, limit)
}

/// Clamp the actuator-position part of a state.
pub fn clamp_state(s: State) -> State {
    State {
        accel: s.accel.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
        curvature: clamp_curvature(s.curvature, s.speed),
        ..s
    }
}

/// Clamp an action to the actuator-rate limits.
pub fn clamp_control(u: Control) -> Control {
    Control {
        jerk: u.jerk.clamp(-MAX_ABS_LON_JERK, MAX_ABS_LON_JERK),
        curvature_rate: u
            .curvature_rate
            .clamp(-MAX_ABS_CURVATURE_RATE, MAX_ABS_CURVATURE_RATE),
    }
}

/// Convert a desired acceleration/curvature target into a bounded
/// jerk/curvature-rate action from the current state.
pub fn action_toward(state: State, accel: f64, curvature: f64, dt: f64) -> Control {
    let state = clamp_state(state);
    let target_accel = accel.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL);
    let target_curvature = clamp_curvature(curvature, state.speed);
    clamp_control(Control {
        jerk: (target_accel - state.accel) / dt,
        curvature_rate: (target_curvature - state.curvature) / dt,
    })
}

/// Ego vehicle simulator.
pub struct Simulator {
    pub state: State,
    pub dt: f64,
}

impl Simulator {
    pub fn new(state: State, dt: f64) -> Self {
        Simulator {
            state: clamp_state(state),
            dt,
        }
    }

    /// Replan from the current state, advance one tick through the shared
    /// forward model, and return the new state. An empty plan holds actuator
    /// positions.
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
        let remaining = self.route.length() - self.route.project([ego.x, ego.y]).0;
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
            map: MapData::default(),
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
    fn step_applies_action_and_state_limits() {
        let s = State {
            speed: 3.0,
            accel: 1.0,
            curvature: 0.05,
            ..Default::default()
        };
        let ns = step(
            s,
            Control {
                jerk: 100.0,
                curvature_rate: 100.0,
            },
            0.1,
        );
        assert!((ns.accel - (s.accel + MAX_ABS_LON_JERK * 0.1)).abs() < 1e-9);
        assert!((ns.curvature - (s.curvature + MAX_ABS_CURVATURE_RATE * 0.1)).abs() < 1e-9);
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
            jerk: 0.0,
            curvature_rate: 1.0,
        };
        let s1 = step(s0, u, 0.1);
        assert!(s1.yaw > 0.0);
    }

    #[test]
    fn limits_clamp_accel_and_jerk() {
        // a wild action from rest: jerk holds the first-tick accel change to
        // MAX_ABS_LON_JERK · dt, well inside the accel bound
        let s = step(
            State {
                speed: 5.0,
                ..Default::default()
            },
            Control {
                jerk: 100.0,
                curvature_rate: 0.0,
            },
            0.1,
        );
        assert!(
            (s.accel - MAX_ABS_LON_JERK * 0.1).abs() < 1e-9,
            "accel {}",
            s.accel
        );
        // once ramped up, accel saturates at the capability bound, not beyond
        let mut s = State {
            speed: 5.0,
            ..Default::default()
        };
        for _ in 0..100 {
            s = step(
                s,
                Control {
                    jerk: 100.0,
                    curvature_rate: 0.0,
                },
                0.1,
            );
        }
        assert!((s.accel - MAX_LON_ACCEL).abs() < 1e-9, "accel {}", s.accel);
        // hard braking clamps to the (larger) deceleration bound
        let brake = step(
            State {
                speed: 5.0,
                accel: MIN_LON_ACCEL,
                ..Default::default()
            },
            Control {
                jerk: -100.0,
                curvature_rate: 0.0,
            },
            0.1,
        );
        assert!(brake.accel >= MIN_LON_ACCEL - 1e-9);
    }

    #[test]
    fn limits_cap_curvature_rate_and_lateral_accel() {
        // steering rate: a full-lock reversal (Δκ = 0.4) can't happen in one
        // tick — at a low speed where lateral accel doesn't bind, the wheel
        // moves only MAX_ABS_CURVATURE_RATE · dt from where it was
        let slow = 3.0;
        assert!(MAX_ABS_LAT_ACCEL / (slow * slow) > MAX_ABS_CURVATURE);
        let s = step(
            State {
                speed: slow,
                curvature: MAX_ABS_CURVATURE,
                ..Default::default()
            },
            Control {
                jerk: 0.0,
                curvature_rate: -100.0,
            },
            0.1,
        );
        let expected = MAX_ABS_CURVATURE - MAX_ABS_CURVATURE_RATE * 0.1;
        assert!(
            (s.curvature - expected).abs() < 1e-9,
            "curv {}",
            s.curvature
        );

        // lateral-accel (grip) cap: at speed, sustained max steering saturates
        // at the curvature giving MAX_ABS_LAT_ACCEL, tighter than the absolute
        // cap
        let fast = 25.0;
        let kappa_lat = MAX_ABS_LAT_ACCEL / (fast * fast);
        assert!(
            kappa_lat < MAX_ABS_CURVATURE,
            "test speed too low to bind lat accel"
        );
        let mut s = State {
            speed: fast,
            ..Default::default()
        };
        for _ in 0..100 {
            s = step(
                s,
                Control {
                    jerk: 0.0,
                    curvature_rate: 1.0,
                },
                0.1,
            );
        }
        assert!(
            (s.curvature - kappa_lat).abs() < 1e-9,
            "curv {}",
            s.curvature
        );
        assert!((s.curvature * fast * fast - MAX_ABS_LAT_ACCEL).abs() < 1e-9);
    }

    #[test]
    fn a_wild_plan_cannot_spin_the_car() {
        // a planner slamming the wheel lock-to-lock every tick at speed: the
        // plant holds curvature, lateral accel, and per-tick steering change to
        // the capability bounds regardless
        let mut sim = Simulator::new(
            State {
                speed: 8.0,
                ..Default::default()
            },
            0.1,
        );
        for k in 0..200 {
            let prev_curvature = sim.state.curvature;
            let u = Control {
                jerk: 0.0,
                curvature_rate: if k % 2 == 0 { 5.0 } else { -5.0 },
            };
            let prev_yaw = sim.state.yaw;
            sim.state = step(sim.state, u, sim.dt);
            let dk = (sim.state.curvature - prev_curvature).abs();
            let yaw_rate = crate::wrap_angle(sim.state.yaw - prev_yaw) / sim.dt;
            let lat_accel = yaw_rate * sim.state.speed;
            assert!(sim.state.curvature.abs() <= MAX_ABS_CURVATURE + 1e-9);
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
