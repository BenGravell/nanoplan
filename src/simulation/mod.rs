//! The kinematic vehicle model and the closed-loop simulator.

use serde::{Deserialize, Serialize};
use web_time::Instant;

use crate::metrics::{self, Metrics};
use crate::planning::{Context, Latency, LatencyStats, Planner, PlannerKind};
use crate::scenarios::{Path, Road, Scenario};

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
/// This is the steering analogue of the longitudinal jerk limit, and enforced
/// just as faithfully: curvature is a control action, not a [`State`] field,
/// exactly like acceleration, so the rate is priced against the *previously
/// applied* control. That only works because planners roll their candidates
/// out through the same forward model the plant advances with ([`drive`]),
/// seeded with the ego's current control ([`Context::ego_control`](crate::planning::Context)),
/// so they plan the steering *ramp* rather than a step the plant clips out
/// from under them.
pub const MAX_ABS_CURVATURE_RATE: f64 = 0.4;

/// Clamp a commanded [`Control`] to the vehicle's physical actuation
/// capability, given the control applied the previous tick (`prev`) and the
/// current `speed`. The plant applies this before integrating, so no planner —
/// however wild its commanded plan — can drive the car harder than a real
/// vehicle:
///
/// - **longitudinal acceleration** held within `MIN_LON_ACCEL..=MAX_LON_ACCEL`;
/// - **longitudinal jerk** (accel rate of change) held within
///   `MAX_ABS_LON_JERK`, so throttle/brake can't step instantly;
/// - **steering angle** (absolute curvature) held within `MAX_ABS_CURVATURE`;
/// - **lateral acceleration** (`speed² · curvature`) held within the grip
///   limit `MAX_ABS_LAT_ACCEL`, which tightens the curvature limit as speed
///   rises (a car can't hold a hairpin at highway speed);
/// - **steering rate** (curvature rate of change) held within
///   `MAX_ABS_CURVATURE_RATE`.
///
/// The accel/jerk pair and the curvature/lat-accel/rate trio each clamp the
/// absolute value first, then rate-limit the change from `prev`, so the result
/// stays inside every bound at once.
pub fn apply_limits(prev: Control, cmd: Control, speed: f64, dt: f64) -> Control {
    // longitudinal: accel bounds, then jerk-limit the change since last tick
    let accel = cmd.accel.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL);
    let max_da = MAX_ABS_LON_JERK * dt;
    let accel = prev.accel + (accel - prev.accel).clamp(-max_da, max_da);

    // steering: absolute curvature, then the lateral-accel (grip) cap
    // (|v²·κ| ≤ bound; no cap at a standstill, where any curvature is zero
    // lateral accel), then rate-limit the change since last tick
    let mut curvature = cmd.curvature.clamp(-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE);
    let v2 = speed * speed;
    if v2 > 1e-6 {
        let kappa_lat = MAX_ABS_LAT_ACCEL / v2;
        curvature = curvature.clamp(-kappa_lat, kappa_lat);
    }
    let max_dk = MAX_ABS_CURVATURE_RATE * dt;
    let curvature = prev.curvature + (curvature - prev.curvature).clamp(-max_dk, max_dk);

    Control { accel, curvature }
}

/// The one forward vehicle model the plant advances with and every planner
/// rolls out through: clamp `cmd` to the actuation limits given the control
/// applied the previous tick ([`apply_limits`]), integrate one [`step`], and
/// return the new state together with the control **actually applied** — which
/// the next call takes back as `prev`.
///
/// Threading the applied control back through is what makes the jerk and
/// steering-rate limits real to a planner rather than a surprise the plant
/// springs afterwards: rolling a candidate out through `drive` predicts the
/// same clipped motion the plant will produce, so the planner plans a *ramp*
/// that fits within the rate limits instead of a step the plant clips out from
/// under it. Because those limits are per-tick differences from `prev`, a
/// planner's first rollout step must seed `prev` with the ego's current
/// applied control — carried on the planning [`Context`](crate::planning::Context).
pub fn drive(state: State, prev: Control, cmd: Control, dt: f64) -> (State, Control) {
    let u = apply_limits(prev, cmd, state.speed, dt);
    (step(state, u, dt), u)
}

/// Ego vehicle simulator.
pub struct Simulator {
    pub state: State,
    pub dt: f64,
    /// Control actually applied last tick, so the plant can rate-limit jerk
    /// and steering rate (see [`apply_limits`]).
    prev_control: Control,
}

impl Simulator {
    /// A simulator starting at rest w.r.t. actuation (no control applied yet).
    pub fn new(state: State, dt: f64) -> Self {
        Simulator {
            state,
            dt,
            prev_control: Control::default(),
        }
    }

    /// The control actually applied to the ego last tick — seed a planner's
    /// rollout with this (via [`Context::ego_control`]) so its jerk and
    /// steering-rate limiting starts from the same place the plant's will.
    pub fn prev_control(&self) -> Control {
        self.prev_control
    }

    /// Replan from the current state, advance one tick through the shared
    /// forward model ([`drive`], which clamps the first planned control to the
    /// vehicle's actuation limits), and return the new state. An empty plan
    /// coasts (zero control).
    pub fn tick(&mut self, planner: &mut dyn Planner, ctx: &Context) -> State {
        let cmd = ctx
            .time("total", || planner.plan(self.state, ctx))
            .first()
            .copied()
            .unwrap_or_default();
        let (state, u) = drive(self.state, self.prev_control, cmd, self.dt);
        self.prev_control = u;
        self.state = state;
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
            ego_control: self.sim.prev_control(),
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
    fn drive_applies_the_limits_and_returns_the_applied_control() {
        // drive is step ∘ apply_limits: the returned control is exactly what
        // the limits produce, and the new state is that control integrated —
        // so a planner rolling out through drive sees the same clipped motion
        // the plant will
        let s = State {
            speed: 12.0,
            ..Default::default()
        };
        let prev = Control {
            accel: 1.0,
            curvature: 0.05,
        };
        let cmd = Control {
            accel: 100.0,
            curvature: 100.0,
        }; // wild command
        let (ns, u) = drive(s, prev, cmd, 0.1);
        assert_eq!(u, apply_limits(prev, cmd, s.speed, 0.1));
        assert_eq!(ns, step(s, u, 0.1));
        // the wild command was actually clipped — steering rate holds it near
        // the previous curvature, not at the demanded lock
        assert!((u.curvature - prev.curvature).abs() <= MAX_ABS_CURVATURE_RATE * 0.1 + 1e-9);
        assert!(u.curvature < MAX_ABS_CURVATURE);
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

    #[test]
    fn limits_clamp_accel_and_jerk() {
        // a wild command from rest (prev = 0): jerk holds the first-tick accel
        // change to MAX_ABS_LON_JERK · dt, well inside the accel bound
        let u = apply_limits(
            Control::default(),
            Control {
                accel: 100.0,
                curvature: 0.0,
            },
            5.0,
            0.1,
        );
        assert!(
            (u.accel - MAX_ABS_LON_JERK * 0.1).abs() < 1e-9,
            "accel {}",
            u.accel
        );
        // once ramped up, accel saturates at the capability bound, not beyond
        let mut prev = Control::default();
        for _ in 0..100 {
            prev = apply_limits(
                prev,
                Control {
                    accel: 100.0,
                    curvature: 0.0,
                },
                5.0,
                0.1,
            );
        }
        assert!(
            (prev.accel - MAX_LON_ACCEL).abs() < 1e-9,
            "accel {}",
            prev.accel
        );
        // hard braking clamps to the (larger) deceleration bound
        let brake = apply_limits(
            Control {
                accel: MIN_LON_ACCEL,
                ..Default::default()
            },
            Control {
                accel: -100.0,
                curvature: 0.0,
            },
            5.0,
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
        let u = apply_limits(
            Control {
                accel: 0.0,
                curvature: MAX_ABS_CURVATURE,
            },
            Control {
                accel: 0.0,
                curvature: -MAX_ABS_CURVATURE,
            },
            slow,
            0.1,
        );
        let expected = MAX_ABS_CURVATURE - MAX_ABS_CURVATURE_RATE * 0.1;
        assert!(
            (u.curvature - expected).abs() < 1e-9,
            "curv {}",
            u.curvature
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
        let mut prev = Control::default();
        for _ in 0..100 {
            prev = apply_limits(
                prev,
                Control {
                    accel: 0.0,
                    curvature: 1.0,
                },
                fast,
                0.1,
            );
        }
        assert!(
            (prev.curvature - kappa_lat).abs() < 1e-9,
            "curv {}",
            prev.curvature
        );
        assert!((prev.curvature * fast * fast - MAX_ABS_LAT_ACCEL).abs() < 1e-9);
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
        let mut prev = Control::default();
        for k in 0..200 {
            let cmd = Control {
                accel: 0.0,
                curvature: if k % 2 == 0 { 5.0 } else { -5.0 },
            };
            let u = apply_limits(prev, cmd, sim.state.speed, sim.dt);
            let prev_yaw = sim.state.yaw;
            let dk = (u.curvature - prev.curvature).abs();
            sim.state = step(sim.state, u, sim.dt);
            prev = u;
            let yaw_rate = crate::wrap_angle(sim.state.yaw - prev_yaw) / sim.dt;
            let lat_accel = yaw_rate * sim.state.speed;
            assert!(u.curvature.abs() <= MAX_ABS_CURVATURE + 1e-9);
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
