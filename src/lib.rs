//! Ultra minimalist motion planner for car-like vehicles.
//!
//! Planned architecture: trajectory trees expanded by sampling-based DDP
//! over a kinematic car model.

pub mod metrics;
mod pi2ddp;
mod planners;
pub mod scenario;

pub use metrics::Metrics;
pub use pi2ddp::Pi2DdpPlanner;
pub use planners::{BezierIdmPlanner, LatticePlanner, Path};
pub use scenario::{Rollout, Scenario, simulate};

use serde::{Deserialize, Serialize};

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

/// Deterministic xorshift* RNG with Box-Muller normals; avoids a rand
/// dependency and keeps batches and tests reproducible.
pub(crate) struct Rng(pub u64);

impl Rng {
    pub fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }

    /// Uniform sample in [lo, hi).
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }
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

/// Everything a planner sees besides the ego state.
pub struct Context<'a> {
    /// Lane centerline the ego should follow, as a polyline.
    pub centerline: &'a [[f64; 2]],
    /// Current states of the other actors.
    pub actors: &'a [State],
    /// Desired cruise speed.
    pub target_speed: f64,
    /// Tick length of the returned control trajectory.
    pub dt: f64,
    /// Requested number of controls (planners may return fewer or more).
    pub horizon: usize,
}

/// A planner turns the current state into a control trajectory.
/// The simulator applies the first control each tick (receding horizon).
pub trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}

/// Strawman planner: straight ahead at constant speed, always.
pub struct StraightPlanner;

impl Planner for StraightPlanner {
    fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
        vec![Control::default(); ctx.horizon]
    }
}

/// Configuration: which planner to run. Lets the app (and later, benchmarks)
/// compare planners on the same scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlannerKind {
    Straight,
    BezierIdm,
    Lattice,
    Pi2Ddp,
}

impl PlannerKind {
    pub const ALL: [PlannerKind; 4] = [
        PlannerKind::Straight,
        PlannerKind::BezierIdm,
        PlannerKind::Lattice,
        PlannerKind::Pi2Ddp,
    ];

    pub fn name(self) -> &'static str {
        match self {
            PlannerKind::Straight => "straight (strawman)",
            PlannerKind::BezierIdm => "bezier + IDM",
            PlannerKind::Lattice => "frenet lattice",
            PlannerKind::Pi2Ddp => "PI2-DDP",
        }
    }

    pub fn build(self) -> Box<dyn Planner> {
        match self {
            PlannerKind::Straight => Box::new(StraightPlanner),
            PlannerKind::BezierIdm => Box::new(BezierIdmPlanner),
            PlannerKind::Lattice => Box::new(LatticePlanner),
            PlannerKind::Pi2Ddp => Box::new(Pi2DdpPlanner::default()),
        }
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
        let u = planner
            .plan(self.state, ctx)
            .first()
            .copied()
            .unwrap_or_default();
        self.state = step(self.state, u, self.dt);
        self.state
    }
}

#[cfg(test)]
pub(crate) fn test_ctx<'a>(centerline: &'a [[f64; 2]], actors: &'a [State]) -> Context<'a> {
    Context {
        centerline,
        actors,
        target_speed: 10.0,
        dt: 0.1,
        horizon: 10,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn strawman_planner_holds_heading_and_speed() {
        let mut sim = Simulator {
            state: State {
                x: 1.0,
                y: 2.0,
                yaw: 0.5,
                speed: 3.0,
            },
            dt: 0.1,
        };
        let centerline = [[0.0, 0.0], [100.0, 0.0]];
        let ctx = test_ctx(&centerline, &[]);
        let mut planner = StraightPlanner;
        for _ in 0..100 {
            sim.tick(&mut planner, &ctx);
        }
        let s = sim.state;
        assert_eq!((s.yaw, s.speed), (0.5, 3.0));
        // 100 ticks of 0.1 s at 3 m/s = 30 m along the initial heading
        assert!((s.x - (1.0 + 30.0 * 0.5f64.cos())).abs() < 1e-9);
        assert!((s.y - (2.0 + 30.0 * 0.5f64.sin())).abs() < 1e-9);
    }

    // ponytail: smoke test that bevy links and boots headless; delete once a real app exists
    #[test]
    fn bevy_app_boots() {
        use bevy::prelude::*;
        App::new().add_plugins(MinimalPlugins).update();
    }
}
