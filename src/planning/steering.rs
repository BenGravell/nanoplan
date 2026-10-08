//! Shared flat-output steering primitives for planners.
//!
//! Planners connect sampled states by choosing cubic Hermite `x(t)` and
//! `y(t)` polynomials, then reading acceleration and curvature back from
//! their derivatives. The curve matches only pose and velocity: acceleration
//! stays a control, not hidden planner state.

use crate::common::kinematics::{clamp_control, commanded_accel_for_net, commanded_accel_to_stop};
use crate::common::measure::dot;
use crate::common::polynomial::CubicPolynomial;
use crate::common::types::{Control, Position, State};
use crate::simulation::world_step_unclamped;

/// Cubic flat-output connector between two states/poses.
///
/// The polynomial in each coordinate matches position and velocity at both
/// endpoints:
///
/// `p(t) = c0 + c1*t + c2*t^2 + c3*t^3`
pub(crate) struct CubicSteer {
    cx: CubicPolynomial,
    cy: CubicPolynomial,
    duration: f64,
}

impl CubicSteer {
    /// Fit a time-parametrized connector between full vehicle states.
    pub(crate) fn from_states(start: &State, goal: &State, duration: f64) -> Self {
        let duration = duration.max(1e-6);
        let v0 = state_velocity(start);
        let v1 = state_velocity(goal);
        Self {
            cx: CubicPolynomial::from_boundary(start.position().x, v0[0], goal.position().x, v1[0], duration),
            cy: CubicPolynomial::from_boundary(start.position().y, v0[1], goal.position().y, v1[1], duration),
            duration,
        }
    }

    /// Fit a unit-interval connector between oriented positions. The
    /// derivative magnitude is tied to chord length.
    pub(crate) fn from_poses(p0: Position, yaw0: f64, p1: Position, yaw1: f64) -> Self {
        let k = p0.distance(p1).max(1e-3) / 2.0;
        let boundary = |yaw: f64| (Position::from_angle(yaw) * k).xy();
        let v0 = boundary(yaw0);
        let v1 = boundary(yaw1);
        Self {
            cx: CubicPolynomial::from_boundary(p0.x, v0[0], p1.x, v1[0], 1.0),
            cy: CubicPolynomial::from_boundary(p0.y, v0[1], p1.y, v1[1], 1.0),
            duration: 1.0,
        }
    }

    pub(crate) fn point(&self, t: f64) -> Position {
        let t = t.clamp(0.0, self.duration);
        Position::new(self.cx.at(t)[0], self.cy.at(t)[0])
    }

    pub(crate) fn curvature(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, self.duration);
        let [_, dx, ddx] = self.cx.at(t);
        let [_, dy, ddy] = self.cy.at(t);
        let velocity = Position::new(dx, dy);
        let acceleration = Position::new(ddx, ddy);
        velocity.cross(acceleration) / velocity.norm().max(1e-6).powi(3)
    }

    /// Flat-output action `(longitudinal acceleration, curvature)`.
    pub(crate) fn control(&self, t: f64) -> Control {
        let (_, acceleration, curvature) = self.flat_motion(t);
        Control {
            acceleration,
            curvature,
        }
    }

    fn flat_motion(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.duration);
        let [_, dx, ddx] = self.cx.at(t);
        let [_, dy, ddy] = self.cy.at(t);
        let velocity = Position::new(dx, dy);
        let acceleration = Position::new(ddx, ddy);
        let speed = velocity.norm();
        if speed <= 0.01 {
            return (speed, acceleration.norm(), 0.0);
        }
        let accel = dot(velocity.xy(), acceleration.xy()) / speed;
        let curvature = velocity.cross(acceleration) / speed.powi(3);
        (speed, accel, curvature)
    }

    pub(crate) fn forward_sign(&self, yaw: f64, probe_t: f64) -> f64 {
        let p0 = self.point(0.0);
        let p1 = self.point(probe_t.min(self.duration));
        let forward = Position::from_angle(yaw);
        if dot((p1 - p0).xy(), forward.xy()) >= 0.0 {
            1.0
        } else {
            -1.0
        }
    }

    /// Sample `n` points from start to end inclusive.
    pub(crate) fn sample(&self, n: usize) -> Vec<Position> {
        (0..n)
            .map(|i| self.point(self.duration * i as f64 / (n - 1) as f64))
            .collect()
    }
}

/// Convert a fitted cubic's analytic flat-output action into direct
/// controls, sampling each segment at its midpoint. `curvature_sign` lets
/// callers flip the curve when they intentionally drive it in reverse.
/// Disable `clamp` to retain infeasible commands and integrate them without
/// limits, leaving feasibility checks to the caller.
pub(crate) fn steer_controls(
    start: State,
    steer: &CubicSteer,
    dt: f64,
    ticks: usize,
    curvature_sign: f64,
    clamp: bool,
) -> (Vec<Control>, State) {
    let mut x = start;
    let controls = (0..ticks)
        .map(|i| {
            let t = (i as f64 + 0.5) * dt;
            let mut u = steer.control(t);
            u.curvature *= curvature_sign;
            u.acceleration = commanded_accel_for_net(u.acceleration, x.speed);
            if clamp {
                u.acceleration = u.acceleration.max(commanded_accel_to_stop(x.speed, dt));
                u = clamp_control(u, x.speed);
            }
            x = world_step_unclamped(x, u, dt);
            u
        })
        .collect();
    (controls, x)
}

fn state_velocity(x: &State) -> [f64; 2] {
    (Position::from_angle(x.pose.yaw) * x.speed).xy()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cubic_matches_state_endpoints() {
        let start = State {
            speed: 5.0,
            ..Default::default()
        };
        let goal = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(8.0, 1.0), 0.1),
            6.0,
        );
        let steer = CubicSteer::from_states(&start, &goal, 1.2);

        let p0 = steer.point(0.0);
        let p1 = steer.point(1.2);
        assert!((p0.x - start.position().x).abs() < 1e-9);
        assert!((p0.y - start.position().y).abs() < 1e-9);
        assert!((p1.x - goal.position().x).abs() < 1e-9);
        assert!((p1.y - goal.position().y).abs() < 1e-9);
    }

    #[test]
    fn cubic_control_reports_acceleration_and_curvature() {
        let start = State {
            speed: 5.0,
            ..Default::default()
        };
        let goal = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(8.0, 1.0), 0.1),
            6.0,
        );
        let steer = CubicSteer::from_states(&start, &goal, 1.2);
        let t = 0.6;
        let (_, accel, curvature) = steer.flat_motion(t);
        let u = steer.control(t);

        assert!((u.acceleration - accel).abs() < 1e-9);
        assert!((u.curvature - curvature).abs() < 1e-9);
    }

    #[test]
    fn steer_controls_sample_analytic_control() {
        let start = State {
            speed: 5.0,
            ..Default::default()
        };
        let goal = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(8.0, 1.0), 0.1),
            6.0,
        );
        let steer = CubicSteer::from_states(&start, &goal, 1.2);
        let (controls, _) = steer_controls(start, &steer, 0.1, 1, 1.0, true);

        let mut expected = steer.control(0.05);
        expected.acceleration = commanded_accel_for_net(expected.acceleration, start.speed);
        assert_eq!(controls[0], clamp_control(expected, start.speed));
    }

    #[test]
    fn steering_compensates_resistance_and_optionally_preserves_infeasible_controls() {
        let start = State {
            speed: 5.0,
            ..Default::default()
        };
        let goal = State::from((Position::new(50.0, 0.0), 0.0, 5.0));
        let steer = CubicSteer::from_states(&start, &goal, 10.0);
        let (_, end) = steer_controls(start, &steer, 0.1, 100, 1.0, false);
        assert!((end.speed - start.speed).abs() < 1e-9);
        assert!(end.position().distance(goal.position()) < 1e-9);

        let goal = State::from((Position::new(5.0, 5.0), 0.0, 5.0));
        let steer = CubicSteer::from_states(&start, &goal, 1.0);
        let (raw, raw_end) = steer_controls(start, &steer, 0.1, 1, 1.0, false);
        let (limited, limited_end) = steer_controls(start, &steer, 0.1, 1, 1.0, true);
        let mut expected = steer.control(0.05);
        expected.acceleration = commanded_accel_for_net(expected.acceleration, start.speed);
        assert_eq!(raw[0], expected);
        assert_ne!(raw[0], limited[0]);
        assert_eq!(limited[0], clamp_control(raw[0], start.speed));
        assert!((raw_end.speed - (start.speed + steer.control(0.05).acceleration * 0.1)).abs() < 1e-9);
        assert!((raw_end.pose.yaw - (start.pose.yaw + start.speed * raw[0].curvature * 0.1)).abs() < 1e-9);
        assert_ne!(raw_end.speed, limited_end.speed);
        assert_ne!(raw_end.pose.yaw, limited_end.pose.yaw);
        assert_eq!(limited_end, crate::simulation::world_step(start, limited[0], 0.1));
    }
}
