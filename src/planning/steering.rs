//! Shared flat-output steering primitives for planners.
//!
//! Planners connect sampled states by choosing cubic Hermite `x(t)` and
//! `y(t)` polynomials, then reading acceleration and curvature back from
//! their derivatives. The curve matches only pose and velocity: acceleration
//! stays a control, not hidden planner state.

use crate::simulation::{Control, State, clamp_control, step};

/// Cubic flat-output connector between two states/poses.
///
/// The polynomial in each coordinate matches position and velocity at both
/// endpoints:
///
/// `p(t) = c0 + c1*t + c2*t^2 + c3*t^3`
pub(crate) struct CubicSteer {
    cx: [f64; 4],
    cy: [f64; 4],
    duration: f64,
}

impl CubicSteer {
    /// Fit a time-parametrized connector between full vehicle states.
    pub(crate) fn from_states(start: &State, goal: &State, duration: f64) -> Self {
        let duration = duration.max(1e-6);
        let v0 = state_velocity(start);
        let v1 = state_velocity(goal);
        Self {
            cx: cubic_coeffs(start.x, v0[0], goal.x, v1[0], duration),
            cy: cubic_coeffs(start.y, v0[1], goal.y, v1[1], duration),
            duration,
        }
    }

    /// Fit a unit-interval connector between oriented positions. The
    /// derivative magnitude is tied to chord length.
    pub(crate) fn from_poses(p0: [f64; 2], yaw0: f64, p1: [f64; 2], yaw1: f64) -> Self {
        let k = dist(p0, p1).max(1e-3) / 2.0;
        let boundary = |yaw: f64| {
            let tangent = [yaw.cos(), yaw.sin()];
            [k * tangent[0], k * tangent[1]]
        };
        let v0 = boundary(yaw0);
        let v1 = boundary(yaw1);
        Self {
            cx: cubic_coeffs(p0[0], v0[0], p1[0], v1[0], 1.0),
            cy: cubic_coeffs(p0[1], v0[1], p1[1], v1[1], 1.0),
            duration: 1.0,
        }
    }

    pub(crate) fn point(&self, t: f64) -> [f64; 2] {
        let t = t.clamp(0.0, self.duration);
        [eval(&self.cx, t), eval(&self.cy, t)]
    }

    pub(crate) fn curvature(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, self.duration);
        let (dx, dy) = (eval_d1(&self.cx, t), eval_d1(&self.cy, t));
        let (ddx, ddy) = (eval_d2(&self.cx, t), eval_d2(&self.cy, t));
        let speed = dx.hypot(dy).max(1e-6);
        (dx * ddy - dy * ddx) / speed.powi(3)
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
        let (dx, dy) = (eval_d1(&self.cx, t), eval_d1(&self.cy, t));
        let (ddx, ddy) = (eval_d2(&self.cx, t), eval_d2(&self.cy, t));
        let speed = dx.hypot(dy);
        if speed <= 0.01 {
            return (speed, ddx.hypot(ddy), 0.0);
        }
        let accel = (dx * ddx + dy * ddy) / speed;
        let curvature = (dx * ddy - dy * ddx) / speed.powi(3);
        (speed, accel, curvature)
    }

    pub(crate) fn forward_sign(&self, yaw: f64, probe_t: f64) -> f64 {
        let p0 = self.point(0.0);
        let p1 = self.point(probe_t.min(self.duration));
        if (p1[0] - p0[0]) * yaw.cos() + (p1[1] - p0[1]) * yaw.sin() >= 0.0 {
            1.0
        } else {
            -1.0
        }
    }

    /// Sample `n` points from start to end inclusive.
    pub(crate) fn sample(&self, n: usize) -> Vec<[f64; 2]> {
        (0..n)
            .map(|i| self.point(self.duration * i as f64 / (n - 1) as f64))
            .collect()
    }
}

/// Convert a fitted cubic's analytic flat-output action into direct
/// controls, sampling each segment at its midpoint. `curvature_sign` lets
/// callers flip the curve when they intentionally drive it in reverse.
pub(crate) fn steer_controls(
    start: State,
    steer: &CubicSteer,
    dt: f64,
    ticks: usize,
    curvature_sign: f64,
) -> (Vec<Control>, State) {
    let mut x = start;
    let controls = (0..ticks)
        .map(|i| {
            let t = (i as f64 + 0.5) * dt;
            let mut u = steer.control(t);
            u.curvature *= curvature_sign;
            let accel_floor = -x.speed / dt;
            u.acceleration = u.acceleration.max(accel_floor);
            let u = clamp_control(u, x.speed);
            x = step(x, u, dt);
            u
        })
        .collect();
    (controls, x)
}

fn state_velocity(x: &State) -> [f64; 2] {
    let tangent = [x.yaw.cos(), x.yaw.sin()];
    [x.speed * tangent[0], x.speed * tangent[1]]
}

fn cubic_coeffs(p0: f64, v0: f64, p1: f64, v1: f64, t: f64) -> [f64; 4] {
    let (t2, t3) = (t * t, t * t * t);
    [
        p0,
        v0,
        (3.0 * (p1 - p0) - (2.0 * v0 + v1) * t) / t2,
        (2.0 * (p0 - p1) + (v0 + v1) * t) / t3,
    ]
}

fn eval(c: &[f64; 4], t: f64) -> f64 {
    c[0] + t * (c[1] + t * (c[2] + t * c[3]))
}

fn eval_d1(c: &[f64; 4], t: f64) -> f64 {
    c[1] + t * (2.0 * c[2] + t * 3.0 * c[3])
}

fn eval_d2(c: &[f64; 4], t: f64) -> f64 {
    2.0 * c[2] + t * 6.0 * c[3]
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
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
        let goal = State {
            x: 8.0,
            y: 1.0,
            yaw: 0.1,
            speed: 6.0,
        };
        let steer = CubicSteer::from_states(&start, &goal, 1.2);

        let p0 = steer.point(0.0);
        let p1 = steer.point(1.2);
        assert!((p0[0] - start.x).abs() < 1e-9);
        assert!((p0[1] - start.y).abs() < 1e-9);
        assert!((p1[0] - goal.x).abs() < 1e-9);
        assert!((p1[1] - goal.y).abs() < 1e-9);
    }

    #[test]
    fn cubic_control_reports_acceleration_and_curvature() {
        let start = State {
            speed: 5.0,
            ..Default::default()
        };
        let goal = State {
            x: 8.0,
            y: 1.0,
            yaw: 0.1,
            speed: 6.0,
        };
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
        let goal = State {
            x: 8.0,
            y: 1.0,
            yaw: 0.1,
            speed: 6.0,
        };
        let steer = CubicSteer::from_states(&start, &goal, 1.2);
        let (controls, _) = steer_controls(start, &steer, 0.1, 1, 1.0);

        assert_eq!(controls[0], clamp_control(steer.control(0.05), start.speed));
    }
}
