//! Shared flat-output steering primitives for sampling-tree planners.
//!
//! Both tree planners connect sampled states by choosing `x(t)` and `y(t)`
//! polynomials, then reading heading, acceleration, and curvature back from
//! their derivatives. This module owns that math once. RRT* uses pose
//! boundaries over a unit interval; treetop RRT uses full [`State`] boundary
//! conditions over physical time.

use crate::simulation::State;

/// Quintic flat-output connector between two states/poses.
///
/// The polynomial in each coordinate matches position, velocity, and
/// acceleration at both endpoints:
///
/// `p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5`
pub(crate) struct QuinticSteer {
    cx: [f64; 6],
    cy: [f64; 6],
    duration: f64,
}

impl QuinticSteer {
    /// Fit a time-parametrized connector between full vehicle states.
    pub(crate) fn from_states(start: &State, goal: &State, duration: f64) -> Self {
        let duration = duration.max(1e-6);
        let (v0, a0) = state_derivatives(start);
        let (v1, a1) = state_derivatives(goal);
        Self {
            cx: quintic_coeffs(start.x, v0[0], a0[0], goal.x, v1[0], a1[0], duration),
            cy: quintic_coeffs(start.y, v0[1], a0[1], goal.y, v1[1], a1[1], duration),
            duration,
        }
    }

    /// Fit a unit-interval connector between oriented positions. The
    /// derivative magnitude is tied to chord length, while endpoint curvature
    /// supplies the second derivative normal to each heading.
    pub(crate) fn from_poses(
        p0: [f64; 2],
        yaw0: f64,
        curvature0: f64,
        p1: [f64; 2],
        yaw1: f64,
        curvature1: f64,
    ) -> Self {
        let k = dist(p0, p1).max(1e-3) / 2.0;
        let boundary = |yaw: f64, curvature: f64| {
            let tangent = [yaw.cos(), yaw.sin()];
            let normal = [-yaw.sin(), yaw.cos()];
            (
                [k * tangent[0], k * tangent[1]],
                [k * k * curvature * normal[0], k * k * curvature * normal[1]],
            )
        };
        let (v0, a0) = boundary(yaw0, curvature0);
        let (v1, a1) = boundary(yaw1, curvature1);
        Self {
            cx: quintic_coeffs(p0[0], v0[0], a0[0], p1[0], v1[0], a1[0], 1.0),
            cy: quintic_coeffs(p0[1], v0[1], a0[1], p1[1], v1[1], a1[1], 1.0),
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

    /// Flat-output kinematics `(speed, tangential_accel, curvature)`.
    pub(crate) fn kinematics(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.duration);
        let (dx, dy) = (eval_d1(&self.cx, t), eval_d1(&self.cy, t));
        let (ddx, ddy) = (eval_d2(&self.cx, t), eval_d2(&self.cy, t));
        let speed = dx.hypot(dy);
        if speed <= 0.01 {
            return (speed, ddx.hypot(ddy), 0.0);
        }
        (
            speed,
            (dx * ddx + dy * ddy) / speed,
            (dx * ddy - dy * ddx) / speed.powi(3),
        )
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

fn state_derivatives(x: &State) -> ([f64; 2], [f64; 2]) {
    let tangent = [x.yaw.cos(), x.yaw.sin()];
    let normal = [-x.yaw.sin(), x.yaw.cos()];
    (
        [x.speed * tangent[0], x.speed * tangent[1]],
        [
            x.accel * tangent[0] + x.speed * x.speed * x.curvature * normal[0],
            x.accel * tangent[1] + x.speed * x.speed * x.curvature * normal[1],
        ],
    )
}

fn quintic_coeffs(p0: f64, v0: f64, a0: f64, p1: f64, v1: f64, a1: f64, t: f64) -> [f64; 6] {
    let (t2, t3, t4, t5) = (t * t, t * t * t, t * t * t * t, t * t * t * t * t);
    [
        p0,
        v0,
        0.5 * a0,
        (20.0 * (p1 - p0) - (8.0 * v1 + 12.0 * v0) * t - (3.0 * a0 - a1) * t2) / (2.0 * t3),
        (30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * t + (3.0 * a0 - 2.0 * a1) * t2) / (2.0 * t4),
        (12.0 * (p1 - p0) - (6.0 * v1 + 6.0 * v0) * t - (a0 - a1) * t2) / (2.0 * t5),
    ]
}

fn eval(c: &[f64; 6], t: f64) -> f64 {
    c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))
}

fn eval_d1(c: &[f64; 6], t: f64) -> f64 {
    c[1] + t * (2.0 * c[2] + t * (3.0 * c[3] + t * (4.0 * c[4] + t * 5.0 * c[5])))
}

fn eval_d2(c: &[f64; 6], t: f64) -> f64 {
    2.0 * c[2] + t * (6.0 * c[3] + t * (12.0 * c[4] + t * 20.0 * c[5]))
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quintic_matches_state_endpoints() {
        let start = State {
            speed: 5.0,
            accel: 0.4,
            curvature: 0.02,
            ..Default::default()
        };
        let goal = State {
            x: 8.0,
            y: 1.0,
            yaw: 0.1,
            speed: 6.0,
            accel: -0.2,
            curvature: 0.01,
        };
        let steer = QuinticSteer::from_states(&start, &goal, 1.2);

        let p0 = steer.point(0.0);
        let p1 = steer.point(1.2);
        assert!((p0[0] - start.x).abs() < 1e-9);
        assert!((p0[1] - start.y).abs() < 1e-9);
        assert!((p1[0] - goal.x).abs() < 1e-9);
        assert!((p1[1] - goal.y).abs() < 1e-9);
        assert!((steer.kinematics(0.0).2 - start.curvature).abs() < 1e-9);
        assert!((steer.kinematics(1.2).2 - goal.curvature).abs() < 1e-9);
    }
}
