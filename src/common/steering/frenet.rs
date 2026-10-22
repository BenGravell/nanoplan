//! Cubic steering between Frenet position and velocity boundaries.

use super::{Motion, control_from_derivatives};
use crate::common::finite::check_finite;
use crate::common::polynomial::CubicPolynomial;
use crate::common::types::{Acceleration, AxisMotion, Control, FrenetPosition, FrenetSpeed, Position, Speed, State};
use crate::track::Path;

pub(crate) const MIN_FRENET_SCALE: f64 = 0.1;
const STATION_SPEED_TOLERANCE_MPS: f64 = 1e-8;

pub(crate) struct FrenetMotion {
    pub(crate) s: CubicPolynomial,
    pub(crate) d: CubicPolynomial,
}

impl FrenetMotion {
    /// Fit position and velocity boundaries in the reference path's Frenet chart.
    pub(crate) fn new(
        (src_pos, src_vel): (FrenetPosition, FrenetSpeed),
        (tgt_pos, tgt_vel): (FrenetPosition, FrenetSpeed),
        duration: f64,
    ) -> Self {
        Self {
            s: CubicPolynomial::from_boundary(src_pos.s, src_vel.s, tgt_pos.s, tgt_vel.s, duration),
            d: CubicPolynomial::from_boundary(src_pos.d, src_vel.d, tgt_pos.d, tgt_vel.d, duration),
        }
    }

    /// Transform Frenet position and derivatives into Cartesian state and net controls.
    pub(crate) fn at(&self, path: &Path, t: f64) -> Option<(State, Control)> {
        let (s, d) = self.derivatives_at(t);
        cartesian_kinematics(path, s, d)
    }
}

impl Motion for FrenetMotion {
    fn derivatives_at(&self, t: f64) -> (AxisMotion, AxisMotion) {
        (self.s.at(t).into(), self.d.at(t).into())
    }
}

/// Transform Frenet coordinates and derivatives with respect to any common parameter.
/// For a spatial parameter, the returned control's curvature is geometric curvature.
pub(crate) fn cartesian_kinematics(path: &Path, s: AxisMotion, d: AxisMotion) -> Option<(State, Control)> {
    if !(0.0..=path.length()).contains(&s.position) || s.velocity < -STATION_SPEED_TOLERANCE_MPS {
        return None;
    }
    let k = path.curvature_at(s.position);
    let dk = path.sharpness_at(s.position);
    let scale = 1.0 - k * d.position;
    if scale <= MIN_FRENET_SCALE {
        return None;
    }
    let vx = scale * s.velocity;
    let ax = scale * s.acceleration - dk * d.position * s.velocity * s.velocity - 2.0 * k * s.velocity * d.velocity;
    let ay = d.acceleration + k * scale * s.velocity * s.velocity;
    let velocity = Speed::new(vx, d.velocity);
    let acceleration = Acceleration::new(ax, ay);
    let speed = velocity.norm();
    let control = control_from_derivatives(velocity, acceleration).unwrap_or(Control {
        acceleration: ax,
        curvature: 0.0,
    });
    let heading = path.heading_at(s.position);
    let left = Position::from_angle(heading + std::f64::consts::FRAC_PI_2);
    let state = State::from((
        path.pose_at(s.position).0 + left * d.position,
        heading + velocity.angle(),
        speed,
    ));
    check_finite((state, control))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::geometry::wrap_angle;

    #[test]
    fn stationary_samples_keep_the_reference_heading_and_tangential_acceleration() {
        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(0.0, 100.0)]);
        for speed in [0.0, 1e-6] {
            let (state, control) =
                cartesian_kinematics(&path, [10.0, speed, 2.0].into(), [1.0, 0.0, 3.0].into()).unwrap();
            assert!((state.pose.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
            assert_eq!(state.speed, speed);
            assert_eq!(control.acceleration, 2.0);
            assert_eq!(control.curvature, 0.0);
        }
    }

    #[test]
    fn straight_reference_matches_cartesian_steering() {
        use crate::common::steering::cartesian::{CartesianMotion, CubicCartesianMotion};

        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(200.0, 200.0)]);
        let src = (FrenetPosition { s: 20.0, d: 2.0 }, FrenetSpeed { s: 8.0, d: 0.3 });
        let tgt = (FrenetPosition { s: 60.0, d: -1.0 }, FrenetSpeed { s: 9.0, d: -0.2 });
        let boundary = |(position, speed): (FrenetPosition, FrenetSpeed)| {
            cartesian_kinematics(
                &path,
                [position.s, speed.s, 0.0].into(),
                [position.d, speed.d, 0.0].into(),
            )
            .unwrap()
            .0
        };
        let frenet = FrenetMotion::new(src, tgt, 5.0);
        let cartesian = CubicCartesianMotion::new(boundary(src).into(), boundary(tgt).into(), 5.0);
        for t in [0.0, 1.25, 2.5, 3.75, 5.0] {
            let (f, fc) = frenet.at(&path, t).unwrap();
            let (c, cc) = cartesian.at(t).unwrap();
            assert!(f.position().distance(c.position()) < 1e-10);
            assert!((f.speed - c.speed).abs() < 1e-10);
            assert!(wrap_angle(f.pose.yaw - c.pose.yaw).abs() < 1e-10);
            assert!((fc.acceleration - cc.acceleration).abs() < 1e-10);
            assert!((fc.curvature - cc.curvature).abs() < 1e-10);
        }
    }

    #[test]
    fn frenet_cubics_match_boundaries_and_follow_a_curved_reference() {
        for end_velocity in [-0.5, 0.0, 0.5] {
            let polynomial = CubicPolynomial::from_boundary(2.0, 0.3, -1.0, end_velocity, 3.0);
            assert_eq!(polynomial.at(0.0)[..2], [2.0, 0.3]);
            let tgt = polynomial.at(3.0);
            assert!((tgt[0] + 1.0).abs() < 1e-10);
            assert!((tgt[1] - end_velocity).abs() < 1e-10);
        }
        let radius = 40.0;
        let points: Vec<_> = (0..1800)
            .map(|i| {
                let angle = i as f64 * 0.002;
                Position::new(radius * angle.cos(), radius * angle.sin())
            })
            .collect();
        let path = Path::new(&points);
        let motion = FrenetMotion {
            s: CubicPolynomial([10.0, 8.0, 0.0, 0.0]),
            d: CubicPolynomial([2.0, 0.0, 0.0, 0.0]),
        };
        for t in [0.0, 2.0, 5.0, 10.0] {
            let (state, control) = motion.at(&path, t).unwrap();
            // A constant Frenet offset traces the concentric circle, not a Cartesian chord.
            assert!((state.position().x.hypot(state.position().y) - 38.0).abs() < 1e-4);
            assert!((state.speed - 7.6).abs() < 1e-8);
            assert!((control.curvature - 1.0 / 38.0).abs() < 1e-8);
            assert!(control.acceleration.abs() < 1e-8);
            let tangent = state.position().y.atan2(state.position().x) + std::f64::consts::FRAC_PI_2;
            assert!(wrap_angle(state.pose.yaw - tangent).abs() < 1e-8);
        }
        // Frenet chart singularities and backwards station motion are not valid trajectories.
        let singular = FrenetMotion {
            d: CubicPolynomial([41.0, 0.0, 0.0, 0.0]),
            ..motion
        };
        assert!(singular.at(&path, 1.0).is_none());
        let reverse = FrenetMotion {
            s: CubicPolynomial([10.0, -1.0, 0.0, 0.0]),
            ..singular
        };
        assert!(reverse.at(&path, 1.0).is_none());
    }

    #[test]
    fn frenet_transform_accounts_for_changing_reference_curvature() {
        let points: Vec<_> = (0..3000)
            .map(|i| {
                let x = i as f64 * 0.05;
                Position::new(x, 0.01 * x * x)
            })
            .collect();
        let path = Path::new(&points);
        let motion = FrenetMotion {
            s: CubicPolynomial([10.0, 8.0, 0.15, 0.0]),
            d: CubicPolynomial([2.0, 0.3, 0.1, 0.0]),
        };
        let t = 1.234;
        let epsilon = 1e-5;
        let (before, _) = motion.at(&path, t - epsilon).unwrap();
        let (state, control) = motion.at(&path, t).unwrap();
        let (after, _) = motion.at(&path, t + epsilon).unwrap();
        assert!(path.sharpness_at(motion.s.at(t)[0]).abs() > 1e-5);
        let acceleration = (after.speed - before.speed) / (2.0 * epsilon);
        let curvature = wrap_angle(after.pose.yaw - before.pose.yaw) / (2.0 * epsilon * state.speed);
        assert!((control.acceleration - acceleration).abs() < 1e-6);
        assert!((control.curvature - curvature).abs() < 1e-5);
    }
}
