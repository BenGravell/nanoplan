//! Cartesian motion sampling and cubic steering between position and velocity boundaries.

use super::{Motion, control_from_derivatives};
use crate::common::finite::check_finite;
use crate::common::polynomial::CubicPolynomial;
use crate::common::types::{Acceleration, AxisMotion, Control, PosVelState, Position, Speed, State};

/// Motion providing Cartesian position, velocity, and acceleration in each axis.
pub(crate) trait CartesianMotion: Motion {
    fn at(&self, t: f64) -> Option<(State, Control)> {
        let (x, y) = self.derivatives_at(t);

        let position = Position::new(x.position, y.position);
        let velocity = Speed::new(x.velocity, y.velocity);
        let acceleration = Acceleration::new(x.acceleration, y.acceleration);

        // Heading and curvature are undefined at a stationary interior point.
        let control = control_from_derivatives(velocity, acceleration)?;
        let state = State::from(PosVelState {
            position,
            speed: velocity,
        });

        check_finite((state, control))
    }
}

pub(crate) struct CubicCartesianMotion {
    x: CubicPolynomial,
    y: CubicPolynomial,
}

impl CubicCartesianMotion {
    pub(crate) fn new(src: PosVelState, tgt: PosVelState, duration: f64) -> Self {
        Self {
            x: CubicPolynomial::from_boundary(src.position.x, src.speed.x, tgt.position.x, tgt.speed.x, duration),
            y: CubicPolynomial::from_boundary(src.position.y, src.speed.y, tgt.position.y, tgt.speed.y, duration),
        }
    }
}

impl Motion for CubicCartesianMotion {
    fn derivatives_at(&self, t: f64) -> (AxisMotion, AxisMotion) {
        (self.x.at(t).into(), self.y.at(t).into())
    }
}

impl CartesianMotion for CubicCartesianMotion {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn general_motion_converts_derivatives_and_rejects_stationary_samples() {
        struct ParabolicMotion;
        impl Motion for ParabolicMotion {
            fn derivatives_at(&self, t: f64) -> (AxisMotion, AxisMotion) {
                (
                    AxisMotion {
                        position: t * t,
                        velocity: 2.0 * t,
                        acceleration: 2.0,
                    },
                    AxisMotion {
                        position: 0.0,
                        velocity: 0.0,
                        acceleration: 0.0,
                    },
                )
            }
        }
        impl CartesianMotion for ParabolicMotion {}

        let (state, control) = ParabolicMotion.at(2.0).unwrap();
        assert_eq!(state.position(), Position::new(4.0, 0.0));
        assert_eq!(state.speed, 4.0);
        assert_eq!(control.acceleration, 2.0);
        assert_eq!(control.curvature, 0.0);
        assert!(ParabolicMotion.at(0.0).is_none());
        assert!(ParabolicMotion.at(0.5e-6).is_none());
        assert!(ParabolicMotion.at(f64::NAN).is_none());
    }

    #[test]
    fn matches_physical_position_and_velocity_boundaries() {
        let src = State::from((Position::new(12.0, -3.0), 0.4, 8.0));
        let tgt = State::from((Position::new(40.0, 15.0), 0.8, 6.0));
        let motion = CubicCartesianMotion::new(src.into(), tgt.into(), 4.0);
        for (t, expected) in [(0.0, src), (4.0, tgt)] {
            let (actual, _) = motion.at(t).unwrap();
            assert!(actual.position().distance(expected.position()) < 1e-10);
            assert!((actual.speed - expected.speed).abs() < 1e-10);
            assert!((actual.pose.yaw - expected.pose.yaw).abs() < 1e-10);
        }
    }
}
