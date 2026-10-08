//! Vehicle acceleration and curvature limits.

use super::{Constraint, Sample};
use crate::common::kinematics::curvature_limit;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

pub(crate) struct Kinodynamic;

impl Constraint for Kinodynamic {
    fn is_violated(&self, sample: &Sample) -> bool {
        self.violation_depth(sample) > 0.0
    }

    fn violation_depth(&self, sample: &Sample) -> f64 {
        let Some((control, speed)) = sample.control else {
            return 0.0;
        };

        if !speed.is_finite() || !control.acceleration.is_finite() || !control.curvature.is_finite() {
            return 1.0;
        }

        scaled_braking_violation(control.acceleration)
            + scaled_acceleration_violation(control.acceleration)
            + scaled_curvature_violation(control.curvature, speed)
    }
}

fn scaled_braking_violation(acceleration: f64) -> f64 {
    (MIN_LON_ACCEL - acceleration).max(0.0) / -MIN_LON_ACCEL
}

fn scaled_acceleration_violation(acceleration: f64) -> f64 {
    (acceleration - MAX_LON_ACCEL).max(0.0) / MAX_LON_ACCEL
}

fn scaled_curvature_violation(curvature: f64, speed: f64) -> f64 {
    let limit = curvature_limit(speed);
    (curvature.abs() - limit).max(0.0) / limit.max(f64::MIN_POSITIVE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::Control;

    #[test]
    fn equal_fractional_excess_has_equal_depth() {
        for speed in [0.0, 5.0, 30.0, -30.0] {
            let limit = curvature_limit(speed);
            for excess in [0.0, 0.1, 1.0, 2.0] {
                for control in [
                    Control::from([MIN_LON_ACCEL * (1.0 + excess), 0.0]),
                    Control::from([MAX_LON_ACCEL * (1.0 + excess), 0.0]),
                    Control::from([0.0, limit * (1.0 + excess)]),
                    Control::from([0.0, -limit * (1.0 + excess)]),
                ] {
                    let sample = Sample::default().with_control(control, speed);
                    assert!((Kinodynamic.violation_depth(&sample) - excess).abs() < 1e-12);
                }
                let sample = Sample::default().with_control(
                    Control::from([MAX_LON_ACCEL * (1.0 + excess), limit * (1.0 + excess)]),
                    speed,
                );
                assert!((Kinodynamic.violation_depth(&sample) - 2.0 * excess).abs() < 1e-12);
            }
        }
    }
}
