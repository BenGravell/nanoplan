use super::Control;
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL,
    MIN_LON_ACCEL, ROLLING_RESISTANCE_ACCEL,
};

const LOW_SPEED_LIMIT_MPS: f64 = 0.001;

/// Signed passive resistance term from rolling resistance and air drag.
/// Positive while moving forward and negative while reversing, so subtracting
/// it from commanded acceleration always opposes the current motion.
pub(crate) fn longitudinal_resistance_accel(speed: f64) -> f64 {
    if speed.abs() <= LOW_SPEED_LIMIT_MPS {
        return 0.0;
    }

    speed.signum() * (ROLLING_RESISTANCE_ACCEL + AERO_DRAG_ACCEL_COEFFICIENT * speed * speed)
}

/// Net longitudinal acceleration experienced under current thrust accel acceleration and drag deceleration at current speed.
pub(super) fn net_longitudinal_accel(thrust_accel: f64, speed: f64) -> f64 {
    thrust_accel - longitudinal_resistance_accel(speed)
}

/// State curvature bound for a given speed.
pub(crate) fn curvature_limit(speed: f64) -> f64 {
    if speed.abs() <= LOW_SPEED_LIMIT_MPS {
        return MAX_ABS_CURVATURE;
    }

    MAX_ABS_CURVATURE.min(MAX_ABS_LAT_ACCEL / (speed * speed))
}

/// Clamp longitudinal thrust acceleration.
fn clamp_accel(accel: f64) -> f64 {
    accel.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL)
}

/// Clamp curvature.
fn clamp_curvature(curvature: f64, speed: f64) -> f64 {
    let limit = curvature_limit(speed);
    curvature.clamp(-limit, limit)
}

/// Clamp a control action.
pub(crate) fn clamp_control(u: Control, speed: f64) -> Control {
    Control {
        acceleration: clamp_accel(u.acceleration),
        curvature: clamp_curvature(u.curvature, speed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::MAX_TERMINAL_SPEED_MPS;

    #[test]
    fn rolling_and_air_drag_create_a_terminal_speed() {
        let terminal = *MAX_TERMINAL_SPEED_MPS;
        assert!(terminal.is_finite() && terminal > 50.0);

        assert_eq!(longitudinal_resistance_accel(LOW_SPEED_LIMIT_MPS), 0.0);
        assert_eq!(longitudinal_resistance_accel(-LOW_SPEED_LIMIT_MPS), 0.0);
        assert!(longitudinal_resistance_accel(terminal * 0.5) < MAX_LON_ACCEL);
        assert!(longitudinal_resistance_accel(terminal * 1.2) > MAX_LON_ACCEL);
        assert!(longitudinal_resistance_accel(-10.0) < 0.0);
    }

    #[test]
    fn limits_cap_curvature_and_lateral_accel() {
        assert_eq!(curvature_limit(LOW_SPEED_LIMIT_MPS), MAX_ABS_CURVATURE);
        assert_eq!(curvature_limit(-LOW_SPEED_LIMIT_MPS), MAX_ABS_CURVATURE);

        let slow = 3.0;
        assert!(MAX_ABS_LAT_ACCEL / (slow * slow) > MAX_ABS_CURVATURE);
        assert_eq!(
            clamp_control(
                Control {
                    acceleration: 0.0,
                    curvature: -100.0
                },
                slow
            )
            .curvature,
            -MAX_ABS_CURVATURE
        );

        let fast = 25.0;
        let kappa_lat = MAX_ABS_LAT_ACCEL / (fast * fast);
        assert!(
            kappa_lat < MAX_ABS_CURVATURE,
            "test speed too low to bind lat accel"
        );
        assert_eq!(
            clamp_control(
                Control {
                    acceleration: 0.0,
                    curvature: 1.0
                },
                fast
            )
            .curvature,
            kappa_lat
        );
    }
}
