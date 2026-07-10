use std::sync::LazyLock;

use super::Control;
use crate::vehicle::{
    AIR_DENSITY_KG_M3, DRAG_AREA_M2, EGO_MASS_KG, GRAVITY_MS2, MAX_ABS_CURVATURE,
    MAX_ABS_CURVATURE_RATE, MAX_ABS_LAT_ACCEL, MAX_ABS_LON_JERK, MAX_LON_ACCEL, MIN_LON_ACCEL,
    ROLLING_RESISTANCE_COEFF,
};

/// Signed passive resistance term from rolling resistance and air drag.
/// Positive while moving forward and negative while reversing, so subtracting
/// it from commanded acceleration always opposes the current motion.
pub fn longitudinal_resistance_accel(speed: f64) -> f64 {
    if speed == 0.0 {
        0.0
    } else {
        speed.signum()
            * (ROLLING_RESISTANCE_COEFF * GRAVITY_MS2
                + 0.5 * AIR_DENSITY_KG_M3 * DRAG_AREA_M2 * speed * speed / EGO_MASS_KG)
    }
}

/// Terminal speed for a constant positive requested acceleration under the
/// simple rolling/aero resistance model.
pub fn terminal_speed_for_accel(accel: f64) -> Option<f64> {
    let rolling = ROLLING_RESISTANCE_COEFF * GRAVITY_MS2;
    let aero = 0.5 * AIR_DENSITY_KG_M3 * DRAG_AREA_M2 / EGO_MASS_KG;
    if accel > rolling {
        Some(((accel - rolling) / aero).sqrt())
    } else {
        None
    }
}

/// Static speed envelope used by pure planner rollouts.
pub static MAX_TERMINAL_SPEED_MPS: LazyLock<f64> =
    LazyLock::new(|| terminal_speed_for_accel(MAX_LON_ACCEL).unwrap());

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

/// Clamp an action to acceleration, steering, and lateral-grip limits.
pub fn clamp_control(u: Control, speed: f64) -> Control {
    Control {
        acceleration: u.acceleration.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
        curvature: clamp_curvature(u.curvature, speed),
    }
}

pub(super) fn rate_limit_control(
    prev: Control,
    requested: Control,
    dt: f64,
    speed: f64,
) -> Control {
    let prev = clamp_control(prev, speed);
    let target = clamp_control(requested, speed);
    Control {
        acceleration: target.acceleration.clamp(
            prev.acceleration - MAX_ABS_LON_JERK * dt,
            prev.acceleration + MAX_ABS_LON_JERK * dt,
        ),
        curvature: target.curvature.clamp(
            prev.curvature - MAX_ABS_CURVATURE_RATE * dt,
            prev.curvature + MAX_ABS_CURVATURE_RATE * dt,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rolling_and_air_drag_create_a_terminal_speed() {
        let terminal = terminal_speed_for_accel(MAX_LON_ACCEL).unwrap();
        assert_eq!(terminal, *MAX_TERMINAL_SPEED_MPS);
        assert!(terminal.is_finite() && terminal > 50.0);

        assert!(longitudinal_resistance_accel(terminal * 0.5) < MAX_LON_ACCEL);
        assert!(longitudinal_resistance_accel(terminal * 1.2) > MAX_LON_ACCEL);
        assert!(longitudinal_resistance_accel(-10.0) < 0.0);
    }

    #[test]
    fn limits_cap_curvature_and_lateral_accel() {
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
