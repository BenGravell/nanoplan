//! Shared vehicle kinematics, resistance, and control limits.

use crate::common::measure::dot;
use crate::common::types::{Acceleration, Control, FrenetAcceleration, Speed, State};
use crate::simulation::world_step;
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL,
    ROLLING_RESISTANCE_ACCEL,
};

pub(crate) const LOW_SPEED_LIMIT_MPS: f64 = 0.001;

/// Where the vehicle ends up after coasting for `ticks` steps of `dt` seconds.
pub(crate) fn zero_action_point(mut x: State, ticks: usize, dt: f64) -> State {
    for _ in 0..ticks {
        x = world_step(x, Control::default(), dt);
    }
    x
}

/// Speed after traveling a given distance under constant acceleration from an initial speed.
pub(crate) fn speed_after_distance(speed: f64, acceleration: f64, distance: f64) -> f64 {
    (speed * speed + 2.0 * acceleration * distance).sqrt()
}

/// Signed lateral acceleration for a speed and path curvature.
pub(crate) fn lateral_acceleration(speed: f64, curvature: f64) -> f64 {
    speed.powi(2) * curvature
}

/// Recover net longitudinal acceleration from Cartesian flat-output derivatives.
/// Assumes velocity.mag() > 0.
pub(crate) fn longitudinal_acceleration_from_cartesian(velocity: Speed, acceleration: Acceleration) -> f64 {
    dot(velocity.xy(), acceleration.xy()) / velocity.mag()
}

/// Recover signed path curvature from Cartesian flat-output derivatives.
/// Assumes velocity.mag() > 0.
pub(crate) fn curvature_from_cartesian(velocity: Speed, acceleration: Acceleration) -> f64 {
    velocity.cross(*acceleration) / velocity.mag().powi(3)
}

/// Commanded accelerations at the current speed.
pub(crate) fn control_to_accelerations(control: Control, speed: f64) -> FrenetAcceleration {
    FrenetAcceleration {
        s: control.acceleration,
        d: lateral_acceleration(speed, control.curvature),
    }
}

/// Signed path curvature required for a lateral acceleration at `speed`.
/// At effectively zero speed, nonzero lateral acceleration requires infinite
/// curvature.
pub(crate) fn curvature_from_lateral_acceleration(speed: f64, lateral_acceleration: f64) -> f64 {
    if speed.abs() <= LOW_SPEED_LIMIT_MPS {
        return if lateral_acceleration == 0.0 {
            0.0
        } else {
            f64::INFINITY.copysign(lateral_acceleration)
        };
    }

    lateral_acceleration / speed.powi(2)
}

/// Signed passive resistance term from rolling resistance and air drag.
/// Positive while moving forward and negative while reversing, so subtracting
/// it from commanded acceleration always opposes the current motion.
pub(crate) fn longitudinal_resistance_accel(speed: f64) -> f64 {
    if speed.abs() <= LOW_SPEED_LIMIT_MPS {
        return 0.0;
    }

    speed.signum() * (ROLLING_RESISTANCE_ACCEL + AERO_DRAG_ACCEL_COEFFICIENT * speed * speed)
}

/// Net longitudinal acceleration under thrust and passive resistance.
pub(crate) fn net_longitudinal_accel(thrust_accel: f64, speed: f64) -> f64 {
    thrust_accel - longitudinal_resistance_accel(speed)
}

/// Requested thrust/braking acceleration needed to produce `net_accel`
/// after passive resistance is applied.
pub(crate) fn commanded_accel_for_net(net_accel: f64, speed: f64) -> f64 {
    net_accel + longitudinal_resistance_accel(speed)
}

/// Unclamped command needed to stop in one step of duration `dt`, including resistance.
pub(crate) fn commanded_accel_to_stop(speed: f64, dt: f64) -> f64 {
    commanded_accel_for_net(-speed / dt, speed)
}

/// State curvature bound for a given speed.
pub(crate) fn curvature_limit(speed: f64) -> f64 {
    MAX_ABS_CURVATURE.min(curvature_from_lateral_acceleration(speed, MAX_ABS_LAT_ACCEL))
}

fn clamp_accel(accel: f64) -> f64 {
    accel.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL)
}

fn clamp_curvature(curvature: f64, speed: f64) -> f64 {
    let limit = curvature_limit(speed);
    curvature.clamp(-limit, limit)
}

/// Clamp a control action to the vehicle's longitudinal and lateral limits.
pub(crate) fn clamp_control(control: Control, speed: f64) -> Control {
    Control {
        acceleration: clamp_accel(control.acceleration),
        curvature: clamp_curvature(control.curvature, speed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::MAX_TERMINAL_SPEED_MPS;

    #[test]
    fn zero_action_point_coasts_straight_and_slows() {
        let x = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(1.0, 2.0), 0.0),
            5.0,
        );
        for ticks in [0, 1, 3, 20] {
            for dt in [0.03, 0.1, 0.2] {
                let expected = (0..ticks).fold(x, |state, _| world_step(state, Control::default(), dt));
                assert_eq!(zero_action_point(x, ticks, dt), expected);
            }
        }
        let z = zero_action_point(x, 20, 0.1);
        assert!(z.position().x > x.position().x && z.position().x < x.position().x + x.speed * 2.0);
        assert!(z.speed < x.speed);
        assert_eq!((z.position().y, z.pose.yaw), (x.position().y, x.pose.yaw));
    }

    #[test]
    fn curvature_and_lateral_acceleration_round_trip() {
        let speed = 20.0;
        let curvature = -0.025;
        let acceleration = lateral_acceleration(speed, curvature);
        assert_eq!(acceleration, -10.0);
        assert_eq!(curvature_from_lateral_acceleration(speed, acceleration), curvature);
    }

    #[test]
    fn low_speed_has_no_lateral_acceleration() {
        assert_eq!(lateral_acceleration(0.0, 0.2), 0.0);
        assert_eq!(curvature_from_lateral_acceleration(0.0, 0.0), 0.0);
        assert!(curvature_from_lateral_acceleration(0.0, 1.0).is_infinite());
        assert!(curvature_from_lateral_acceleration(LOW_SPEED_LIMIT_MPS, -1.0).is_infinite());
    }

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
    fn commanded_and_net_longitudinal_acceleration_round_trip() {
        let speed = 30.0;
        for net_accel in [-5.0, 0.0, 4.0] {
            let commanded = commanded_accel_for_net(net_accel, speed);
            assert_eq!(net_longitudinal_accel(commanded, speed), net_accel);
        }
    }

    #[test]
    fn limits_cap_curvature_and_lateral_accel() {
        assert_eq!(curvature_limit(LOW_SPEED_LIMIT_MPS), MAX_ABS_CURVATURE);
        assert_eq!(curvature_limit(-LOW_SPEED_LIMIT_MPS), MAX_ABS_CURVATURE);

        let slow = 3.0;
        assert!(curvature_from_lateral_acceleration(slow, MAX_ABS_LAT_ACCEL) > MAX_ABS_CURVATURE);
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
        let kappa_lat = curvature_from_lateral_acceleration(fast, MAX_ABS_LAT_ACCEL);
        assert!(kappa_lat < MAX_ABS_CURVATURE, "test speed too low to bind lat accel");
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
