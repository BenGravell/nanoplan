//! Shared vehicle kinematics, resistance, and control limits.

use crate::common::types::{Control, State};
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL,
    MIN_LON_ACCEL, ROLLING_RESISTANCE_ACCEL,
};

pub(crate) const LOW_SPEED_LIMIT_MPS: f64 = 0.001;

/// Native state and control, plus time and derived kinematics for one trajectory.
///
/// All series are tick-aligned and have the same length. Construct this once
/// at a trajectory boundary and share it with consumers instead of recovering
/// controls from finite differences of states.
#[derive(Debug, Clone)]
pub(crate) struct TrajectoryKinematics {
    pub(crate) states: Vec<State>,
    pub(crate) controls: Vec<Control>,
    pub(crate) time: Vec<f64>,
    pub(crate) lateral_acceleration: Vec<f64>,
    pub(crate) dt: f64,
}

impl TrajectoryKinematics {
    pub(crate) fn new(states: Vec<State>, controls: Vec<Control>, dt: f64) -> Self {
        assert_eq!(states.len(), controls.len());
        let time = (0..states.len()).map(|i| i as f64 * dt).collect();
        let lateral_acceleration = states
            .iter()
            .zip(&controls)
            .map(|(state, control)| lateral_acceleration(state.speed, control.curvature))
            .collect();
        Self {
            states,
            controls,
            time,
            lateral_acceleration,
            dt,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.states.len()
    }
}

/// Signed lateral acceleration for a speed and path curvature.
pub(crate) fn lateral_acceleration(speed: f64, curvature: f64) -> f64 {
    speed.powi(2) * curvature
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

/// State curvature bound for a given speed.
pub(crate) fn curvature_limit(speed: f64) -> f64 {
    MAX_ABS_CURVATURE.min(curvature_from_lateral_acceleration(
        speed,
        MAX_ABS_LAT_ACCEL,
    ))
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
    fn curvature_and_lateral_acceleration_round_trip() {
        let speed = 20.0;
        let curvature = -0.025;
        let acceleration = lateral_acceleration(speed, curvature);
        assert_eq!(acceleration, -10.0);
        assert_eq!(
            curvature_from_lateral_acceleration(speed, acceleration),
            curvature
        );
    }

    #[test]
    fn low_speed_has_no_lateral_acceleration() {
        assert_eq!(lateral_acceleration(0.0, 0.2), 0.0);
        assert_eq!(curvature_from_lateral_acceleration(0.0, 0.0), 0.0);
        assert!(curvature_from_lateral_acceleration(0.0, 1.0).is_infinite());
        assert!(curvature_from_lateral_acceleration(LOW_SPEED_LIMIT_MPS, -1.0).is_infinite());
    }

    #[test]
    fn trajectory_kinematics_come_directly_from_aligned_controls() {
        let states = vec![
            State {
                speed: 10.0,
                ..Default::default()
            },
            State {
                speed: 20.0,
                ..Default::default()
            },
        ];
        let controls = vec![
            Control {
                acceleration: 2.0,
                curvature: -0.01,
            },
            Control {
                acceleration: -3.0,
                curvature: 0.02,
            },
        ];

        let trajectory = TrajectoryKinematics::new(states, controls, 0.1);

        assert_eq!(trajectory.time, [0.0, 0.1]);
        assert_eq!(
            trajectory.controls,
            [
                Control {
                    acceleration: 2.0,
                    curvature: -0.01,
                },
                Control {
                    acceleration: -3.0,
                    curvature: 0.02,
                },
            ]
        );
        assert_eq!(trajectory.lateral_acceleration, [-1.0, 8.0]);
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
