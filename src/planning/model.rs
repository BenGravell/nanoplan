//! Planner-internal state/control model.
//!
//! This is deliberately only the bicycle kinematics plus vehicle limits that
//! are expressible from the current [`State`] and requested [`Control`].
//! Drag dynamics, actuator slew rate, and collisions live in
//! [`crate::simulation`]; planner rollouts only use the static terminal-speed
//! envelope exported by the world physics.

use crate::simulation::physics::MAX_TERMINAL_SPEED_MPS;
use crate::simulation::{Control, State};
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};

/// State curvature bound for a given speed: the tighter of steering geometry
/// and lateral acceleration.
pub fn curvature_limit(speed: f64) -> f64 {
    let v2 = speed * speed;
    if v2 > 1e-6 {
        MAX_ABS_CURVATURE.min(MAX_ABS_LAT_ACCEL / v2)
    } else {
        MAX_ABS_CURVATURE
    }
}

/// Clamp a planner action to state/control-space vehicle limits.
pub fn clamp_control(u: Control, speed: f64) -> Control {
    Control {
        acceleration: u.acceleration.clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
        curvature: u
            .curvature
            .clamp(-curvature_limit(speed), curvature_limit(speed)),
    }
}

/// Pure kinematic bicycle step for planner rollouts.
pub fn step(s: State, u: Control, dt: f64) -> State {
    let u = clamp_control(u, s.speed);
    let terminal = *MAX_TERMINAL_SPEED_MPS;
    let speed = (s.speed + u.acceleration * dt).clamp(-terminal, terminal);
    State {
        x: s.x + s.speed * s.yaw.cos() * dt,
        y: s.y + s.speed * s.yaw.sin() * dt,
        yaw: s.yaw + s.speed * u.curvature * dt,
        speed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planner_step_has_no_drag_or_actuator_memory() {
        let s = State {
            speed: 10.0,
            ..Default::default()
        };
        let u = Control {
            acceleration: 0.0,
            curvature: 0.0,
        };
        assert_eq!(
            step(s, u, 0.1),
            State {
                x: 1.0,
                speed: 10.0,
                ..Default::default()
            }
        );
    }

    #[test]
    fn planner_step_clamps_state_control_limits() {
        let s = State {
            speed: 25.0,
            ..Default::default()
        };
        let next = step(
            s,
            Control {
                acceleration: 100.0,
                curvature: 10.0,
            },
            0.1,
        );
        assert!((next.speed - (s.speed + MAX_LON_ACCEL * 0.1)).abs() < 1e-9);
        assert!((next.yaw - s.speed * curvature_limit(s.speed) * 0.1).abs() < 1e-9);
    }

    #[test]
    fn planner_step_allows_reverse_speed() {
        let next = step(
            State {
                speed: 1.0,
                ..Default::default()
            },
            Control {
                acceleration: MIN_LON_ACCEL,
                curvature: 0.0,
            },
            1.0,
        );
        assert!(next.speed < 0.0, "speed was {}", next.speed);
    }

    #[test]
    fn planner_step_clamps_speed_to_terminal_envelope() {
        let terminal = *MAX_TERMINAL_SPEED_MPS;
        let forward = step(
            State {
                speed: terminal - 1.0,
                ..Default::default()
            },
            Control {
                acceleration: MAX_LON_ACCEL,
                curvature: 0.0,
            },
            1.0,
        );
        assert_eq!(forward.speed, terminal);

        let reverse = step(
            State {
                speed: -terminal + 1.0,
                ..Default::default()
            },
            Control {
                acceleration: MIN_LON_ACCEL,
                curvature: 0.0,
            },
            1.0,
        );
        assert_eq!(reverse.speed, -terminal);
    }
}
