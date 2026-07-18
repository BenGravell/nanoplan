use super::physics::{clamp_control, longitudinal_resistance_accel};
use super::{Control, State};

/// Advance the high-fidelity world plant by one Euler step of length `dt`,
/// using an already-applied direct control. This includes passive
/// longitudinal resistance. [`crate::simulation::Simulator`] owns collision
/// response around it.
pub(crate) fn world_step(s: State, u: Control, dt: f64) -> State {
    let u = clamp_control(u, s.speed);
    let net_accel = u.acceleration - longitudinal_resistance_accel(s.speed);
    State {
        x: s.x + s.speed * s.yaw.cos() * dt,
        y: s.y + s.speed * s.yaw.sin() * dt,
        yaw: s.yaw + s.speed * u.curvature * dt,
        speed: s.speed + net_accel * dt,
    }
}

/// Stores the statically limited control currently applied by the simulator.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CommandLimiter {
    pub(crate) applied: Control,
}

impl CommandLimiter {
    pub(crate) fn new() -> Self {
        CommandLimiter {
            applied: Control::default(),
        }
    }

    pub(crate) fn step(&mut self, state: State, command: Control, dt: f64) -> State {
        self.applied = clamp_control(command, state.speed);
        world_step(state, self.applied, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::physics::terminal_speed_for_accel;
    use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};

    #[test]
    fn world_step_applies_static_limits() {
        let s = State {
            speed: 3.0,
            ..Default::default()
        };
        let ns = world_step(
            s,
            Control {
                acceleration: 100.0,
                curvature: 100.0,
            },
            0.1,
        );
        assert!(
            (ns.speed - (s.speed + (MAX_LON_ACCEL - longitudinal_resistance_accel(s.speed)) * 0.1))
                .abs()
                < 1e-9
        );
        assert!((ns.yaw - s.speed * MAX_ABS_CURVATURE * 0.1).abs() < 1e-9);
    }

    #[test]
    fn coasts_straight_with_resistance() {
        let s0 = State {
            speed: 1.0,
            ..Default::default()
        };
        let s1 = world_step(s0, Control::default(), 0.1);
        assert_eq!(
            s1,
            State {
                x: 0.1,
                speed: 1.0 - longitudinal_resistance_accel(1.0) * 0.1,
                ..Default::default()
            }
        );
    }

    #[test]
    fn turns_left_with_positive_curvature() {
        let s0 = State {
            speed: 1.0,
            ..Default::default()
        };
        let u = Control {
            acceleration: 0.0,
            curvature: 1.0,
        };
        let s1 = world_step(s0, u, 0.1);
        assert!(s1.yaw > 0.0);
    }

    #[test]
    fn limits_clamp_accel() {
        let s = world_step(
            State {
                speed: 5.0,
                ..Default::default()
            },
            Control {
                acceleration: 100.0,
                curvature: 0.0,
            },
            0.1,
        );
        assert!(
            (s.speed - (5.0 + (MAX_LON_ACCEL - longitudinal_resistance_accel(5.0)) * 0.1)).abs()
                < 1e-9,
            "speed {}",
            s.speed
        );
        let brake = world_step(
            State {
                speed: 5.0,
                ..Default::default()
            },
            Control {
                acceleration: -100.0,
                curvature: 0.0,
            },
            0.1,
        );
        assert!(
            (brake.speed - (5.0 + (MIN_LON_ACCEL - longitudinal_resistance_accel(5.0)) * 0.1))
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn terminal_speed_controls_acceleration_sign() {
        let terminal = terminal_speed_for_accel(MAX_LON_ACCEL).unwrap();
        let below = world_step(
            State {
                speed: terminal * 0.5,
                ..Default::default()
            },
            Control {
                acceleration: MAX_LON_ACCEL,
                curvature: 0.0,
            },
            0.1,
        );
        assert!(below.speed > terminal * 0.5);

        let above = world_step(
            State {
                speed: terminal * 1.2,
                ..Default::default()
            },
            Control {
                acceleration: MAX_LON_ACCEL,
                curvature: 0.0,
            },
            0.1,
        );
        assert!(above.speed < terminal * 1.2);
    }

    #[test]
    fn world_step_allows_reverse_speed_and_resists_it() {
        let reversed = world_step(
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
        assert!(reversed.speed < 0.0, "speed was {}", reversed.speed);

        let coasting = world_step(reversed, Control::default(), 0.1);
        assert!(
            coasting.speed > reversed.speed,
            "{} did not resist reverse motion from {}",
            coasting.speed,
            reversed.speed
        );
    }

    #[test]
    fn limits_cap_curvature_and_lateral_accel_integration() {
        let slow = 3.0;
        let s = world_step(
            State {
                speed: slow,
                ..Default::default()
            },
            Control {
                acceleration: 0.0,
                curvature: -100.0,
            },
            0.1,
        );
        let expected_yaw = -slow * MAX_ABS_CURVATURE * 0.1;
        assert!((s.yaw - expected_yaw).abs() < 1e-9, "yaw {}", s.yaw);

        let fast = 25.0;
        let kappa_lat = MAX_ABS_LAT_ACCEL / (fast * fast);
        let s = world_step(
            State {
                speed: fast,
                ..Default::default()
            },
            Control {
                acceleration: 0.0,
                curvature: 1.0,
            },
            0.1,
        );
        let applied_curvature = s.yaw / (fast * 0.1);
        assert!(
            (applied_curvature - kappa_lat).abs() < 1e-9,
            "curv {}",
            applied_curvature
        );
        assert!((applied_curvature * fast * fast - MAX_ABS_LAT_ACCEL).abs() < 1e-9);
    }

    #[test]
    fn simulator_applies_commands_without_rate_limits() {
        let mut limiter = CommandLimiter::new();
        let s0 = State {
            speed: 1.0,
            ..Default::default()
        };
        let dt = 0.01;
        let s1 = limiter.step(
            s0,
            Control {
                acceleration: 100.0,
                curvature: 5.0,
            },
            dt,
        );
        assert_eq!(limiter.applied.acceleration, MAX_LON_ACCEL);
        assert_eq!(limiter.applied.curvature, MAX_ABS_CURVATURE);
        assert!(
            (s1.speed
                - (1.0 + (limiter.applied.acceleration - longitudinal_resistance_accel(1.0)) * dt))
                .abs()
                < 1e-9
        );
    }
}
