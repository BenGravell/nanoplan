use super::{Control, State};
use crate::common::kinematics::{clamp_control, net_longitudinal_accel};

/// Speed reached after `ticks` maximum-acceleration integration steps.
pub(crate) fn speed_after_max_accel(mut speed: f64, ticks: usize, dt: f64) -> f64 {
    if dt <= 0.0 {
        return speed;
    }

    for _ in 0..ticks {
        speed += net_longitudinal_accel(crate::vehicle::MAX_LON_ACCEL, speed) * dt;
    }
    speed
}

/// Advance the high-fidelity world plant by one Euler step of length `dt`,
/// using an already-applied direct control. This includes passive
/// longitudinal resistance. The live world resolves collisions after all
/// dynamic bodies have advanced.
pub(crate) fn world_step(s: State, u: Control, dt: f64) -> State {
    let u = clamp_control(u, s.speed);
    let net_accel = net_longitudinal_accel(u.acceleration, s.speed);
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
    use crate::common::kinematics::{
        curvature_from_lateral_acceleration, lateral_acceleration, longitudinal_resistance_accel,
    };
    use crate::vehicle::{
        MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MAX_TERMINAL_SPEED_MPS, MIN_LON_ACCEL,
    };

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
    fn max_accel_baseline_uses_the_worlds_drag_limited_update() {
        let dt = 0.1;
        let initial = 20.0;
        let one_step = initial + net_longitudinal_accel(MAX_LON_ACCEL, initial) * dt;
        assert!((speed_after_max_accel(initial, 1, dt) - one_step).abs() < 1e-12);

        let mut expected = initial;
        for _ in 0..20 {
            expected += net_longitudinal_accel(MAX_LON_ACCEL, expected) * dt;
        }
        assert!((speed_after_max_accel(initial, 20, dt) - expected).abs() < 1e-12);
        assert!(expected < initial + MAX_LON_ACCEL * 2.0);
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
        let terminal = *MAX_TERMINAL_SPEED_MPS;
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
        let kappa_lat = curvature_from_lateral_acceleration(fast, MAX_ABS_LAT_ACCEL);
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
        assert!((lateral_acceleration(fast, applied_curvature) - MAX_ABS_LAT_ACCEL).abs() < 1e-9);
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
