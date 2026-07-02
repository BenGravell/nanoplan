//! Ultra minimalist motion planner for car-like vehicles.
//!
//! Planned architecture: trajectory trees expanded by sampling-based DDP
//! over a kinematic bicycle model. Zero dependencies.

/// State of a car-like vehicle: position, heading, and speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub speed: f64,
}

/// Control input: longitudinal acceleration and front steering angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Control {
    pub accel: f64,
    pub steer: f64,
}

/// Advance the kinematic bicycle model by one Euler step of length `dt`.
///
/// `wheelbase` is the distance between front and rear axles.
pub fn step(state: State, control: Control, wheelbase: f64, dt: f64) -> State {
    State {
        x: state.x + state.speed * state.heading.cos() * dt,
        y: state.y + state.speed * state.heading.sin() * dt,
        heading: state.heading + state.speed * control.steer.tan() / wheelbase * dt,
        speed: state.speed + control.accel * dt,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drives_straight() {
        let s0 = State {
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            speed: 1.0,
        };
        let u = Control {
            accel: 0.0,
            steer: 0.0,
        };
        let s1 = step(s0, u, 2.5, 0.1);
        assert_eq!(
            s1,
            State {
                x: 0.1,
                y: 0.0,
                heading: 0.0,
                speed: 1.0
            }
        );
    }

    #[test]
    fn turns_left_when_steering_left() {
        let s0 = State {
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            speed: 1.0,
        };
        let u = Control {
            accel: 0.0,
            steer: 0.3,
        };
        let s1 = step(s0, u, 2.5, 0.1);
        assert!(s1.heading > 0.0);
    }

    // ponytail: smoke test that bevy links and boots headless; delete once a real app exists
    #[test]
    fn bevy_app_boots() {
        use bevy::prelude::*;
        App::new().add_plugins(MinimalPlugins).update();
    }
}
