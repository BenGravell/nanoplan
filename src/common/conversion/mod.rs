//! Standard conversions between domain types and numeric representations.

use crate::common::types::algebra::vector::{V2, V4};
use crate::common::types::{AxisMotion, Control, PosVelState, Pose, Position, Speed, State};

impl From<[f64; 3]> for AxisMotion {
    fn from([position, velocity, acceleration]: [f64; 3]) -> Self {
        Self {
            position,
            velocity,
            acceleration,
        }
    }
}

impl From<State> for PosVelState {
    fn from(state: State) -> Self {
        let (sin, cos) = state.pose.yaw.sin_cos();
        Self {
            position: state.position(),
            speed: Speed::new(cos * state.speed, sin * state.speed),
        }
    }
}

impl From<PosVelState> for State {
    /// Uses velocity direction as heading and a nonnegative speed.
    /// At rest, heading defaults to zero. Original heading and speed sign
    /// cannot be recovered from a velocity vector alone.
    fn from(state: PosVelState) -> Self {
        let speed = state.speed.norm();
        let yaw = if speed == 0.0 { 0.0 } else { state.speed.angle() };
        State::from((state.position, yaw, speed))
    }
}

impl From<V4> for State {
    fn from(v: V4) -> Self {
        State {
            pose: Pose::new(Position::new(v[0], v[1]), v[2]),
            speed: v[3],
        }
    }
}

impl From<Position> for State {
    fn from(p: Position) -> Self {
        (p, 0.0, 0.0).into()
    }
}

impl From<(Position, f64, f64)> for State {
    fn from((position, yaw, speed): (Position, f64, f64)) -> Self {
        State::new(Pose::new(position, yaw), speed)
    }
}

impl From<Pose> for State {
    fn from(p: Pose) -> Self {
        (Position::from(p), p.yaw, 0.0).into()
    }
}

impl From<Position> for Pose {
    fn from(position: Position) -> Self {
        Pose::new(position, 0.0)
    }
}

impl From<State> for Pose {
    fn from(s: State) -> Self {
        s.pose
    }
}

impl From<&State> for Pose {
    fn from(s: &State) -> Self {
        (*s).into()
    }
}

impl From<V2> for Position {
    fn from(p: V2) -> Self {
        Position::new(p[0], p[1])
    }
}

impl From<Position> for V2 {
    fn from(p: Position) -> Self {
        p.xy()
    }
}

impl From<State> for Position {
    fn from(s: State) -> Self {
        s.pose.position
    }
}

impl From<&State> for Position {
    fn from(s: &State) -> Self {
        (*s).into()
    }
}

impl From<Pose> for Position {
    fn from(p: Pose) -> Self {
        p.position
    }
}

impl From<V2> for Control {
    fn from(v: V2) -> Self {
        Control {
            acceleration: v[0],
            curvature: v[1],
        }
    }
}

impl From<State> for V4 {
    fn from(state: State) -> Self {
        [
            state.pose.position.x,
            state.pose.position.y,
            state.pose.yaw,
            state.speed,
        ]
    }
}

impl From<&State> for V4 {
    fn from(state: &State) -> Self {
        (*state).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_vector_conversion_preserves_all_components() {
        let vector = [12.0, -3.0, 0.4, -8.0];
        let state: State = vector.into();
        let owned: V4 = state.into();
        let borrowed: V4 = (&state).into();
        assert_eq!(owned, vector);
        assert_eq!(borrowed, vector);
    }

    #[test]
    fn conversions_preserve_position_and_velocity() {
        for speed in [-8.0, 0.0, 8.0] {
            let original = State::from((Position::new(12.0, -3.0), 0.4, speed));
            let cartesian = PosVelState::from(original);
            assert_eq!(cartesian.position, original.position());
            assert!((cartesian.speed.x - 0.4_f64.cos() * speed).abs() < 1e-10);
            assert!((cartesian.speed.y - 0.4_f64.sin() * speed).abs() < 1e-10);

            let restored = State::from(cartesian);
            let roundtrip = PosVelState::from(restored);
            assert_eq!(roundtrip.position, cartesian.position);
            assert!((roundtrip.speed.x - cartesian.speed.x).abs() < 1e-10);
            assert!((roundtrip.speed.y - cartesian.speed.y).abs() < 1e-10);
            assert_eq!(restored.speed, speed.abs());
            if speed > 0.0 {
                assert!((restored.pose.yaw - original.pose.yaw).abs() < 1e-10);
            } else if speed == 0.0 {
                assert_eq!(restored.pose.yaw, 0.0);
            }
        }
    }
}
