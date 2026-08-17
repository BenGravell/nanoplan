use super::vector::V4;
use super::{Pose, Position};

/// Vehicle state at the rear midpoint: pose and speed.
#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct State {
    pub(crate) pose: Pose,
    pub(crate) speed: f64,
}

impl State {
    pub(crate) const fn new(pose: Pose, speed: f64) -> Self {
        Self { pose, speed }
    }

    pub(crate) fn pose(self) -> Pose {
        self.pose
    }

    pub(crate) fn position(self) -> Position {
        self.pose.position
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

pub(crate) fn state(s: &State) -> V4 {
    [s.pose.position.x, s.pose.position.y, s.pose.yaw, s.speed]
}
