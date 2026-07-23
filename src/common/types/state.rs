use super::vector::V4;
use super::{Pose, Position};

/// Vehicle state at the rear midpoint: pose and speed.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct State {
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) yaw: f64,
    pub(crate) speed: f64,
}

impl State {
    pub(crate) fn pose(self) -> Pose {
        self.into()
    }

    pub(crate) fn position(self) -> Position {
        self.into()
    }
}

impl From<V4> for State {
    fn from(v: V4) -> Self {
        State {
            x: v[0],
            y: v[1],
            yaw: v[2],
            speed: v[3],
        }
    }
}

impl From<Position> for State {
    fn from(p: Position) -> Self {
        State {
            x: p.x,
            y: p.y,
            ..Default::default()
        }
    }
}

impl From<Pose> for State {
    fn from(p: Pose) -> Self {
        State {
            x: p.x,
            y: p.y,
            yaw: p.yaw,
            ..Default::default()
        }
    }
}

pub(crate) fn state(s: &State) -> V4 {
    [s.x, s.y, s.yaw, s.speed]
}
