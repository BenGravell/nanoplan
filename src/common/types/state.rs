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
