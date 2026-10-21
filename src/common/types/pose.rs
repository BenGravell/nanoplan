use super::Position;

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Pose {
    pub(crate) position: Position,
    pub(crate) yaw: f64,
}

impl Pose {
    pub(crate) const fn new(position: Position, yaw: f64) -> Self {
        Self { position, yaw }
    }
}
