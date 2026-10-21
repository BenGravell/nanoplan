use super::{Position, Speed};

/// Vehicle state represented by Cartesian position and velocity.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct PosVelState {
    pub(crate) position: Position,
    pub(crate) speed: Speed,
}
