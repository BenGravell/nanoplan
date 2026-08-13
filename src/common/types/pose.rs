use super::{Position, State};

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
