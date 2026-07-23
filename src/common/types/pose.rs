use super::{Position, State};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Pose {
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) yaw: f64,
}

impl Pose {
    pub(crate) const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }
}

impl From<Position> for Pose {
    fn from(p: Position) -> Self {
        Pose::new(p.x, p.y, 0.0)
    }
}

impl From<State> for Pose {
    fn from(s: State) -> Self {
        Pose::new(s.x, s.y, s.yaw)
    }
}

impl From<&State> for Pose {
    fn from(s: &State) -> Self {
        (*s).into()
    }
}
