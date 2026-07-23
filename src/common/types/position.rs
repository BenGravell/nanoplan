use super::{Pose, State};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Position {
    pub(crate) x: f64,
    pub(crate) y: f64,
}

impl Position {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Position { x, y }
    }

    pub(crate) const fn xy(self) -> [f64; 2] {
        [self.x, self.y]
    }

    pub(crate) fn distance(self, other: Position) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

impl From<[f64; 2]> for Position {
    fn from(p: [f64; 2]) -> Self {
        Position::new(p[0], p[1])
    }
}

impl From<Position> for [f64; 2] {
    fn from(p: Position) -> Self {
        p.xy()
    }
}

impl From<State> for Position {
    fn from(s: State) -> Self {
        Position::new(s.x, s.y)
    }
}

impl From<&State> for Position {
    fn from(s: &State) -> Self {
        (*s).into()
    }
}

impl From<Pose> for Position {
    fn from(p: Pose) -> Self {
        Position::new(p.x, p.y)
    }
}
