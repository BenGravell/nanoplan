use super::{Pose, State};
use std::ops::{Add, Mul, Sub};

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Position {
    pub(crate) x: f64,
    pub(crate) y: f64,
}

impl Position {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Position { x, y }
    }

    /// Unit-circle position for an angle in radians.
    pub(crate) fn from_angle(angle_rad: f64) -> Self {
        let (y, x) = angle_rad.sin_cos();
        Position::new(x, y)
    }

    pub(crate) const fn xy(self) -> [f64; 2] {
        [self.x, self.y]
    }

    pub(crate) fn distance(self, other: Position) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    pub(crate) const fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Add for Position {
    type Output = Position;

    fn add(self, other: Position) -> Position {
        Position::new(self.x + other.x, self.y + other.y)
    }
}

impl Sub for Position {
    type Output = Position;

    fn sub(self, other: Position) -> Position {
        Position::new(self.x - other.x, self.y - other.y)
    }
}

impl Mul<f64> for Position {
    type Output = Position;

    fn mul(self, scalar: f64) -> Position {
        Position::new(self.x * scalar, self.y * scalar)
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
