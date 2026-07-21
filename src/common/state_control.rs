use crate::common::vector::{V2, V4};

/// 2D world position.
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

/// Position and heading.
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

impl From<Position> for State {
    fn from(p: Position) -> Self {
        State {
            x: p.x,
            y: p.y,
            ..Default::default()
        }
    }
}

impl From<Pose> for Position {
    fn from(p: Pose) -> Self {
        Position::new(p.x, p.y)
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

/// Control action: longitudinal acceleration and path curvature.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Control {
    pub(crate) acceleration: f64,
    pub(crate) curvature: f64,
}

impl From<V2> for Control {
    fn from(v: V2) -> Self {
        Control {
            acceleration: v[0],
            curvature: v[1],
        }
    }
}
