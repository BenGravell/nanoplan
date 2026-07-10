use serde::{Deserialize, Serialize};

/// 2D world position.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
}

impl Position {
    pub const fn new(x: f64, y: f64) -> Self {
        Position { x, y }
    }

    pub const fn xy(self) -> [f64; 2] {
        [self.x, self.y]
    }

    pub fn distance(self, other: Position) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

/// Position and heading without speed.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose {
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub const fn xy(self) -> [f64; 2] {
        [self.x, self.y]
    }
}

/// Vehicle state: pose and speed.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct State {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub yaw: f64,
    #[serde(default)]
    pub speed: f64,
}

impl State {
    pub fn pose(self) -> Pose {
        self.into()
    }

    pub fn position(self) -> Position {
        self.into()
    }

    pub fn with_pose(self, pose: Pose) -> Self {
        State {
            x: pose.x,
            y: pose.y,
            yaw: pose.yaw,
            ..self
        }
    }

    pub fn with_position(self, position: Position) -> Self {
        State {
            x: position.x,
            y: position.y,
            ..self
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

/// Control action: longitudinal acceleration and path curvature.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Control {
    #[serde(default)]
    pub acceleration: f64,
    #[serde(default)]
    pub curvature: f64,
}
