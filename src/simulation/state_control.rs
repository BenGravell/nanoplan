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

/// Position and heading without speed.
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

/// Vehicle state: pose and speed.
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
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Control {
    pub(crate) acceleration: f64,
    pub(crate) curvature: f64,
}
