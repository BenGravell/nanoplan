use super::{Pose, State, vector::V2};
use std::ops::{Add, Div, Mul, Sub};

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Position {
    pub(crate) x: f64,
    pub(crate) y: f64,
}

/// Position in Cartesian coordinates.
impl Position {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Position { x, y }
    }

    /// Unit-circle position for an angle in radians.
    pub(crate) fn from_angle(angle_rad: f64) -> Self {
        let (y, x) = angle_rad.sin_cos();
        Position::new(x, y)
    }

    pub(crate) const fn xy(self) -> V2 {
        [self.x, self.y]
    }

    pub(crate) fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub(crate) const fn norm_squared(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    /// Signed z-component of the 2D cross product: the oriented parallelogram area.
    /// See [Wikipedia](https://en.wikipedia.org/wiki/Cross_product) and the
    /// [area proof](https://proofwiki.org/wiki/Area_of_Triangle_in_Determinant_Form).
    pub(crate) fn cross(self, other: Self) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Angle of this vector from the positive x-axis, in radians.
    pub(crate) fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }

    /// Unit vector in the same direction. Panics for a zero vector.
    pub(crate) fn unit(self) -> Self {
        let norm = self.norm();
        assert_ne!(norm, 0.0, "cannot normalize a zero vector");
        self / norm
    }

    pub(crate) fn distance(self, other: Position) -> f64 {
        (self - other).norm()
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

impl Div<f64> for Position {
    type Output = Position;

    fn div(self, scalar: f64) -> Position {
        Position::new(self.x / scalar, self.y / scalar)
    }
}

impl From<V2> for Position {
    fn from(p: V2) -> Self {
        Position::new(p[0], p[1])
    }
}

impl From<Position> for V2 {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic(expected = "cannot normalize a zero vector")]
    fn zero_vector_has_no_unit_direction() {
        Position::default().unit();
    }

    #[test]
    fn unit_vector_has_unit_norm() {
        assert_eq!(Position::new(3.0, 4.0).norm_squared(), 25.0);
        let unit = Position::new(3.0, 4.0).unit();
        assert!((unit.norm() - 1.0).abs() < 1e-12);
        assert!((Position::new(0.0, 1.0).angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert_eq!(Position::new(1.0, 0.0).cross(Position::new(0.0, 1.0)), 1.0);
        assert_eq!(Position::new(0.0, 1.0).cross(Position::new(1.0, 0.0)), -1.0);
    }
}
