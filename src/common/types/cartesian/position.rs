use super::super::Vec2d;
use std::ops::{Add, Deref, DerefMut, Div, Mul, Sub};

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(target_family = "wasm", serde(transparent))]
pub(crate) struct Position(Vec2d);

impl Deref for Position {
    type Target = Vec2d;

    fn deref(&self) -> &Vec2d {
        &self.0
    }
}

impl DerefMut for Position {
    fn deref_mut(&mut self) -> &mut Vec2d {
        &mut self.0
    }
}

/// Position in a Cartesian frame.
impl Position {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Self(Vec2d::new(x, y))
    }

    pub(crate) const fn is_finite(self) -> bool {
        self.0.is_finite()
    }

    /// Unit-circle position for an angle in radians.
    pub(crate) fn from_angle(angle_rad: f64) -> Self {
        let (y, x) = angle_rad.sin_cos();
        Position::new(x, y)
    }

    pub(crate) fn cross(self, other: Self) -> f64 {
        self.0.cross(other.0)
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
