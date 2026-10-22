use super::algebra::vector::V2;

/// Two-dimensional vector with shared coordinate operations.
#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Vec2d {
    pub(crate) x: f64,
    pub(crate) y: f64,
}

impl Vec2d {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub(crate) fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub(crate) const fn norm_squared(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub(crate) const fn xy(self) -> V2 {
        [self.x, self.y]
    }

    /// Signed z-component of the two-dimensional cross product.
    pub(crate) fn cross(self, other: Self) -> f64 {
        self.x * other.y - self.y * other.x
    }

    pub(crate) fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }

    pub(crate) const fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::types::{Position, Speed};

    #[test]
    fn norm_is_shared_and_avoids_intermediate_overflow() {
        assert_eq!(Vec2d::new(3.0, 4.0).norm(), 5.0);
        assert_eq!(Position::new(3.0, 4.0).norm(), 5.0);
        assert_eq!(Speed::new(3.0, 4.0).norm(), 5.0);
        let mut velocity = Speed::new(3.0, 4.0);
        velocity.x = 6.0;
        velocity.y = 8.0;
        assert_eq!(velocity.norm(), 10.0);
        assert_eq!(Vec2d::default().norm(), 0.0);
        assert!(Vec2d::new(1e200, 1e200).norm().is_finite());
    }
}
