use super::super::Vec2d;
use std::ops::{Deref, DerefMut};

/// Acceleration components in a Cartesian frame, in m/s².
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Acceleration(Vec2d);

impl Acceleration {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Self(Vec2d::new(x, y))
    }
}

impl Deref for Acceleration {
    type Target = Vec2d;

    fn deref(&self) -> &Vec2d {
        &self.0
    }
}

impl DerefMut for Acceleration {
    fn deref_mut(&mut self) -> &mut Vec2d {
        &mut self.0
    }
}
