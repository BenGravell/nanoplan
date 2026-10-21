use super::Vec2d;
use std::ops::{Deref, DerefMut};

/// Cartesian velocity components, in metres per second.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Speed(Vec2d);

impl Speed {
    pub(crate) const fn new(x: f64, y: f64) -> Self {
        Self(Vec2d::new(x, y))
    }
}

impl Deref for Speed {
    type Target = Vec2d;

    fn deref(&self) -> &Vec2d {
        &self.0
    }
}

impl DerefMut for Speed {
    fn deref_mut(&mut self) -> &mut Vec2d {
        &mut self.0
    }
}
