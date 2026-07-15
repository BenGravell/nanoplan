use crate::common::measure::dot;

/// Rectangular footprint dimensions in meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Footprint {
    pub length: f64,
    pub width: f64,
}

impl Footprint {
    pub const fn new(length: f64, width: f64) -> Self {
        Self { length, width }
    }

    /// Half-extent of this rectangle along a world-space axis.
    pub fn support_radius(self, yaw: f64, axis: [f64; 2]) -> f64 {
        let n = axis[0].hypot(axis[1]).max(1e-9);
        let axis = [axis[0] / n, axis[1] / n];
        let forward = [yaw.cos(), yaw.sin()];
        let left = [-forward[1], forward[0]];
        0.5 * self.length * dot(axis, forward).abs() + 0.5 * self.width * dot(axis, left).abs()
    }
}
