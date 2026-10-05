//! Drivable road bounds and violation depth.

use super::{Constraint, Sample};

pub(super) struct DrivableArea {
    pub(super) half_width: f64,
}

impl Constraint for DrivableArea {
    fn is_violated(&self, sample: &Sample) -> bool {
        let (right, left) = sample.road_bounds.unwrap_or((-self.half_width, self.half_width));
        sample.lateral < right || sample.lateral > left
    }

    fn violation_depth(&self, sample: &Sample) -> f64 {
        let (right, left) = sample.road_bounds.unwrap_or((-self.half_width, self.half_width));
        (right - sample.lateral).max(0.0) + (sample.lateral - left).max(0.0)
    }
}
