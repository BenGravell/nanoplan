use crate::common::types::AxisMotion;

/// Position, velocity, and acceleration along two axes in the motion's coordinate system.
pub(crate) trait Motion {
    fn derivatives_at(&self, t: f64) -> (AxisMotion, AxisMotion);
}
