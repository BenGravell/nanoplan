use crate::common::math::wrap_angle;
use crate::simulation::Pose;

use super::distance::dist;

/// Signed curvature estimated from the heading change between two poses.
pub(crate) fn curvature_between(previous: Pose, current: Pose) -> f64 {
    let distance = dist(previous, current);
    if distance <= f64::EPSILON {
        0.0
    } else {
        wrap_angle(current.yaw - previous.yaw) / distance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_curvature_comes_from_heading_change_over_distance() {
        let previous = Pose::default();
        let current = Pose::new(2.0, 0.0, 0.2);

        assert!((curvature_between(previous, current) - 0.1).abs() < 1e-12);
    }
}
