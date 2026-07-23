use crate::common::math::wrap_angle;
use crate::simulation::Pose;

use super::distance::dist;

/// Unsigned curvature through three points, via the Menger curvature
/// formula (twice the signed area over the product of the side lengths).
pub(crate) fn curvature_of(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2]) -> f64 {
    let area2 = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    let (a, b, c) = (
        (p1[0] - p0[0]).hypot(p1[1] - p0[1]),
        (p2[0] - p1[0]).hypot(p2[1] - p1[1]),
        (p2[0] - p0[0]).hypot(p2[1] - p0[1]),
    );
    let denom = a * b * c;
    if denom < 1e-9 {
        0.0
    } else {
        2.0 * area2.abs() / denom
    }
}

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
    fn menger_curvature_handles_straight_and_turning_points() {
        assert_eq!(curvature_of([0.0, 0.0], [1.0, 0.0], [2.0, 0.0]), 0.0);
        assert!(curvature_of([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]) > 0.0);
    }

    #[test]
    fn pose_curvature_comes_from_heading_change_over_distance() {
        let previous = Pose::default();
        let current = Pose::new(2.0, 0.0, 0.2);

        assert!((curvature_between(previous, current) - 0.1).abs() < 1e-12);
    }
}
