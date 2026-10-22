use crate::common::geometry::angle::wrap_angle;
use crate::simulation::{Pose, Position};

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

/// Signed [Menger curvature](https://en.wikipedia.org/wiki/Menger_curvature) of three consecutive points.
pub(crate) fn menger_curvature(a: Position, b: Position, c: Position) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let bc = c - b;
    let (ab_length, bc_length, ac_length) = (ab.norm(), bc.norm(), ac.norm());
    assert!(
        ab_length > f64::EPSILON && bc_length > f64::EPSILON && ac_length > f64::EPSILON,
        "Menger curvature requires distinct points"
    );
    2.0 * ab.cross(ac) / (ab_length * bc_length * ac_length)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_curvature_comes_from_heading_change_over_distance() {
        let previous = Pose::default();
        let current = Pose::new(crate::simulation::Position::new(2.0, 0.0), 0.2);

        assert!((curvature_between(previous, current) - 0.1).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "Menger curvature requires distinct points")]
    fn menger_curvature_rejects_points_too_close_together() {
        menger_curvature(
            Position::default(),
            Position::new(f64::EPSILON / 2.0, 0.0),
            Position::new(1.0, 0.0),
        );
    }

    #[test]
    fn counterclockwise_circle_has_positive_menger_curvature() {
        let a = Position::new(0.0, -1.0);
        let b = Position::new(1.0, 0.0);
        let c = Position::new(0.0, 1.0);
        assert!((menger_curvature(a, b, c) - 1.0).abs() < 1e-12);
    }
}
