//! Shared geometry helpers and physical/rendered footprints.

pub(crate) mod barrier;
pub(crate) mod curvature;
pub(crate) mod distance;
mod footprint;
#[cfg(any(test, feature = "track-pregeneration"))]
mod polygon;
mod road_polygon;

use crate::common::measure::dot;
use crate::simulation::{Pose, Position};
use crate::vehicle::{BODY_LENGTH_M, BODY_WIDTH_M};

pub(crate) use footprint::Footprint;
#[cfg(any(test, feature = "track-pregeneration"))]
pub(crate) use polygon::{polygons_overlap, segments_intersect};
pub(crate) use road_polygon::RoadPolygon;

/// 2017 Ford GT body footprint.
pub(crate) const EGO_FOOTPRINT: Footprint = Footprint::new(BODY_LENGTH_M, BODY_WIDTH_M);
pub(crate) const CAR_FOOTPRINT: Footprint = EGO_FOOTPRINT;

/// Circumscribed ego radius for callers that need a scalar bound.
pub(crate) const EGO_COLLISION_RADIUS_M: f64 = 2.5908902910003735;
pub(crate) const CAR_COLLISION_RADIUS_M: f64 = EGO_COLLISION_RADIUS_M;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Overlap {
    /// Unit vector pointing from the second footprint toward the first.
    pub(crate) normal: [f64; 2],
    pub(crate) depth: f64,
}

/// Minimum translation vector for two rendered footprints, if they overlap.
pub(crate) fn overlap_mtv(
    a_rear: Pose,
    a_footprint: Footprint,
    b_rear: Pose,
    b_footprint: Footprint,
) -> Option<Overlap> {
    let a = a_footprint.center(a_rear);
    let b = b_footprint.center(b_rear);
    let delta = [a.position.x - b.position.x, a.position.y - b.position.y];
    let mut best = Overlap {
        normal: [1.0, 0.0],
        depth: f64::INFINITY,
    };

    for axis in axes(a.yaw).into_iter().chain(axes(b.yaw)) {
        let separation = dot(delta, axis);
        let depth =
            a_footprint.support_radius(a.yaw, axis) + b_footprint.support_radius(b.yaw, axis) - separation.abs();
        if depth <= 0.0 {
            return None;
        }
        if depth < best.depth {
            let sign = if separation >= 0.0 { 1.0 } else { -1.0 };
            best = Overlap {
                normal: [axis[0] * sign, axis[1] * sign],
                depth,
            };
        }
    }
    Some(best)
}

pub(crate) fn footprints_overlap(a: Pose, a_footprint: Footprint, b: Pose, b_footprint: Footprint) -> bool {
    overlap_mtv(a, a_footprint, b, b_footprint).is_some()
}

fn axes(yaw: f64) -> [[f64; 2]; 2] {
    let forward = Position::from_angle(yaw);
    [forward.xy(), [-forward.y, forward.x]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangles_touching_edges_do_not_overlap() {
        let gap = CAR_FOOTPRINT.length;
        assert!(!footprints_overlap(
            Pose::new(crate::simulation::Position::new(0.0, 0.0), 0.0),
            CAR_FOOTPRINT,
            Pose::new(crate::simulation::Position::new(gap, 0.0), 0.0),
            CAR_FOOTPRINT
        ));
        assert!(footprints_overlap(
            Pose::new(crate::simulation::Position::new(0.0, 0.0), 0.0),
            CAR_FOOTPRINT,
            Pose::new(crate::simulation::Position::new(gap - 0.01, 0.0), 0.0),
            CAR_FOOTPRINT
        ));
    }

    #[test]
    fn pose_is_the_rear_of_the_footprint() {
        let rear = Pose::new(crate::simulation::Position::new(2.0, 3.0), std::f64::consts::FRAC_PI_2);
        let center = CAR_FOOTPRINT.center(rear);

        assert!((center.position.x - 2.0).abs() < 1e-12);
        assert!((center.position.y - (3.0 + CAR_FOOTPRINT.length / 2.0)).abs() < 1e-12);
    }
}

/// Estimate the path heading at `b` by bisecting the directions of `a → b` and `b → c`.
/// Panics if either segment has zero length; an exact reversal has no unique bisector.
pub(crate) fn vertex_heading(a: Position, b: Position, c: Position) -> f64 {
    let ab = (b - a).unit();
    let bc = (c - b).unit();
    let tangent = ab + bc;
    tangent.angle()
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

/// Wrap an angle to [-pi, pi).
pub(crate) fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Shortest signed rotation from `from` to `to`.
pub(crate) fn angle_delta(from: f64, to: f64) -> f64 {
    wrap_angle(to - from)
}

#[cfg(test)]
mod helper_tests {
    use super::*;

    #[test]
    fn wrap_angle_returns_principal_angle() {
        assert_eq!(wrap_angle(0.0), 0.0);
        assert!((wrap_angle(3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1e-12);
        assert!((wrap_angle(-3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn angle_delta_takes_the_short_arc() {
        let from = std::f64::consts::PI - 0.2;
        let to = -std::f64::consts::PI + 0.2;
        assert!((angle_delta(from, to) - 0.4).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "Menger curvature requires distinct points")]
    fn curvature_rejects_points_too_close_together() {
        menger_curvature(
            Position::default(),
            Position::new(f64::EPSILON / 2.0, 0.0),
            Position::new(1.0, 0.0),
        );
    }

    #[test]
    fn counterclockwise_circle_has_positive_curvature_and_tangent_heading() {
        let a = Position::new(0.0, -1.0);
        let b = Position::new(1.0, 0.0);
        let c = Position::new(0.0, 1.0);
        assert!((vertex_heading(a, b, c) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((menger_curvature(a, b, c) - 1.0).abs() < 1e-12);
    }
}
