//! Shared geometry helpers and physical/rendered footprints.

pub(crate) mod barrier;
pub(crate) mod curvature;
mod footprint;
mod polygon;
mod road_polygon;

use crate::common::measure::dot;
use crate::simulation::Pose;
use crate::vehicle::{BODY_LENGTH_M, BODY_WIDTH_M};

pub(crate) use footprint::Footprint;
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
    let delta = [a.x - b.x, a.y - b.y];
    let mut best = Overlap {
        normal: [1.0, 0.0],
        depth: f64::INFINITY,
    };

    for axis in axes(a.yaw).into_iter().chain(axes(b.yaw)) {
        let separation = dot(delta, axis);
        let depth = a_footprint.support_radius(a.yaw, axis)
            + b_footprint.support_radius(b.yaw, axis)
            - separation.abs();
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

pub(crate) fn footprints_overlap(
    a: Pose,
    a_footprint: Footprint,
    b: Pose,
    b_footprint: Footprint,
) -> bool {
    overlap_mtv(a, a_footprint, b, b_footprint).is_some()
}

fn axes(yaw: f64) -> [[f64; 2]; 2] {
    let forward = [yaw.cos(), yaw.sin()];
    [forward, [-forward[1], forward[0]]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangles_touching_edges_do_not_overlap() {
        let gap = CAR_FOOTPRINT.length;
        assert!(!footprints_overlap(
            Pose::new(0.0, 0.0, 0.0),
            CAR_FOOTPRINT,
            Pose::new(gap, 0.0, 0.0),
            CAR_FOOTPRINT
        ));
        assert!(footprints_overlap(
            Pose::new(0.0, 0.0, 0.0),
            CAR_FOOTPRINT,
            Pose::new(gap - 0.01, 0.0, 0.0),
            CAR_FOOTPRINT
        ));
    }

    #[test]
    fn pose_is_the_rear_of_the_footprint() {
        let rear = Pose::new(2.0, 3.0, std::f64::consts::FRAC_PI_2);
        let center = CAR_FOOTPRINT.center(rear);

        assert!((center.x - 2.0).abs() < 1e-12);
        assert!((center.y - (3.0 + CAR_FOOTPRINT.length / 2.0)).abs() < 1e-12);
    }
}
