//! Shared physical/rendered footprints.

pub(crate) mod barrier;
mod footprint;
mod polygon;

use crate::common::measure::dot;
use crate::simulation::{Pose, State};

pub(crate) use footprint::Footprint;
pub(crate) use polygon::{polygons_overlap, segments_intersect};

/// Representative passenger-car footprint.
pub(crate) const EGO_FOOTPRINT: Footprint = Footprint::new(5.176, 2.297);
pub(crate) const CAR_FOOTPRINT: Footprint = EGO_FOOTPRINT;

/// Circumscribed ego radius for callers that need a scalar bound.
pub(crate) const EGO_COLLISION_RADIUS_M: f64 = 2.8313947534739836;
pub(crate) const CAR_COLLISION_RADIUS_M: f64 = EGO_COLLISION_RADIUS_M;

/// Constant-speed, constant-heading projection.
pub(crate) fn project(s: &State, t: f64) -> State {
    State {
        x: s.x + s.speed * s.yaw.cos() * t,
        y: s.y + s.speed * s.yaw.sin() * t,
        ..*s
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Overlap {
    /// Unit vector pointing from the second footprint toward the first.
    pub(crate) normal: [f64; 2],
    pub(crate) depth: f64,
}

/// Minimum translation vector for two rendered footprints, if they overlap.
pub(crate) fn overlap_mtv(
    a: Pose,
    a_footprint: Footprint,
    b: Pose,
    b_footprint: Footprint,
) -> Option<Overlap> {
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
}
