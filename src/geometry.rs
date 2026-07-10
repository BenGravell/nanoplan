//! Shared physical/rendered footprints.

use crate::simulation::{Pose, State};

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

    pub const fn size_m(self) -> [f64; 2] {
        [self.length, self.width]
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

/// Pacifica footprint from scenarios/nuplan/vehicle_parameters.py.
pub const EGO_FOOTPRINT: Footprint = Footprint::new(5.176, 2.297);
pub const CAR_FOOTPRINT: Footprint = EGO_FOOTPRINT;
pub const TRUCK_FOOTPRINT: Footprint = Footprint::new(9.5, 2.5);
pub const BIKE_FOOTPRINT: Footprint = Footprint::new(1.8, 0.6);
pub const PEDESTRIAN_FOOTPRINT: Footprint = Footprint::new(0.6, 0.6);

/// Circumscribed ego radius for callers that need a scalar bound.
pub const EGO_COLLISION_RADIUS_M: f64 = 2.8313947534739836;
pub const CAR_COLLISION_RADIUS_M: f64 = EGO_COLLISION_RADIUS_M;

/// Constant-speed, constant-heading projection.
pub fn project(s: &State, t: f64) -> State {
    State {
        x: s.x + s.speed * s.yaw.cos() * t,
        y: s.y + s.speed * s.yaw.sin() * t,
        ..*s
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Overlap {
    /// Unit vector pointing from the second footprint toward the first.
    pub normal: [f64; 2],
    pub depth: f64,
}

/// Minimum translation vector for two rendered footprints, if they overlap.
pub fn overlap_mtv(
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

pub fn footprints_overlap(
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

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
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
