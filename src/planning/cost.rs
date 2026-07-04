//! Shared trajectory-cost function used by the search-based planners
//! (Frenet lattice, PI²-DDP, RRT*), so "cost" means the same thing across
//! all three instead of each inventing its own weights. Grounded in the
//! same quantities [`crate::metrics`] scores scenario quality by — the
//! hard-collision threshold, the drivable-area bound, the overspeed
//! tolerance, and the comfort accel bounds — plus a prediction of actors
//! that reuses [`crate::metrics::project`], the same constant-velocity
//! model `metrics::ttc` scores against. A planner disagreeing with the
//! metrics about what "good" means would be optimizing for the wrong
//! thing.
//!
//! Every planner in this codebase finds trajectories by sampling candidates
//! and comparing their scalar cost, never by differentiating cost or
//! dynamics — a deliberate design choice. This module's interface enforces
//! it structurally: [`point_cost`] takes already-known numbers (position,
//! speed, curvature, accel) and returns a plain `f64`. Those numbers may
//! come from a closed-form fact about an already-*fixed* candidate curve
//! (RRT*'s differential-flatness steering evaluates its own curvature), or
//! from a purely numerical estimate off sampled points ([`curvature_of`],
//! below) — but nothing here, or in any planner, differentiates cost or
//! dynamics with respect to a search variable to decide where to look next.

use crate::metrics::{CAR_RADIUS_M, comfort, drivable_area, speed_limit};
use crate::simulation::State;

/// Center-to-center clearance below which two cars are treated as collided
/// — the same threshold `metrics::collisions` and `metrics::ttc` score
/// against.
pub(crate) const COLLISION_DIAMETER_M: f64 = 2.0 * CAR_RADIUS_M;

/// Finite stand-in for a hard violation, for callers whose statistics can't
/// tolerate an actual infinity propagating through them (PI²-DDP's
/// min/max-normalized rollout weighting).
pub(crate) const HARD_VIOLATION_PENALTY: f64 = 1e4;

/// One sample along a candidate trajectory: enough geometry and kinematics
/// to price it against the road and predicted actors. Fields a planner
/// doesn't track default to zero, which is always the "no penalty from this
/// term" value.
#[derive(Default)]
pub(crate) struct Sample {
    /// World-frame position, for actor-proximity checks.
    pub xy: [f64; 2],
    /// Signed Frenet offset from the centerline.
    pub lateral: f64,
    /// Signed heading error from the lane direction at this point.
    pub heading_err: f64,
    pub speed: f64,
    pub curvature: f64,
    pub accel: f64,
    /// Seconds from now this sample is reached, for actor prediction.
    pub t: f64,
}

/// Unsigned curvature through three points, via the Menger curvature
/// formula (twice the signed area of the triangle they form, over the
/// product of its three side lengths) — a purely numerical estimate off
/// sampled points, for planners (the lattice) with no closed-form curve to
/// evaluate directly.
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

/// Cost of one sample against the road and every actor's predicted
/// position at `sample.t` (constant velocity and heading — `metrics::project`,
/// the same model `metrics::ttc` uses). Returns `f64::INFINITY` for a hard
/// violation — collision, or off the drivable area — that a planner should
/// reject outright rather than merely disfavor; see [`HARD_VIOLATION_PENALTY`]
/// for callers that need a finite number instead.
pub(crate) fn point_cost(sample: &Sample, target_speed: f64, actors: &[State]) -> f64 {
    if sample.lateral.abs() > drivable_area::ROAD_HALF_WIDTH_M {
        return f64::INFINITY;
    }
    let mut proximity = 0.0;
    for a in actors {
        let predicted = crate::metrics::project(a, sample.t);
        let gap = (sample.xy[0] - predicted.x).hypot(sample.xy[1] - predicted.y);
        if gap < COLLISION_DIAMETER_M {
            return f64::INFINITY;
        }
        // smooth inverse-square repulsion inside the safe zone, so a search
        // that samples near an actor without touching it still prefers
        // more clearance. Weighted heavily for the same reason as the
        // road-edge hinge below: the lattice and RRT* both hard-reject a
        // colliding candidate outright (via the `f64::INFINITY` above) and
        // never select one, so this soft term barely matters to them — but
        // PI²-DDP's continuous, sampling-based search has no such hard
        // accept/reject step, so a weak gradient here is its only actor-
        // avoidance margin.
        proximity += 1.0 / (gap * gap);
    }

    // drivable-area proximity: soft push away from the road edge, on top of
    // the hard reject above. Weighted heavily: planners with their own
    // tighter hard bound (RRT*'s `DRIVABLE_HALF_WIDTH_M`) or a sampling grid
    // that never reaches this threshold (the lattice's `LATERALS_M`) barely
    // ever evaluate it, but PI²-DDP has no such structural bound of its
    // own — this hinge is the only thing keeping its continuous search
    // on the road, so it needs to bite hard, not softly.
    let edge = (sample.lateral.abs() - 0.7 * drivable_area::ROAD_HALF_WIDTH_M).max(0.0);

    // speed tracking, scaled by the same overspeed tolerance speed_limit
    // scores against
    let overspeed = (sample.speed - target_speed) / speed_limit::MAX_OVERSPEED_MS;

    // comfort: penalize exceeding nuPlan's empirical accel bounds, each
    // scaled by its own bound so "just over the line" costs about the same
    // as `overspeed` above — a large weight on a raw, unscaled accel
    // overshoot would swamp collision avoidance, since curvature sharp
    // enough to dodge an actor can be an order of magnitude past a comfort
    // bound. Lateral accel is speed² · curvature — algebraically the same
    // quantity `comfort::Kinematics` measures as `yaw_rate * speed`, since
    // this kinematic model defines yaw_rate = speed * curvature.
    let lon_scale = (comfort::MAX_LON_ACCEL - comfort::MIN_LON_ACCEL) / 2.0;
    let lon_over = ((sample.accel - comfort::MAX_LON_ACCEL).max(0.0)
        + (comfort::MIN_LON_ACCEL - sample.accel).max(0.0))
        / lon_scale;
    let lat_accel = sample.speed * sample.speed * sample.curvature;
    let lat_over =
        (lat_accel.abs() - comfort::MAX_ABS_LAT_ACCEL).max(0.0) / comfort::MAX_ABS_LAT_ACCEL;

    200.0 * proximity
        + 200.0 * edge * edge
        + overspeed * overspeed
        + 2.0 * sample.heading_err * sample.heading_err
        + lon_over * lon_over
        + lat_over * lat_over
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_collision() {
        let s = Sample {
            xy: [0.0, 0.0],
            ..Default::default()
        };
        let actor = State {
            x: 1.0,
            ..Default::default()
        };
        assert!(point_cost(&s, 10.0, &[actor]).is_infinite());
    }

    #[test]
    fn rejects_off_road() {
        let s = Sample {
            lateral: 10.0,
            ..Default::default()
        };
        assert!(point_cost(&s, 10.0, &[]).is_infinite());
    }

    #[test]
    fn clear_road_at_target_speed_is_cheap_and_finite() {
        let s = Sample {
            speed: 10.0,
            ..Default::default()
        };
        let c = point_cost(&s, 10.0, &[]);
        assert!(c.is_finite());
        assert!(c < 1.0, "cost {c}");
    }

    #[test]
    fn menger_curvature_of_a_straight_line_is_zero() {
        assert_eq!(curvature_of([0.0, 0.0], [1.0, 0.0], [2.0, 0.0]), 0.0);
    }

    #[test]
    fn menger_curvature_of_a_right_angle_turn_is_positive() {
        assert!(curvature_of([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]) > 0.0);
    }
}
