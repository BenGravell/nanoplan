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
//! The cost splits into two parts with different standing. Hard rules —
//! collision and leaving the drivable area — are infinitely bad by fiat
//! ([`features`] returns `None`, [`point_cost`] returns `f64::INFINITY`);
//! no data ever adjusts them. Everything else is a linear combination
//! [`WEIGHTS`]` · `[`features`], and the weights are learnable: the
//! [`crate::tuning`] module re-fits them to expert nuPlan trajectories with
//! maximum-entropy IRL, on the assumption that the human demonstrations were
//! drawn from a well-tuned cost of exactly this form.
//!
//! nanoplan provides no analytic derivatives of its cost or dynamics — a
//! deliberate design choice this module's interface enforces structurally:
//! [`point_cost`] takes already-known numbers (position, speed, curvature,
//! accel) and returns a plain `f64`, and nothing may demand a gradient of
//! it. Most planners find trajectories by sampling candidates and comparing
//! this scalar, never differentiating anything; where one needs curvature
//! as an input it evaluates a closed-form fact about an already-*fixed*
//! candidate curve (RRT*'s differential-flatness steering) or estimates it
//! numerically off sampled points ([`curvature_of`], below). The one
//! genuine optimizer — the treetop-derived iLQR
//! ([`crate::planning::treetop::ilqr`]) — respects the same interface: it
//! consumes this exact black-box scalar and differentiates it by central
//! finite differences, so this function stays the single definition of
//! "good" with no analytically-differentiated twin to drift away from it.

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

/// Number of soft cost features; the length of [`WEIGHTS`], [`FEATURE_NAMES`],
/// and the array [`features`] returns.
pub(crate) const N_FEATURES: usize = 6;

/// Display names of the soft features, index-aligned with [`WEIGHTS`].
pub(crate) const FEATURE_NAMES: [&str; N_FEATURES] = [
    "actor_proximity",
    "road_edge",
    "overspeed",
    "heading_err",
    "lon_accel_over",
    "lat_accel_over",
];

/// Weights of the soft features: `point_cost = WEIGHTS · features`. Hand-set
/// originally; `cargo run --release --bin tune` re-fits them to expert nuPlan
/// trajectories with maximum-entropy IRL (see [`crate::tuning`]) and prints a
/// replacement for this line. The hard collision/off-road rejection is *not*
/// in here — it is a fixed rule of [`features`], never a learned weight.
pub(crate) const WEIGHTS: [f64; N_FEATURES] = [200.0, 200.0, 1.0, 2.0, 1.0, 1.0];

/// Soft feature vector of one sample, or `None` for a hard violation —
/// collision with an actor's predicted position, or off the drivable area —
/// which is infinitely bad by fiat, not by any weight. Actor prediction is
/// constant velocity and heading (`metrics::project`, the same model
/// `metrics::ttc` uses). Each feature is already squared/hinged so the cost
/// is *linear* in [`WEIGHTS`] — the form the IRL tuner learns.
pub(crate) fn features(
    sample: &Sample,
    target_speed: f64,
    actors: &[State],
) -> Option<[f64; N_FEATURES]> {
    if sample.lateral.abs() > drivable_area::ROAD_HALF_WIDTH_M {
        return None;
    }
    let mut proximity = 0.0;
    for a in actors {
        let predicted = crate::metrics::project(a, sample.t);
        let gap = (sample.xy[0] - predicted.x).hypot(sample.xy[1] - predicted.y);
        if gap < COLLISION_DIAMETER_M {
            return None;
        }
        // smooth inverse-square repulsion inside the safe zone, so a search
        // that samples near an actor without touching it still prefers
        // more clearance. Weighted heavily for the same reason as the
        // road-edge hinge below: the lattice and RRT* both hard-reject a
        // colliding candidate outright (via the `None` above) and never
        // select one, so this soft term barely matters to them — but
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

    Some([
        proximity,
        edge * edge,
        overspeed * overspeed,
        sample.heading_err * sample.heading_err,
        lon_over * lon_over,
        lat_over * lat_over,
    ])
}

/// Cost of one sample against the road and every actor's predicted position
/// at `sample.t`: [`WEIGHTS`] dotted with [`features`]. Returns
/// `f64::INFINITY` for a hard violation — collision, or off the drivable
/// area — that a planner should reject outright rather than merely disfavor;
/// see [`HARD_VIOLATION_PENALTY`] for callers that need a finite number
/// instead.
pub(crate) fn point_cost(sample: &Sample, target_speed: f64, actors: &[State]) -> f64 {
    match features(sample, target_speed, actors) {
        None => f64::INFINITY,
        Some(f) => WEIGHTS.iter().zip(f).map(|(w, x)| w * x).sum(),
    }
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
    fn point_cost_is_weights_dot_features() {
        // pins the linear form the IRL tuner relies on: every finite cost is
        // exactly WEIGHTS · features, with every soft term engaged
        let s = Sample {
            xy: [0.0, 0.0],
            lateral: 4.5, // inside the road, past the edge hinge
            heading_err: 0.3,
            speed: 14.0,    // over a 10 m/s target
            curvature: 0.1, // lat accel 19.6, past the comfort bound
            accel: 4.0,     // past MAX_LON_ACCEL
            t: 1.0,
        };
        let actor = State {
            x: 5.0,
            ..Default::default()
        };
        let f = features(&s, 10.0, &[actor]).unwrap();
        assert!(f.iter().all(|&x| x > 0.0), "features {f:?}");
        let dot: f64 = WEIGHTS.iter().zip(f).map(|(w, x)| w * x).sum();
        assert_eq!(point_cost(&s, 10.0, &[actor]), dot);
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
