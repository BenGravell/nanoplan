//! Shared trajectory-cost function used by the search-based planners
//! (Frenet lattice, PI²-DDP, RRT*), so "cost" means the same thing across
//! all three instead of each inventing its own weights. Grounded in the
//! same quantities [`crate::metrics`] scores scenario quality by — the
//! hard-collision threshold, the drivable-area bound, the overspeed
//! tolerance, and the comfort accel bounds — plus a prediction of actors
//! via [`crate::metrics::predict`]. That is the lane-aware kinematic model:
//! an actor travelling along the route is rolled forward following the
//! lane's curve and eased back toward its center, so on a bend the planner
//! prices it where it will actually be rather than off on the straight
//! tangent the constant-velocity `metrics::ttc` model assumes. An actor not
//! associated with the lane (oncoming, crossing) still falls back to that
//! same constant-velocity projection. The planner predicting more accurately
//! than the deliberately-simple TTC metric is the point — the ground-truth
//! `metrics::collisions` score over the real actor traces is what a better
//! prediction ultimately has to improve.
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

use crate::metrics::{CAR_RADIUS_M, comfort, lane_keeping, speed_limit};
use crate::scenarios::Path;
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
pub(crate) const N_FEATURES: usize = 7;

/// Display names of the soft features, index-aligned with [`WEIGHTS`].
pub(crate) const FEATURE_NAMES: [&str; N_FEATURES] = [
    "actor_proximity",
    "road_edge",
    "overspeed",
    "heading_err",
    "lon_accel_over",
    "lat_accel_over",
    "lane_keeping",
];

/// Weights of the soft features: `point_cost = WEIGHTS · features`. Hand-set
/// originally; `cargo run --release --bin tune` re-fits them to expert nuPlan
/// trajectories with maximum-entropy IRL (see [`crate::tuning`]) and prints a
/// replacement for this line. The hard collision/off-road rejection is *not*
/// in here — it is a fixed rule of [`features`], never a learned weight.
///
/// `lane_keeping` is weighted well below the collision/road-edge terms so it
/// pulls the car to its lane center on an open road without ever fighting an
/// obstacle swerve that has to leave the lane (which is priced against the
/// far larger `actor_proximity`/off-road terms).
pub(crate) const WEIGHTS: [f64; N_FEATURES] = [200.0, 200.0, 1.0, 2.0, 1.0, 1.0, 3.0];

/// Soft feature vector of one sample, or `None` for a hard violation —
/// collision with an actor's predicted position, or off the drivable area —
/// which is infinitely bad by fiat, not by any weight. Actors are predicted
/// with [`crate::metrics::predict`] against `lane`: one travelling along the
/// route follows the lane's curve back toward its center, while oncoming or
/// crossing traffic (or a `None` lane) falls back to the constant-velocity
/// projection `metrics::ttc` uses. Each feature is already squared/hinged so
/// the cost is *linear* in [`WEIGHTS`] — the form the IRL tuner learns.
///
/// `road_half_width` is the actual half-width of the road this sample sits
/// on ([`crate::scenarios::Road::half_width`]) — the same lateral bound the
/// `drivable_area` metric scores against — so the off-road reject and the
/// road-edge push-back below fire at the true road edge, not a fixed
/// default. On a narrow street that is well inside the old 5.5 m constant.
pub(crate) fn features(
    sample: &Sample,
    target_speed: f64,
    road_half_width: f64,
    actors: &[State],
    lane: Option<&Path>,
) -> Option<[f64; N_FEATURES]> {
    if sample.lateral.abs() > road_half_width {
        return None;
    }
    let mut proximity = 0.0;
    for a in actors {
        let predicted = crate::metrics::predict(a, lane, sample.t);
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
    let edge = (sample.lateral.abs() - 0.7 * road_half_width).max(0.0);

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

    // lane keeping: a hinge on straddling the lane line into the next lane —
    // zero anywhere inside the ego's own lane, growing once the offset passes
    // a lane half-width, normalized so being a full lane width off costs ~1
    // before the weight. Hinged at the lane edge (not at center) on purpose:
    // it must not fight the planners' own centerline pulls or perturb normal
    // in-lane driving — that would, among other things, destabilize iLQR's
    // finite-difference search, whose trajectories live near center. The
    // within-lane *bias* the `lane_keeping` metric also scores is left to
    // those centerline pulls; a per-sample cost has no window to measure it.
    let lane_over =
        (sample.lateral.abs() - lane_keeping::LANE_HALF_WIDTH_M).max(0.0) / lane_keeping::LANE_HALF_WIDTH_M;

    Some([
        proximity,
        edge * edge,
        overspeed * overspeed,
        sample.heading_err * sample.heading_err,
        lon_over * lon_over,
        lat_over * lat_over,
        lane_over * lane_over,
    ])
}

/// Cost of one sample against the road and every actor's predicted position
/// at `sample.t`: [`WEIGHTS`] dotted with [`features`]. Returns
/// `f64::INFINITY` for a hard violation — collision, or off the drivable
/// area — that a planner should reject outright rather than merely disfavor;
/// see [`HARD_VIOLATION_PENALTY`] for callers that need a finite number
/// instead.
pub(crate) fn point_cost(
    sample: &Sample,
    target_speed: f64,
    road_half_width: f64,
    actors: &[State],
    lane: Option<&Path>,
) -> f64 {
    match features(sample, target_speed, road_half_width, actors, lane) {
        None => f64::INFINITY,
        Some(f) => WEIGHTS.iter().zip(f).map(|(w, x)| w * x).sum(),
    }
}

/// How far *inside* a hard violation `sample` sits, in meters: how far past
/// `road_half_width` it is off-road, plus how far inside
/// [`COLLISION_DIAMETER_M`] it is of each predicted actor. Zero exactly at
/// the violation boundary. This is the depth the escape slope in
/// [`soft_point_cost`] scales, so that penalty is continuous with the flat
/// one at the edge.
pub(crate) fn violation_depth(
    sample: &Sample,
    road_half_width: f64,
    actors: &[State],
    lane: Option<&Path>,
) -> f64 {
    let mut depth = (sample.lateral.abs() - road_half_width).max(0.0);
    for a in actors {
        let p = crate::metrics::predict(a, lane, sample.t);
        let gap = (sample.xy[0] - p.x).hypot(sample.xy[1] - p.y);
        depth += (COLLISION_DIAMETER_M - gap).max(0.0);
    }
    depth
}

/// [`point_cost`] with hard violations made finite by a *depth-scaled escape
/// slope*: `HARD_VIOLATION_PENALTY · (1 + `[`violation_depth`]`)` instead of
/// `f64::INFINITY`, for the continuous, sampling-based optimizers (the judo
/// samplers, PI²-DDP, iLQR) whose reward statistics can't absorb an infinity.
///
/// The escape slope matters as much as the finiteness: a *flat*
/// `HARD_VIOLATION_PENALTY` plateau gives those optimizers no gradient once
/// their rollouts are all in violation — on a tight bend where every sampled
/// candidate briefly clips the road edge, or once the closed loop has already
/// drifted off-road, a reward-weighted average (CEM, MPPI) then has nothing
/// pulling it back onto the road and can settle there. Making the penalty
/// grow with depth restores that gradient, so "less off-road" always scores
/// better than "more off-road" and the search climbs back in.
pub(crate) fn soft_point_cost(
    sample: &Sample,
    target_speed: f64,
    road_half_width: f64,
    actors: &[State],
    lane: Option<&Path>,
) -> f64 {
    let c = point_cost(sample, target_speed, road_half_width, actors, lane);
    if c.is_finite() {
        c
    } else {
        HARD_VIOLATION_PENALTY * (1.0 + violation_depth(sample, road_half_width, actors, lane))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Standard drivable half-width for the cost tests — the default road
    /// width ([`crate::metrics::drivable_area::ROAD_HALF_WIDTH_M`]).
    const HW: f64 = 5.5;

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
        assert!(point_cost(&s, 10.0, HW, &[actor], None).is_infinite());
    }

    #[test]
    fn rejects_off_road() {
        let s = Sample {
            lateral: 10.0,
            ..Default::default()
        };
        assert!(point_cost(&s, 10.0, HW, &[], None).is_infinite());
    }

    #[test]
    fn off_road_bound_tracks_the_road_width() {
        // a sample 4.0 m off-center is on a wide road but off a narrow one:
        // the reject follows the road it is given, not a fixed constant
        let s = Sample {
            lateral: 4.0,
            speed: 10.0,
            ..Default::default()
        };
        assert!(point_cost(&s, 10.0, 5.5, &[], None).is_finite());
        assert!(point_cost(&s, 10.0, 3.5, &[], None).is_infinite());
    }

    #[test]
    fn lane_keeping_feature_is_zero_in_lane_and_grows_when_straddling() {
        let feat = |lateral: f64| features(
            &Sample { lateral, speed: 10.0, ..Default::default() },
            10.0,
            HW,
            &[],
            None,
        )
        .unwrap()[6];
        // centered and anywhere inside the ego's own lane: no lane-keeping cost
        assert_eq!(feat(0.0), 0.0);
        assert_eq!(feat(lane_keeping::LANE_HALF_WIDTH_M - 0.01), 0.0);
        // straddling into the next lane costs, and more the further across
        assert!(feat(lane_keeping::LANE_HALF_WIDTH_M + 0.5) > 0.0);
        assert!(feat(3.5) > feat(2.5));
    }

    #[test]
    fn soft_cost_matches_point_cost_when_feasible() {
        let s = Sample {
            speed: 10.0,
            ..Default::default()
        };
        let hard = point_cost(&s, 10.0, HW, &[], None);
        assert!(hard.is_finite());
        assert_eq!(soft_point_cost(&s, 10.0, HW, &[], None), hard);
    }

    #[test]
    fn soft_cost_escape_slope_deepens_with_the_violation() {
        // two off-road samples, one further out than the other: both are hard
        // violations (infinite under point_cost) but soft_point_cost prices
        // the deeper one higher, giving a sampling optimizer a gradient back
        // toward the road instead of a flat penalty plateau.
        let near = Sample { lateral: HW + 0.5, ..Default::default() };
        let far = Sample { lateral: HW + 3.0, ..Default::default() };
        assert!(point_cost(&near, 10.0, HW, &[], None).is_infinite());
        assert!(point_cost(&far, 10.0, HW, &[], None).is_infinite());
        let c_near = soft_point_cost(&near, 10.0, HW, &[], None);
        let c_far = soft_point_cost(&far, 10.0, HW, &[], None);
        assert!(c_near.is_finite() && c_far.is_finite());
        assert!(c_far > c_near, "escape slope not monotonic: {c_near} vs {c_far}");
        // continuous with the flat penalty exactly at the edge (zero depth)
        let edge = Sample { lateral: HW, speed: 10.0, ..Default::default() };
        assert!((soft_point_cost(&edge, 10.0, HW, &[], None)
            - point_cost(&edge, 10.0, HW, &[], None))
        .abs() < 1e-9);
    }

    #[test]
    fn clear_road_at_target_speed_is_cheap_and_finite() {
        let s = Sample {
            speed: 10.0,
            ..Default::default()
        };
        let c = point_cost(&s, 10.0, HW, &[], None);
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
        let f = features(&s, 10.0, HW, &[actor], None).unwrap();
        assert!(f.iter().all(|&x| x > 0.0), "features {f:?}");
        let dot: f64 = WEIGHTS.iter().zip(f).map(|(w, x)| w * x).sum();
        assert_eq!(point_cost(&s, 10.0, HW, &[actor], None), dot);
    }

    #[test]
    fn lane_association_predicts_actors_around_a_bend() {
        // a lane running east then turning north at (50, 0)
        let lane = Path::new(&[[0.0, 0.0], [50.0, 0.0], [50.0, 50.0]]);
        // an actor on the eastbound leg, driving along the lane at 10 m/s
        let actor = State {
            x: 40.0,
            y: 0.0,
            yaw: 0.0,
            speed: 10.0,
        };
        // a sample sitting 20 m of arc length ahead of the actor — up on the
        // *northbound* leg, at (50, 10). Two seconds out the actor reaches it.
        let s = Sample {
            xy: [50.0, 10.0],
            lateral: 0.0,
            t: 2.0,
            ..Default::default()
        };
        // straight-line prediction sends the actor to (60, 0), nowhere near
        // the sample: no collision.
        assert!(point_cost(&s, 10.0, HW, &[actor], None).is_finite());
        // lane-aware prediction follows the bend to (50, 10), right on top of
        // the sample: a collision the straight-line model misses.
        assert!(point_cost(&s, 10.0, HW, &[actor], Some(&lane)).is_infinite());
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
