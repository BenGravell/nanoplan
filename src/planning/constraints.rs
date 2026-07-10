//! Hard trajectory constraints shared by planners.

use crate::metrics::COLLISION_CLEARANCE_M;
use crate::planning::cost::{N_FEATURES, WEIGHTS, soft_features};
use crate::prediction::predict;
use crate::scenarios::Path;
use crate::simulation::State;

/// Center-to-center clearance below which point-sample planners treat two
/// cars as collided. Physics and metrics use the real rectangular footprint;
/// this is the narrow proxy for planners that only carry a point sample.
pub(crate) const COLLISION_DIAMETER_M: f64 = COLLISION_CLEARANCE_M;

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

/// One hard rule a candidate sample must satisfy. Each rule reports both a
/// boolean reject and a depth so optimizers can use the same violation
/// boundary with a finite escape slope instead of re-encoding it.
pub(crate) trait HardConstraint {
    fn is_violated(&self, sample: &Sample) -> bool;
    fn violation_depth(&self, sample: &Sample) -> f64;
}

struct DrivableArea {
    half_width: f64,
}

impl HardConstraint for DrivableArea {
    fn is_violated(&self, sample: &Sample) -> bool {
        sample.lateral.abs() > self.half_width
    }

    fn violation_depth(&self, sample: &Sample) -> f64 {
        (sample.lateral.abs() - self.half_width).max(0.0)
    }
}

struct CollisionFree<'a> {
    actors: &'a [State],
    lane: Option<&'a Path>,
}

impl HardConstraint for CollisionFree<'_> {
    fn is_violated(&self, sample: &Sample) -> bool {
        self.actors.iter().any(|a| {
            let predicted = predict(a, self.lane, sample.t);
            (sample.xy[0] - predicted.x).hypot(sample.xy[1] - predicted.y) < COLLISION_DIAMETER_M
        })
    }

    fn violation_depth(&self, sample: &Sample) -> f64 {
        self.actors
            .iter()
            .map(|a| {
                let p = predict(a, self.lane, sample.t);
                let gap = (sample.xy[0] - p.x).hypot(sample.xy[1] - p.y);
                (COLLISION_DIAMETER_M - gap).max(0.0)
            })
            .sum()
    }
}

/// The hard constraints shared by every planner: stay on the drivable
/// surface and clear predicted actors. Search planners call
/// [`HardConstraints::point_cost`] and reject infinity; optimizers call
/// [`HardConstraints::soft_point_cost`] to keep the same boundary but turn
/// violations into a finite escape slope.
pub(crate) struct HardConstraints<'a> {
    drivable: DrivableArea,
    collision: CollisionFree<'a>,
}

impl<'a> HardConstraints<'a> {
    pub(crate) fn new(road_half_width: f64, actors: &'a [State], lane: Option<&'a Path>) -> Self {
        HardConstraints {
            drivable: DrivableArea {
                half_width: road_half_width,
            },
            collision: CollisionFree { actors, lane },
        }
    }

    pub(crate) fn violation_depth(&self, sample: &Sample) -> f64 {
        self.drivable.violation_depth(sample) + self.collision.violation_depth(sample)
    }

    /// Soft feature vector of one sample, or `None` for a hard violation.
    /// Each feature is already squared/hinged so the cost is linear in
    /// [`WEIGHTS`] — the form the IRL tuner learns.
    pub(crate) fn features(&self, sample: &Sample, target_speed: f64) -> Option<[f64; N_FEATURES]> {
        if self.drivable.is_violated(sample) {
            return None;
        }

        let proximity = actor_proximity(sample, self.collision.actors, self.collision.lane)?;
        Some(soft_features(
            sample,
            target_speed,
            self.drivable.half_width,
            proximity,
        ))
    }

    /// [`WEIGHTS`] dotted with [`HardConstraints::features`], or
    /// `f64::INFINITY` for a hard violation a planner should reject.
    pub(crate) fn point_cost(&self, sample: &Sample, target_speed: f64) -> f64 {
        match self.features(sample, target_speed) {
            None => f64::INFINITY,
            Some(f) => WEIGHTS.iter().zip(f).map(|(w, x)| w * x).sum(),
        }
    }

    /// Finite, depth-scaled stand-in for a hard violation.
    pub(crate) fn violation_penalty(&self, sample: &Sample) -> f64 {
        HARD_VIOLATION_PENALTY * (1.0 + self.violation_depth(sample))
    }

    /// [`HardConstraints::point_cost`] with hard violations made finite by
    /// [`HardConstraints::violation_penalty`], for optimizers whose reward
    /// statistics or finite differences cannot absorb an infinity.
    pub(crate) fn soft_point_cost(&self, sample: &Sample, target_speed: f64) -> f64 {
        let c = self.point_cost(sample, target_speed);
        if c.is_finite() {
            c
        } else {
            self.violation_penalty(sample)
        }
    }
}

fn actor_proximity(sample: &Sample, actors: &[State], lane: Option<&Path>) -> Option<f64> {
    let mut proximity = 0.0;
    for a in actors {
        let predicted = predict(a, lane, sample.t);
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
    Some(proximity)
}
