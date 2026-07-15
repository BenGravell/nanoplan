//! Hard trajectory constraints shared by planners.

use crate::metrics::{COLLISION_CLEARANCE_M, METRICS, aggregation, comfort, progress};
use crate::prediction::predict;
use crate::simulation::State;
use crate::track::Path;

/// Center-to-center clearance below which point-sample planners treat two
/// cars as collided. Physics and metrics use the real rectangular footprint;
/// this is the narrow proxy for planners that only carry a point sample.
pub(crate) const COLLISION_DIAMETER_M: f64 = COLLISION_CLEARANCE_M;

/// Finite stand-in for a hard violation, for numeric optimizers that cannot
/// propagate infinity through statistics or finite differences.
pub(crate) const HARD_VIOLATION_PENALTY: f64 = 1e4;

/// One sample along a candidate trajectory: enough geometry and kinematics
/// to price it against the road and predicted actors. Fields a planner
/// doesn't track default to zero, which is always the "no penalty from this
/// term" value.
#[derive(Default)]
pub(crate) struct Sample {
    /// World-frame position, for actor collision checks.
    pub(crate) xy: [f64; 2],
    /// Signed Frenet offset from the centerline.
    pub(crate) lateral: f64,
    /// Signed heading error from the lane direction at this point.
    pub(crate) heading_err: f64,
    pub(crate) speed: f64,
    pub(crate) curvature: f64,
    pub(crate) accel: f64,
    /// Seconds from now this sample is reached, for actor prediction.
    pub(crate) t: f64,
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

    /// Per-sample complement of the production composite metric, or infinity
    /// when its safety multiplier is zero.
    pub(crate) fn point_cost(&self, sample: &Sample) -> f64 {
        if self.drivable.is_violated(sample) || self.collision.is_violated(sample) {
            return f64::INFINITY;
        }
        let forward_speed = sample.speed * sample.heading_err.cos();
        let scores = [
            1.0,
            progress::speed_score(forward_speed),
            comfort::accel_score(sample.accel, sample.speed.powi(2) * sample.curvature),
        ];
        1.0 - aggregation::composite(&METRICS, &scores)
    }

    /// Finite, depth-scaled stand-in for a hard violation.
    pub(crate) fn violation_penalty(&self, sample: &Sample) -> f64 {
        HARD_VIOLATION_PENALTY * (1.0 + self.violation_depth(sample))
    }

    /// [`HardConstraints::point_cost`] with hard violations made finite by
    /// [`HardConstraints::violation_penalty`], for optimizers whose reward
    /// statistics or finite differences cannot absorb an infinity.
    pub(crate) fn soft_point_cost(&self, sample: &Sample) -> f64 {
        let c = self.point_cost(sample);
        if c.is_finite() {
            c
        } else {
            self.violation_penalty(sample)
        }
    }
}
