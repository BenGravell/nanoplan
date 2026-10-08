//! Collision checks against predicted actors and road barriers.

use super::{Constraint, Sample};
use crate::common::geometry::barrier::collide_with_road_barriers;
use crate::common::geometry::{
    CAR_COLLISION_RADIUS_M, CAR_FOOTPRINT, EGO_COLLISION_RADIUS_M, EGO_FOOTPRINT, Footprint, footprints_overlap,
};
use crate::planning::Context;
use crate::prediction::predict;
use crate::simulation::{Pose, State};
use crate::track::{Path, Road};

/// Center-to-center clearance below which point-sample planners treat two
/// cars as collided. Physics uses the real rectangular footprint;
/// this is the narrow proxy for planners that only carry a point sample.
const COLLISION_DIAMETER_M: f64 = crate::common::geometry::CAR_FOOTPRINT.width;

pub(super) struct CollisionFree<'a> {
    pub(super) actors: &'a [State],
    pub(super) track: &'a Path,
}

impl Constraint for CollisionFree<'_> {
    fn is_violated(&self, sample: &Sample) -> bool {
        self.is_violated_at(sample, sample.t)
    }

    fn violation_depth(&self, sample: &Sample) -> f64 {
        self.violation_depth_at(sample, sample.t)
    }
}

impl CollisionFree<'_> {
    pub(super) fn is_violated_at(&self, sample: &Sample, actor_time: f64) -> bool {
        self.actors.iter().any(|a| {
            let predicted = predict(a, self.track, actor_time);
            sample.position.distance(predicted.into()) < COLLISION_DIAMETER_M
        })
    }

    pub(super) fn violation_depth_at(&self, sample: &Sample, actor_time: f64) -> f64 {
        self.actors
            .iter()
            .map(|a| {
                let p = predict(a, self.track, actor_time);
                let gap = sample.position.distance(p.into());
                (COLLISION_DIAMETER_M - gap).max(0.0)
            })
            .sum()
    }
}

/// Check the swept ego footprint without applying the collision response.
pub(super) fn road_barrier_collision(previous: State, state: State, footprint: Footprint, road: &Road) -> bool {
    collide_with_road_barriers(previous, state, footprint, road) != state
}

pub(crate) fn actor_collision(pose: Pose, time: f64, ctx: &Context) -> bool {
    ctx.actors.iter().any(|actor| {
        ctx.work(1);
        // Live actors have already advanced when the current ego command is
        // applied. Check their supplied poses as well as the prediction.
        if time <= ctx.road.dt && footprints_overlap(pose, EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT) {
            return true;
        }
        let predicted = predict(actor, ctx.path(), time).pose();
        EGO_FOOTPRINT
            .center(pose)
            .position
            .distance(CAR_FOOTPRINT.center(predicted).position)
            < EGO_COLLISION_RADIUS_M + CAR_COLLISION_RADIUS_M
            && footprints_overlap(pose, EGO_FOOTPRINT, predicted, CAR_FOOTPRINT)
    })
}
