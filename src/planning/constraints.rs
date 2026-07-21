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
    pub(crate) lon_jerk: f64,
    pub(crate) lat_jerk: f64,
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
    track: &'a Path,
}

impl HardConstraint for CollisionFree<'_> {
    fn is_violated(&self, sample: &Sample) -> bool {
        self.is_violated_at(sample, sample.t)
    }

    fn violation_depth(&self, sample: &Sample) -> f64 {
        self.violation_depth_at(sample, sample.t)
    }
}

impl CollisionFree<'_> {
    fn is_violated_at(&self, sample: &Sample, actor_time: f64) -> bool {
        self.actors.iter().any(|a| {
            let predicted = predict(a, self.track, actor_time);
            (sample.xy[0] - predicted.x).hypot(sample.xy[1] - predicted.y) < COLLISION_DIAMETER_M
        })
    }

    fn violation_depth_at(&self, sample: &Sample, actor_time: f64) -> f64 {
        self.actors
            .iter()
            .map(|a| {
                let p = predict(a, self.track, actor_time);
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
    initial_speed: f64,
    dt: f64,
}

impl<'a> HardConstraints<'a> {
    pub(crate) fn new(
        road_half_width: f64,
        actors: &'a [State],
        track: &'a Path,
        initial_speed: f64,
        dt: f64,
    ) -> Self {
        HardConstraints {
            drivable: DrivableArea {
                half_width: road_half_width,
            },
            collision: CollisionFree { actors, track },
            initial_speed,
            dt,
        }
    }

    /// Per-sample complement of the production composite metric, or infinity
    /// when its safety multiplier is zero.
    pub(crate) fn point_cost(&self, sample: &Sample) -> f64 {
        self.point_cost_with_actor_time(sample, sample.t)
    }

    fn point_cost_with_actor_time(&self, sample: &Sample, actor_time: f64) -> f64 {
        if self.drivable.is_violated(sample) || self.collision.is_violated_at(sample, actor_time) {
            return f64::INFINITY;
        }
        let forward_speed = sample.speed * sample.heading_err.cos();
        let tick = (sample.t / self.dt).round().max(0.0) as usize;
        let scores = [
            1.0,
            progress::speed_score(forward_speed, self.initial_speed, tick, self.dt),
            comfort::jerk_score(sample.lon_jerk, sample.lat_jerk),
        ];
        1.0 - aggregation::composite(&METRICS, &scores)
    }

    /// Finite, depth-scaled stand-in for a hard violation.
    pub(crate) fn violation_penalty(&self, sample: &Sample) -> f64 {
        self.violation_penalty_with_actor_time(sample, sample.t)
    }

    fn violation_penalty_with_actor_time(&self, sample: &Sample, actor_time: f64) -> f64 {
        let depth = self.drivable.violation_depth(sample)
            + self.collision.violation_depth_at(sample, actor_time);
        HARD_VIOLATION_PENALTY * (1.0 + depth)
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

    /// Soft point cost when `actors` were already predicted to `sample.t`.
    pub(crate) fn soft_point_cost_with_predicted_actors(&self, sample: &Sample) -> f64 {
        let c = self.point_cost_with_actor_time(sample, 0.0);
        if c.is_finite() {
            c
        } else {
            self.violation_penalty_with_actor_time(sample, 0.0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HALF_WIDTH_M: f64 = 5.5;
    const DT: f64 = 0.1;
    const INITIAL_SPEED: f64 = 10.0;

    fn point_cost(sample: &Sample, actors: &[State]) -> f64 {
        let track = Path::new(&[[0.0, 0.0], [100.0, 0.0]]);
        HardConstraints::new(HALF_WIDTH_M, actors, &track, INITIAL_SPEED, DT).point_cost(sample)
    }

    #[test]
    fn planner_cost_is_the_composite_complement() {
        let sample = Sample {
            speed: 12.0,
            lon_jerk: 20.0,
            lat_jerk: 15.0,
            t: 1.0,
            ..Default::default()
        };
        let scores = [
            1.0,
            progress::speed_score(
                sample.speed,
                INITIAL_SPEED,
                (sample.t / DT).round() as usize,
                DT,
            ),
            comfort::jerk_score(sample.lon_jerk, sample.lat_jerk),
        ];
        assert_eq!(
            point_cost(&sample, &[]),
            1.0 - aggregation::composite(&METRICS, &scores)
        );
    }

    #[test]
    fn safety_gate_rejects_collision_and_off_road() {
        let actor = State {
            x: 1.0,
            ..Default::default()
        };
        assert!(point_cost(&Sample::default(), &[actor]).is_infinite());
        assert!(
            point_cost(
                &Sample {
                    lateral: 10.0,
                    ..Default::default()
                },
                &[]
            )
            .is_infinite()
        );
    }

    #[test]
    fn soft_violation_cost_has_an_escape_slope() {
        let track = Path::new(&[[0.0, 0.0], [100.0, 0.0]]);
        let constraints = HardConstraints::new(HALF_WIDTH_M, &[], &track, INITIAL_SPEED, DT);
        let near = Sample {
            lateral: HALF_WIDTH_M + 0.5,
            ..Default::default()
        };
        let far = Sample {
            lateral: HALF_WIDTH_M + 3.0,
            ..Default::default()
        };
        assert!(constraints.soft_point_cost(&far) > constraints.soft_point_cost(&near));
    }
}
