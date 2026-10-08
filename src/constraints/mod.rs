//! Hard trajectory constraints shared by planners.

pub(crate) mod collision;
mod drivable_area;
mod kinodynamic;

use collision::CollisionFree;
use drivable_area::DrivableArea;
pub(crate) use kinodynamic::Kinodynamic;

use crate::common::geometry::Footprint;
use crate::metrics::progress_score;
use crate::simulation::{Control, Position, State};
use crate::track::{Path, Road};

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
    pub(crate) position: Position,
    /// Signed Frenet offset from the centerline.
    pub(crate) lateral: f64,
    /// Local signed road bounds when the planner retains varying widths.
    pub(crate) road_bounds: Option<(f64, f64)>,
    /// Frenet s coordinate along the track.
    pub(crate) station: f64,
    /// Command and speed at its application, before integration. None for
    /// geometry-only target samples that have no associated command.
    pub(crate) control: Option<(Control, f64)>,
    /// Seconds from now this sample is reached, for actor prediction.
    pub(crate) t: f64,
}

impl Sample {
    pub(crate) fn with_control(mut self, control: Control, speed: f64) -> Self {
        self.control = Some((control, speed));
        self
    }
}

/// One hard rule a candidate sample must satisfy. Each rule reports both a
/// boolean reject and a depth so optimizers can use the same violation
/// boundary with a finite escape slope instead of re-encoding it.
pub(crate) trait Constraint {
    fn is_violated(&self, sample: &Sample) -> bool;
    fn violation_depth(&self, sample: &Sample) -> f64;
}

/// The hard constraints shared by every planner: stay on the drivable
/// surface, clear predicted actors, and respect vehicle kinodynamic limits.
/// Search planners call
/// [`Constraints::point_cost`] and reject infinity; optimizers call
/// [`Constraints::soft_point_cost`] to keep the same boundary but turn
/// violations into a finite escape slope.
pub(crate) struct Constraints<'a> {
    drivable: DrivableArea,
    collision: CollisionFree<'a>,
    initial_speed: f64,
    initial_station: f64,
}

impl<'a> Constraints<'a> {
    pub(crate) fn new(
        road_half_width: f64,
        actors: &'a [State],
        track: &'a Path,
        initial_speed: f64,
        initial_station: f64,
    ) -> Self {
        Constraints {
            drivable: DrivableArea {
                half_width: road_half_width,
            },
            collision: CollisionFree { actors, track },
            initial_speed,
            initial_station,
        }
    }

    /// Reject a sample without assigning a quality cost.
    pub(crate) fn is_violated(&self, sample: &Sample) -> bool {
        Kinodynamic.is_violated(sample) || self.drivable.is_violated(sample) || self.collision.is_violated(sample)
    }

    /// Reject an ego transition for swept road-barrier contact or sample violations.
    pub(crate) fn is_transition_violated(
        &self,
        previous: State,
        state: State,
        footprint: Footprint,
        road: &Road,
        sample: &Sample,
    ) -> bool {
        collision::road_barrier_collision(previous, state, footprint, road) || self.is_violated(sample)
    }

    /// Progress cost for a feasible sample; hard violations return infinity.
    pub(crate) fn point_cost(&self, sample: &Sample) -> f64 {
        self.point_cost_with_actor_time(sample, sample.t)
    }

    fn point_cost_with_actor_time(&self, sample: &Sample, actor_time: f64) -> f64 {
        if Kinodynamic.is_violated(sample)
            || self.drivable.is_violated(sample)
            || self.collision.is_violated_at(sample, actor_time)
        {
            return f64::INFINITY;
        }
        if sample.t == 0.0 {
            return 0.0;
        }
        -progress_score(sample.station - self.initial_station, self.initial_speed, sample.t)
    }

    /// Finite, depth-scaled stand-in for a hard violation.
    pub(crate) fn violation_penalty(&self, sample: &Sample) -> f64 {
        self.violation_penalty_with_actor_time(sample, sample.t)
    }

    fn violation_penalty_with_actor_time(&self, sample: &Sample, actor_time: f64) -> f64 {
        let depth = Kinodynamic.violation_depth(sample)
            + self.drivable.violation_depth(sample)
            + self.collision.violation_depth_at(sample, actor_time);
        HARD_VIOLATION_PENALTY * (1.0 + depth)
    }

    /// [`Constraints::point_cost`] with hard violations made finite by
    /// [`Constraints::violation_penalty`], for optimizers whose reward
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
    use crate::common::kinematics::curvature_limit;
    use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

    const HALF_WIDTH_M: f64 = 5.5;
    const DT: f64 = 0.1;
    const INITIAL_SPEED: f64 = 10.0;

    fn point_cost(sample: &Sample, actors: &[State]) -> f64 {
        let track = Path::new(&[Position::new(0.0, 0.0), Position::new(100.0, 0.0)]);
        Constraints::new(HALF_WIDTH_M, actors, &track, INITIAL_SPEED, 0.0).point_cost(sample)
    }

    #[test]
    fn kinodynamic_limits_reject_and_penalize_infeasible_commands() {
        let track = Path::new(&[Position::new(0.0, 0.0), Position::new(100.0, 0.0)]);
        let constraints = Constraints::new(HALF_WIDTH_M, &[], &track, INITIAL_SPEED, 0.0);
        for speed in [0.0, 5.0, 30.0, -30.0] {
            for acceleration in [MIN_LON_ACCEL, MAX_LON_ACCEL] {
                for curvature in [-curvature_limit(speed), curvature_limit(speed)] {
                    let sample = Sample::default().with_control(
                        Control {
                            acceleration,
                            curvature,
                        },
                        speed,
                    );
                    assert!(!constraints.is_violated(&sample));
                    assert!(constraints.point_cost(&sample).is_finite());
                }
            }
            for control in [
                Control {
                    acceleration: MIN_LON_ACCEL - 0.1,
                    curvature: 0.0,
                },
                Control {
                    acceleration: MAX_LON_ACCEL + 0.1,
                    curvature: 0.0,
                },
                Control {
                    acceleration: 0.0,
                    curvature: curvature_limit(speed) + 0.01,
                },
                Control {
                    acceleration: 0.0,
                    curvature: -curvature_limit(speed) - 0.01,
                },
                Control {
                    acceleration: f64::NAN,
                    curvature: 0.0,
                },
                Control {
                    acceleration: 0.0,
                    curvature: f64::INFINITY,
                },
            ] {
                let sample = Sample::default().with_control(control, speed);
                assert!(constraints.is_violated(&sample));
                assert!(constraints.point_cost(&sample).is_infinite());
                let penalty = constraints.soft_point_cost(&sample);
                assert!(penalty.is_finite() && penalty > HARD_VIOLATION_PENALTY);
                assert_eq!(constraints.soft_point_cost_with_predicted_actors(&sample), penalty);
            }
        }
        let near = Sample::default().with_control(Control::from([MAX_LON_ACCEL + 0.1, 0.0]), 10.0);
        let far = Sample::default().with_control(Control::from([MAX_LON_ACCEL + 1.0, 0.0]), 10.0);
        assert!(constraints.soft_point_cost(&far) > constraints.soft_point_cost(&near));
    }

    #[test]
    fn transition_checks_footprint_and_sample_constraints() {
        let road = Road::new(vec![[-20.0, 0.0], [100.0, 0.0]], HALF_WIDTH_M, DT);
        let path = road.path();
        let constraints = Constraints::new(HALF_WIDTH_M, &[], &path, INITIAL_SPEED, 0.0);
        let previous = State::from((Position::new(0.0, 0.0), 0.0, INITIAL_SPEED));
        let state = State::from((Position::new(1.0, 0.0), 0.0, INITIAL_SPEED));
        let sample = Sample {
            position: state.position(),
            ..Default::default()
        };
        assert!(!constraints.is_transition_violated(
            previous,
            state,
            crate::common::geometry::EGO_FOOTPRINT,
            &road,
            &sample
        ));

        // The center remains on the road, but the footprint reaches the barrier.
        let touching = State::from((Position::new(1.0, HALF_WIDTH_M - 0.1), 0.0, INITIAL_SPEED));
        let sample = Sample {
            position: touching.position(),
            lateral: touching.position().y,
            ..Default::default()
        };
        assert!(!constraints.is_violated(&sample));
        assert!(constraints.is_transition_violated(
            previous,
            touching,
            crate::common::geometry::EGO_FOOTPRINT,
            &road,
            &sample
        ));

        let actors = [state];
        let constraints = Constraints::new(HALF_WIDTH_M, &actors, &path, INITIAL_SPEED, 0.0);
        let sample = Sample {
            position: state.position(),
            ..Default::default()
        };
        assert!(constraints.is_transition_violated(
            previous,
            state,
            crate::common::geometry::EGO_FOOTPRINT,
            &road,
            &sample
        ));
    }

    #[test]
    fn feasible_cost_only_rewards_progress() {
        let mut sample = Sample {
            station: 10.0,
            t: 1.0,
            ..Default::default()
        };
        let cost = point_cost(&sample, &[]);
        assert_eq!(cost, -progress_score(10.0, INITIAL_SPEED, 1.0));
        sample.station = 12.0;
        assert!(point_cost(&sample, &[]) < cost);
        sample.station = 6.75;
        assert_eq!(point_cost(&sample, &[]), 1.0);
        sample.station = 16.5;
        assert_eq!(point_cost(&sample, &[]), -2.0);

        let track = Path::new(&[Position::new(0.0, 0.0), Position::new(100.0, 0.0)]);
        let constraints = Constraints::new(HALF_WIDTH_M, &[], &track, INITIAL_SPEED, 30.0);
        sample.station = 40.0;
        assert_eq!(constraints.point_cost(&sample), cost);
    }

    #[test]
    fn safety_gate_rejects_collision_and_off_road() {
        let actor = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(1.0, 0.0), 0.0),
            0.0,
        );
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
        let track = Path::new(&[Position::new(0.0, 0.0), Position::new(100.0, 0.0)]);
        let constraints = Constraints::new(HALF_WIDTH_M, &[], &track, INITIAL_SPEED, 0.0);
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
