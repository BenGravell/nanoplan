//! Shared planner objective.
//!
//! Every planner prices a feasible sample with the complement of the
//! production metrics composite: safety gates the candidate, then progress
//! and comfort determine its cost. Actor states are projected with the shared
//! lane-aware predictor before applying the safety gate. Optimizers that
//! cannot carry infinity use the same boundary with a finite escape slope.

pub(crate) use crate::planning::constraints::{HardConstraints, Sample};

/// Unsigned curvature through three points, via the Menger curvature
/// formula (twice the signed area over the product of the side lengths).
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metrics::{METRICS, aggregation, comfort, progress};
    use crate::simulation::State;
    use crate::track::Path;

    const HALF_WIDTH_M: f64 = 5.5;

    fn point_cost(sample: &Sample, actors: &[State], lane: Option<&Path>) -> f64 {
        HardConstraints::new(HALF_WIDTH_M, actors, lane).point_cost(sample)
    }

    #[test]
    fn planner_cost_is_the_composite_complement() {
        let sample = Sample {
            speed: 12.0,
            accel: 2.0,
            curvature: 0.1,
            ..Default::default()
        };
        let scores = [
            1.0,
            progress::speed_score(sample.speed),
            comfort::accel_score(sample.accel, sample.speed.powi(2) * sample.curvature),
        ];
        assert_eq!(
            point_cost(&sample, &[], None),
            1.0 - aggregation::composite(&METRICS, &scores)
        );
    }

    #[test]
    fn safety_gate_rejects_collision_and_off_road() {
        let actor = State {
            x: 1.0,
            ..Default::default()
        };
        assert!(point_cost(&Sample::default(), &[actor], None).is_infinite());
        assert!(
            point_cost(
                &Sample {
                    lateral: 10.0,
                    ..Default::default()
                },
                &[],
                None
            )
            .is_infinite()
        );
    }

    #[test]
    fn faster_forward_progress_costs_less() {
        let cost = |speed| {
            point_cost(
                &Sample {
                    speed,
                    ..Default::default()
                },
                &[],
                None,
            )
        };
        assert!(cost(20.0) < cost(10.0));
    }

    #[test]
    fn soft_violation_cost_has_an_escape_slope() {
        let constraints = HardConstraints::new(HALF_WIDTH_M, &[], None);
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

    #[test]
    fn lane_association_predicts_actors_around_a_bend() {
        let lane = Path::new(&[[0.0, 0.0], [50.0, 0.0], [50.0, 50.0]]);
        let actor = State {
            x: 40.0,
            speed: 10.0,
            ..Default::default()
        };
        let sample = Sample {
            xy: [50.0, 10.0],
            t: 2.0,
            ..Default::default()
        };
        assert!(point_cost(&sample, &[actor], None).is_finite());
        assert!(point_cost(&sample, &[actor], Some(&lane)).is_infinite());
    }

    #[test]
    fn menger_curvature_handles_straight_and_turning_points() {
        assert_eq!(curvature_of([0.0, 0.0], [1.0, 0.0], [2.0, 0.0]), 0.0);
        assert!(curvature_of([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]) > 0.0);
    }
}
