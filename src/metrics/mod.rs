//! Frenet progress above constant-speed coasting is the sole metric.

use crate::common::kinematics::TrajectoryKinematics;
#[cfg(test)]
use crate::simulation::{Control, Position, State};
#[cfg(test)]
use crate::track::Road;
use crate::vehicle::MAX_LON_ACCEL;

pub(crate) fn evaluate(trajectory: &TrajectoryKinematics) -> f64 {
    let Some(ego) = trajectory.states.first() else {
        return 0.0;
    };

    let last = trajectory.len() - 1;
    if last == 0 {
        return 1.0;
    }

    progress_score(
        trajectory.projections[last].s - trajectory.projections[0].s,
        ego.speed,
        trajectory.time[last],
    )
}

/// Frenet progress above zero-acceleration coasting, normalized by maximum-acceleration gain.
pub(crate) fn progress_score(progress: f64, initial_speed: f64, t: f64) -> f64 {
    assert!(t > 0.0, "progress_score requires t > 0");
    let progress_gain = progress - initial_speed * t;
    let max_progress_gain = 0.5 * MAX_LON_ACCEL * t * t;
    progress_gain / max_progress_gain
}

#[cfg(test)]
pub(crate) fn evaluate_trace(states: &[State], controls: &[Control], road: &Road) -> f64 {
    let trajectory = TrajectoryKinematics::new(states.to_vec(), controls.to_vec(), road.dt, &road.path());
    evaluate(&trajectory)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic(expected = "progress_score requires t > 0")]
    fn progress_score_rejects_zero_time() {
        progress_score(0.0, 0.0, 0.0);
    }

    #[test]
    fn scores_cumulative_station_delta_with_an_analytic_baseline() {
        let road = Road::new(vec![[-20.0, 0.0], [0.0, 0.0], [0.0, 100.0]], 5.5, 1.0);
        // Same speed at each sample: only station progress affects the score.
        let states = [
            State::from((Position::new(0.0, 10.0), 0.0, 10.0)),
            State::from((Position::new(0.0, 20.0), 0.0, 10.0)),
            State::from((Position::new(0.0, 40.0), 0.0, 10.0)),
        ];
        let score = evaluate_trace(&states, &[Control::default(); 3], &road);
        assert_eq!(score, 10.0 / 13.0);
        // For constant commanded acceleration without resistance, the signed
        // gain is 0.5 * acceleration * t², regardless of initial speed.
        for initial_speed in [-10.0, 0.0, 10.0] {
            for acceleration in [-MAX_LON_ACCEL, 0.0, MAX_LON_ACCEL, 2.0 * MAX_LON_ACCEL] {
                let progress = initial_speed * 2.0 + 0.5 * acceleration * 4.0;
                assert_eq!(
                    progress_score(progress, initial_speed, 2.0),
                    acceleration / MAX_LON_ACCEL
                );
            }
        }
        let t = 0.01;
        assert_eq!(progress_score(0.5 * MAX_LON_ACCEL * t * t, 0.0, t), 1.0);
        for states in [&states[..0], &states[..1]] {
            let score = evaluate_trace(states, &vec![Control::default(); states.len()], &road);
            assert_eq!(score, if states.is_empty() { 0.0 } else { 1.0 });
        }
    }
}
