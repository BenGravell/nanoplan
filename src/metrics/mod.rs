//! Frenet progress above constant-speed coasting is the sole metric.

use crate::common::types::State;
use crate::planning::planner_math::STATE_SAMPLE_PROJECTION_RADIUS_M;
use crate::track::Path;
use crate::vehicle::MAX_LON_ACCEL;

/// Score endpoint progress without constructing per-state kinematics or projections.
/// Optional station hints disambiguate endpoints on overlapping road segments.
pub(crate) fn evaluate(states: &[State], dt: f64, path: &Path, station_hints: Option<[f64; 2]>) -> f64 {
    let Some(ego) = states.first() else {
        return 0.0;
    };
    if states.len() == 1 {
        return 1.0;
    }
    let station = |state: State, endpoint: usize| {
        station_hints
            .map_or_else(
                || path.project(state.position()),
                |hints| path.project_near(state.position(), hints[endpoint], STATE_SAMPLE_PROJECTION_RADIUS_M),
            )
            .s
    };
    let progress = station(*states.last().unwrap(), 1) - station(*ego, 0);
    progress_score(progress, ego.speed, (states.len() - 1) as f64 * dt)
}

/// Frenet progress above zero-acceleration coasting, normalized by maximum-acceleration gain.
pub(crate) fn progress_score(progress: f64, initial_speed: f64, t: f64) -> f64 {
    assert!(t > 0.0, "progress_score requires t > 0");
    let progress_gain = progress - initial_speed * t;
    let max_progress_gain = 0.5 * MAX_LON_ACCEL * t * t;
    progress_gain / max_progress_gain
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::Position;
    use crate::track::Road;

    #[test]
    #[should_panic(expected = "progress_score requires t > 0")]
    fn progress_score_rejects_zero_time() {
        progress_score(0.0, 0.0, 0.0);
    }

    #[test]
    fn projection_work_is_independent_of_trajectory_length() {
        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(100.0, 0.0)]);
        let endpoints = [
            State::from((Position::new(10.0, 0.0), 0.0, 8.0)),
            State::from(Position::new(80.0, 0.0)),
        ];
        path.project(endpoints[0].position()); // Prepare the spatial index before measuring.
        let latency = crate::planning::Latency::default();
        let short_score = latency.time("score", || evaluate(&endpoints, 10.0, &path, None));
        let short_work = latency.take()[0].clocks;
        let mut states = vec![State::default(); 1001];
        states[0] = endpoints[0];
        states[1000] = endpoints[1];
        let long_score = latency.time("score", || evaluate(&states, 0.01, &path, None));
        assert_eq!(long_score, short_score);
        assert!(short_work > 0);
        assert_eq!(latency.take()[0].clocks, short_work);
    }

    #[test]
    fn endpoint_hints_preserve_progress_on_overlapping_road_segments() {
        let points = [
            [-100.0, 0.01],
            [200.0, 0.01],
            [200.0, 100.0],
            [-100.0, 100.0],
            [-100.0, 0.0],
            [200.0, 0.0],
        ];
        let path = Path::new(&points.map(Position::from));
        let states = [
            State::from((Position::new(60.0, 0.0), 0.0, 8.0)),
            State::from(Position::new(80.0, 0.01)),
        ];
        let expected = progress_score(20.0, 8.0, 2.0);
        assert_eq!(evaluate(&states, 2.0, &path, Some([160.0, 180.0])), expected);
        assert_ne!(evaluate(&states, 2.0, &path, None), expected);
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
        let score = evaluate(&states, road.dt, &road.path(), None);
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
            let score = evaluate(states, road.dt, &road.path(), None);
            assert_eq!(score, if states.is_empty() { 0.0 } else { 1.0 });
        }
    }
}
