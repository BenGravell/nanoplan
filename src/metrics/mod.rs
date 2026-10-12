//! Frenet station delta in meters is the sole metric.

use crate::common::types::State;
use crate::planning::planner_math::STATE_SAMPLE_PROJECTION_RADIUS_M;
use crate::track::Path;

/// Score endpoint progress without constructing per-state kinematics or projections.
/// Optional station hints disambiguate endpoints on overlapping road segments.
pub(crate) fn evaluate(states: &[State], path: &Path, station_hints: Option<[f64; 2]>) -> f64 {
    let Some(ego) = states.first() else {
        return 0.0;
    };
    if states.len() == 1 {
        return 0.0;
    }
    let station = |state: State, endpoint: usize| {
        station_hints
            .map_or_else(
                || path.project(state.position()),
                |hints| path.project_near(state.position(), hints[endpoint], STATE_SAMPLE_PROJECTION_RADIUS_M),
            )
            .s
    };
    progress_score(station(*states.last().unwrap(), 1) - station(*ego, 0))
}

/// Shared objective: signed Frenet station delta in meters.
pub(crate) fn progress_score(station_delta: f64) -> f64 {
    station_delta
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::Position;
    use crate::track::Road;

    #[test]
    fn projection_work_is_independent_of_trajectory_length() {
        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(100.0, 0.0)]);
        let endpoints = [
            State::from((Position::new(10.0, 0.0), 0.0, 8.0)),
            State::from(Position::new(80.0, 0.0)),
        ];
        path.project(endpoints[0].position()); // Prepare the spatial index before measuring.
        let latency = crate::planning::Latency::default();
        let short_score = latency.time("score", || evaluate(&endpoints, &path, None));
        let short_work = latency.take()[0].clocks;
        let mut states = vec![State::default(); 1001];
        states[0] = endpoints[0];
        states[1000] = endpoints[1];
        let long_score = latency.time("score", || evaluate(&states, &path, None));
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
        let expected = 20.0;
        assert_eq!(evaluate(&states, &path, Some([160.0, 180.0])), expected);
        assert_ne!(evaluate(&states, &path, None), expected);
    }

    #[test]
    fn scores_station_delta_in_meters() {
        let road = Road::new(vec![[-20.0, 0.0], [0.0, 0.0], [0.0, 100.0]], 5.5, 1.0);
        // Same speed at each sample: only station progress affects the score.
        let states = [
            State::from((Position::new(0.0, 10.0), 0.0, 10.0)),
            State::from((Position::new(0.0, 20.0), 0.0, 10.0)),
            State::from((Position::new(0.0, 40.0), 0.0, 10.0)),
        ];
        let score = evaluate(&states, &road.path(), None);
        assert_eq!(score, 30.0);
        // Coasting for the same duration at higher speed earns more progress.
        let faster = [
            State::from((Position::new(0.0, 10.0), 0.0, 30.0)),
            State::from((Position::new(0.0, 40.0), 0.0, 30.0)),
        ];
        let slower = [states[0], states[1]];
        assert!(evaluate(&faster, &road.path(), None) > evaluate(&slower, &road.path(), None));
        let reverse = [states[2], states[0]];
        assert_eq!(evaluate(&reverse, &road.path(), None), -30.0);
        for states in [&states[..0], &states[..1]] {
            let score = evaluate(states, &road.path(), None);
            assert_eq!(score, 0.0);
        }
    }
}
