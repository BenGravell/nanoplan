//! Forward progress is the sole quality metric. Safety is enforced by planner
//! constraints and the vehicle dynamics, rather than a weighted score.

use crate::common::kinematics::TrajectoryKinematics;
use crate::simulation::speed_after_max_accel;
#[cfg(test)]
use crate::simulation::{Control, Position, State};
use crate::track::Road;

#[derive(Debug, Clone, Default)]
pub(crate) struct Metrics {
    pub(crate) score_per_tick: Vec<f64>,
    pub(crate) score: f64,
}

pub(crate) fn evaluate(trajectory_kinematics: &TrajectoryKinematics, road: &Road) -> Metrics {
    let n = trajectory_kinematics.len();
    if n == 0 {
        return Metrics::default();
    }
    let path = road.path();
    let station: Vec<f64> = trajectory_kinematics
        .states
        .iter()
        .map(|s| path.project(s.position()).0)
        .collect();
    // ponytail: score the sole metric directly, without a per-tick context.
    let mut score_per_tick: Vec<f64> = station
        .windows(2)
        .enumerate()
        .map(|(tick, pair)| {
            speed_score(
                (pair[1] - pair[0]) / trajectory_kinematics.dt,
                trajectory_kinematics.states[0].speed,
                tick,
                trajectory_kinematics.dt,
            )
        })
        .collect();
    // The final sample reuses the preceding interval and its baseline tick.
    let last = score_per_tick
        .last()
        .copied()
        .unwrap_or_else(|| speed_score(0.0, trajectory_kinematics.states[0].speed, 0, trajectory_kinematics.dt));
    score_per_tick.push(last);
    let score = score_per_tick.iter().sum::<f64>() / n as f64;
    Metrics { score_per_tick, score }
}

/// Normalized speed score.
pub(crate) fn speed_score(speed: f64, initial_speed: f64, ticks: usize, dt: f64) -> f64 {
    let baseline = speed_after_max_accel(initial_speed, ticks, dt);
    if baseline <= 0.0 {
        f64::from(speed >= baseline)
    } else {
        (speed / baseline).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
pub(crate) fn evaluate_trace(states: &[State], controls: &[Control], road: &Road) -> Metrics {
    let trajectory = TrajectoryKinematics::new(states.to_vec(), controls.to_vec(), road.dt);
    evaluate(&trajectory, road)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::MAX_TERMINAL_SPEED_MPS;
    use crate::simulation::{Control, world_step};
    use crate::vehicle::MAX_LON_ACCEL;

    const CENTERLINE: [crate::simulation::Position; 2] = [
        crate::simulation::Position::new(-20.0, 0.0),
        crate::simulation::Position::new(400.0, 0.0),
    ];
    const DT: f64 = 0.1;
    const TEST_HALF_WIDTH_M: f64 = 5.5;

    fn road() -> Road {
        Road::new(CENTERLINE.to_vec(), 10.0, TEST_HALF_WIDTH_M, DT)
    }

    fn cruise(speed: f64, ticks: usize) -> Vec<State> {
        (0..=ticks)
            .map(|i| State::from((Position::new(speed * DT * i as f64, 0.0), 0.0, speed)))
            .collect()
    }

    fn evaluate_coasting(ego: &[State], road: &Road) -> Metrics {
        evaluate_trace(ego, &vec![Control::default(); ego.len()], road)
    }

    #[test]
    fn perfect_cruise_scores_one_every_tick() {
        let ego = cruise(*MAX_TERMINAL_SPEED_MPS, 20);
        let m = evaluate_coasting(&ego, &road());
        assert!(m.score_per_tick.iter().all(|s| (s - 1.0).abs() < 1e-9));
        assert!((m.score - 1.0).abs() < 1e-9);
    }

    #[test]
    fn progress_uses_full_acceleration_from_the_initial_speed() {
        let initial = *MAX_TERMINAL_SPEED_MPS / 2.0;
        let mut trace = vec![State {
            speed: initial,
            ..Default::default()
        }];
        for _ in 0..20 {
            trace.push(world_step(
                *trace.last().unwrap(),
                Control {
                    acceleration: MAX_LON_ACCEL,
                    ..Default::default()
                },
                DT,
            ));
        }
        let controls = vec![
            Control {
                acceleration: MAX_LON_ACCEL,
                ..Default::default()
            };
            trace.len()
        ];
        let accelerated = evaluate_trace(&trace, &controls, &road());
        assert!((accelerated.score - 1.0).abs() < 1e-9);

        let held_trace = cruise(initial, 20);
        let held = evaluate_coasting(&held_trace, &road());
        assert!(held.score < accelerated.score);
    }

    #[test]
    fn progress_is_independent_of_control_jerk() {
        let ego = cruise(10.0, 20);
        let steady = evaluate_coasting(&ego, &road());
        let controls: Vec<Control> = (0..ego.len())
            .map(|i| Control {
                acceleration: if i % 2 == 0 { 6.0 } else { -6.0 },
                curvature: if i % 2 == 0 { 0.1 } else { -0.1 },
            })
            .collect();
        let changing = evaluate_trace(&ego, &controls, &road());
        assert_eq!(changing.score_per_tick, steady.score_per_tick);
        assert_eq!(changing.score, steady.score);
    }

    #[test]
    fn progress_clamps_backward_motion_to_zero() {
        let mut ego = cruise(10.0, 200);
        ego.reverse();
        let m = evaluate_coasting(&ego, &road());
        assert_eq!(m.score_per_tick[50], 0.0);
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn full_acceleration_is_the_fair_baseline() {
        let dt = 0.1;
        let initial = 12.0;
        let baseline = speed_after_max_accel(initial, 20, dt);
        assert_eq!(speed_score(baseline, initial, 20, dt), 1.0);
        assert!(speed_score(initial, initial, 20, dt) < 1.0);
    }

    #[test]
    fn faster_forward_progress_scores_higher() {
        let score = |speed| speed_score(speed, 10.0, 10, 0.1);
        assert!(score(20.0) > score(10.0));
    }

    #[test]
    fn short_traces_keep_tick_aligned_scores() {
        let road = road();
        let empty = evaluate_coasting(&[], &road);
        assert!(empty.score_per_tick.is_empty());
        assert_eq!(empty.score, 0.0);

        for (speed, expected) in [(0.0, 1.0), (10.0, 0.0)] {
            let single = evaluate_coasting(&cruise(speed, 0), &road);
            assert_eq!(single.score_per_tick, [expected]);
            assert_eq!(single.score, expected);
        }

        let pair = evaluate_coasting(&cruise(10.0, 1), &road);
        assert_eq!(pair.score_per_tick, [1.0, 1.0]);
        assert_eq!(pair.score, 1.0);
    }
}
