//! Forward progress is the sole quality metric. Safety is enforced by planner
//! constraints and the vehicle dynamics, rather than a weighted score.

pub(crate) mod progress;

use crate::common::kinematics::TrajectoryKinematics;
#[cfg(test)]
use crate::simulation::{Control, Position, State};
use crate::track::{Path, Road};

pub(crate) struct TickCtx<'a> {
    pub(crate) trajectory_kinematics: &'a TrajectoryKinematics,
    pub(crate) station: &'a [f64],
}

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
    let path = Path::new(road.centerline());
    let station: Vec<f64> = trajectory_kinematics
        .states
        .iter()
        .map(|s| path.project(s.position()).0)
        .collect();
    let ctx = TickCtx {
        trajectory_kinematics,
        station: &station,
    };
    let score_per_tick: Vec<f64> = (0..n).map(|i| progress::score(&ctx, i)).collect();
    let score = score_per_tick.iter().sum::<f64>() / n as f64;
    Metrics { score_per_tick, score }
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
}
