use super::super::FrenetAcceleration;
use super::Trajectory;
use crate::common::kinematics::control_to_accelerations;

/// Native state and control, plus time and derived kinematics for one trajectory.
///
/// All series are tick-aligned and have the same length. Construct this once
/// at a trajectory boundary and share it with consumers instead of recovering
/// controls from finite differences of states.
#[derive(Debug, Clone)]
pub(crate) struct TrajectoryKinematics {
    pub(crate) trajectory: Trajectory,
    pub(crate) time: Vec<f64>,
    pub(crate) accelerations: Vec<FrenetAcceleration>,
    pub(crate) dt: f64,
}

impl TrajectoryKinematics {
    pub(crate) fn new(trajectory: Trajectory, dt: f64) -> Self {
        assert_eq!(trajectory.states.len(), trajectory.controls.len());
        let time = (0..trajectory.states.len()).map(|i| i as f64 * dt).collect();
        let accelerations = trajectory
            .states
            .iter()
            .zip(&trajectory.controls)
            .map(|(state, control)| control_to_accelerations(*control, state.speed))
            .collect();
        Self {
            trajectory,
            time,
            accelerations,
            dt,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.trajectory.states.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::types::{Control, State};

    #[test]
    fn trajectory_kinematics_come_directly_from_aligned_controls() {
        let states = vec![State::from([-2.0, 5.0, 0.0, 10.0]), State::from([3.0, 15.0, 0.0, 20.0])];
        let controls = vec![
            Control {
                acceleration: 2.0,
                curvature: -0.01,
            },
            Control {
                acceleration: -3.0,
                curvature: 0.02,
            },
        ];

        let trajectory = TrajectoryKinematics::new(Trajectory { states, controls }, 0.1);

        assert_eq!(trajectory.time, [0.0, 0.1]);
        assert_eq!(
            trajectory.trajectory.controls,
            [
                Control {
                    acceleration: 2.0,
                    curvature: -0.01,
                },
                Control {
                    acceleration: -3.0,
                    curvature: 0.02,
                },
            ]
        );
        assert_eq!(
            trajectory.accelerations,
            [
                FrenetAcceleration { s: 2.0, d: -1.0 },
                FrenetAcceleration { s: -3.0, d: 8.0 }
            ]
        );
    }
}
