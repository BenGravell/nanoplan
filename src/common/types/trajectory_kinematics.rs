use super::{Control, FrenetAcceleration, State};
use crate::common::kinematics::control_to_accelerations;

/// Native state and control, plus time and derived kinematics for one trajectory.
///
/// All series are tick-aligned and have the same length. Construct this once
/// at a trajectory boundary and share it with consumers instead of recovering
/// controls from finite differences of states.
#[derive(Debug, Clone)]
pub(crate) struct TrajectoryKinematics {
    pub(crate) states: Vec<State>,
    pub(crate) controls: Vec<Control>,
    pub(crate) time: Vec<f64>,
    pub(crate) accelerations: Vec<FrenetAcceleration>,
    pub(crate) dt: f64,
}

impl TrajectoryKinematics {
    pub(crate) fn new(states: Vec<State>, controls: Vec<Control>, dt: f64) -> Self {
        assert_eq!(states.len(), controls.len());
        let time = (0..states.len()).map(|i| i as f64 * dt).collect();
        let accelerations = states
            .iter()
            .zip(&controls)
            .map(|(state, control)| control_to_accelerations(*control, state.speed))
            .collect();
        Self {
            states,
            controls,
            time,
            accelerations,
            dt,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.states.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        let trajectory = TrajectoryKinematics::new(states, controls, 0.1);

        assert_eq!(trajectory.time, [0.0, 0.1]);
        assert_eq!(
            trajectory.controls,
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
