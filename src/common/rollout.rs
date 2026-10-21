//! Shared vehicle integration of raw controls and analytic steering motions.

use crate::common::steering::{ConstraintHandlingStrategy, motion_to_control};
use crate::common::types::{Control, State, Trajectory};
use crate::constraints::{Constraint, Kinodynamic, Sample};
use crate::simulation::world_step_unclamped;

/// Roll out controls without clipping, rejecting invalid commands or states.
pub(crate) fn rollout(
    ego: State,
    dt: f64,
    ticks: usize,
    mut control_at: impl FnMut(usize, State) -> Option<Control>,
) -> Option<Trajectory> {
    let mut trajectory = Trajectory {
        states: Vec::with_capacity(ticks + 1),
        controls: Vec::with_capacity(ticks),
    };
    trajectory.states.push(ego);
    let mut state = ego;
    for tick in 0..ticks {
        let control = control_at(tick, state)?;
        if Kinodynamic.is_violated(&Sample::default().with_control(control, state.speed)) {
            return None;
        }
        state = world_step_unclamped(state, control, dt);
        if !state.speed.is_finite() || !state.pose.yaw.is_finite() || !state.position().is_finite() {
            return None;
        }
        trajectory.controls.push(control);
        trajectory.states.push(state);
    }
    Some(trajectory)
}

/// Integrate a launch prefix followed by midpoint-sampled analytic motion.
/// The observer runs once per attempted step, including launch and rejected steps.
pub(crate) fn motion(
    ego: State,
    dt: f64,
    ticks: usize,
    launch: &[Control],
    duration: f64,
    motion_at: impl Fn(f64) -> Option<(State, Control)>,
    mut on_tick: impl FnMut(),
) -> Option<Trajectory> {
    rollout(ego, dt, ticks, |tick, actual| {
        on_tick();
        if tick < launch.len() {
            return Some(launch[tick]);
        }
        let t = (((tick - launch.len()) as f64 + 0.5) * dt).min(duration);
        motion_to_control(motion_at(t)?, actual, dt, ConstraintHandlingStrategy::Reject)
    })
}
