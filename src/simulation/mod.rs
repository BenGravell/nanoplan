//! Kinematic vehicle model and collision handling.

mod collision;
mod integration;

pub(crate) use crate::common::kinematics::{clamp_control, curvature_limit};
pub(crate) use crate::common::types::{Control, Pose, Position, State};
pub(crate) use crate::vehicle::MAX_TERMINAL_SPEED_MPS;
pub(crate) use collision::{DynamicBody, collide_dynamic_bodies};
pub(crate) use integration::{CommandLimiter, speed_after_max_accel, world_step};

/// The ego vehicle plant: state and actuator memory.
pub(crate) struct Simulator {
    pub(crate) state: State,
    pub(crate) dt: f64,
    limiter: CommandLimiter,
}

impl Simulator {
    pub(crate) fn new(state: State, dt: f64) -> Self {
        Self {
            state,
            dt,
            limiter: CommandLimiter::new(),
        }
    }

    pub(crate) fn actuation(&self) -> Control {
        self.limiter.applied
    }

    pub(crate) fn preview(&self, commands: &[Control], ticks: usize) -> Vec<State> {
        let mut state = self.state;
        let mut limiter = self.limiter;
        commands
            .iter()
            .take(ticks)
            .map(|&command| {
                state = limiter.step(state, command, self.dt);
                state
            })
            .collect()
    }

    pub(crate) fn step(&mut self, command: Control) -> State {
        self.state = self.limiter.step(self.state, command, self.dt);
        self.state
    }
}
