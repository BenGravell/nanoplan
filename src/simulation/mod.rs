//! Kinematic vehicle model and collision handling.

mod collision;
mod integration;
pub(crate) mod physics;
mod state_control;

pub(crate) use collision::collide_with_actors;
pub(crate) use integration::CommandLimiter;
pub(crate) use integration::world_step;
pub(crate) use physics::{MAX_TERMINAL_SPEED_MPS, clamp_control, curvature_limit};
pub(crate) use state_control::{Control, Pose, Position, State};

/// The ego vehicle plant: state, actuator memory, and collision response.
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

    pub(crate) fn step(
        &mut self,
        command: Control,
        road: &crate::track::Road,
        actors: impl IntoIterator<Item = (State, crate::geometry::Footprint)>,
    ) -> State {
        let prev = self.state;
        let next = crate::geometry::barrier::collide_with_road_barriers(
            prev,
            self.limiter.step(self.state, command, self.dt),
            road,
        );
        let next = collide_with_actors(next, actors);
        self.state = crate::geometry::barrier::collide_with_road_barriers(prev, next, road);
        self.state
    }
}
