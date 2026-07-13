//! Kinematic vehicle model and collision handling.

use crate::planning::{Context, Planner};

mod collision;
mod integration;
pub(crate) mod physics;
mod state_control;

pub use crate::barrier::{BARRIER_RESTITUTION, Barrier, collide_with_barriers, road_side_barriers};
pub use crate::vehicle::{
    AIR_DENSITY_KG_M3, DRAG_AREA_M2, EGO_MASS_KG, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL,
    MAX_LON_ACCEL, MIN_LON_ACCEL, ROLLING_RESISTANCE_COEFF,
};
pub(crate) use collision::{collide_with_actors, collide_with_car_actors};
pub(crate) use integration::CommandLimiter;
pub use state_control::{Control, Pose, Position, State};

/// Replan and advance one fixed-size tick.
pub struct Simulator {
    pub state: State,
    pub dt: f64,
    limiter: CommandLimiter,
}

impl Simulator {
    pub fn new(state: State, dt: f64) -> Self {
        Self {
            state,
            dt,
            limiter: CommandLimiter::new(),
        }
    }

    pub fn tick(&mut self, planner: &mut dyn Planner, ctx: &Context) -> State {
        let u = ctx
            .time("total", || planner.plan(self.state, ctx))
            .first()
            .copied()
            .unwrap_or_default();
        let prev = self.state;
        let next = crate::barrier::collide_with_road_barriers(
            prev,
            self.limiter.step(self.state, u, self.dt),
            ctx.road,
        );
        let next = collide_with_car_actors(next, ctx.actors.iter().copied());
        self.state = crate::barrier::collide_with_road_barriers(prev, next, ctx.road);
        self.state
    }
}
