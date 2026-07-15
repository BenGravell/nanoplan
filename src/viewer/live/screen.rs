use crate::simulation::State;
use bevy::prelude::*;

pub(crate) const PX_PER_M: f32 = 6.0;

pub(crate) fn px(state: &State) -> Vec2 {
    Vec2::new(state.x as f32, state.y as f32) * PX_PER_M
}

pub(crate) fn ppx(position: [f64; 2]) -> Vec2 {
    Vec2::new(position[0] as f32, position[1] as f32) * PX_PER_M
}
