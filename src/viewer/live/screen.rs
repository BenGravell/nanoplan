use crate::simulation::{Position, State};
use bevy::prelude::*;

pub(crate) const PX_PER_M: f32 = 6.0;

pub(crate) fn px(state: &State) -> Vec2 {
    ppx(state.position())
}

pub(crate) fn ppx(position: Position) -> Vec2 {
    Vec2::new(position.x as f32, position.y as f32) * PX_PER_M
}
