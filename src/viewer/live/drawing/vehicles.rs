use crate::geometry::{CAR_FOOTPRINT, Footprint};
use crate::simulation::State;
use crate::world::SmartActor;
use bevy::prelude::*;

use super::super::rendering::interpolate_state;
use super::super::screen::{PX_PER_M, px};

const MAX_ACTOR_INTERPOLATION_M: f64 = 20.0;

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos,
    ego: &State,
    actors: &[SmartActor],
    previous_actors: &[(usize, State)],
    render_alpha: f64,
) {
    draw_vehicle(gizmos, ego, CAR_FOOTPRINT, Color::srgb(0.08, 0.1, 0.1));
    for actor in actors {
        let state = previous_actors
            .iter()
            .find(|(id, _)| *id == actor.id)
            .map(|(_, previous)| {
                if (actor.state.x - previous.x).hypot(actor.state.y - previous.y)
                    > MAX_ACTOR_INTERPOLATION_M
                {
                    actor.state
                } else {
                    interpolate_state(*previous, actor.state, render_alpha)
                }
            })
            .unwrap_or(actor.state);
        draw_vehicle(gizmos, &state, CAR_FOOTPRINT, Color::srgb(0.35, 0.38, 0.38));
    }
}

fn draw_vehicle(gizmos: &mut Gizmos, state: &State, footprint: Footprint, color: Color) {
    let size = Vec2::new(footprint.length as f32, footprint.width as f32);
    let iso = Isometry2d::new(px(state), Rot2::radians(state.yaw as f32));
    gizmos.rect_2d(iso, size * PX_PER_M, color);
    gizmos.line_2d(
        iso * Vec2::ZERO,
        iso * Vec2::new(size.x * PX_PER_M / 2.0, 0.0),
        color,
    );
}
