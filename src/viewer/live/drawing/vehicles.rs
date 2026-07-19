use crate::common::math::wrap_angle;
use crate::geometry::{CAR_FOOTPRINT, Footprint};
use crate::simulation::State;
use crate::vehicle::{
    FRONT_TIRE_DIAMETER_M, FRONT_TIRE_WIDTH_M, FRONT_TRACK_M, MAX_ABS_CURVATURE,
    REAR_TIRE_DIAMETER_M, REAR_TIRE_WIDTH_M, REAR_TRACK_M, WHEELBASE_M,
};
use crate::world::SmartActor;
use bevy::prelude::*;

use super::super::rendering::interpolate_state;
use super::super::screen::{PX_PER_M, px};

const MAX_ACTOR_INTERPOLATION_M: f64 = 20.0;
const TIRE_COLOR: Color = Color::srgb(0.02, 0.02, 0.02);

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos,
    ego: &State,
    ego_curvature: f64,
    actors: &[SmartActor],
    previous_actors: &[(usize, State)],
    render_alpha: f64,
) {
    draw_vehicle(
        gizmos,
        ego,
        ego_curvature,
        CAR_FOOTPRINT,
        Color::srgb(0.08, 0.1, 0.1),
    );
    for actor in actors {
        let previous = previous_actors
            .iter()
            .find(|(id, _)| *id == actor.id)
            .map(|(_, state)| *state);
        let distance = previous.map_or(f64::INFINITY, |state| {
            (actor.state.x - state.x).hypot(actor.state.y - state.y)
        });
        let state = if distance <= MAX_ACTOR_INTERPOLATION_M {
            interpolate_state(previous.unwrap(), actor.state, render_alpha)
        } else {
            actor.state
        };
        let curvature = previous.map_or(0.0, |previous| actor_curvature(previous, actor.state));
        draw_vehicle(
            gizmos,
            &state,
            curvature,
            CAR_FOOTPRINT,
            Color::srgb(0.35, 0.38, 0.38),
        );
    }
}

fn draw_vehicle(
    gizmos: &mut Gizmos,
    state: &State,
    curvature: f64,
    footprint: Footprint,
    color: Color,
) {
    let size = Vec2::new(footprint.length as f32, footprint.width as f32);
    let center = footprint.center(state.pose());
    let iso = Isometry2d::new(px(&center.into()), Rot2::radians(state.yaw as f32));
    gizmos.rect_2d(iso, size * PX_PER_M, color);
    gizmos.line_2d(
        iso * Vec2::ZERO,
        iso * Vec2::new(size.x * PX_PER_M / 2.0, 0.0),
        color,
    );
    let steering = (curvature * footprint.length).atan() as f32;
    for (x, track, diameter, width, angle) in [
        (
            -WHEELBASE_M / 2.0,
            REAR_TRACK_M,
            REAR_TIRE_DIAMETER_M,
            REAR_TIRE_WIDTH_M,
            0.0,
        ),
        (
            WHEELBASE_M / 2.0,
            FRONT_TRACK_M,
            FRONT_TIRE_DIAMETER_M,
            FRONT_TIRE_WIDTH_M,
            steering,
        ),
    ] {
        for y in [-track / 2.0, track / 2.0] {
            gizmos.rect_2d(
                Isometry2d::new(
                    iso * (Vec2::new(x, y) * PX_PER_M),
                    Rot2::radians(state.yaw as f32 + angle),
                ),
                Vec2::new(diameter, width) * PX_PER_M,
                TIRE_COLOR,
            );
        }
    }
}

fn actor_curvature(previous: State, current: State) -> f64 {
    let distance = (current.x - previous.x).hypot(current.y - previous.y);
    if distance <= f64::EPSILON || distance > MAX_ACTOR_INTERPOLATION_M {
        0.0
    } else {
        (wrap_angle(current.yaw - previous.yaw) / distance)
            .clamp(-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actor_tire_curvature_comes_from_heading_change_over_distance() {
        let previous = State::default();
        let current = State {
            x: 2.0,
            yaw: 0.2,
            ..Default::default()
        };

        assert!((actor_curvature(previous, current) - 0.1).abs() < 1e-12);
    }
}
