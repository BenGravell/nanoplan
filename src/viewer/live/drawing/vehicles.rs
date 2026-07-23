use crate::common::interp::interpolate_state;
use crate::geometry::curvature::curvature_between;
use crate::geometry::{CAR_FOOTPRINT, Footprint};
use crate::simulation::State;
use crate::vehicle::{
    FRONT_TIRE_DIAMETER_M, FRONT_TIRE_WIDTH_M, FRONT_TRACK_M, MAX_ABS_CURVATURE,
    REAR_TIRE_DIAMETER_M, REAR_TIRE_WIDTH_M, REAR_TRACK_M, WHEELBASE_M,
};
use crate::world::SmartActor;
use bevy::prelude::*;

use super::super::screen::{PX_PER_M, px};
use crate::viewer::colors::{ACTOR_VEHICLE, EGO_VEHICLE, VEHICLE_TIRE};

// Do not interpolate discontinuous actor updates (for example teleports or reused IDs),
// which would otherwise cause visible sweeps and unrealistic steering curvature.
const MAX_ACTOR_INTERPOLATION_M: f64 = 20.0;

pub(in crate::viewer::live) fn draw_ego(gizmos: &mut Gizmos, ego: &State, curvature: f64) {
    draw_vehicle(gizmos, ego, curvature, CAR_FOOTPRINT, EGO_VEHICLE);
}

pub(in crate::viewer::live) fn draw_actor(
    gizmos: &mut Gizmos,
    actor: &SmartActor,
    previous_actors: &[(usize, State)],
    render_alpha: f64,
) {
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
    let curvature = if distance <= MAX_ACTOR_INTERPOLATION_M {
        previous.map_or(0.0, |previous| {
            curvature_between(previous.pose(), actor.state.pose())
                .clamp(-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE)
        })
    } else {
        0.0
    };
    draw_vehicle(gizmos, &state, curvature, CAR_FOOTPRINT, ACTOR_VEHICLE);
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
                VEHICLE_TIRE,
            );
        }
    }
}
