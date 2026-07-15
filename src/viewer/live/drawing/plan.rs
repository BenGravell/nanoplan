use crate::simulation::State;
use bevy::gizmos::config::GizmoConfigStore;
use bevy::prelude::*;

use super::super::screen::px;
use crate::viewer::colors::ACCENT;

const WIDTH: f32 = 3.0;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct PlannedTrajectoryGizmos;

pub(crate) fn configure(mut configs: ResMut<GizmoConfigStore>) {
    configs.config_mut::<PlannedTrajectoryGizmos>().0.line.width = WIDTH;
}

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos<PlannedTrajectoryGizmos>,
    ego: &State,
    plan: &[State],
) {
    gizmos.linestrip_2d(std::iter::once(ego).chain(plan).map(px), ACCENT);
}
