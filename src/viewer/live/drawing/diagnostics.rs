use crate::planning::DiagnosticsData;
use bevy::gizmos::config::GizmoConfigStore;
use bevy::prelude::*;

use super::super::Live;
use super::super::screen::{PX_PER_M, ppx};

const COLOR: Color = Color::srgba(0.0, 0.0, 0.0, 0.4);
const POINT_RADIUS_M: f32 = 0.14;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct DiagnosticTrajectoryGizmos;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct DiagnosticPointGizmos;

pub(crate) fn configure(live: NonSend<Live>, mut configs: ResMut<GizmoConfigStore>) {
    configs
        .config_mut::<DiagnosticTrajectoryGizmos>()
        .0
        .line
        .width = 1.5;
    configs.config_mut::<DiagnosticPointGizmos>().0.line.width =
        POINT_RADIUS_M * PX_PER_M * live.camera.zoom * 1.2;
}

pub(in crate::viewer::live) fn draw(
    trajectories: &mut Gizmos<DiagnosticTrajectoryGizmos>,
    points: &mut Gizmos<DiagnosticPointGizmos>,
    diagnostics: &DiagnosticsData,
    show_trajectories: bool,
    show_points: bool,
) {
    if show_trajectories {
        for trajectory in &diagnostics.trajectories {
            trajectories.linestrip_2d(trajectory.iter().copied().map(ppx), COLOR);
        }
    }
    if show_points {
        for &point in &diagnostics.points {
            points.circle_2d(ppx(point), 0.5 * POINT_RADIUS_M * PX_PER_M, COLOR);
        }
    }
}
