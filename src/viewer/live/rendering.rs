use crate::simulation::State;
use crate::world::LiveWorld;
use bevy::prelude::*;

use super::Live;
use super::camera::{followed_camera_center, smooth_angle};
use super::drawing::{
    DiagnosticPointGizmos, DiagnosticTrajectoryGizmos, EgoCarpetGizmos, PlannedTrajectoryGizmos,
    RoadSurfaceMesh, carpet, diagnostics, grid, plan, track, vehicles,
};
use super::screen::px;
use crate::viewer::ui::controls::metrics::preview_metrics;
use crate::viewer::{DT, UiState};

const CAMERA_SMOOTH_DURATION_S: f32 = 0.5;

pub(super) struct RenderSnapshot {
    pub(super) ego: State,
    pub(super) actors: Vec<(usize, State)>,
}

impl RenderSnapshot {
    pub(super) fn capture(world: &LiveWorld) -> Self {
        Self {
            ego: world.ego(),
            actors: world
                .actors
                .iter()
                .map(|actor| (actor.id, actor.state))
                .collect(),
        }
    }
}

pub(crate) fn draw(
    mut gizmos: Gizmos,
    mut road_surface: ResMut<RoadSurfaceMesh>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut carpet_gizmos: Gizmos<EgoCarpetGizmos>,
    mut planned_trajectory: Gizmos<PlannedTrajectoryGizmos>,
    mut diagnostic_trajectories: Gizmos<DiagnosticTrajectoryGizmos>,
    mut diagnostic_points: Gizmos<DiagnosticPointGizmos>,
    state: Res<UiState>,
    mut live: NonSendMut<Live>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
    window: Single<&Window>,
    time: Res<Time>,
) {
    // Standard fixed-step interpolation. Rendering stays one simulation tick
    // behind so it can blend completed states without predicting physics.
    let render_alpha = if live.paused {
        1.0
    } else {
        (live.acc as f64 / DT).clamp(0.0, 1.0)
    };
    let ego = interpolate_state(live.previous.ego, live.world.ego(), render_alpha);
    let target_center = live.camera.follow.then_some(px(&ego));
    let target_rotation = live
        .camera
        .align_heading
        .then_some(ego.yaw as f32 - std::f32::consts::FRAC_PI_2);
    let blend = if live.camera.smooth {
        1.0 - (-time.delta_secs() / CAMERA_SMOOTH_DURATION_S).exp()
    } else {
        1.0
    };
    if let Some(target) = target_center {
        live.camera.center = live.camera.center.lerp(target, blend);
    }
    if let Some(target) = target_rotation {
        live.camera.rotation = smooth_angle(live.camera.rotation, target, blend);
    }
    let camera_center = if target_center.is_some() {
        followed_camera_center(live.camera, ego, window.height())
    } else {
        live.camera.center
    };
    camera.translation = camera_center.extend(camera.translation.z);
    camera.rotation = Quat::from_rotation_z(live.camera.rotation);
    camera.scale = Vec3::splat(1.0 / live.camera.zoom);

    if state.show_grid {
        grid::draw(&mut gizmos, live.camera, &window);
    }

    let carpet_metrics = state
        .carpet_visualization
        .is_metric()
        .then(|| preview_metrics(&live));
    let world = &live.world;
    track::draw(
        &mut gizmos,
        &mut meshes,
        &mut road_surface,
        &world.track,
        world.track_progress,
        state.show_stations,
        state.show_centerline,
    );
    if state.show_carpet && !world.plan.is_empty() {
        carpet::draw(
            &mut carpet_gizmos,
            ego,
            &world.plan,
            world.dt(),
            state.carpet_visualization,
            carpet_metrics.as_ref(),
        );
    }
    diagnostics::draw(
        &mut diagnostic_trajectories,
        &mut diagnostic_points,
        &world.diagnostics,
        state.show_diag_trajectories && state.planner.has_diagnostics(),
        state.show_diag_points && state.planner.has_diagnostics(),
    );

    if state.show_plan && !world.plan.is_empty() {
        plan::draw(&mut planned_trajectory, &ego, &world.plan);
    }
    vehicles::draw(
        &mut gizmos,
        &ego,
        world.actuation().curvature,
        &world.actors,
        &live.previous.actors,
        render_alpha,
    );
}

pub(super) fn interpolate_state(previous: State, current: State, alpha: f64) -> State {
    let yaw_delta = (current.yaw - previous.yaw + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    State {
        x: previous.x + (current.x - previous.x) * alpha,
        y: previous.y + (current.y - previous.y) * alpha,
        yaw: previous.yaw + yaw_delta * alpha,
        speed: previous.speed + (current.speed - previous.speed) * alpha,
    }
}
