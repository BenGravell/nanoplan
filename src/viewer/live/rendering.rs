use bevy::gizmos::config::GizmoConfigStore;
use bevy::prelude::*;
use nanoplan::world::LiveWorld;
use nanoplan::{CAR_FOOTPRINT, Path, State};

use super::Live;
use super::camera::{CameraState, CameraTarget, followed_camera_center, smooth_angle};
use super::carpet::{self, EgoCarpetGizmos};
use super::primitives::{ACCENT, draw_agent, draw_car};
use super::screen::{PX_PER_M, ppx, px};
use crate::viewer::{DT, UiState};

const CAMERA_SMOOTH_RATE: f32 = 8.0;
const MAX_ACTOR_INTERPOLATION_M: f64 = 20.0;
const DIAGNOSTIC_COLOR: Color = Color::srgba(0.2, 0.85, 0.95, 0.28);
const DIAGNOSTIC_POINT_RADIUS_M: f32 = 0.14;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct DiagnosticTrajectoryGizmos;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct DiagnosticPointGizmos;

pub(super) struct RenderSnapshot {
    pub(super) ego: State,
    actors: Vec<(usize, State)>,
}

impl RenderSnapshot {
    pub(super) fn capture(world: &LiveWorld) -> Self {
        Self {
            ego: world.ego,
            actors: world
                .actors
                .iter()
                .map(|actor| (actor.id, actor.state))
                .collect(),
        }
    }
}

pub(crate) fn configure_diagnostics(live: NonSend<Live>, mut configs: ResMut<GizmoConfigStore>) {
    configs
        .config_mut::<DiagnosticTrajectoryGizmos>()
        .0
        .line
        .width = 1.0;
    configs.config_mut::<DiagnosticPointGizmos>().0.line.width =
        DIAGNOSTIC_POINT_RADIUS_M * PX_PER_M * live.camera.zoom * 1.05;
}

pub(crate) fn draw(
    mut gizmos: Gizmos,
    mut carpet_gizmos: Gizmos<EgoCarpetGizmos>,
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
    let ego = interpolate_state(live.previous.ego, live.world.ego, render_alpha);
    let (position, yaw) = match live.camera.target {
        CameraTarget::Ego => (px(&ego), ego.yaw),
        CameraTarget::Track => {
            let path = Path::new(&live.world.road.centerline);
            let (s, _) = path.project(ego);
            let (position, yaw) = path.pose_at(s);
            (ppx(position), yaw)
        }
    };
    let target_center = live.camera.follow.then_some(position);
    let target_rotation = live
        .camera
        .align_heading
        .then_some(yaw as f32 - std::f32::consts::FRAC_PI_2);
    let blend = if live.camera.smooth {
        1.0 - (-CAMERA_SMOOTH_RATE * time.delta_secs()).exp()
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
        draw_grid(&mut gizmos, live.camera, &window);
    }

    let world = &live.world;
    if state.show_carpet && !world.plan.is_empty() {
        carpet::draw(&mut carpet_gizmos, ego, &world.plan, world.dt);
    }

    let xs =
        ((world.ego.x - 250.0) / 5.0).floor() as i64..=((world.ego.x + 750.0) / 5.0).ceil() as i64;
    let samples: Vec<_> = xs
        .map(|i| {
            let x = i as f64 * 5.0;
            let (position, yaw) = world.track.pose(x);
            (position, yaw, world.track.half_width(x))
        })
        .collect();
    for sign in [-1.0, 1.0] {
        gizmos.linestrip_2d(
            samples.iter().map(|&(position, yaw, width)| {
                ppx([
                    position[0] - sign * width * yaw.sin(),
                    position[1] + sign * width * yaw.cos(),
                ])
            }),
            Color::srgb(0.6, 0.6, 0.6),
        );
    }
    if state.show_centerline {
        gizmos.linestrip_2d(
            samples.iter().map(|&(position, _, _)| ppx(position)),
            Color::srgb(0.25, 0.5, 0.35),
        );
    }
    for &(position, yaw, width) in &samples {
        let offset = [width * yaw.sin(), -width * yaw.cos()];
        gizmos.line_2d(
            ppx([position[0] - offset[0], position[1] - offset[1]]),
            ppx([position[0] + offset[0], position[1] + offset[1]]),
            Color::srgba(0.6, 0.6, 0.6, 0.2),
        );
    }

    if state.show_diag_trajectories && state.planner.has_diagnostics() {
        for trajectory in &world.diagnostics.trajectories {
            diagnostic_trajectories
                .linestrip_2d(trajectory.iter().copied().map(ppx), DIAGNOSTIC_COLOR);
        }
    }
    if state.show_diag_points && state.planner.has_diagnostics() {
        for &point in &world.diagnostics.points {
            diagnostic_points.circle_2d(
                ppx(point),
                0.5 * DIAGNOSTIC_POINT_RADIUS_M * PX_PER_M,
                DIAGNOSTIC_COLOR,
            );
        }
    }

    if state.show_plan && !world.plan.is_empty() {
        gizmos.linestrip_2d(std::iter::once(&ego).chain(&world.plan).map(px), ACCENT);
    }
    draw_car(&mut gizmos, &ego, Color::srgb(0.08, 0.1, 0.1));
    for actor in &world.actors {
        let state = live
            .previous
            .actors
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
        draw_agent(
            &mut gizmos,
            &state,
            CAR_FOOTPRINT,
            Color::srgb(0.35, 0.38, 0.38),
        );
    }
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

fn draw_grid(gizmos: &mut Gizmos, camera: CameraState, window: &Window) {
    let extent = window.width().hypot(window.height()) / camera.zoom;
    let wide_grid = camera.zoom < 0.25;
    let spacing = if wide_grid { 50.0 } else { 10.0 } * PX_PER_M;
    let major_every = if wide_grid { 2 } else { 5 };
    let min_x = ((camera.center.x - extent) / spacing).floor() as i64;
    let max_x = ((camera.center.x + extent) / spacing).ceil() as i64;
    let min_y = ((camera.center.y - extent) / spacing).floor() as i64;
    let max_y = ((camera.center.y + extent) / spacing).ceil() as i64;
    for x in min_x..=max_x {
        let color = if x.rem_euclid(major_every) == 0 {
            Color::srgba(0.2, 0.48, 0.68, 0.11)
        } else {
            Color::srgba(0.2, 0.4, 0.55, 0.035)
        };
        let x = x as f32 * spacing;
        gizmos.line_2d(
            Vec2::new(x, camera.center.y - extent),
            Vec2::new(x, camera.center.y + extent),
            color,
        );
    }
    for y in min_y..=max_y {
        let color = if y.rem_euclid(major_every) == 0 {
            Color::srgba(0.2, 0.48, 0.68, 0.11)
        } else {
            Color::srgba(0.2, 0.4, 0.55, 0.035)
        };
        let y = y as f32 * spacing;
        gizmos.line_2d(
            Vec2::new(camera.center.x - extent, y),
            Vec2::new(camera.center.x + extent, y),
            color,
        );
    }
}
