//! Interactive driving viewer.

use std::num::NonZeroU32;

use crate::planning::PlannerKind;
use bevy::camera::CameraOutputMode;
use bevy::prelude::*;
use bevy::render::camera::ExtractedCamera;
use bevy::render::error_handler::{RenderError, RenderErrorHandler, RenderErrorPolicy};
use bevy::render::{
    Extract, ExtractSchedule, RenderApp, camera::extract_cameras, view::window::ExtractedWindows,
};
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

mod color_conversion;
mod colors;
mod live;
mod ui;

#[cfg(test)]
use colors::CANVAS_RGB;
use colors::NON_DRIVABLE_RGB;

pub(crate) const DT: f64 = 0.1;
const VIEW_MSAA: Msaa = Msaa::Sample4;
const RESIZE_DEBOUNCE_SECONDS: f32 = 0.2;
pub(crate) const MIN_VIEWPORT_WIDTH: f32 = 667.0;
pub(crate) const MIN_VIEWPORT_ASPECT_RATIO: f32 = 16.0 / 9.0;

#[derive(Resource, Default)]
pub(crate) struct DrivingCanvas {
    pub(crate) rect: Option<Rect>,
}

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum CarpetVisualization {
    #[default]
    Time,
    Speed,
    LongitudinalAcceleration,
    LateralAcceleration,
    Curvature,
    Safety,
    Progress,
    Comfort,
    Overall,
}

impl CarpetVisualization {
    pub(crate) fn is_metric(self) -> bool {
        matches!(
            self,
            Self::Safety | Self::Progress | Self::Comfort | Self::Overall
        )
    }
}

#[derive(Resource)]
pub(crate) struct UiState {
    pub(crate) started: bool,
    pub(crate) tutorial: bool,
    pub(crate) track: usize,
    pub(crate) planner: PlannerKind,
    pub(crate) preview_s: f32,
    pub(crate) opponents: usize,
    pub(crate) show_grid: bool,
    pub(crate) show_stations: bool,
    pub(crate) show_centerline: bool,
    pub(crate) show_carpet: bool,
    pub(crate) show_plan: bool,
    pub(crate) carpet_visualization: CarpetVisualization,
    pub(crate) show_diag_points: bool,
    pub(crate) show_diag_trajectories: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            started: false,
            tutorial: false,
            track: 0,
            planner: PlannerKind::Basic,
            preview_s: 3.0,
            opponents: 5,
            show_grid: true,
            show_stations: true,
            show_centerline: false,
            show_carpet: true,
            show_plan: false,
            carpet_visualization: CarpetVisualization::Time,
            show_diag_points: false,
            show_diag_trajectories: false,
        }
    }
}

pub(crate) fn run() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "nanoplan".into(),
                fit_canvas_to_parent: true,
                recognize_pinch_gesture: true,
                desired_maximum_frame_latency: NonZeroU32::new(1),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(ResizeDebouncePlugin)
        .insert_resource(RenderErrorHandler(recover_failed_resize))
        .add_plugins(EguiPlugin::default())
        .init_gizmo_group::<live::EgoCarpetGizmos>()
        .init_gizmo_group::<live::PlannedTrajectoryGizmos>()
        .init_gizmo_group::<live::DiagnosticTrajectoryGizmos>()
        .init_gizmo_group::<live::DiagnosticPointGizmos>()
        .insert_resource(ClearColor(Color::srgb_u8(
            NON_DRIVABLE_RGB.0,
            NON_DRIVABLE_RGB.1,
            NON_DRIVABLE_RGB.2,
        )))
        .init_resource::<UiState>()
        .init_resource::<DrivingCanvas>()
        .init_non_send::<live::Live>()
        .add_systems(
            Startup,
            (
                |mut commands: Commands| {
                    commands.spawn((Camera2d, VIEW_MSAA));
                },
                live::setup_grid,
                live::setup_road_surface,
            ),
        )
        .add_systems(EguiPrimaryContextPass, ui::ui)
        .add_systems(
            Update,
            (
                live::camera_input,
                live::update,
                live::configure_carpet,
                live::configure_diagnostics,
                live::configure_plan,
                live::draw,
            )
                .chain()
                .run_if(driving),
        )
        .run();
}

struct ResizeDebouncePlugin;

impl Plugin for ResizeDebouncePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ResizeDebounce>()
            .add_systems(Update, debounce_resize);
        app.sub_app_mut(RenderApp).add_systems(
            ExtractSchedule,
            hold_surface_size_during_resize.after(extract_cameras),
        );
    }
}

#[derive(Resource, Default)]
struct ResizeDebounce {
    displayed: UVec2,
    observed: UVec2,
    fallback: UVec2,
    quiet_for: f32,
}

impl ResizeDebounce {
    fn observe(&mut self, size: UVec2, delta_seconds: f32) -> bool {
        if self.displayed == UVec2::ZERO {
            self.displayed = size;
            self.observed = size;
            self.fallback = size;
        } else if size != self.observed {
            if self.observed == self.displayed {
                self.fallback = self.displayed;
            }
            self.observed = size;
            self.quiet_for = 0.0;
        } else if self.observed != self.displayed {
            self.quiet_for += delta_seconds;
            if self.quiet_for >= RESIZE_DEBOUNCE_SECONDS {
                self.displayed = self.observed;
            }
        }

        self.observed != self.displayed
    }

    fn rollback(&mut self) -> Option<UVec2> {
        (self.fallback != UVec2::ZERO && self.fallback != self.displayed).then(|| {
            self.displayed = self.fallback;
            self.observed = self.fallback;
            self.quiet_for = 0.0;
            self.fallback
        })
    }
}

fn debounce_resize(
    time: Res<Time>,
    window: Single<&Window>,
    mut camera: Single<&mut Camera>,
    mut resize: ResMut<ResizeDebounce>,
) {
    let size = UVec2::new(
        window.resolution.physical_width().max(1),
        window.resolution.physical_height().max(1),
    );
    camera.is_active = !resize.observe(size, time.delta_secs());
}

fn hold_surface_size_during_resize(
    resize: Extract<Res<ResizeDebounce>>,
    mut windows: ResMut<ExtractedWindows>,
    mut cameras: Query<&mut ExtractedCamera>,
) {
    if resize.observed == resize.displayed {
        return;
    }
    for mut camera in &mut cameras {
        camera.output_mode = CameraOutputMode::Skip;
    }
    if let Some(window) = windows.primary.and_then(|entity| windows.get_mut(&entity)) {
        window.physical_width = resize.displayed.x;
        window.physical_height = resize.displayed.y;
        window.size_changed = false;
    }
}

fn recover_failed_resize(
    error: &RenderError,
    main_world: &mut World,
    _render_world: &mut World,
) -> RenderErrorPolicy {
    let fallback = main_world.resource_mut::<ResizeDebounce>().rollback();
    let disabled_msaa =
        error.ty == bevy::render::error_handler::ErrorType::OutOfMemory && disable_msaa(main_world);
    if fallback.is_none() && !disabled_msaa {
        error!(
            "Rendering stopped after unrecoverable {:?} error; the app will remain open",
            error.ty
        );
        return RenderErrorPolicy::StopRendering;
    }

    if let Some(fallback) = fallback {
        let mut windows = main_world.query_filtered::<&mut Window, With<PrimaryWindow>>();
        if let Ok(mut window) = windows.single_mut(main_world) {
            window
                .resolution
                .set_physical_resolution(fallback.x, fallback.y);
        }
    }
    let mut cameras = main_world.query::<&mut Camera>();
    for mut camera in cameras.iter_mut(main_world) {
        camera.is_active = true;
    }
    match fallback {
        Some(fallback) => warn!(
            "Discarding failed {:?} resize and restoring framebuffer size {}x{}{}",
            error.ty,
            fallback.x,
            fallback.y,
            if disabled_msaa {
                " without multisampling"
            } else {
                ""
            }
        ),
        None => warn!("GPU memory is too limited for MSAA; disabling multisampling"),
    }

    // The resize only invalidates allocations made for this frame. Recreating the
    // whole renderer here briefly needs a second GPU device and can itself OOM.
    RenderErrorPolicy::Ignore
}

fn disable_msaa(world: &mut World) -> bool {
    let mut changed = false;
    let mut cameras = world.query::<(&mut Camera, &mut Msaa)>();
    for (mut camera, mut msaa) in cameras.iter_mut(world) {
        if *msaa != Msaa::Off {
            *msaa = Msaa::Off;
            camera.is_active = true;
            changed = true;
        }
    }
    changed
}

fn driving(window: Single<&Window>, state: Res<UiState>) -> bool {
    state.started && viewport_supported(window.width(), window.height())
}

fn viewport_supported(width: f32, height: f32) -> bool {
    width >= MIN_VIEWPORT_WIDTH && width / height >= MIN_VIEWPORT_ASPECT_RATIO
}
