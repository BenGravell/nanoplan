//! Interactive driving viewer.

use crate::planning::PlannerKind;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

mod colors;
mod live;
mod ui;

pub(crate) const DT: f64 = 0.1;
const CANVAS_RGB: (u8, u8, u8) = (237, 242, 235);

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum TimeseriesGroup {
    #[default]
    Signals,
    Metrics,
}

#[derive(Resource)]
pub(crate) struct UiState {
    pub(crate) track: usize,
    pub(crate) planner: PlannerKind,
    pub(crate) preview_s: f32,
    pub(crate) show_grid: bool,
    pub(crate) show_stations: bool,
    pub(crate) show_centerline: bool,
    pub(crate) show_carpet: bool,
    pub(crate) show_plan: bool,
    pub(crate) timeseries_group: TimeseriesGroup,
    pub(crate) show_diag_points: bool,
    pub(crate) show_diag_trajectories: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            track: 0,
            planner: PlannerKind::BezierToppra,
            preview_s: 3.0,
            show_grid: true,
            show_stations: true,
            show_centerline: false,
            show_carpet: true,
            show_plan: false,
            timeseries_group: TimeseriesGroup::Signals,
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
                resize_constraints: WindowResizeConstraints {
                    min_width: 568.0,
                    min_height: 320.0,
                    ..default()
                },
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .init_gizmo_group::<live::EgoCarpetGizmos>()
        .init_gizmo_group::<live::PlannedTrajectoryGizmos>()
        .init_gizmo_group::<live::DiagnosticTrajectoryGizmos>()
        .init_gizmo_group::<live::DiagnosticPointGizmos>()
        .insert_resource(ClearColor(Color::srgb_u8(
            CANVAS_RGB.0,
            CANVAS_RGB.1,
            CANVAS_RGB.2,
        )))
        .init_resource::<UiState>()
        .init_non_send::<live::Live>()
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
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
                .run_if(landscape),
        )
        .run();
}

fn landscape(window: Single<&Window>) -> bool {
    !is_portrait(window.width(), window.height())
}

fn is_portrait(width: f32, height: f32) -> bool {
    height > width
}
