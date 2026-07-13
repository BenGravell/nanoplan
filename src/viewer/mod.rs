//! Interactive endless-track viewer.

use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use nanoplan::PlannerKind;

mod draw;
mod live;
mod ui;

pub(crate) const DT: f64 = 0.1;

#[derive(Resource)]
pub(crate) struct UiState {
    pub planner: PlannerKind,
    pub target_speed: f32,
    pub preview_s: f32,
    pub show_diag_points: bool,
    pub show_diag_trajectories: bool,
}

pub fn run() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "nanoplan".into(),
                fit_canvas_to_parent: true,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(UiState {
            planner: PlannerKind::BezierIdm,
            target_speed: 8.0,
            preview_s: 3.0,
            show_diag_points: false,
            show_diag_trajectories: false,
        })
        .init_non_send::<live::Live>()
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
        .add_systems(EguiPrimaryContextPass, ui::ui)
        .add_systems(Update, (live::update, live::draw).chain())
        .run();
}
