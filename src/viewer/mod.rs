//! Interactive viewer: scrub through a simulated scenario and preview the
//! planned ego future and predicted actor motion.

use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use nanoplan::planning::PLANNING_HORIZON_S;
use nanoplan::{PlannerKind, Scenario, simulate};

mod draw;
mod rollouts;
mod scenarios;
mod ui;
#[cfg(target_arch = "wasm32")]
mod web;

use rollouts::{ActiveJob, RolloutCache, step_active_job};
#[cfg(not(target_arch = "wasm32"))]
use ui::ScenarioLoader;

pub(crate) const DT: f64 = 0.1;
pub(crate) const DURATION_S: f64 = 20.0;
pub(crate) const PREVIEW_MAX_S: f64 = PLANNING_HORIZON_S;

#[derive(Resource)]
pub(crate) struct Scenarios(pub Vec<Scenario>);

#[derive(Resource)]
pub(crate) struct UiState {
    pub scenario: usize,
    pub planner: PlannerKind,
    pub time_s: f32,
    pub preview_s: f32,
    /// Show the current planner's diagnostic sample points (only recorded
    /// while `preview_s > 0`, since that's the only replan collecting them).
    pub show_diag_points: bool,
    /// Show the current planner's diagnostic trajectories/connectors.
    pub show_diag_trajectories: bool,
}

pub fn run() {
    let scenes = scenarios::all_scenarios();
    let mut cache = RolloutCache::default();
    cache.0.insert(
        (0, PlannerKind::Straight),
        simulate(&scenes[0], PlannerKind::Straight, DURATION_S, DT),
    );
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "nanoplan".into(),
            // fill the browser window on wasm; no effect on desktop
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .insert_resource(Scenarios(scenes))
    .insert_resource(UiState {
        scenario: 0,
        planner: PlannerKind::Straight,
        time_s: 0.0,
        preview_s: 0.0,
        show_diag_points: false,
        show_diag_trajectories: false,
    })
    .insert_resource(cache)
    .init_non_send::<ActiveJob>()
    .add_systems(Startup, |mut commands: Commands| {
        commands.spawn(Camera2d);
    })
    .add_systems(EguiPrimaryContextPass, ui::ui)
    .add_systems(Update, (step_active_job, draw::draw).chain());
    #[cfg(not(target_arch = "wasm32"))]
    app.insert_resource(ScenarioLoader::default());
    #[cfg(target_arch = "wasm32")]
    app.init_non_send::<web::WebScenarioFetch>()
        .add_systems(Startup, web::spawn_fetch)
        .add_systems(Update, web::absorb_fetch);
    app.run();
}
