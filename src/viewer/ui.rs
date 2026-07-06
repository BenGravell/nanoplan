//! The egui control panel: the scrub/open-world mode switch, scenario and
//! planner selection, the scenario-loading widget, sliders, and the
//! metrics/latency tables.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use nanoplan::metrics::METRICS;
use nanoplan::{IncrementalSim, PlannerKind};

use super::live::Live;
use super::loader::Loader;
use super::rollouts::{ActiveJob, RolloutCache};
use super::{DT, DURATION_S, Mode, PREVIEW_MAX_S, Scenarios, UiState};

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut scenes: ResMut<Scenarios>,
    mut state: ResMut<UiState>,
    cache: Res<RolloutCache>,
    mut job: NonSendMut<ActiveJob>,
    mut loader: NonSendMut<Loader>,
    mut live: NonSendMut<Live>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let prev = (state.scenario, state.planner);
    egui::Window::new("nanoplan").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut state.mode, Mode::Scrub, "scenarios");
            ui.selectable_value(&mut state.mode, Mode::Live, "open world");
        });
        ui.separator();
        match state.mode {
            Mode::Scrub => scrub_panel(ui, &mut scenes, &mut state, &cache, &mut job, &mut loader),
            Mode::Live => live_panel(ui, &mut state, &mut live),
        }
    });
    // map clicks are goal placement in open-world mode — not while the
    // pointer is interacting with the panel itself
    state.pointer_over_ui = ctx.egui_wants_pointer_input() || ctx.is_pointer_over_egui();
    if (state.scenario, state.planner) != prev {
        state.time_s = 0.0;
    }
    // future preview active: frame the whole screen in the accent color
    if state.mode == Mode::Scrub && state.preview_s > 0.0 {
        let painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Foreground,
            egui::Id::new("preview_frame"),
        ));
        let accent = egui::Color32::from_rgb(255, 64, 217);
        painter.rect_stroke(
            ctx.content_rect(),
            0,
            egui::Stroke::new(10.0, accent),
            egui::StrokeKind::Inside,
        );
    }
}

fn planner_combo(ui: &mut egui::Ui, selected: &mut PlannerKind) {
    egui::ComboBox::from_label("planner")
        .selected_text(selected.name())
        .show_ui(ui, |ui| {
            for kind in PlannerKind::ALL {
                ui.selectable_value(selected, kind, kind.name());
            }
        });
}

/// The realtime open-world controls and readouts; the world itself is
/// advanced by `live::live_update`, not here.
fn live_panel(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live) {
    planner_combo(ui, &mut state.live_planner);
    ui.add(egui::Slider::new(&mut state.live_target_speed, 3.0..=13.0).text("cruise speed [m/s]"));
    ui.horizontal(|ui| {
        ui.checkbox(&mut live.paused, "pause");
        if ui.button("new map").clicked() {
            let (seed, planner) = (live.seed + 1, state.live_planner);
            live.regenerate(seed, planner);
        }
        if ui.button("clear goal").clicked() {
            live.world.clear_goal();
        }
    });
    ui.label("click the map to place the goal; scroll to zoom");
    ui.separator();
    let w = &live.world;
    egui::Grid::new("live_stats").show(ui, |ui| {
        ui.label("ego speed");
        ui.label(format!("{:.1} m/s", w.ego.speed));
        ui.end_row();
        ui.label("route to goal");
        ui.label(match w.remaining_m() {
            Some(m) => format!("{m:.0} m"),
            None => "no goal — stopped".into(),
        });
        ui.end_row();
        ui.label("plan latency");
        ui.label(format!("{:.1} ms", w.last_plan_ms));
        ui.end_row();
    });
}

fn scrub_panel(
    ui: &mut egui::Ui,
    scenes: &mut Scenarios,
    state: &mut UiState,
    cache: &RolloutCache,
    job: &mut ActiveJob,
    loader: &mut Loader,
) {
    egui::ComboBox::from_label("scenario")
        .selected_text(&scenes.0[state.scenario].name)
        .show_ui(ui, |ui| {
            for (i, sc) in scenes.0.iter().enumerate() {
                ui.selectable_value(&mut state.scenario, i, &sc.name);
            }
        });
    planner_combo(ui, &mut state.planner);
    // scenario loading: the platform source renders its own widget and
    // reports loads; merging and the status line are platform-independent
    if let Some(result) = loader.source.widget(ui) {
        loader.status = Some(match result {
            Ok(loaded) => {
                let n = loaded.len();
                state.scenario = scenes.0.len();
                scenes.0.extend(loaded);
                Ok(format!(
                    "loaded {n} scenario{}",
                    if n == 1 { "" } else { "s" }
                ))
            }
            Err(e) => Err(e),
        });
    }
    if let Some(status) = &loader.status {
        let (color, msg) = match status {
            Ok(msg) => (egui::Color32::from_rgb(120, 210, 140), msg),
            Err(msg) => (egui::Color32::from_rgb(230, 100, 100), msg),
        };
        ui.colored_label(color, msg);
    }
    ui.add(egui::Slider::new(&mut state.time_s, 0.0..=DURATION_S as f32).text("time [s]"));
    ui.add(
        egui::Slider::new(&mut state.preview_s, 0.0..=PREVIEW_MAX_S as f32)
            .text("future preview [s]"),
    );
    if state.planner.has_diagnostics() {
        ui.horizontal(|ui| {
            ui.checkbox(&mut state.show_diag_points, "diagnostic points");
            ui.checkbox(&mut state.show_diag_trajectories, "diagnostic trajectories");
        });
        if state.preview_s == 0.0 && (state.show_diag_points || state.show_diag_trajectories) {
            ui.label("(needs future preview > 0 — that's what replans and records them)");
        }
    }
    ui.separator();

    let key = (state.scenario, state.planner);
    match (cache.0.get(&key), &job.0) {
        (Some(rollout), _) => {
            let idx = (state.time_s as f64 / DT).round() as usize;
            let (tick_scores, tick_score) = rollout.metrics.at(idx);
            egui::Grid::new("metrics").show(ui, |ui| {
                ui.label("");
                ui.label("@t");
                ui.label("agg");
                ui.end_row();
                for ((spec, tick), avg) in METRICS
                    .iter()
                    .zip(tick_scores)
                    .zip(rollout.metrics.aggregate)
                {
                    ui.label(spec.label);
                    ui.label(format!("{tick:.2}"));
                    ui.label(format!("{avg:.2}"));
                    ui.end_row();
                }
                ui.strong("closed-loop score");
                ui.strong(format!("{tick_score:.2}"));
                ui.strong(format!("{:.2}", rollout.metrics.score));
                ui.end_row();
            });
            ui.separator();
            ui.label("planner latency");
            egui::Grid::new("latency").show(ui, |ui| {
                ui.label("seam");
                ui.label("mean [ms]");
                ui.label("max [ms]");
                ui.end_row();
                for seam in &rollout.latency.seams {
                    ui.label(seam.name);
                    ui.label(format!("{:.3}", seam.mean_ms()));
                    ui.label(format!("{:.3}", seam.max_ms));
                    ui.end_row();
                }
            });
        }
        (None, Some((active_key, sim))) if *active_key == key => {
            ui.add(egui::ProgressBar::new(sim.progress()).text("simulating…"));
        }
        (None, _) => {
            if ui.button("Simulate").clicked() {
                job.0 = Some((
                    key,
                    IncrementalSim::start(&scenes.0[key.0], key.1, DURATION_S, DT),
                ));
            }
        }
    }
}
