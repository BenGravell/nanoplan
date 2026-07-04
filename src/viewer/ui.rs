//! The egui control panel: scenario/planner selection, the nuPlan
//! scenario-loading widget, sliders, and the metrics/latency tables.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use nanoplan::metrics::METRICS;
use nanoplan::{IncrementalSim, PlannerKind};

use super::loader::Loader;
use super::rollouts::{ActiveJob, RolloutCache};
use super::{DT, DURATION_S, PREVIEW_MAX_S, Scenarios, UiState};

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut scenes: ResMut<Scenarios>,
    mut state: ResMut<UiState>,
    cache: Res<RolloutCache>,
    mut job: NonSendMut<ActiveJob>,
    mut loader: NonSendMut<Loader>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let prev = (state.scenario, state.planner);
    egui::Window::new("nanoplan").show(ctx, |ui| {
        egui::ComboBox::from_label("scenario")
            .selected_text(&scenes.0[state.scenario].name)
            .show_ui(ui, |ui| {
                for (i, sc) in scenes.0.iter().enumerate() {
                    ui.selectable_value(&mut state.scenario, i, &sc.name);
                }
            });
        egui::ComboBox::from_label("planner")
            .selected_text(state.planner.name())
            .show_ui(ui, |ui| {
                for kind in PlannerKind::ALL {
                    ui.selectable_value(&mut state.planner, kind, kind.name());
                }
            });
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
    });
    if (state.scenario, state.planner) != prev {
        state.time_s = 0.0;
    }
    // future preview active: frame the whole screen in the accent color
    if state.preview_s > 0.0 {
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
