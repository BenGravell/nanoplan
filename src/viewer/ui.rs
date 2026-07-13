use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use nanoplan::PlannerKind;
use nanoplan::planning::PLANNING_HORIZON_S;

use super::UiState;
use super::live::Live;

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut live: NonSendMut<Live>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::Window::new("nanoplan").show(ctx, |ui| {
        egui::ComboBox::from_label("planner")
            .selected_text(state.planner.name())
            .show_ui(ui, |ui| {
                for kind in PlannerKind::ALL {
                    ui.selectable_value(&mut state.planner, kind, kind.name());
                }
            });
        ui.add(egui::Slider::new(&mut state.target_speed, 3.0..=13.0).text("target speed [m/s]"));
        ui.add(
            egui::Slider::new(&mut state.preview_s, 0.0..=PLANNING_HORIZON_S as f32)
                .text("future preview [s]"),
        );
        if state.planner.has_diagnostics() {
            ui.checkbox(&mut state.show_diag_points, "diagnostic points");
            ui.checkbox(&mut state.show_diag_trajectories, "diagnostic trajectories");
            if state.preview_s == 0.0 && (state.show_diag_points || state.show_diag_trajectories) {
                ui.label("diagnostics need a future preview");
            }
        }
        ui.horizontal(|ui| {
            ui.checkbox(&mut live.paused, "pause");
            if ui.button("new track").clicked() {
                let seed = live.seed + 1;
                live.regenerate(seed, state.planner);
            }
        });
        ui.label("scroll to zoom");
        ui.separator();
        ui.label(format!("speed: {:.1} m/s", live.world.ego.speed));
        ui.label(format!("latest plan: {:.1} ms", live.world.last_plan_ms));
        ui.label("planner latency");
        egui::Grid::new("latency").show(ui, |ui| {
            ui.label("seam");
            ui.label("mean [ms]");
            ui.label("max [ms]");
            ui.end_row();
            for seam in &live.latency.seams {
                ui.label(seam.name);
                ui.label(format!("{:.3}", seam.mean_ms()));
                ui.label(format!("{:.3}", seam.max_ms));
                ui.end_row();
            }
        });
    });
}
