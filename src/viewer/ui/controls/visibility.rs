use bevy_egui::egui;

use super::super::super::colors::RED;
use crate::viewer::UiState;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState) {
    ui.checkbox(&mut state.show_grid, "Grid");
    ui.checkbox(&mut state.show_stations, "Track stations");
    ui.checkbox(&mut state.show_centerline, "Track centerline");
    ui.checkbox(&mut state.show_carpet, "Ego carpet");
    ui.checkbox(&mut state.show_plan, "Planned path");
    if state.planner.has_diagnostics() {
        ui.checkbox(&mut state.show_diag_points, "Search points");
        ui.checkbox(&mut state.show_diag_trajectories, "Candidate trajectories");
        if state.preview_s == 0.0 && (state.show_diag_points || state.show_diag_trajectories) {
            ui.colored_label(RED, "Set future preview above zero to record diagnostics.");
        }
    }
}
