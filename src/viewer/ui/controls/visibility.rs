use bevy_egui::egui;

use crate::planning::PLANNING_HORIZON_S;

use super::super::super::colors::{DIM, ORANGE};
use super::super::style::caps_font;
use crate::viewer::{CarpetVisualization, UiState};

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState, compact: bool) {
    ui.checkbox(&mut state.show_grid, "Grid");
    ui.checkbox(&mut state.show_stations, "Track stations");
    ui.checkbox(&mut state.show_centerline, "Track centerline");

    ui.add_space(6.0);
    ui.label(
        egui::RichText::new("FUTURE")
            .font(caps_font(11.0))
            .color(DIM),
    );
    ui.add(
        egui::Slider::new(&mut state.preview_s, 0.0..=PLANNING_HORIZON_S as f32)
            .step_by(0.5)
            .text(if compact {
                "preview"
            } else {
                "future preview [s]"
            })
            .trailing_fill(true),
    );
    ui.checkbox(&mut state.show_carpet, "Ego carpet");
    let previous = state.carpet_visualization;
    ui.label("Ego carpet color");
    egui::ComboBox::from_id_salt("ego_carpet_color")
        .selected_text(option_label(state.carpet_visualization))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            for visualization in ALL_VISUALIZATIONS {
                ui.selectable_value(
                    &mut state.carpet_visualization,
                    visualization,
                    option_label(visualization),
                );
            }
        });
    if state.carpet_visualization != previous {
        state.show_carpet = true;
    }
    ui.checkbox(&mut state.show_plan, "Planned path");
    if state.planner.has_diagnostics() {
        ui.checkbox(&mut state.show_diag_points, "Search points");
        ui.checkbox(&mut state.show_diag_trajectories, "Candidate trajectories");
        if state.preview_s == 0.0 && (state.show_diag_points || state.show_diag_trajectories) {
            ui.colored_label(ORANGE, "Set future preview above zero to show diagnostics.");
        }
    }
}

const ALL_VISUALIZATIONS: [CarpetVisualization; 9] = [
    CarpetVisualization::Time,
    CarpetVisualization::Speed,
    CarpetVisualization::LongitudinalAcceleration,
    CarpetVisualization::LateralAcceleration,
    CarpetVisualization::Curvature,
    CarpetVisualization::Safety,
    CarpetVisualization::Progress,
    CarpetVisualization::Comfort,
    CarpetVisualization::Overall,
];

fn option_label(visualization: CarpetVisualization) -> &'static str {
    match visualization {
        CarpetVisualization::Speed => "Speed",
        CarpetVisualization::Time => "Time",
        CarpetVisualization::LongitudinalAcceleration => "Longitudinal acceleration",
        CarpetVisualization::LateralAcceleration => "Lateral acceleration",
        CarpetVisualization::Curvature => "Curvature",
        CarpetVisualization::Safety => "Safety",
        CarpetVisualization::Progress => "Progress",
        CarpetVisualization::Comfort => "Comfort",
        CarpetVisualization::Overall => "Overall",
    }
}
