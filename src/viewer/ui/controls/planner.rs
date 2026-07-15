use bevy_egui::egui;
use nanoplan::PlannerKind;
use nanoplan::planning::PLANNING_HORIZON_S;

use super::super::super::colors::DIM;
use super::super::style::caps_font;
use crate::viewer::UiState;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState, compact: bool) {
    ui.label(
        egui::RichText::new("ACTIVE PLANNER")
            .font(caps_font(11.0))
            .color(DIM),
    );
    egui::ComboBox::from_id_salt("planner")
        .selected_text(state.planner.name())
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            for kind in PlannerKind::ALL {
                ui.selectable_value(&mut state.planner, kind, kind.name());
            }
        });
    ui.add_space(6.0);
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
}
