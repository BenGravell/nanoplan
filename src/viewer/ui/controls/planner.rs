use crate::planning::PlannerKind;
use bevy_egui::egui;

use super::super::super::colors::DIM_TEXT;
use super::super::style::caps_font;
use crate::viewer::UiState;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState) {
    ui.label(
        egui::RichText::new("ACTIVE PLANNER")
            .font(caps_font(11.0))
            .color(DIM_TEXT),
    );
    egui::ComboBox::from_id_salt("planner")
        .selected_text(state.planner.name())
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            for kind in PlannerKind::ALL {
                ui.selectable_value(&mut state.planner, kind, kind.name());
            }
        });
}
