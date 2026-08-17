use crate::planning::{COMPUTE_BUDGET_BREAKPOINTS, PlannerKind};
use bevy_egui::egui;

use super::super::super::colors::DIM_TEXT;
use super::super::style::caps_font;
use super::super::widgets::breakpoint_slider;
use crate::viewer::UiState;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState, content_width: f32) {
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
    ui.add_space(10.0);
    ui.label(
        egui::RichText::new("COMPUTE BUDGET")
            .font(caps_font(11.0))
            .color(DIM_TEXT),
    );
    let budget = breakpoint_slider::show(
        ui,
        content_width,
        &mut state.compute_budget_percent,
        &COMPUTE_BUDGET_BREAKPOINTS,
        " %",
    )
    .on_hover_text("100% is a calibrated 100 ms allowance; lower values trade search quality for speed.");
    budget.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Slider, true, "Compute budget"));
}
