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
    ui.add_space(6.0);
    if state.planner.supports_post_optimization() {
        ui.checkbox(&mut state.trajectory_optimization, "Trajectory optimization (iLQR)")
            .on_hover_text("Refine the generated trajectory. Adds planning time; keeps the original if refinement is unsafe or worse.");
    } else {
        ui.add_enabled(false, egui::Checkbox::new(&mut true, "Trajectory optimization (iLQR)"))
            .on_disabled_hover_text("This planner already includes iLQR.");
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use egui_kittest::{Harness, kittest::Queryable};

    #[test]
    fn trajectory_optimization_toggle_preserves_the_selected_generator() {
        let mut configured = false;
        let mut harness = Harness::builder().with_size(egui::vec2(320.0, 360.0)).build_ui_state(
            move |ui, state: &mut UiState| {
                if !configured {
                    crate::viewer::ui::style::configure(ui.ctx());
                    configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                show(ui, state, 280.0);
            },
            UiState::default(),
        );
        harness.run();
        let kind = harness.state().planner;
        assert!(!harness.state().trajectory_optimization);
        harness.get_by_label("Trajectory optimization (iLQR)").click();
        harness.run();
        assert!(harness.state().trajectory_optimization);
        assert_eq!(harness.state().planner, kind);
        harness.get_by_label("Trajectory optimization (iLQR)").click();
        harness.run();
        assert!(!harness.state().trajectory_optimization);
    }
}
