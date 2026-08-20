use bevy_egui::egui;

use crate::viewer::UiState;
use crate::viewer::live::Live;
use crate::viewer::ui::style::scroll_area;

mod camera;
pub(crate) mod metrics;
mod opponents;
mod planner;
mod timing;
mod visibility;

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum ControlTab {
    #[default]
    Planner,
    Opponents,
    Camera,
    Visibility,
    Metrics,
    Timing,
}

pub(super) fn control_deck(
    ui: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
    compact: bool,
    content_width: f32,
) {
    ui.set_max_width(content_width);
    let selector = egui::ComboBox::from_id_salt("control_tab")
        .selected_text(active_tab.label())
        .width(content_width)
        .height(ui.available_height())
        .show_ui(ui, |ui| {
            for tab in ControlTab::ALL {
                ui.selectable_value(active_tab, tab, tab.label());
            }
        });
    selector
        .response
        .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::ComboBox, true, "OPTIONS"));
    ui.add_space(if compact { 6.0 } else { 9.0 });

    scroll_area(ui, egui::ScrollArea::vertical().max_width(content_width), |ui| {
        ui.set_width(content_width);
        match *active_tab {
            ControlTab::Planner => planner::show(ui, state, content_width),
            ControlTab::Opponents => opponents::show(ui, state, live, content_width),
            ControlTab::Camera => camera::show(ui, live, compact, content_width),
            ControlTab::Visibility => visibility::show(ui, state, compact, content_width),
            ControlTab::Metrics => metrics::show(ui, live),
            ControlTab::Timing => timing::show(ui, live),
        }
    });
}

impl ControlTab {
    const ALL: [Self; 6] = [
        Self::Planner,
        Self::Opponents,
        Self::Camera,
        Self::Visibility,
        Self::Metrics,
        Self::Timing,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::Planner => "PLANNER",
            Self::Opponents => "OPPONENTS",
            Self::Camera => "CAMERA",
            Self::Visibility => "VIZ",
            Self::Metrics => "METRICS",
            Self::Timing => "TIMING",
        }
    }
}
