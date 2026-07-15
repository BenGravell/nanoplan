use bevy_egui::egui;

use super::super::colors::TEXT;
use super::style::caps_font;
use crate::viewer::UiState;
use crate::viewer::live::Live;

mod camera;
pub(super) mod metrics;
mod planner;
mod visibility;

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum ControlTab {
    #[default]
    Planner,
    Camera,
    Visibility,
    Metrics,
}

pub(super) fn control_deck(
    ui: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
    compact: bool,
) {
    super::style::brand_header(ui, compact);
    ui.add_space(if compact { 4.0 } else { 12.0 });
    transport_controls(ui, state, live);
    ui.add_space(if compact { 6.0 } else { 9.0 });

    let tabs = [
        (ControlTab::Planner, "PLANNER"),
        (ControlTab::Camera, "CAMERA"),
        (ControlTab::Visibility, "VIZ"),
        (ControlTab::Metrics, "METRICS"),
    ];
    let columns = if compact { 2 } else { 4 };
    for row in tabs.chunks(columns) {
        let width = equal_button_width(ui, columns);
        ui.horizontal(|ui| {
            for &(tab, title) in row {
                let selected = *active_tab == tab;
                if ui
                    .add_sized(
                        [width, 32.0],
                        egui::Button::new(
                            egui::RichText::new(title)
                                .font(caps_font(12.0))
                                .color(if selected { egui::Color32::WHITE } else { TEXT }),
                        )
                        .selected(selected),
                    )
                    .clicked()
                {
                    *active_tab = tab;
                }
            }
        });
    }
    ui.add_space(if compact { 3.0 } else { 9.0 });

    egui::ScrollArea::vertical().show(ui, |ui| match *active_tab {
        ControlTab::Planner => planner::show(ui, state, compact),
        ControlTab::Camera => camera::show(ui, live),
        ControlTab::Visibility => visibility::show(ui, state),
        ControlTab::Metrics => metrics::show(ui, live),
    });
}

fn transport_controls(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live) {
    let width = equal_button_width(ui, 2);
    ui.horizontal(|ui| {
        let pause_label = if live.paused { "RESUME" } else { "PAUSE" };
        if ui
            .add_sized(
                [width, 36.0],
                egui::Button::new(egui::RichText::new(pause_label).size(13.0)),
            )
            .clicked()
        {
            live.toggle_pause();
        }
        if ui
            .add_sized(
                [width, 36.0],
                egui::Button::new(egui::RichText::new("↻ NEW TRACK").size(13.0)),
            )
            .clicked()
        {
            live.regenerate(live.seed + 1, state.planner);
        }
    });
}

fn equal_button_width(ui: &egui::Ui, count: usize) -> f32 {
    (ui.available_width() - ui.spacing().item_spacing.x * (count - 1) as f32) / count as f32
}
