use crate::track::{GENERATED_TRACK_NAME, TRACK_CATALOG};
use bevy_egui::egui;

use super::super::colors::TEXT;
use super::style::caps_font;
use crate::viewer::UiState;
use crate::viewer::live::Live;

mod camera;
pub(crate) mod metrics;
mod planner;
mod opponents;
mod visibility;

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum ControlTab {
    #[default]
    Planner,
    Opponents,
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
    transport_controls(ui, state, live);
    ui.add_space(if compact { 6.0 } else { 9.0 });

    let tabs = if compact {
        [
            Some((ControlTab::Planner, "PLANNER")),
            Some((ControlTab::Camera, "CAMERA")),
            Some((ControlTab::Opponents, "OPPONENTS")),
            Some((ControlTab::Visibility, "VIZ")),
            None,
            Some((ControlTab::Metrics, "METRICS")),
        ]
    } else {
        [
            Some((ControlTab::Planner, "PLANNER")),
            Some((ControlTab::Opponents, "OPPONENTS")),
            Some((ControlTab::Camera, "CAMERA")),
            Some((ControlTab::Visibility, "VIZ")),
            None,
            Some((ControlTab::Metrics, "METRICS")),
        ]
    };
    let columns = if compact { 2 } else { 3 };
    for row in tabs.chunks(columns) {
        let width = equal_button_width(ui, columns);
        ui.horizontal(|ui| {
            for entry in row {
                let Some((tab, title)) = entry else {
                    ui.allocate_space(egui::vec2(width, 32.0));
                    continue;
                };
                let selected = *active_tab == *tab;
                if ui
                    .add_sized(
                        [width, 32.0],
                        egui::Button::new(
                            egui::RichText::new(*title)
                                .font(caps_font(12.0))
                                .color(if selected { egui::Color32::WHITE } else { TEXT }),
                        )
                        .selected(selected),
                    )
                    .clicked()
                {
                    *active_tab = *tab;
                }
            }
        });
    }
    ui.add_space(if compact { 3.0 } else { 9.0 });

    egui::ScrollArea::vertical().show(ui, |ui| match *active_tab {
        ControlTab::Planner => planner::show(ui, state),
        ControlTab::Opponents => opponents::show(ui, state, live),
        ControlTab::Camera => camera::show(ui, live),
        ControlTab::Visibility => visibility::show(ui, state, compact),
        ControlTab::Metrics => metrics::show(ui, live),
    });
}

fn transport_controls(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live) {
    let previous_track = state.track;
    egui::ComboBox::from_label("TRACK")
        .selected_text(if state.track == 0 {
            GENERATED_TRACK_NAME
        } else {
            TRACK_CATALOG[state.track - 1].name
        })
        .width(ui.available_width() - 64.0)
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut state.track, 0, GENERATED_TRACK_NAME);
            for (index, track) in TRACK_CATALOG.iter().enumerate() {
                ui.selectable_value(&mut state.track, index + 1, track.name);
            }
        });
    if state.track != previous_track {
        live.regenerate_with_actor_count(
            live.seed,
            state.planner,
            state.track,
            state.opponents,
        );
    }
    ui.add_space(6.0);

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
            live.regenerate_with_actor_count(
                live.seed + 1,
                state.planner,
                state.track,
                state.opponents,
            );
        }
    });
}

fn equal_button_width(ui: &egui::Ui, count: usize) -> f32 {
    (ui.available_width() - ui.spacing().item_spacing.x * (count - 1) as f32) / count as f32
}
