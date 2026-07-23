use crate::track::{GENERATED_TRACK_NAME, TRACK_CATALOG, TRACK_PRESETS};
use bevy_egui::egui;

use crate::viewer::UiState;
use crate::viewer::live::Live;

mod camera;
pub(crate) mod metrics;
mod opponents;
mod planner;
mod timing;
mod visibility;

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum ControlTab {
    Track,
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
) {
    let selector = egui::ComboBox::from_id_salt("control_tab")
        .selected_text(active_tab.label())
        .width(ui.available_width())
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

    let content_width = ui.available_width();
    egui::ScrollArea::vertical()
        .max_width(content_width)
        .show(ui, |ui| {
            ui.set_width(content_width);
            match *active_tab {
                ControlTab::Track => track_controls(ui, state, live, compact),
                ControlTab::Planner => planner::show(ui, state),
                ControlTab::Opponents => opponents::show(ui, state, live),
                ControlTab::Camera => camera::show(ui, live, compact, content_width),
                ControlTab::Visibility => visibility::show(ui, state, compact, content_width),
                ControlTab::Metrics => metrics::show(ui, live),
                ControlTab::Timing => timing::show(ui, live),
            }
        });
}

impl ControlTab {
    const ALL: [Self; 7] = [
        Self::Track,
        Self::Planner,
        Self::Opponents,
        Self::Camera,
        Self::Visibility,
        Self::Metrics,
        Self::Timing,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::Track => "TRACK",
            Self::Planner => "PLANNER",
            Self::Opponents => "OPPONENTS",
            Self::Camera => "CAMERA",
            Self::Visibility => "VIZ",
            Self::Metrics => "METRICS",
            Self::Timing => "TIMING",
        }
    }
}

fn track_controls(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live, compact: bool) {
    let previous_track = state.track;
    egui::ComboBox::from_id_salt("track")
        .selected_text(if compact && state.track == 0 {
            "Generated"
        } else {
            track_name(state.track)
        })
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut state.track, 0, GENERATED_TRACK_NAME);
            for (index, track) in TRACK_PRESETS.iter().enumerate() {
                ui.selectable_value(&mut state.track, index + 1, track.name);
            }
            for (index, track) in TRACK_CATALOG.iter().enumerate() {
                ui.selectable_value(
                    &mut state.track,
                    index + TRACK_PRESETS.len() + 1,
                    track.name,
                );
            }
        });
    if state.track != previous_track {
        live.regenerate_with_actor_count(live.seed, state.planner, state.track, state.opponents);
    }
    if state.track == 0 {
        ui.add_space(6.0);

        let response = ui.add_sized(
            [ui.available_width(), 36.0],
            egui::Button::new(egui::RichText::new("↻ NEW TRACK").size(13.0)),
        );
        if response.clicked() {
            live.regenerate_with_actor_count(
                live.seed + 1,
                state.planner,
                state.track,
                state.opponents,
            );
        }
    }
}

fn track_name(index: usize) -> &'static str {
    if index == 0 {
        GENERATED_TRACK_NAME
    } else if index <= TRACK_PRESETS.len() {
        TRACK_PRESETS[index - 1].name
    } else {
        TRACK_CATALOG[index - TRACK_PRESETS.len() - 1].name
    }
}
