use bevy_egui::egui;

use super::super::super::colors::DIM_TEXT;
use super::super::style::caps_font;
use super::super::widgets::stacked_slider;
use crate::viewer::UiState;
use crate::viewer::live::Live;
use crate::world::MAX_ACTORS;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live, content_width: f32) {
    ui.label(egui::RichText::new("OPPONENTS").font(caps_font(11.0)).color(DIM_TEXT));
    let value = state.opponents.to_string();
    let count = stacked_slider::show(
        ui,
        content_width,
        value,
        egui::Slider::new(&mut state.opponents, 0..=MAX_ACTORS).trailing_fill(true),
    );
    stacked_slider::paint_ticks(ui, &count, MAX_ACTORS + 1, state.opponents);
    count.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Slider, true, "Opponent count"));
    if live.world.actors.len() != state.opponents {
        live.set_actor_count(state.opponents);
    }
}

#[cfg(test)]
#[path = "opponents_tests.rs"]
mod tests;
