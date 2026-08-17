use bevy_egui::egui;

use super::super::super::colors::DIM_TEXT;
use super::super::style::caps_font;
use super::super::widgets::stacked_slider;
use crate::viewer::UiState;
use crate::viewer::live::Live;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live, content_width: f32) {
    ui.label(egui::RichText::new("OPPONENTS").font(caps_font(11.0)).color(DIM_TEXT));
    let value = state.opponents.to_string();
    let count = stacked_slider::show(
        ui,
        content_width,
        value,
        egui::Slider::new(&mut state.opponents, 0..=15).trailing_fill(true),
    );
    count.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Slider, true, "Opponent count"));
    if live.world.actors.len() != state.opponents {
        live.set_actor_count(state.opponents);
    }
}
