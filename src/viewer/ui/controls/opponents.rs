use bevy_egui::egui;

use super::super::super::colors::DIM;
use super::super::style::caps_font;
use crate::viewer::UiState;
use crate::viewer::live::Live;

pub(super) fn show(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live) {
    ui.label(
        egui::RichText::new("OPPONENTS")
            .font(caps_font(11.0))
            .color(DIM),
    );

    ui.add(
        egui::Slider::new(&mut state.opponents, 0..=15)
            .text("count")
            .trailing_fill(true),
    );
    if live.world.actors.len() != state.opponents {
        live.regenerate_with_actor_count(
            live.seed,
            state.planner,
            state.track,
            state.opponents,
        );
    }
}
