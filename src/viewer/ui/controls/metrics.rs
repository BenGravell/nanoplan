use crate::metrics::evaluate;
use bevy_egui::egui;

use super::super::super::colors::{DIM_TEXT, TEXT};
use super::super::style::caps_font;
use crate::viewer::live::Live;

pub(super) fn show(ui: &mut egui::Ui, live: &Live) {
    let score = preview_score(live);
    section_heading(ui, "PLANNER METRICS");
    metric(ui, "PROGRESS", format!("{:.1}%", score * 100.0));
}

pub(super) fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.add(egui::Label::new(egui::RichText::new(label).font(caps_font(11.0)).color(DIM_TEXT)).wrap());
    ui.add(egui::Label::new(egui::RichText::new(value).monospace()).wrap());
    ui.add_space(4.0);
}

pub(super) fn section_heading(ui: &mut egui::Ui, heading: &str) {
    ui.add_space(6.0);
    ui.add(egui::Label::new(egui::RichText::new(heading).font(caps_font(12.0)).color(TEXT)).wrap());
}

pub(crate) fn preview_score(live: &Live) -> f64 {
    let trajectory = &live.world.trajectory;
    evaluate(&trajectory.states, trajectory.dt, &live.world.road.path(), None)
}

#[cfg(test)]
#[path = "metrics_tests.rs"]
mod tests;
