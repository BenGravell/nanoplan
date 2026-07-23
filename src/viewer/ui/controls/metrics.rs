use crate::common::kinematics::TrajectoryKinematics;
use crate::metrics::{Metrics, evaluate};
use crate::prediction::predict;
use crate::simulation::State;
use crate::track::Path;
use bevy_egui::egui;

use super::super::super::colors::{DIM_TEXT, TEXT};
use super::super::style::caps_font;
use crate::viewer::live::Live;

pub(super) fn show(ui: &mut egui::Ui, live: &Live) {
    let metrics = preview_metrics(live);
    section_heading(ui, "PLANNER METRICS");
    for (label, score) in ["SAFETY", "PROGRESS", "COMFORT", "OVERALL"]
        .into_iter()
        .zip(metrics.aggregate.into_iter().chain([metrics.score]))
    {
        metric(ui, label, format!("{:.1}%", score * 100.0));
    }
}

pub(super) fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.add(
        egui::Label::new(
            egui::RichText::new(label)
                .font(caps_font(11.0))
                .color(DIM_TEXT),
        )
        .wrap(),
    );
    ui.add(egui::Label::new(egui::RichText::new(value).monospace()).wrap());
    ui.add_space(4.0);
}

pub(super) fn section_heading(ui: &mut egui::Ui, heading: &str) {
    ui.add_space(6.0);
    ui.add(
        egui::Label::new(
            egui::RichText::new(heading)
                .font(caps_font(12.0))
                .color(TEXT),
        )
        .wrap(),
    );
}

pub(crate) fn preview_metrics(live: &Live) -> Metrics {
    preview_metrics_for_trajectory(live, &live.world.trajectory)
}

pub(crate) fn preview_metrics_for_trajectory(
    live: &Live,
    trajectory: &TrajectoryKinematics,
) -> Metrics {
    let track = Path::new(live.world.road.centerline());
    let actors: Vec<Vec<State>> = live
        .world
        .actors
        .iter()
        .map(|actor| {
            trajectory
                .time
                .iter()
                .map(|&time| predict(&actor.state, &track, time))
                .collect()
        })
        .collect();
    evaluate(trajectory, &actors, &live.world.road)
}
