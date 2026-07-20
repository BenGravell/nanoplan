use crate::metrics::{Metrics, evaluate};
use crate::prediction::predict;
use crate::simulation::State;
use crate::track::Path;
use bevy_egui::egui;

use super::super::super::colors::{DIM, TEXT};
use super::super::style::caps_font;
use crate::viewer::live::Live;

pub(super) fn show(ui: &mut egui::Ui, live: &Live) {
    let metrics = preview_metrics(live);
    section_heading(ui, "PLANNER METRICS");
    egui::Grid::new("planner_metrics")
        .num_columns(2)
        .spacing(egui::vec2(28.0, 7.0))
        .show(ui, |ui| {
            for (label, score) in ["SAFETY", "PROGRESS", "COMFORT", "OVERALL"]
                .into_iter()
                .zip(metrics.aggregate.into_iter().chain([metrics.score]))
            {
                metric(ui, label, format!("{:.1}%", score * 100.0));
            }
        });
    section_heading(ui, "DRIVING");
    let actuation = live.world.actuation();
    egui::Grid::new("live_metrics")
        .num_columns(2)
        .spacing(egui::vec2(28.0, 7.0))
        .show(ui, |ui| {
            metric(ui, "SPEED", format!("{:.1} m/s", live.world.ego().speed));
            metric(
                ui,
                "ACCELERATION",
                format!("{:+.2} m/s²", actuation.acceleration),
            );
            metric(ui, "CURVATURE", format!("{:+.4} m⁻¹", actuation.curvature));
            metric(
                ui,
                "LATEST PLAN",
                format!("{:.2} ms", live.world.last_plan_ms),
            );
        });
    section_heading(ui, "PLANNER LATENCY SEAMS");
    egui::Grid::new("latency")
        .num_columns(2)
        .spacing(egui::vec2(28.0, 7.0))
        .show(ui, |ui| {
            for seam in &live.latency.seams {
                metric(
                    ui,
                    seam.name,
                    format!("mean {:.3} ms · max {:.3} ms", seam.mean_ms(), seam.max_ms),
                );
            }
        });
}

fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(egui::RichText::new(label).font(caps_font(11.0)).color(DIM));
    ui.monospace(value);
    ui.end_row();
}

fn section_heading(ui: &mut egui::Ui, heading: &str) {
    ui.add_space(6.0);
    ui.label(
        egui::RichText::new(heading)
            .font(caps_font(12.0))
            .color(TEXT),
    );
}

pub(crate) fn preview_metrics(live: &Live) -> Metrics {
    let ego: Vec<State> = std::iter::once(live.world.ego())
        .chain(live.world.plan.iter().skip(1).copied())
        .collect();
    let track = Path::new(&live.world.road.centerline);
    let actors: Vec<Vec<State>> = live
        .world
        .actors
        .iter()
        .map(|actor| {
            (0..ego.len())
                .map(|i| predict(&actor.state, &track, i as f64 * live.world.dt()))
                .collect()
        })
        .collect();
    evaluate(&ego, &actors, &live.world.road)
}
