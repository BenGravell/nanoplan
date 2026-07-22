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
    for (label, score) in ["SAFETY", "PROGRESS", "COMFORT", "OVERALL"]
        .into_iter()
        .zip(metrics.aggregate.into_iter().chain([metrics.score]))
    {
        metric(ui, label, format!("{:.1}%", score * 100.0));
    }
    section_heading(ui, "DRIVING");
    let actuation = live.world.actuation();
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
    section_heading(ui, "PLANNER LATENCY SEAMS");
    for seam in &live.latency.seams {
        metric(
            ui,
            seam.name,
            format!("mean {:.3} ms · max {:.3} ms", seam.mean_ms(), seam.max_ms),
        );
    }
}

fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.add(egui::Label::new(egui::RichText::new(label).font(caps_font(11.0)).color(DIM)).wrap());
    ui.add(egui::Label::new(egui::RichText::new(value).monospace()).wrap());
    ui.add_space(4.0);
}

fn section_heading(ui: &mut egui::Ui, heading: &str) {
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
    let ego: Vec<State> = std::iter::once(live.world.ego())
        .chain(live.world.plan.iter().skip(1).copied())
        .collect();
    let controls = if live.world.plan_controls.is_empty() {
        vec![live.world.actuation()]
    } else {
        live.world.plan_controls.clone()
    };
    let track = Path::new(live.world.road.centerline());
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
    evaluate(&ego, &controls, &actors, &live.world.road)
}
