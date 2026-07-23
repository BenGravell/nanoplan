use bevy_egui::egui;

use super::metrics::{metric, section_heading};
use crate::viewer::live::Live;

pub(super) fn show(ui: &mut egui::Ui, live: &Live) {
    section_heading(ui, "PLANNING");
    metric(
        ui,
        "LATEST PLAN",
        format!("{:.2} ms", live.world.last_plan_ms),
    );

    section_heading(ui, "FRAME");
    metric(
        ui,
        "WHOLE FRAME FPS",
        format!(
            "{:.1} FPS · {:.2} ms",
            live.frame_rate.fps(),
            live.frame_rate.milliseconds()
        ),
    );

    section_heading(ui, "LATENCY SEAMS");
    for seam in &live.latency.seams {
        metric(
            ui,
            seam.name,
            format!("mean {:.3} ms · max {:.3} ms", seam.mean_ms(), seam.max_ms),
        );
    }
}
