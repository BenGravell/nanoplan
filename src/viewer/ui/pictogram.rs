use bevy_egui::egui;

use super::super::colors::ORANGE;

#[derive(Clone, Copy)]
pub(super) enum Pictogram {
    Track,
    Planner,
    Preview,
    Diagnostics,
    Pause,
    Zoom,
}

pub(super) fn pictogram(ui: &mut egui::Ui, icon: Pictogram, size: f32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
    let center = rect.center();
    let radius = size * 0.34;
    let stroke = egui::Stroke::new((size * 0.065).max(1.3), ORANGE);
    ui.painter().circle_stroke(center, radius, stroke);

    match icon {
        Pictogram::Track => track(ui.painter(), center, radius, stroke),
        Pictogram::Planner => planner(ui.painter(), center, radius, stroke),
        Pictogram::Preview => preview(ui.painter(), center, radius, stroke),
        Pictogram::Diagnostics => diagnostics(ui.painter(), center, radius, stroke),
        Pictogram::Pause => pause(ui.painter(), center, radius, stroke),
        Pictogram::Zoom => zoom(ui.painter(), center, radius, stroke),
    }
}

fn track(painter: &egui::Painter, center: egui::Pos2, radius: f32, stroke: egui::Stroke) {
    painter.circle_stroke(center, radius * 0.55, stroke);
    painter.line_segment(
        [
            center + egui::vec2(0.0, -radius),
            center + egui::vec2(0.0, -radius * 0.55),
        ],
        stroke,
    );
}

fn planner(painter: &egui::Painter, center: egui::Pos2, radius: f32, stroke: egui::Stroke) {
    painter.line_segment([center + egui::vec2(-radius * 0.65, radius * 0.5), center], stroke);
    painter.line_segment([center, center + egui::vec2(radius * 0.65, -radius * 0.5)], stroke);
    painter.circle_filled(center, stroke.width * 1.25, ORANGE);
}

fn preview(painter: &egui::Painter, center: egui::Pos2, radius: f32, stroke: egui::Stroke) {
    painter.line_segment(
        [
            center + egui::vec2(-radius * 0.55, 0.0),
            center + egui::vec2(radius * 0.5, 0.0),
        ],
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(radius * 0.15, -radius * 0.35),
            center + egui::vec2(radius * 0.5, 0.0),
        ],
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(radius * 0.15, radius * 0.35),
            center + egui::vec2(radius * 0.5, 0.0),
        ],
        stroke,
    );
}

fn diagnostics(painter: &egui::Painter, center: egui::Pos2, radius: f32, stroke: egui::Stroke) {
    for offset in [egui::vec2(-0.45, 0.3), egui::vec2(0.0, -0.4), egui::vec2(0.5, 0.25)] {
        painter.circle_filled(center + offset * radius, stroke.width * 1.35, ORANGE);
    }
}

fn pause(painter: &egui::Painter, center: egui::Pos2, radius: f32, stroke: egui::Stroke) {
    for x in [-radius * 0.25, radius * 0.25] {
        painter.line_segment(
            [
                center + egui::vec2(x, -radius * 0.45),
                center + egui::vec2(x, radius * 0.45),
            ],
            stroke,
        );
    }
}

fn zoom(painter: &egui::Painter, center: egui::Pos2, radius: f32, stroke: egui::Stroke) {
    painter.circle_stroke(
        center + egui::vec2(-radius * 0.15, -radius * 0.15),
        radius * 0.38,
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(radius * 0.12, radius * 0.12),
            center + egui::vec2(radius * 0.55, radius * 0.55),
        ],
        stroke,
    );
}
