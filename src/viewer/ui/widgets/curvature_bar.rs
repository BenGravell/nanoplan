use bevy_egui::egui;
use nanoplan::math::signed_fraction;
use nanoplan::simulation::curvature_limit;

use super::super::super::colors::{BLUE, FAINT, TEXT};

pub(in crate::viewer::ui) fn draw(
    painter: &egui::Painter,
    track: egui::Rect,
    label_position: egui::Pos2,
    curvature: f64,
    speed: f64,
) {
    let zero = track.center().x;
    painter.rect_filled(track, 2.0, FAINT);
    painter.line_segment(
        [
            egui::pos2(zero, track.top() - 5.0),
            egui::pos2(zero, track.bottom() + 5.0),
        ],
        egui::Stroke::new(1.0, TEXT),
    );
    let limit = curvature_limit(speed) as f32;
    let fraction = signed_fraction(curvature as f32, limit, limit);
    let end = zero - fraction * track.width() / 2.0;
    painter.rect_filled(
        egui::Rect::from_x_y_ranges(end.min(zero)..=end.max(zero), track.y_range()),
        2.0,
        BLUE,
    );
    painter.text(
        label_position,
        egui::Align2::CENTER_TOP,
        format!("CURV {curvature:+.3}"),
        egui::FontId::monospace(10.0),
        TEXT,
    );
}
