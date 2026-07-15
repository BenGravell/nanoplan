use crate::common::math::signed_fraction;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};
use bevy_egui::egui;

use super::super::super::colors::{BLUE, FAINT, RED, TEXT};

pub(in crate::viewer::ui) fn draw(
    painter: &egui::Painter,
    track: egui::Rect,
    label_position: egui::Pos2,
    acceleration: f64,
) {
    let zero = track.center().y;
    painter.rect_filled(track, 2.0, FAINT);
    painter.line_segment(
        [
            egui::pos2(track.left() - 5.0, zero),
            egui::pos2(track.right() + 5.0, zero),
        ],
        egui::Stroke::new(1.0, TEXT),
    );
    let fraction = signed_fraction(
        acceleration as f32,
        MAX_LON_ACCEL as f32,
        -MIN_LON_ACCEL as f32,
    );
    let end = zero - fraction * track.height() / 2.0;
    painter.rect_filled(
        egui::Rect::from_x_y_ranges(track.x_range(), end.min(zero)..=end.max(zero)),
        2.0,
        if fraction >= 0.0 { BLUE } else { RED },
    );
    painter.text(
        label_position,
        egui::Align2::LEFT_CENTER,
        format!("A {acceleration:+.1}"),
        egui::FontId::monospace(11.0),
        TEXT,
    );
}
