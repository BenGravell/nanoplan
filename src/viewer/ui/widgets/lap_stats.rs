//! Current-session lap timing for the driving HUD.

use bevy_egui::egui;

use crate::viewer::colors::{DIM, TEXT};
use crate::viewer::live::LapStats;

use super::super::style::caps_font;

pub(crate) fn draw(painter: &egui::Painter, rect: egui::Rect, stats: LapStats) {
    let scale = rect.height() / 121.0;
    painter.text(
        rect.right_top(),
        egui::Align2::RIGHT_TOP,
        "LAP STATS",
        caps_font(10.0 * scale),
        DIM,
    );

    let label_font = caps_font(8.0 * scale);
    let value_font = egui::FontId::monospace(9.0 * scale);
    let first_row = 17.0 * scale;
    let row_step = 15.0 * scale;
    for (row, (label, value)) in [
        ("CURRENT", format_time(Some(stats.current_s))),
        ("PREVIOUS", format_time(stats.previous_s)),
        ("BEST", format_time(stats.best_s)),
        ("LAPS", stats.completed.to_string()),
    ]
    .into_iter()
    .enumerate()
    {
        let y = rect.top() + first_row + row as f32 * row_step;
        painter.text(
            egui::pos2(rect.left(), y),
            egui::Align2::LEFT_TOP,
            label,
            label_font.clone(),
            DIM,
        );
        painter.text(
            egui::pos2(rect.right(), y),
            egui::Align2::RIGHT_TOP,
            value,
            value_font.clone(),
            TEXT,
        );
    }
}

fn format_time(seconds: Option<f64>) -> String {
    let Some(seconds) = seconds else {
        return "--:--.-".to_owned();
    };
    let tenths = (seconds.max(0.0) * 10.0).floor() as u64;
    format!("{}:{:02}.{}", tenths / 600, (tenths / 10) % 60, tenths % 10)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_format_is_stable_and_compact() {
        assert_eq!(format_time(None), "--:--.-");
        assert_eq!(format_time(Some(0.0)), "0:00.0");
        assert_eq!(format_time(Some(125.69)), "2:05.6");
    }
}
