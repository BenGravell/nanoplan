use bevy_egui::egui;

use crate::viewer::colors::ORANGE;

const BACKGROUND_ASPECT_RATIO: f32 = 16.0 / 9.0;

pub(crate) fn paint_chevron(ui: &egui::Ui, screen: egui::Rect, center_y: f32, time: f64) {
    let (offset, scale) = chevron_animation(time);
    let center_x = 0.06 + offset;
    let half_width = 0.004 * scale;
    let half_height = 0.011 * scale;
    ui.painter().add(egui::Shape::line(
        [
            normalized_pos(screen, center_x - half_width, center_y - half_height),
            normalized_pos(screen, center_x + half_width, center_y),
            normalized_pos(screen, center_x - half_width, center_y + half_height),
        ]
        .to_vec(),
        egui::Stroke::new(screen.height() * 0.005 * scale, ORANGE),
    ));
}

pub(crate) fn chevron_animation(time: f64) -> (f32, f32) {
    let sine = (time as f32 * std::f32::consts::TAU * 0.75).sin();
    let pulse = sine.signum() * (1.0 - (1.0 - sine.abs()).powi(2));
    (pulse * 0.003_5, 1.0 + pulse * 0.12)
}

fn normalized_pos(screen: egui::Rect, x: f32, y: f32) -> egui::Pos2 {
    screen.left_top() + egui::vec2(screen.height() * BACKGROUND_ASPECT_RATIO * x, screen.height() * y)
}
