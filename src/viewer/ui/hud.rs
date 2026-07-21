use bevy_egui::egui;

use super::widgets::{friction_box, lap_stats, speedometer};
use crate::viewer::live::Live;

pub(super) fn draw(ui: &mut egui::Ui, live: &Live, _compact: bool) {
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ui.available_height()),
        egui::Sense::hover(),
    );
    draw_rows(ui, rect, live);
    ui.interact(rect, ui.id().with("driving_hud"), egui::Sense::hover())
        .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Driving HUD"));
}

fn draw_rows(ui: &egui::Ui, rect: egui::Rect, live: &Live) {
    let painter = ui.painter_at(rect);
    let speed = live.world.ego().speed;
    let [lap_row, friction_row, speedometer_row] = rows(rect);
    lap_stats::draw(&painter, lap_row, live.lap_stats);
    friction_box::draw(&painter, friction_row, &live.friction_box, speed);
    speedometer::draw(&painter, speedometer_row, speed);
}

fn rows(rect: egui::Rect) -> [egui::Rect; 3] {
    let first_bottom = egui::lerp(rect.top()..=rect.bottom(), 1.0 / 3.0);
    let second_bottom = egui::lerp(rect.top()..=rect.bottom(), 2.0 / 3.0);
    [
        egui::Rect::from_x_y_ranges(rect.x_range(), rect.top()..=first_bottom),
        egui::Rect::from_x_y_ranges(rect.x_range(), first_bottom..=second_bottom),
        egui::Rect::from_x_y_ranges(rect.x_range(), second_bottom..=rect.bottom()),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn widgets_get_three_equal_rows() {
        for size in [egui::vec2(120.0, 363.0), egui::vec2(200.0, 1060.0)] {
            let hud = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
            let rows = rows(hud);

            assert!(rows.iter().all(|row| hud.contains_rect(*row)));
            assert!((rows[0].height() - rows[1].height()).abs() < 1e-4);
            assert!((rows[1].height() - rows[2].height()).abs() < 1e-4);
            assert_eq!(rows[0].top(), hud.top());
            assert_eq!(rows[2].bottom(), hud.bottom());
        }
    }
}
