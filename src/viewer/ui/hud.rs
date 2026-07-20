use bevy_egui::egui;

use super::widgets::{friction_box, lap_stats, speedometer};
use crate::viewer::live::Live;

pub(super) fn draw(ui: &mut egui::Ui, live: &Live, compact: bool) {
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ui.available_height()),
        egui::Sense::hover(),
    );
    if compact {
        draw_compact(ui, rect, live);
    } else {
        draw_full(ui, rect, live);
    }
    ui.interact(rect, ui.id().with("driving_hud"), egui::Sense::hover())
        .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Driving HUD"));
}

fn draw_compact(ui: &egui::Ui, rect: egui::Rect, live: &Live) {
    let painter = ui.painter_at(rect);
    lap_stats::draw(
        &painter,
        egui::Rect::from_min_size(rect.min, egui::vec2(rect.width(), 77.0)),
        live.lap_stats,
    );
    friction_box::draw(
        &painter,
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), rect.top() + 82.0),
            egui::vec2(rect.width(), 110.0),
        ),
        &live.friction_box,
        live.world.ego().speed,
    );
    speedometer::draw(&painter, speedometer_rect(rect), live.world.ego().speed);
}

fn draw_full(ui: &egui::Ui, rect: egui::Rect, live: &Live) {
    let speed = live.world.ego().speed;
    let painter = ui.painter_at(rect);
    lap_stats::draw(
        &painter,
        egui::Rect::from_min_size(rect.min, egui::vec2(rect.width(), 77.0)),
        live.lap_stats,
    );
    let friction_top = egui::lerp(rect.top()..=rect.bottom() - 300.0, 0.5);
    friction_box::draw(
        &painter,
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), friction_top),
            egui::vec2(rect.width(), 184.0),
        ),
        &live.friction_box,
        speed,
    );
    speedometer::draw(&painter, speedometer_rect(rect), speed);
}

fn speedometer_rect(rect: egui::Rect) -> egui::Rect {
    const HEIGHT_TO_WIDTH: f32 = 3.0 / 5.0;
    let width = rect.width().min(rect.height() / HEIGHT_TO_WIDTH);
    let height = width * HEIGHT_TO_WIDTH;
    egui::Rect::from_min_size(
        egui::pos2(rect.left(), rect.bottom() - height),
        egui::vec2(width, height),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speedometer_always_gets_a_three_by_five_area_inside_the_hud() {
        for size in [egui::vec2(120.0, 363.0), egui::vec2(200.0, 1060.0)] {
            let hud = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
            let gauge = speedometer_rect(hud);

            assert_eq!(gauge.height() / gauge.width(), 3.0 / 5.0);
            assert!(hud.contains_rect(gauge));
            assert_eq!(gauge.bottom(), hud.bottom());
        }
    }
}
