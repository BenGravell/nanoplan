use bevy_egui::egui;

use super::widgets::{friction_box, speedometer};
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
    friction_box::draw(
        &painter,
        egui::Rect::from_min_size(rect.min, egui::vec2(rect.width(), 110.0)),
        &live.friction_box,
        live.world.ego().speed,
    );
    speedometer::draw(
        &painter,
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), rect.bottom() - 74.0),
            egui::vec2(rect.width(), 74.0),
        ),
        live.world.ego().speed,
    );
}

fn draw_full(ui: &egui::Ui, rect: egui::Rect, live: &Live) {
    let speed = live.world.ego().speed;
    let painter = ui.painter_at(rect);
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
    speedometer::draw(
        &painter,
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), rect.bottom() - 116.0),
            egui::vec2(rect.width(), 116.0),
        ),
        speed,
    );
}
