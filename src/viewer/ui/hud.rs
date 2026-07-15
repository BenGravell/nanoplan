use bevy_egui::egui;

use super::super::colors::{DIM, TEXT};
use super::style::caps_font;
use super::widgets::friction_box;
use crate::viewer::live::Live;

pub(super) fn draw(ui: &mut egui::Ui, live: &Live, compact: bool) {
    let height = if compact {
        145.0
    } else {
        (ui.available_height() * 0.6).clamp(360.0, 560.0)
    };
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), height),
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
    painter.text(
        rect.center_top(),
        egui::Align2::CENTER_TOP,
        format!("SPEED  {:.1} m/s", live.world.ego().speed),
        egui::FontId::monospace(9.0),
        TEXT,
    );
    friction_box::draw(
        &painter,
        egui::Rect::from_min_max(
            egui::pos2(rect.left(), rect.top() + 18.0),
            rect.right_bottom(),
        ),
        &live.friction_box,
        live.world.ego().speed,
    );
}

fn draw_full(ui: &egui::Ui, rect: egui::Rect, live: &Live) {
    let speed = live.world.ego().speed;
    let painter = ui.painter_at(rect);
    let center_x = rect.center().x;
    painter.text(
        egui::pos2(center_x, rect.top()),
        egui::Align2::CENTER_TOP,
        "SPEED",
        caps_font(10.0),
        DIM,
    );
    painter.text(
        egui::pos2(center_x, rect.top() + 10.0),
        egui::Align2::CENTER_TOP,
        format!("{:04.1}", speed),
        egui::FontId::monospace(28.0),
        TEXT,
    );
    painter.text(
        egui::pos2(center_x, rect.top() + 40.0),
        egui::Align2::CENTER_TOP,
        "m/s",
        egui::FontId::monospace(9.0),
        DIM,
    );
    let friction_top = egui::lerp(rect.top() + 60.0..=rect.bottom() - 184.0, 0.5);
    friction_box::draw(
        &painter,
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), friction_top),
            egui::vec2(rect.width(), 184.0),
        ),
        &live.friction_box,
        speed,
    );
}
