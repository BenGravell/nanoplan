use bevy_egui::egui;

use super::super::colors::{DIM, TEXT};
use super::style::caps_font;
use super::widgets::{acceleration_bar, curvature_bar, friction_box};
use crate::viewer::live::Live;

pub(super) fn draw(ui: &mut egui::Ui, live: &Live, compact: bool) {
    let height = if compact {
        112.0
    } else {
        (ui.available_height() * 0.42).clamp(280.0, 400.0)
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
    let actuation = live.world.actuation();
    for (i, (label, value)) in [
        ("SPEED", format!("{:.1}", live.world.ego().speed)),
        ("ACCEL", format!("{:+.1}", actuation.acceleration)),
        ("CURV", format!("{:+.3}", actuation.curvature)),
    ]
    .into_iter()
    .enumerate()
    {
        let x = egui::lerp(rect.left()..=rect.right(), (i as f32 + 0.5) / 3.0);
        painter.text(
            egui::pos2(x, rect.top()),
            egui::Align2::CENTER_TOP,
            label,
            caps_font(8.0),
            DIM,
        );
        painter.text(
            egui::pos2(x, rect.top() + 12.0),
            egui::Align2::CENTER_TOP,
            value,
            egui::FontId::monospace(9.0),
            TEXT,
        );
    }
    friction_box::draw(
        &painter,
        egui::Rect::from_min_max(
            egui::pos2(rect.left(), rect.top() + 31.0),
            rect.right_bottom(),
        ),
        &live.friction_box,
        live.world.ego().speed,
    );
}

fn draw_full(ui: &egui::Ui, rect: egui::Rect, live: &Live) {
    let speed = live.world.ego().speed;
    let actuation = live.world.actuation();
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
    acceleration_bar::draw(
        &painter,
        egui::Rect::from_min_max(
            egui::pos2(center_x - 6.0, rect.top() + 58.0),
            egui::pos2(center_x + 6.0, rect.top() + 120.0),
        ),
        egui::pos2(center_x + 18.0, rect.top() + 89.0),
        actuation.acceleration,
    );
    curvature_bar::draw(
        &painter,
        egui::Rect::from_min_max(
            egui::pos2(center_x - 68.0, rect.top() + 148.0),
            egui::pos2(center_x + 68.0, rect.top() + 160.0),
        ),
        egui::pos2(center_x, rect.top() + 169.0),
        actuation.curvature,
        speed,
    );
    friction_box::draw(
        &painter,
        egui::Rect::from_min_max(
            egui::pos2(rect.left(), rect.top() + 190.0),
            rect.right_bottom(),
        ),
        &live.friction_box,
        speed,
    );
}
