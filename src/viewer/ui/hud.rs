use bevy_egui::egui;

use super::super::colors::{DIM, PANEL, TEXT};
use super::style::caps_font;
use super::widgets::{acceleration_bar, curvature_bar, friction_box};
use crate::viewer::live::Live;

pub(super) fn compact_hud(ui: &mut egui::Ui, live: &Live, width: f32) {
    let actuation = live.world.actuation();
    egui::Panel::right("driving_hud")
        .exact_size(width)
        .resizable(false)
        .frame(
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(egui::Margin::same(8)),
        )
        .show(ui, |ui| {
            ui.vertical_centered(|ui| {
                for (label, value) in [
                    ("SPEED", format!("{:.1} m/s", live.world.ego.speed)),
                    ("ACCEL", format!("{:+.1} m/s²", actuation.acceleration)),
                    ("CURV", format!("{:+.3} m⁻¹", actuation.curvature)),
                ] {
                    ui.label(egui::RichText::new(label).font(caps_font(10.0)).color(DIM));
                    ui.monospace(value);
                    ui.add_space(4.0);
                }
                let (plot, _) = ui.allocate_exact_size(
                    egui::vec2(ui.available_width(), ui.available_height()),
                    egui::Sense::hover(),
                );
                friction_box::draw(ui.painter(), plot, &live.friction_box, live.world.ego.speed);
            });
            accessibility(ui);
        });
}

pub(super) fn hud(ui: &mut egui::Ui, live: &Live, width: f32) {
    const CONTENT_HEIGHT: f32 = 472.0;

    let speed = live.world.ego.speed;
    let actuation = live.world.actuation();
    egui::Panel::right("driving_hud")
        .exact_size(width)
        .resizable(false)
        .frame(egui::Frame::new().fill(PANEL))
        .show(ui, |ui| {
            let rect = ui.max_rect();
            let painter = ui.painter_at(rect);
            let center_x = rect.center().x;
            let top = (rect.center().y - CONTENT_HEIGHT / 2.0).max(rect.top());
            painter.text(
                egui::pos2(center_x, top + 10.0),
                egui::Align2::CENTER_TOP,
                "SPEED",
                caps_font(10.0),
                DIM,
            );
            painter.text(
                egui::pos2(center_x, top + 20.0),
                egui::Align2::CENTER_TOP,
                format!("{:04.1}", speed),
                egui::FontId::monospace(34.0),
                TEXT,
            );
            painter.text(
                egui::pos2(center_x, top + 55.0),
                egui::Align2::CENTER_TOP,
                "m/s",
                egui::FontId::monospace(10.0),
                DIM,
            );

            acceleration_bar::draw(
                &painter,
                egui::Rect::from_min_max(
                    egui::pos2(center_x - 6.0, top + 98.0),
                    egui::pos2(center_x + 6.0, top + 178.0),
                ),
                egui::pos2(center_x + 18.0, top + 138.0),
                actuation.acceleration,
            );

            curvature_bar::draw(
                &painter,
                egui::Rect::from_min_max(
                    egui::pos2(center_x - 68.0, top + 224.0),
                    egui::pos2(center_x + 68.0, top + 236.0),
                ),
                egui::pos2(center_x, top + 245.0),
                actuation.curvature,
                speed,
            );

            friction_box::draw(
                &painter,
                egui::Rect::from_min_size(
                    egui::pos2(rect.left(), top + 288.0),
                    egui::vec2(rect.width(), 184.0),
                ),
                &live.friction_box,
                speed,
            );
            accessibility(ui);
        });
}

fn accessibility(ui: &egui::Ui) {
    ui.interact(
        ui.max_rect(),
        ui.id().with("accessibility"),
        egui::Sense::hover(),
    )
    .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Driving HUD"));
}
