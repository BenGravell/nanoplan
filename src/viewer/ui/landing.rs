use bevy_egui::egui;

use super::super::colors::ORANGE;
use super::style::caps_font;

const BACKGROUND: egui::Color32 = egui::Color32::from_rgb(7, 16, 31);
const BLUE_LINE: egui::Color32 = egui::Color32::from_rgb(31, 72, 103);
const MUTED: egui::Color32 = egui::Color32::from_rgb(143, 157, 170);

pub(super) fn show(root: &mut egui::Ui, started: &mut bool) {
    let screen = root.max_rect();
    let painter = root.painter();
    painter.rect_filled(screen, 0.0, BACKGROUND);

    // A spare, track-like horizon gives the title screen a late-90s console feel.
    let horizon = screen.center().y + screen.height() * 0.1;
    for offset in [0.0, 20.0, 40.0] {
        painter.line_segment(
            [
                egui::pos2(screen.left(), horizon + offset),
                egui::pos2(screen.right(), horizon - 110.0 + offset),
            ],
            egui::Stroke::new(if offset == 0.0 { 2.0 } else { 1.0 }, BLUE_LINE),
        );
    }

    egui::Area::new("landing_brand".into())
        .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, -90.0))
        .show(root.ctx(), |ui| {
            ui.set_width((screen.width() * 0.68).clamp(420.0, 760.0));
            ui.vertical_centered(|ui| {
                egui::Frame::new()
                    .fill(ORANGE)
                    .inner_margin(egui::Margin::symmetric(24, 8))
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new("NANOPLAN")
                                .font(caps_font((screen.height() * 0.09).clamp(44.0, 84.0)))
                                .color(egui::Color32::WHITE),
                        );
                    });
                ui.add_space(12.0);
                ui.label(
                    egui::RichText::new("REAL-TIME MOTION PLANNING")
                        .font(caps_font(14.0))
                        .color(MUTED),
                );
            });
        });

    egui::Area::new("landing_menu".into())
        .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 120.0))
        .show(root.ctx(), |ui| {
            ui.set_width((screen.width() * 0.38).clamp(300.0, 440.0));
            let button = egui::Button::new(
                egui::RichText::new("START DRIVING")
                    .font(caps_font(19.0))
                    .color(egui::Color32::WHITE),
            )
            .fill(ORANGE)
            .corner_radius(0);
            if ui.add_sized([ui.available_width(), 52.0], button).clicked() {
                *started = true;
            }
            ui.add_space(12.0);
            ui.vertical_centered(|ui| {
                ui.label(
                    egui::RichText::new("ENTER / SPACE")
                        .font(caps_font(11.0))
                        .color(MUTED),
                );
            });
        });

    if root
        .input(|input| input.key_pressed(egui::Key::Enter) || input.key_pressed(egui::Key::Space))
    {
        *started = true;
    }

    painter.text(
        egui::pos2(screen.right() - 24.0, screen.bottom() - 20.0),
        egui::Align2::RIGHT_BOTTOM,
        "DRIVE  -  PLAN  -  IMPROVE",
        caps_font(11.0),
        MUTED.gamma_multiply(0.7),
    );
}
