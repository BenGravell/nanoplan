use bevy_egui::egui;

use super::super::colors::{DIM, ORANGE, SURFACE, TEXT};
use super::style::caps_font;

const CAMERA_CONTROLS: [(&str, &str); 5] = [
    ("MMB / WASD", "PAN"),
    ("RMB / Q E", "ROTATE"),
    ("WHEEL", "ZOOM"),
    ("F", "FOLLOW"),
    ("R", "RESET"),
];

pub(super) fn show(root: &mut egui::Ui, open: &mut bool) {
    root.painter().rect_filled(root.max_rect(), 0.0, SURFACE);

    let screen = root.max_rect();
    let compact = screen.height() < 500.0;
    let content_width = (screen.width() * 0.55).clamp(360.0, 720.0);
    let content = egui::Rect::from_center_size(
        screen.center(),
        egui::vec2(content_width, (screen.height() * 0.9).clamp(300.0, 600.0)),
    );
    root.scope_builder(egui::UiBuilder::new().max_rect(content), |ui| {
        if compact {
            ui.spacing_mut().interact_size.y = 24.0;
        }
        ui.vertical_centered(|ui| {
            ui.label(
                egui::RichText::new("TUTORIAL")
                    .font(caps_font(if compact { 28.0 } else { 36.0 }))
                    .color(TEXT),
            );
            ui.add_space(if compact { 2.0 } else { 8.0 });
            ui.label(
                egui::RichText::new("CAMERA CONTROLS")
                    .font(caps_font(15.0))
                    .color(ORANGE),
            );
            ui.add_space(if compact { 8.0 } else { 18.0 });

            egui::Grid::new("tutorial_camera_controls")
                .num_columns(2)
                .spacing(egui::vec2(40.0, if compact { 4.0 } else { 14.0 }))
                .show(ui, |ui| {
                    for (input, action) in CAMERA_CONTROLS {
                        ui.label(egui::RichText::new(input).font(caps_font(14.0)).color(DIM));
                        ui.monospace(action);
                        ui.end_row();
                    }
                });

            ui.add_space(if compact { 10.0 } else { 24.0 });
            if ui
                .button(egui::RichText::new("BACK").font(caps_font(13.0)))
                .clicked()
            {
                *open = false;
            }
        });
    });

    if root.input(|input| input.key_pressed(egui::Key::Escape)) {
        *open = false;
    }
}
