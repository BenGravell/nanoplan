use bevy_egui::egui;

use super::super::super::colors::{DIM, TEXT};
use super::super::style::caps_font;
use crate::viewer::live::{CameraTarget, Live, MAX_ZOOM, MIN_ZOOM};

pub(super) fn show(ui: &mut egui::Ui, live: &mut Live) {
    ui.label(
        egui::RichText::new("FOLLOW TARGET")
            .font(caps_font(11.0))
            .color(DIM),
    );
    ui.checkbox(&mut live.camera.follow, "Follow camera");
    ui.horizontal(|ui| {
        ui.selectable_value(&mut live.camera.target, CameraTarget::Ego, "Ego");
        ui.selectable_value(
            &mut live.camera.target,
            CameraTarget::Track,
            "Track centerline",
        );
    });
    let heading = match live.camera.target {
        CameraTarget::Ego => "Align to ego heading",
        CameraTarget::Track => "Align to track heading",
    };
    ui.checkbox(&mut live.camera.align_heading, heading);
    ui.checkbox(&mut live.camera.smooth, "Smooth motion");
    ui.add(
        egui::Slider::new(&mut live.camera.zoom, MIN_ZOOM..=MAX_ZOOM)
            .logarithmic(true)
            .text("zoom")
            .custom_formatter(|value, _| format!("{:.0}%", value * 100.0))
            .trailing_fill(true),
    );
    ui.horizontal(|ui| {
        if ui.button("-15°").clicked() {
            live.camera.rotation -= 15.0_f32.to_radians();
            live.camera.align_heading = false;
        }
        if ui.button("NORTH UP").clicked() {
            live.camera.rotation = 0.0;
            live.camera.align_heading = false;
        }
        if ui.button("+15°").clicked() {
            live.camera.rotation += 15.0_f32.to_radians();
            live.camera.align_heading = false;
        }
        if ui.button("RESET").clicked() {
            live.reset_camera();
        }
    });
    section_heading(ui, "CONTROLS");
    egui::Grid::new("camera_controls").show(ui, |ui| {
        for (input, action) in [
            ("MMB / WASD", "PAN"),
            ("RMB / Q E", "ROTATE"),
            ("WHEEL", "ZOOM"),
            ("F", "FOLLOW"),
            ("R", "RESET"),
        ] {
            ui.label(egui::RichText::new(input).font(caps_font(11.0)).color(DIM));
            ui.monospace(action);
            ui.end_row();
        }
    });
}

fn section_heading(ui: &mut egui::Ui, heading: &str) {
    ui.add_space(6.0);
    ui.label(
        egui::RichText::new(heading)
            .font(caps_font(12.0))
            .color(TEXT),
    );
}
