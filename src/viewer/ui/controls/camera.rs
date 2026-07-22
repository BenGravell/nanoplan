use bevy_egui::egui;

use super::super::super::colors::DIM_TEXT;
use super::super::style::caps_font;
use crate::viewer::live::{Live, MAX_ZOOM, MIN_ZOOM};

pub(super) fn show(ui: &mut egui::Ui, live: &mut Live, compact: bool, content_width: f32) {
    ui.label(
        egui::RichText::new("FOLLOW")
            .font(caps_font(11.0))
            .color(DIM_TEXT),
    );
    ui.checkbox(
        &mut live.camera.follow,
        if compact { "Follow" } else { "Follow camera" },
    );
    ui.checkbox(
        &mut live.camera.align_heading,
        if compact {
            "Align heading"
        } else {
            "Align to ego heading"
        },
    );
    ui.checkbox(
        &mut live.camera.smooth,
        if compact { "Smooth" } else { "Smooth motion" },
    );
    ui.label(
        egui::RichText::new("ZOOM")
            .font(caps_font(11.0))
            .color(DIM_TEXT),
    );
    let zoom = ui
        .scope(|ui| {
            if compact {
                // Reserve space for the percentage field and size the track
                // from the actual narrow-rail content width.
                ui.spacing_mut().slider_width = (content_width - 76.0).max(24.0);
            }
            ui.add(
                egui::Slider::new(&mut live.camera.zoom, MIN_ZOOM..=MAX_ZOOM)
                    .logarithmic(true)
                    .custom_formatter(|value, _| format!("{:.0}%", value * 100.0))
                    .trailing_fill(true),
            )
        })
        .inner;
    zoom.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Slider, true, "Zoom control"));
    let labels = [
        "-15°",
        if compact { "NORTH" } else { "NORTH UP" },
        "+15°",
        "RESET",
    ];
    let mut clicked = [false; 4];
    ui.scope(|ui| {
        if compact {
            ui.spacing_mut().button_padding.x = 4.0;
        }
        let button_width = (content_width - ui.spacing().item_spacing.x) / 2.0;
        for row in 0..2 {
            ui.horizontal(|ui| {
                for column in 0..2 {
                    let index = row * 2 + column;
                    let height = ui.spacing().interact_size.y;
                    let text =
                        egui::RichText::new(labels[index]).size(if compact { 12.0 } else { 15.0 });
                    clicked[index] = ui
                        .add_sized([button_width, height], egui::Button::new(text))
                        .clicked();
                }
            });
        }
    });
    if clicked[0] {
        live.camera.rotation -= 15.0_f32.to_radians();
        live.camera.align_heading = false;
    }
    if clicked[1] {
        live.camera.rotation = 0.0;
        live.camera.align_heading = false;
    }
    if clicked[2] {
        live.camera.rotation += 15.0_f32.to_radians();
        live.camera.align_heading = false;
    }
    if clicked[3] {
        live.reset_camera();
    }
}
