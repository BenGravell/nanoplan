use bevy_egui::egui;

use super::super::colors::{DIM_TEXT, ORANGE, SURFACE, TEXT};
use super::pictogram::{Pictogram, pictogram};
use super::style::caps_font;

const CAMERA_CONTROLS: [(&str, &str); 7] = [
    ("MMB / WASD", "PAN"),
    ("RMB / Q E", "ROTATE"),
    ("WHEEL", "ZOOM"),
    ("F", "FOLLOW"),
    ("R", "RESET"),
    ("SPACE / ESC", "PAUSE"),
    ("T", "FRAME TIME"),
];

const INTRODUCTION: [(Pictogram, &str, &str); 6] = [
    (Pictogram::Track, "TRACK", "Selects a circuit."),
    (Pictogram::Planner, "PLANNER", "Changes the active motion planner."),
    (
        Pictogram::Preview,
        "FUTURE PREVIEW",
        "Sets how many seconds of the current plan are drawn.",
    ),
    (
        Pictogram::Diagnostics,
        "DIAGNOSTIC POINTS / TRAJECTORIES",
        "Show a planner's diagnostic geometry.",
    ),
    (Pictogram::Pause, "PAUSE", "Freezes the simulation."),
    (Pictogram::Zoom, "SCROLL", "Zooms the camera."),
];

#[derive(Clone, Copy, Default)]
enum Page {
    #[default]
    Introduction,
    Controls,
}

pub(super) fn show(root: &mut egui::Ui, open: &mut bool) {
    root.painter().rect_filled(root.max_rect(), 0.0, SURFACE);

    let page_id = egui::Id::new("tutorial_page");
    let mut page = root.data_mut(|data| data.get_temp::<Page>(page_id).unwrap_or_default());
    let screen = root.max_rect();
    let compact = screen.height() < 500.0;
    let content_width = (screen.width() * 0.72).clamp(420.0, 880.0).min(screen.width() - 24.0);
    let content = egui::Rect::from_center_size(
        screen.center(),
        egui::vec2(content_width, (screen.height() * 0.92).clamp(280.0, 680.0)),
    );
    root.scope_builder(egui::UiBuilder::new().max_rect(content), |ui| {
        if compact {
            ui.spacing_mut().interact_size.y = 20.0;
            ui.spacing_mut().item_spacing.y = 3.0;
            ui.spacing_mut().button_padding.y = 3.0;
        }
        ui.vertical_centered(|ui| {
            ui.label(
                egui::RichText::new("TUTORIAL")
                    .font(caps_font(if compact { 22.0 } else { 36.0 }))
                    .color(TEXT),
            );
            ui.label(
                egui::RichText::new(match page {
                    Page::Introduction => "01 / 02  ·  INTRODUCTION",
                    Page::Controls => "02 / 02  ·  CONTROLS",
                })
                .font(caps_font(if compact { 11.0 } else { 14.0 }))
                .color(ORANGE),
            );
            ui.add_space(if compact { 2.0 } else { 10.0 });

            match page {
                Page::Introduction => introduction(ui, compact),
                Page::Controls => controls(ui, compact),
            }

            ui.add_space(if compact { 2.0 } else { 12.0 });
            ui.horizontal(|ui| {
                if ui.button(egui::RichText::new("BACK").font(caps_font(13.0))).clicked() {
                    *open = false;
                }
                match page {
                    Page::Introduction => {
                        if ui
                            .button(egui::RichText::new("CONTROLS  →").font(caps_font(13.0)))
                            .clicked()
                        {
                            page = Page::Controls;
                        }
                    }
                    Page::Controls => {
                        if ui
                            .button(egui::RichText::new("←  INTRODUCTION").font(caps_font(13.0)))
                            .clicked()
                        {
                            page = Page::Introduction;
                        }
                    }
                }
            });
        });
    });

    if root.input(|input| input.key_pressed(egui::Key::Escape)) {
        *open = false;
    }
    if *open {
        root.data_mut(|data| data.insert_temp(page_id, page));
    } else {
        root.data_mut(|data| data.remove::<Page>(page_id));
    }
}

fn introduction(ui: &mut egui::Ui, compact: bool) {
    ui.label(
        egui::RichText::new("The ego and traffic race on various circuits.")
            .size(if compact { 13.0 } else { 18.0 })
            .color(TEXT),
    );
    ui.add_space(if compact { 2.0 } else { 10.0 });
    ui.columns(2, |columns| {
        for (column, items) in columns.iter_mut().zip(INTRODUCTION.chunks(3)) {
            for &(icon, title, description) in items {
                introduction_item(column, icon, title, description, compact);
            }
        }
    });
}

fn introduction_item(ui: &mut egui::Ui, icon: Pictogram, title: &str, description: &str, compact: bool) {
    ui.horizontal_top(|ui| {
        pictogram(ui, icon, if compact { 25.0 } else { 34.0 });
        ui.vertical(|ui| {
            ui.label(
                egui::RichText::new(title)
                    .font(caps_font(if compact { 10.0 } else { 12.0 }))
                    .color(ORANGE),
            );
            ui.label(
                egui::RichText::new(description)
                    .size(if compact { 11.0 } else { 14.0 })
                    .color(DIM_TEXT),
            );
        });
    });
    ui.add_space(if compact { 2.0 } else { 10.0 });
}

fn controls(ui: &mut egui::Ui, compact: bool) {
    ui.label(
        egui::RichText::new("Move the camera, follow the ego, or inspect performance while the race runs.")
            .size(if compact { 12.0 } else { 16.0 })
            .color(DIM_TEXT),
    );
    ui.add_space(if compact { 2.0 } else { 12.0 });
    egui::Grid::new("tutorial_camera_controls")
        .num_columns(2)
        .spacing(egui::vec2(40.0, if compact { 1.0 } else { 12.0 }))
        .show(ui, |ui| {
            for (input, action) in CAMERA_CONTROLS {
                ui.label(egui::RichText::new(input).font(caps_font(14.0)).color(DIM_TEXT));
                ui.monospace(action);
                ui.end_row();
            }
        });
}
