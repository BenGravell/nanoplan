use bevy_egui::egui;

use super::super::colors::{
    HOVER as YELLOW, LANDING_BACKGROUND as BACKGROUND, LANDING_BLUE as BLUE,
    LANDING_DARK_BLUE as DARK_BLUE, LANDING_MUTED as MUTED, ORANGE,
};
use super::style::caps_font;

pub(super) fn show(root: &mut egui::Ui, started: &mut bool) {
    let screen = root.max_rect();
    let painter = root.painter();
    painter.rect_filled(screen, 0.0, BACKGROUND);
    background_graphics(painter, screen);
    root.ctx()
        .request_repaint_after(std::time::Duration::from_millis(16));

    egui::Area::new("landing_brand".into())
        .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, -90.0))
        .show(root.ctx(), |ui| {
            ui.set_width((screen.width() * 0.68).clamp(420.0, 760.0));
            ui.vertical_centered(|ui| {
                egui::Frame::new()
                    .fill(BACKGROUND)
                    .inner_margin(egui::Margin::same(32))
                    .show(ui, |ui| {
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
        .anchor(
            egui::Align2::CENTER_CENTER,
            egui::vec2(0.0, (screen.height() / 6.0).min(120.0)),
        )
        .show(root.ctx(), |ui| {
            ui.set_width((screen.width() * 0.38).clamp(300.0, 440.0));
            let response = egui::Frame::new()
                .fill(BACKGROUND)
                .inner_margin(egui::Margin::same(32))
                .show(ui, |ui| {
                    ui.scope(|ui| {
                        let widgets = &mut ui.style_mut().visuals.widgets;
                        widgets.inactive.bg_fill = ORANGE;
                        widgets.inactive.weak_bg_fill = ORANGE;
                        widgets.inactive.fg_stroke.color = egui::Color32::WHITE;
                        widgets.hovered.bg_fill = YELLOW;
                        widgets.hovered.weak_bg_fill = YELLOW;
                        widgets.hovered.fg_stroke.color = DARK_BLUE;
                        ui.visuals_mut().override_text_color = None;
                        ui.add_sized(
                            [ui.available_width(), 52.0],
                            egui::Button::new(
                                egui::RichText::new("START DRIVING").font(caps_font(19.0)),
                            )
                            .corner_radius(0),
                        )
                    })
                    .inner
                })
                .inner;
            corner_brackets(
                ui.painter(),
                response.rect,
                ui.input(|input| input.time) as f32,
            );
            if response.clicked() {
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

fn corner_brackets(painter: &egui::Painter, rect: egui::Rect, time: f32) {
    let pulse = (time * 4.0).sin() * 0.5 + 0.5;
    let rect = rect.expand(10.0 + pulse * 2.0);
    let length = 12.0 + pulse * 4.0;
    let color = egui::Color32::from_rgba_unmultiplied(
        ORANGE.r(),
        ORANGE.g(),
        ORANGE.b(),
        (170.0 + pulse * 85.0) as u8,
    );
    let stroke = egui::Stroke::new(3.0, color);
    for points in [
        vec![
            egui::pos2(rect.left() + length, rect.top()),
            rect.left_top(),
            egui::pos2(rect.left(), rect.top() + length),
        ],
        vec![
            egui::pos2(rect.right() - length, rect.top()),
            rect.right_top(),
            egui::pos2(rect.right(), rect.top() + length),
        ],
        vec![
            egui::pos2(rect.left(), rect.bottom() - length),
            rect.left_bottom(),
            egui::pos2(rect.left() + length, rect.bottom()),
        ],
        vec![
            egui::pos2(rect.right(), rect.bottom() - length),
            rect.right_bottom(),
            egui::pos2(rect.right() - length, rect.bottom()),
        ],
    ] {
        painter.add(egui::Shape::line(points, stroke));
    }
}

fn background_graphics(painter: &egui::Painter, screen: egui::Rect) {
    // Large cropped planes keep the composition asymmetric and fast.
    polygon(
        painter,
        screen,
        &[
            (0.0, 0.68),
            (0.19, 0.54),
            (0.34, 0.57),
            (0.09, 1.0),
            (0.0, 1.0),
        ],
        DARK_BLUE,
    );
    polygon(
        painter,
        screen,
        &[
            (0.0, 0.78),
            (0.28, 0.55),
            (0.39, 0.58),
            (0.16, 1.0),
            (0.0, 1.0),
        ],
        BLUE,
    );
    polygon(
        painter,
        screen,
        &[(0.73, 1.0), (0.88, 0.53), (0.94, 0.47), (0.84, 1.0)],
        ORANGE,
    );
    polygon(
        painter,
        screen,
        &[(0.88, 1.0), (0.95, 0.59), (1.0, 0.51), (1.0, 1.0)],
        BLUE,
    );

    // Thin trapezoids echo vehicle silhouettes and high-speed UI dividers.
    polygon(
        painter,
        screen,
        &[(0.0, 0.17), (0.29, 0.17), (0.25, 0.19), (0.0, 0.19)],
        ORANGE,
    );
    polygon(
        painter,
        screen,
        &[(0.76, 0.24), (1.0, 0.17), (1.0, 0.2), (0.8, 0.26)],
        BLUE,
    );
    for offset in [0.0, 0.018, 0.036] {
        painter.line_segment(
            [
                point(screen, 0.04, 0.39 + offset),
                point(screen, 0.38, 0.28 + offset),
            ],
            egui::Stroke::new(if offset == 0.0 { 3.0 } else { 1.0 }, BLUE),
        );
    }
}

fn point(screen: egui::Rect, x: f32, y: f32) -> egui::Pos2 {
    egui::pos2(
        screen.left() + screen.width() * x,
        screen.top() + screen.height() * y,
    )
}

fn polygon(
    painter: &egui::Painter,
    screen: egui::Rect,
    points: &[(f32, f32)],
    color: egui::Color32,
) {
    painter.add(egui::Shape::convex_polygon(
        points.iter().map(|&(x, y)| point(screen, x, y)).collect(),
        color,
        egui::Stroke::NONE,
    ));
}
