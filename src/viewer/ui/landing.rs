use bevy_egui::egui;

use super::super::colors::{BLACK, HOVER, LANDING_BACKGROUND as BACKGROUND, ORANGE, WHITE};
use super::style::caps_font;

const BACKGROUND_ASPECT_RATIO: f32 = 2560.0 / 1440.0;
const TITLE_ASPECT_RATIO: f32 = 146.65262 / 23.909071;

pub(super) fn show(root: &mut egui::Ui, started: &mut bool) {
    let screen = root.max_rect();
    let content_left = (screen.width() * 0.06).clamp(24.0, 96.0);
    background_graphics(root);
    title_graphic(root);
    root.ctx()
        .request_repaint_after(std::time::Duration::from_millis(16));

    egui::Area::new("landing_menu".into())
        .anchor(
            egui::Align2::LEFT_CENTER,
            egui::vec2(content_left, (screen.height() / 6.0).min(120.0)),
        )
        .show(root.ctx(), |ui| {
            ui.set_width((screen.width() * 0.34).clamp(300.0, 440.0));
            let response = egui::Frame::new()
                .fill(BACKGROUND)
                .inner_margin(egui::Margin::same(32))
                .show(ui, |ui| {
                    ui.scope(|ui| {
                        let widgets = &mut ui.style_mut().visuals.widgets;
                        widgets.inactive.bg_fill = ORANGE;
                        widgets.inactive.weak_bg_fill = ORANGE;
                        widgets.inactive.fg_stroke.color = WHITE;
                        widgets.hovered.bg_fill = HOVER;
                        widgets.hovered.weak_bg_fill = HOVER;
                        widgets.hovered.fg_stroke.color = BLACK;
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
        });

    if root
        .input(|input| input.key_pressed(egui::Key::Enter) || input.key_pressed(egui::Key::Space))
    {
        *started = true;
    }
}

pub(super) fn background_rect(screen: egui::Rect, anchor: egui::Align2) -> egui::Rect {
    let size = egui::vec2(screen.height() * BACKGROUND_ASPECT_RATIO, screen.height());
    anchor.align_size_within_rect(size, screen)
}

fn background_graphics(ui: &egui::Ui) {
    for (image, anchor) in [
        (
            egui::Image::new(egui::include_image!(
                "../../../assets/landing/nanoplan_bkgd_bt_rt_corner.svg"
            )),
            egui::Align2::RIGHT_BOTTOM,
        ),
        (
            egui::Image::new(egui::include_image!(
                "../../../assets/landing/nanoplan_bkgd_bt_lf_corner.svg"
            )),
            egui::Align2::LEFT_BOTTOM,
        ),
        (
            egui::Image::new(egui::include_image!(
                "../../../assets/landing/nanoplan_bkgd_tp_lf_corner.svg"
            )),
            egui::Align2::LEFT_TOP,
        ),
    ] {
        paint_svg(ui, image, background_rect(ui.max_rect(), anchor));
    }
}

pub(super) fn title_rect(screen: egui::Rect) -> egui::Rect {
    let scale = screen.height() / 1080.0;
    egui::Rect::from_min_size(
        screen.left_top() + egui::vec2(80.0, 160.0) * scale,
        egui::vec2(760.0, 760.0 / TITLE_ASPECT_RATIO) * scale,
    )
}

fn title_graphic(ui: &egui::Ui) {
    paint_svg(
        ui,
        egui::Image::new(egui::include_image!(
            "../../../assets/landing/nanoplan_title.svg"
        )),
        title_rect(ui.max_rect()),
    );
}

fn paint_svg(ui: &egui::Ui, image: egui::Image<'_>, rect: egui::Rect) {
    let raster_size = background_raster_size(
        rect.size(),
        ui.pixels_per_point(),
        ui.input(|input| input.max_texture_side),
    );
    if let Ok(egui::load::TexturePoll::Ready { texture }) =
        image.load_for_size(ui.ctx(), raster_size)
    {
        ui.painter().image(
            texture.id,
            rect,
            egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0)),
            WHITE,
        );
    }
}

pub(super) fn background_raster_size(
    display_size: egui::Vec2,
    pixels_per_point: f32,
    max_texture_side: usize,
) -> egui::Vec2 {
    let max_side_in_points = max_texture_side as f32 / pixels_per_point;
    display_size * (max_side_in_points / display_size.max_elem()).min(1.0)
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
