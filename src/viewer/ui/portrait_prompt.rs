use crate::common::math::smoothstep;
use bevy_egui::egui;

use super::super::ViewportConstraints;
use super::super::colors::{DIM_TEXT, FAINT, ORANGE, PANEL, SURFACE, TEXT};
use super::super::{MIN_VIEWPORT_ASPECT_HEIGHT, MIN_VIEWPORT_ASPECT_WIDTH, MIN_VIEWPORT_WIDTH};
use super::style::caps_font;

// Below a conventional 320 px phone width, tighten the card for narrow windows and foldables.
const COMPACT_BREAKPOINT: f32 = 320.0;
const CARD_MARGIN: i8 = 24;
const COMPACT_CARD_MARGIN: i8 = 12;
const CARD_MAX_WIDTH: f32 = 380.0;
const CARD_STROKE_WIDTH: f32 = 1.0;
const CARD_CORNER_RADIUS: u8 = 12;
const ARROW_MAX_SIZE: f32 = 190.0;
const TITLE_FONT_SIZE: f32 = 22.0;
const COMPACT_TITLE_FONT_SIZE: f32 = 18.0;
const BODY_FONT_SIZE: f32 = 16.0;
const COMPACT_BODY_FONT_SIZE: f32 = 14.0;
const TITLE_BODY_GAP: f32 = 8.0;

const LOOP_DURATION_S: f32 = 3.0;
const EXPANSION_DURATION_S: f32 = 2.2;
const EXPANSION_END_PAUSE_S: f32 = 0.2;
const FADE_DURATION_S: f32 = 0.3;
const ARROW_DESIGN_WIDTH: f32 = 210.0;
const ARROW_MIN_HALF_WIDTH: f32 = 45.0;
const ARROW_MAX_HALF_WIDTH: f32 = 90.0;
const SHAFT_HALF_HEIGHT: f32 = 5.5;
const HEAD_HALF_HEIGHT: f32 = 22.0;
const HEAD_LENGTH: f32 = 29.0;
const ROTATION_RADIANS: f32 = std::f32::consts::FRAC_PI_2;
const ARC_RADIUS: f32 = 70.0;
const ARC_HALF_WIDTH: f32 = 5.5;
const ARC_START_DEG: f32 = 105.0;
const ARC_SWEEP_DEG: f32 = 250.0;
const ARC_SEGMENTS: usize = 48;
const ROTATION_HEAD_HALF_WIDTH: f32 = 22.0;
const MIN_MITER_DOT: f32 = 0.25;

pub(super) fn show(root: &mut egui::Ui, is_mobile: bool, constraints: ViewportConstraints) {
    root.painter().rect_filled(root.max_rect(), 0.0, PANEL);
    let compact = root.max_rect().width() < COMPACT_BREAKPOINT;
    let mobile_portrait = is_mobile && root.max_rect().height() > root.max_rect().width();
    let (title, reason) = prompt_copy(constraints, mobile_portrait);
    let margin = if compact {
        COMPACT_CARD_MARGIN
    } else {
        CARD_MARGIN
    };

    egui::Area::new("portrait_prompt".into())
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(root.ctx(), |ui| {
            egui::Frame::new()
                .fill(SURFACE)
                .stroke(egui::Stroke::new(CARD_STROKE_WIDTH, FAINT))
                .corner_radius(CARD_CORNER_RADIUS)
                .inner_margin(egui::Margin::same(margin))
                .show(ui, |ui| {
                    ui.set_width(
                        (root.max_rect().width() - 2.0 * f32::from(margin)).min(CARD_MAX_WIDTH),
                    );
                    ui.vertical_centered(|ui| {
                        let (rect, _) = ui.allocate_exact_size(
                            egui::vec2(
                                ui.available_width(),
                                ui.available_width().min(ARROW_MAX_SIZE),
                            ),
                            egui::Sense::hover(),
                        );
                        if mobile_portrait {
                            rotation_arrow(ui, rect);
                        } else {
                            width_arrow(ui, rect);
                        }
                        ui.label(
                            egui::RichText::new(title)
                                .font(caps_font(if compact {
                                    COMPACT_TITLE_FONT_SIZE
                                } else {
                                    TITLE_FONT_SIZE
                                }))
                                .color(TEXT),
                        );
                        ui.add_space(TITLE_BODY_GAP);
                        ui.label(
                            egui::RichText::new(reason)
                                .size(if compact {
                                    COMPACT_BODY_FONT_SIZE
                                } else {
                                    BODY_FONT_SIZE
                                })
                                .color(DIM_TEXT),
                        );
                    });
                });
        });
}

pub(super) fn prompt_copy(
    constraints: ViewportConstraints,
    mobile_portrait: bool,
) -> (&'static str, String) {
    if mobile_portrait {
        (
            "TURN YOUR DEVICE SIDEWAYS",
            "Nanoplan requires landscape orientation.".to_owned(),
        )
    } else if !constraints.has_minimum_width && !constraints.has_minimum_aspect_ratio {
        (
            "MAKE YOUR WINDOW WIDER",
            format!(
                "Nanoplan requires a viewport at least {MIN_VIEWPORT_WIDTH:.0} px wide with a \
                 {MIN_VIEWPORT_ASPECT_WIDTH}:{MIN_VIEWPORT_ASPECT_HEIGHT} aspect ratio."
            ),
        )
    } else if !constraints.has_minimum_width {
        (
            "MAKE YOUR WINDOW WIDER",
            format!("Nanoplan requires a viewport at least {MIN_VIEWPORT_WIDTH:.0} px wide."),
        )
    } else {
        (
            "MAKE YOUR WINDOW WIDER",
            format!(
                "Nanoplan requires a viewport with at least a \
                 {MIN_VIEWPORT_ASPECT_WIDTH}:{MIN_VIEWPORT_ASPECT_HEIGHT} aspect ratio."
            ),
        )
    }
}

fn rotation_arrow(ui: &egui::Ui, rect: egui::Rect) {
    let scale = (rect.width() / ARROW_DESIGN_WIDTH).min(1.0);
    let time = ui.input(|input| input.time) as f32;
    let elapsed = time % LOOP_DURATION_S;
    let progress = (elapsed / EXPANSION_DURATION_S).min(1.0);
    let smooth_progress = smoothstep(f64::from(progress)) as f32;
    let rotation = ROTATION_RADIANS * smooth_progress;
    let fade_start = EXPANSION_DURATION_S + EXPANSION_END_PAUSE_S;
    let alpha = arrow_alpha(elapsed, fade_start);
    let color = egui::Color32::from_rgba_unmultiplied(ORANGE.r(), ORANGE.g(), ORANGE.b(), alpha);
    let center = rect.center();
    let start_angle = ARC_START_DEG.to_radians() + rotation;
    let sweep = ARC_SWEEP_DEG.to_radians();
    let end_angle = start_angle + sweep;
    let direction = egui::vec2(-end_angle.sin(), end_angle.cos());
    let normal = egui::vec2(-direction.y, direction.x);
    let mut arrow = egui::Mesh::default();
    let mut outer_arc = Vec::with_capacity(ARC_SEGMENTS + 1);
    let mut inner_arc = Vec::with_capacity(ARC_SEGMENTS + 1);
    for step in 0..=ARC_SEGMENTS {
        let angle = start_angle + sweep * step as f32 / ARC_SEGMENTS as f32;
        let radial = egui::vec2(angle.cos(), angle.sin());
        let outer = center + radial * scale * (ARC_RADIUS + ARC_HALF_WIDTH);
        let inner = center + radial * scale * (ARC_RADIUS - ARC_HALF_WIDTH);
        outer_arc.push(outer);
        inner_arc.push(inner);
        arrow.colored_vertex(outer, color);
        arrow.colored_vertex(inner, color);
        if step > 0 {
            let previous = (2 * (step - 1)) as u32;
            let current = (2 * step) as u32;
            arrow.add_triangle(previous, previous + 1, current);
            arrow.add_triangle(current, previous + 1, current + 1);
        }
    }
    let base = center + scale * ARC_RADIUS * egui::vec2(end_angle.cos(), end_angle.sin());
    let head_outer = base - scale * normal * ROTATION_HEAD_HALF_WIDTH;
    let head_inner = base + scale * normal * ROTATION_HEAD_HALF_WIDTH;
    let head_tip = base + scale * direction * HEAD_LENGTH;
    let head = arrow.vertices.len() as u32;
    arrow.colored_vertex(head_outer, color);
    arrow.colored_vertex(head_inner, color);
    arrow.colored_vertex(head_tip, color);
    arrow.add_triangle(head, head + 1, head + 2);

    let mut outline = outer_arc;
    outline.extend([head_outer, head_tip, head_inner]);
    outline.extend(inner_arc.into_iter().rev());
    add_outline_antialiasing(&mut arrow, &outline, color, ui.ctx().pixels_per_point());
    ui.painter().add(egui::Shape::mesh(arrow));
}

fn width_arrow(ui: &egui::Ui, rect: egui::Rect) {
    let scale = (rect.width() / ARROW_DESIGN_WIDTH).min(1.0);
    let time = ui.input(|input| input.time) as f32;
    let elapsed = time % LOOP_DURATION_S;
    let progress = (elapsed / EXPANSION_DURATION_S).min(1.0);
    let smooth_progress = smoothstep(f64::from(progress)) as f32;
    let half_width =
        ARROW_MIN_HALF_WIDTH + (ARROW_MAX_HALF_WIDTH - ARROW_MIN_HALF_WIDTH) * smooth_progress;
    let fade_start = EXPANSION_DURATION_S + EXPANSION_END_PAUSE_S;
    let alpha = arrow_alpha(elapsed, fade_start);
    let color = egui::Color32::from_rgba_unmultiplied(ORANGE.r(), ORANGE.g(), ORANGE.b(), alpha);
    let center = rect.center();
    let left_tip = center - egui::vec2(scale * half_width, 0.0);
    let right_tip = center + egui::vec2(scale * half_width, 0.0);
    let left_base_x = left_tip.x + scale * HEAD_LENGTH;
    let right_base_x = right_tip.x - scale * HEAD_LENGTH;
    let head_height = scale * HEAD_HALF_HEIGHT;
    let shaft_height = scale * SHAFT_HALF_HEIGHT;
    let outline = [
        left_tip,
        egui::pos2(left_base_x, center.y - head_height),
        egui::pos2(left_base_x, center.y - shaft_height),
        egui::pos2(right_base_x, center.y - shaft_height),
        egui::pos2(right_base_x, center.y - head_height),
        right_tip,
        egui::pos2(right_base_x, center.y + head_height),
        egui::pos2(right_base_x, center.y + shaft_height),
        egui::pos2(left_base_x, center.y + shaft_height),
        egui::pos2(left_base_x, center.y + head_height),
    ];
    let mut arrow = egui::Mesh::default();
    for point in outline {
        arrow.colored_vertex(point, color);
    }
    arrow.add_triangle(0, 1, 9);
    arrow.add_triangle(2, 3, 7);
    arrow.add_triangle(2, 7, 8);
    arrow.add_triangle(4, 5, 6);
    add_outline_antialiasing(&mut arrow, &outline, color, ui.ctx().pixels_per_point());
    ui.painter().add(egui::Shape::mesh(arrow));
}

fn arrow_alpha(elapsed: f32, fade_start: f32) -> u8 {
    if elapsed < fade_start {
        u8::MAX
    } else {
        (f32::from(u8::MAX) * (1.0 - (elapsed - fade_start) / FADE_DURATION_S).clamp(0.0, 1.0))
            as u8
    }
}

fn add_outline_antialiasing(
    mesh: &mut egui::Mesh,
    outline: &[egui::Pos2],
    color: egui::Color32,
    pixels_per_point: f32,
) {
    let clockwise = outline
        .iter()
        .zip(outline.iter().cycle().skip(1))
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f32>()
        > 0.0;
    let aa = pixels_per_point.recip();
    let fringe = mesh.vertices.len() as u32;
    for index in 0..outline.len() {
        let previous = outline[(index + outline.len() - 1) % outline.len()];
        let point = outline[index];
        let next = outline[(index + 1) % outline.len()];
        let outward = |edge: egui::Vec2| {
            let edge = edge.normalized();
            if clockwise {
                egui::vec2(edge.y, -edge.x)
            } else {
                egui::vec2(-edge.y, edge.x)
            }
        };
        let before = outward(point - previous);
        let after = outward(next - point);
        let miter = (before + after).normalized();
        let offset = miter * (aa / miter.dot(after).max(MIN_MITER_DOT));
        mesh.colored_vertex(point, color);
        mesh.colored_vertex(point + offset, egui::Color32::TRANSPARENT);
    }
    for index in 0..outline.len() as u32 {
        let next = (index + 1) % outline.len() as u32;
        mesh.add_triangle(
            fringe + 2 * index,
            fringe + 2 * index + 1,
            fringe + 2 * next,
        );
        mesh.add_triangle(
            fringe + 2 * next,
            fringe + 2 * index + 1,
            fringe + 2 * next + 1,
        );
    }
}
