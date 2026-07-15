use bevy_egui::egui;
use nanoplan::math::smoothstep;

use super::{DIM, FAINT, ORANGE, PANEL, SURFACE, TEXT, brand_header, caps_font};

const LOOP_DURATION_S: f32 = 3.0;
const ROTATION_DURATION_S: f32 = 2.2;
const ROTATION_END_PAUSE_S: f32 = 0.2;
const FADE_DURATION_S: f32 = 0.3;
const ROTATION_RADIANS: f32 = std::f32::consts::FRAC_PI_2;
const ARC_RADIUS: f32 = 70.0;
const ARC_HALF_WIDTH: f32 = 5.5;
const ARC_START_DEG: f32 = 105.0;
const ARC_SWEEP_DEG: f32 = 250.0;
const ARC_SEGMENTS: usize = 48;
const HEAD_HALF_WIDTH: f32 = 22.0;
const HEAD_LENGTH: f32 = 29.0;
const MIN_MITER_DOT: f32 = 0.25;

pub(super) fn show(root: &mut egui::Ui) {
    root.painter().rect_filled(root.max_rect(), 0.0, PANEL);
    brand_header(root, false);

    egui::Area::new("portrait_prompt".into())
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(root.ctx(), |ui| {
            egui::Frame::new()
                .fill(SURFACE)
                .stroke(egui::Stroke::new(1.0, FAINT))
                .corner_radius(12)
                .inner_margin(egui::Margin::same(24))
                .show(ui, |ui| {
                    ui.set_width((root.max_rect().width() - 48.0).min(380.0));
                    ui.vertical_centered(|ui| {
                        let (rect, _) = ui.allocate_exact_size(
                            egui::vec2(ui.available_width(), 190.0),
                            egui::Sense::hover(),
                        );
                        rotation_arrow(ui, rect);
                        ui.label(
                            egui::RichText::new("TURN YOUR DEVICE SIDEWAYS")
                                .font(caps_font(22.0))
                                .color(TEXT),
                        );
                        ui.add_space(8.0);
                        ui.label(
                            egui::RichText::new("Nanoplan requires landscape orientation.")
                                .size(16.0)
                                .color(DIM),
                        );
                    });
                });
        });
}

fn rotation_arrow(ui: &egui::Ui, rect: egui::Rect) {
    let time = ui.input(|input| input.time) as f32;
    let elapsed = time % LOOP_DURATION_S;
    let progress = (elapsed / ROTATION_DURATION_S).min(1.0);
    let smooth_progress = smoothstep(f64::from(progress)) as f32;
    let rotation = ROTATION_RADIANS * smooth_progress;
    let fade_start = ROTATION_DURATION_S + ROTATION_END_PAUSE_S;
    let alpha = if elapsed < fade_start {
        u8::MAX
    } else {
        (f32::from(u8::MAX) * (1.0 - (elapsed - fade_start) / FADE_DURATION_S).clamp(0.0, 1.0))
            as u8
    };
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
        let outer = center + radial * (ARC_RADIUS + ARC_HALF_WIDTH);
        let inner = center + radial * (ARC_RADIUS - ARC_HALF_WIDTH);
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
    let base = center + ARC_RADIUS * egui::vec2(end_angle.cos(), end_angle.sin());
    let head_outer = base - normal * HEAD_HALF_WIDTH;
    let head_inner = base + normal * HEAD_HALF_WIDTH;
    let head_tip = base + direction * HEAD_LENGTH;
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
