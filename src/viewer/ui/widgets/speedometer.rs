//! Speed gauge with chamfered arm.

use bevy_egui::egui;
use colorgrad::Gradient;

use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::viewer::colors::{DIM, FAINT, GUPPY, TEXT};

use super::super::style::caps_font;

const LOW_THICKNESS: f32 = 11.0;
const HIGH_THICKNESS: f32 = 21.0;
const CHAMFER: f32 = 30.0;

pub(crate) fn draw(painter: &egui::Painter, rect: egui::Rect, velocity: f64) {
    let speed = velocity.abs();
    let fraction = speed_fraction(speed);
    let inset = 3.0;
    let points = [
        egui::pos2(rect.left() + inset, rect.bottom() - inset),
        egui::pos2(rect.right() - CHAMFER - inset, rect.bottom() - inset),
        egui::pos2(rect.right() - inset, rect.bottom() - CHAMFER - inset),
        egui::pos2(rect.right() - inset, rect.top() + inset),
    ];

    draw_ribbon(painter, &points, 1.0, |_| FAINT);
    draw_ribbon(painter, &points, fraction, gauge_color);

    let center = rect.center() - egui::vec2(3.0, 2.0);
    painter.text(
        egui::pos2(center.x, rect.top() + 18.0),
        egui::Align2::CENTER_TOP,
        "SPEED",
        caps_font(10.0),
        DIM,
    );
    painter.text(
        egui::pos2(center.x, rect.top() + 31.0),
        egui::Align2::CENTER_TOP,
        format!("{speed:04.1}"),
        egui::FontId::monospace((rect.height() * 0.25).clamp(20.0, 30.0)),
        TEXT,
    );
    painter.text(
        egui::pos2(center.x, rect.top() + rect.height() * 0.66),
        egui::Align2::CENTER_TOP,
        "m/s",
        egui::FontId::monospace(9.0),
        DIM,
    );
}

fn speed_fraction(speed: f64) -> f32 {
    (speed / *MAX_TERMINAL_SPEED_MPS).clamp(0.0, 1.0) as f32
}

fn draw_ribbon(
    painter: &egui::Painter,
    path: &[egui::Pos2],
    fraction: f32,
    color: impl Fn(f32) -> egui::Color32,
) {
    if fraction <= 0.0 {
        return;
    }
    let lengths: Vec<_> = path.windows(2).map(|p| p[0].distance(p[1])).collect();
    let total: f32 = lengths.iter().sum();
    let bottom_length = lengths[0];
    let filled = total * fraction.clamp(0.0, 1.0);
    let mut samples = vec![(path[0], 0.0)];
    let mut traversed = 0.0;
    for (segment, &length) in path.windows(2).zip(&lengths) {
        if traversed + length < filled {
            traversed += length;
            samples.push((segment[1], traversed));
        } else {
            samples.push((
                segment[0].lerp(
                    segment[1],
                    ((filled - traversed) / length.max(f32::EPSILON)).clamp(0.0, 1.0),
                ),
                filled,
            ));
            break;
        }
    }

    let mut mesh = egui::Mesh::default();
    for index in 0..samples.len() {
        let t = samples[index].1 / total;
        let inward = inward_miter(&samples, index);
        let outer = samples[index].0;
        let inner = outer + inward * thickness(samples[index].1, bottom_length);
        let vertex_color = color(t);
        mesh.colored_vertex(outer, vertex_color);
        mesh.colored_vertex(inner, vertex_color);
        if index > 0 {
            let current = (index * 2) as u32;
            mesh.add_triangle(current - 2, current - 1, current);
            mesh.add_triangle(current, current - 1, current + 1);
        }
    }
    painter.add(egui::Shape::mesh(mesh));
}

fn gauge_color(fraction: f32) -> egui::Color32 {
    let [r, g, b, _] = GUPPY.at(1.0 - fraction.clamp(0.0, 1.0)).to_rgba8();
    egui::Color32::from_rgb(r, g, b)
}

fn thickness(distance: f32, bottom_length: f32) -> f32 {
    egui::lerp(
        LOW_THICKNESS..=HIGH_THICKNESS,
        (distance / bottom_length.max(f32::EPSILON)).clamp(0.0, 1.0),
    )
}

fn inward_miter(samples: &[(egui::Pos2, f32)], index: usize) -> egui::Vec2 {
    let inward_normal = |direction: egui::Vec2| egui::vec2(direction.y, -direction.x).normalized();
    if index == 0 {
        return inward_normal(samples[1].0 - samples[0].0);
    }
    if index + 1 == samples.len() {
        return inward_normal(samples[index].0 - samples[index - 1].0);
    }
    let before = inward_normal(samples[index].0 - samples[index - 1].0);
    let after = inward_normal(samples[index + 1].0 - samples[index].0);
    let miter = (before + after).normalized();
    miter / miter.dot(after).max(0.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speed_is_absolute_and_bounded_by_terminal_speed() {
        assert_eq!(speed_fraction(0.0), 0.0);
        assert_eq!(speed_fraction(-*MAX_TERMINAL_SPEED_MPS / 2.0), 0.5);
        assert_eq!(speed_fraction(*MAX_TERMINAL_SPEED_MPS), 1.0);
        assert_eq!(speed_fraction(*MAX_TERMINAL_SPEED_MPS * 2.0), 1.0);
    }

    #[test]
    fn guppy_gauge_runs_from_blue_to_orange() {
        assert_eq!(gauge_color(0.0), egui::Color32::from_rgb(30, 204, 191));
        assert_eq!(gauge_color(1.0), egui::Color32::from_rgb(250, 145, 79));
    }

    #[test]
    fn bottom_flares_then_chamfer_and_right_edge_stay_at_max_thickness() {
        let bottom = 100.0;
        assert_eq!(thickness(0.0, bottom), LOW_THICKNESS);
        assert_eq!(
            thickness(bottom / 2.0, bottom),
            (LOW_THICKNESS + HIGH_THICKNESS) / 2.0
        );
        assert_eq!(thickness(bottom, bottom), HIGH_THICKNESS);
        assert_eq!(thickness(bottom + 50.0, bottom), HIGH_THICKNESS);
    }
}
