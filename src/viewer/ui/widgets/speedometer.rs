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
    let Some(mesh) = ribbon_mesh(path, fraction, color) else {
        return;
    };
    painter.add(egui::Shape::mesh(mesh));
}

fn ribbon_mesh(
    path: &[egui::Pos2],
    fraction: f32,
    color: impl Fn(f32) -> egui::Color32,
) -> Option<egui::Mesh> {
    if fraction <= 0.0 || path.len() < 2 {
        return None;
    }
    let lengths: Vec<_> = path.windows(2).map(|p| p[0].distance(p[1])).collect();
    let total: f32 = lengths.iter().sum();
    if total <= f32::EPSILON {
        return None;
    }
    let bottom_length = lengths[0];
    let filled = total * fraction.clamp(0.0, 1.0);
    let distances: Vec<_> = lengths
        .iter()
        .scan(0.0, |distance, &length| {
            let start = *distance;
            *distance += length;
            Some(start)
        })
        .chain(std::iter::once(total))
        .collect();
    let inner: Vec<_> = path
        .iter()
        .enumerate()
        .map(|(index, &point)| {
            point + inward_miter(path, index) * thickness(distances[index], bottom_length)
        })
        .collect();
    let mut samples = vec![(path[0], inner[0], 0.0)];

    for (index, (segment, &length)) in path.windows(2).zip(&lengths).enumerate() {
        let segment_fraction =
            ((filled - distances[index]) / length.max(f32::EPSILON)).clamp(0.0, 1.0);
        if segment_fraction <= 0.0 {
            break;
        }

        let direction = (segment[1] - segment[0]).normalized();
        let outer = segment[0].lerp(segment[1], segment_fraction);
        let inner = normal_intersection(outer, direction, inner[index], inner[index + 1]);
        samples.push((
            outer,
            inner,
            egui::lerp(distances[index]..=distances[index + 1], segment_fraction),
        ));

        if segment_fraction < 1.0 {
            break;
        }
    }

    let mut mesh = egui::Mesh::default();
    for (index, &(outer, inner, distance)) in samples.iter().enumerate() {
        let vertex_color = color(distance / total);
        mesh.colored_vertex(outer, vertex_color);
        mesh.colored_vertex(inner, vertex_color);
        if index > 0 {
            let current = (index * 2) as u32;
            mesh.add_triangle(current - 2, current - 1, current);
            mesh.add_triangle(current, current - 1, current + 1);
        }
    }
    Some(mesh)
}

fn normal_intersection(
    outer: egui::Pos2,
    direction: egui::Vec2,
    inner_start: egui::Pos2,
    inner_end: egui::Pos2,
) -> egui::Pos2 {
    let inner_direction = inner_end - inner_start;
    let t = direction.dot(outer - inner_start) / direction.dot(inner_direction).max(f32::EPSILON);
    inner_start.lerp(inner_end, t.clamp(0.0, 1.0))
}

fn inward_miter(path: &[egui::Pos2], index: usize) -> egui::Vec2 {
    let inward_normal = |direction: egui::Vec2| egui::vec2(direction.y, -direction.x).normalized();
    if index == 0 {
        return inward_normal(path[1] - path[0]);
    }
    if index + 1 == path.len() {
        return inward_normal(path[index] - path[index - 1]);
    }
    let before = inward_normal(path[index] - path[index - 1]);
    let after = inward_normal(path[index + 1] - path[index]);
    let miter = (before + after).normalized();
    miter / miter.dot(after).max(0.5)
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

    #[test]
    fn crossing_a_corner_does_not_move_the_completed_segment() {
        let path = [
            egui::pos2(0.0, 100.0),
            egui::pos2(100.0, 100.0),
            egui::pos2(130.0, 70.0),
        ];
        let total = 100.0 + 30.0 * 2.0_f32.sqrt();
        let at_corner = ribbon_mesh(&path, 100.0 / total, |_| egui::Color32::WHITE).unwrap();
        let past_corner = ribbon_mesh(&path, 101.0 / total, |_| egui::Color32::WHITE).unwrap();

        assert_eq!(at_corner.vertices, past_corner.vertices[..4]);
        assert_eq!(at_corner.indices, past_corner.indices[..6]);
    }

    #[test]
    fn fill_edge_pivots_around_the_inner_corner() {
        let path = [
            egui::pos2(0.0, 100.0),
            egui::pos2(100.0, 100.0),
            egui::pos2(130.0, 70.0),
        ];
        let corner = path[1] + inward_miter(&path, 1) * HIGH_THICKNESS;
        let middle_outer = egui::pos2(50.0, 100.0);
        let middle = normal_intersection(
            middle_outer,
            egui::Vec2::X,
            path[0] + inward_miter(&path, 0) * LOW_THICKNESS,
            corner,
        );
        let before = normal_intersection(
            egui::pos2(99.0, 100.0),
            egui::Vec2::X,
            path[0] + inward_miter(&path, 0) * LOW_THICKNESS,
            corner,
        );
        let diagonal = (path[2] - path[1]).normalized();
        let after = normal_intersection(
            path[1] + diagonal,
            diagonal,
            corner,
            path[2] + inward_miter(&path, 2) * HIGH_THICKNESS,
        );

        assert!(egui::Vec2::X.dot(middle - middle_outer).abs() < 1e-5);
        assert_eq!(before, corner);
        assert_eq!(after, corner);
    }
}
