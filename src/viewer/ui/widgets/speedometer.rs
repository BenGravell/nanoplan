//! Speed gauge with chamfered arm.

use bevy_egui::egui;
use colorgrad::Gradient;

use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::viewer::colors::{DIM, FAINT, GUPPY, TEXT};

const LOW_THICKNESS: f32 = 11.0;
const HIGH_THICKNESS: f32 = 21.0;
const CHAMFER: f32 = 30.0;
const INSET: f32 = 3.0;

pub(crate) fn draw(painter: &egui::Painter, rect: egui::Rect, velocity: f64) {
    let speed = velocity.abs();
    let fraction = speed_fraction(speed);
    let points = [
        egui::pos2(rect.left() + INSET, rect.bottom() - INSET),
        egui::pos2(rect.right() - CHAMFER - INSET, rect.bottom() - INSET),
        egui::pos2(rect.right() - INSET, rect.bottom() - CHAMFER - INSET),
        egui::pos2(rect.right() - INSET, rect.top() + INSET),
    ];

    draw_ribbon(painter, &points, 1.0, |_| FAINT);
    draw_ribbon(painter, &points, fraction, gauge_color);

    // Center labels in the area left by the bottom and right gauge arms.
    // Centering them in the outer rect pushes wide values into the right arm
    // on narrow rails.
    let free_area = free_area(rect);
    let center = free_area.center();
    let value = format!("{speed:04.1}");
    let mut value_font = egui::FontId::monospace((rect.height() * 0.25).clamp(20.0, 30.0));
    let value_width = painter
        .layout_no_wrap(value.clone(), value_font.clone(), TEXT)
        .size()
        .x;
    if value_width > free_area.width() {
        value_font.size *= free_area.width() / value_width;
    }
    let value_bottom = center.y + value_font.size * 0.5;
    painter.text(center, egui::Align2::CENTER_CENTER, value, value_font, TEXT);
    painter.text(
        egui::pos2(center.x, value_bottom),
        egui::Align2::CENTER_TOP,
        "m/s",
        egui::FontId::monospace(12.0),
        DIM,
    );
}

fn free_area(rect: egui::Rect) -> egui::Rect {
    egui::Rect::from_min_max(
        rect.min + egui::Vec2::splat(INSET),
        rect.max - egui::Vec2::splat(INSET + HIGH_THICKNESS),
    )
}

fn speed_fraction(speed: f64) -> f32 {
    (speed.abs() / *MAX_TERMINAL_SPEED_MPS).clamp(0.0, 1.0) as f32
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
    let outer_lengths: Vec<_> = path.windows(2).map(|p| p[0].distance(p[1])).collect();
    let bottom_length = outer_lengths[0];
    let outer_distances: Vec<_> = outer_lengths
        .iter()
        .scan(0.0, |distance, &length| {
            let start = *distance;
            *distance += length;
            Some(start)
        })
        .chain(std::iter::once(outer_lengths.iter().sum()))
        .collect();
    let inner: Vec<_> = path
        .iter()
        .enumerate()
        .map(|(index, &point)| {
            point + inward_miter(path, index) * thickness(outer_distances[index], bottom_length)
        })
        .collect();
    let sections = ribbon_sections(path, &inner);
    let lengths: Vec<_> = sections
        .windows(2)
        .map(|section| section_center(section[0]).distance(section_center(section[1])))
        .collect();
    let total: f32 = lengths.iter().sum();
    if total <= f32::EPSILON {
        return None;
    }
    let filled = total * fraction.clamp(0.0, 1.0);
    let mut samples = vec![(sections[0].0, sections[0].1, 0.0)];
    let mut distance = 0.0;

    for (section, &length) in sections.windows(2).zip(&lengths) {
        let section_fraction = ((filled - distance) / length.max(f32::EPSILON)).clamp(0.0, 1.0);
        if section_fraction <= 0.0 {
            break;
        }
        samples.push((
            section[0].0.lerp(section[1].0, section_fraction),
            section[0].1.lerp(section[1].1, section_fraction),
            distance + length * section_fraction,
        ));
        if section_fraction < 1.0 {
            break;
        }
        distance += length;
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

fn ribbon_sections(outer: &[egui::Pos2], inner: &[egui::Pos2]) -> Vec<(egui::Pos2, egui::Pos2)> {
    let mut sections = vec![(outer[0], inner[0])];
    for index in 1..outer.len() - 1 {
        let blend = outer[index]
            .distance(inner[index])
            .min(outer[index - 1].distance(outer[index]) * 0.45)
            .min(outer[index].distance(outer[index + 1]) * 0.45)
            .min(inner[index - 1].distance(inner[index]) * 0.45)
            .min(inner[index].distance(inner[index + 1]) * 0.45);
        let inner_before = point_before(inner[index], inner[index - 1], blend);
        let outer_before = project_onto_segment(inner_before, outer[index - 1], outer[index]);
        let outer_after = point_before(outer[index], outer[index + 1], blend);
        let inner_after = project_onto_segment(outer_after, inner[index], inner[index + 1]);

        // The edge is perpendicular on either side of the corner. Within the
        // blend, both boundary tracers advance at constant rates; their
        // different corner times rotate the connecting edge without a pause.
        let outer_transition = [outer_before, outer[index], outer_after];
        let inner_transition = [inner_before, inner[index], inner_after];
        sections.push((outer_before, inner_before));
        let mut stops = vec![0.0, 1.0];
        stops.push(path_corner_fraction(&outer_transition));
        stops.push(path_corner_fraction(&inner_transition));
        stops.sort_by(f32::total_cmp);
        stops.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
        sections.extend(stops.into_iter().skip(1).map(|fraction| {
            (
                trace_path(&outer_transition, fraction),
                trace_path(&inner_transition, fraction),
            )
        }));
    }
    sections.push((*outer.last().unwrap(), *inner.last().unwrap()));
    sections
}

fn point_before(start: egui::Pos2, toward: egui::Pos2, distance: f32) -> egui::Pos2 {
    start + (toward - start).normalized() * distance
}

fn path_corner_fraction(path: &[egui::Pos2; 3]) -> f32 {
    let first = path[0].distance(path[1]);
    first / (first + path[1].distance(path[2])).max(f32::EPSILON)
}

fn trace_path(path: &[egui::Pos2; 3], fraction: f32) -> egui::Pos2 {
    let first = path[0].distance(path[1]);
    let second = path[1].distance(path[2]);
    let distance = (first + second) * fraction.clamp(0.0, 1.0);
    if distance <= first {
        path[0].lerp(path[1], distance / first.max(f32::EPSILON))
    } else {
        path[1].lerp(path[2], (distance - first) / second.max(f32::EPSILON))
    }
}

fn project_onto_segment(point: egui::Pos2, start: egui::Pos2, end: egui::Pos2) -> egui::Pos2 {
    let segment = end - start;
    let t = segment.dot(point - start) / segment.length_sq().max(f32::EPSILON);
    start.lerp(end, t.clamp(0.0, 1.0))
}

fn section_center(section: (egui::Pos2, egui::Pos2)) -> egui::Pos2 {
    section.0.lerp(section.1, 0.5)
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
        assert_eq!(gauge_color(0.0), egui::Color32::from_rgb(42, 182, 196));
        assert_eq!(gauge_color(1.0), egui::Color32::from_rgb(254, 107, 44));
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
        let outer_distances = [0.0, 100.0, 100.0 + 30.0 * 2.0_f32.sqrt()];
        let inner: Vec<_> = path
            .iter()
            .enumerate()
            .map(|(index, &point)| {
                point + inward_miter(&path, index) * thickness(outer_distances[index], 100.0)
            })
            .collect();
        let sections = ribbon_sections(&path, &inner);
        let lengths: Vec<_> = sections
            .windows(2)
            .map(|section| section_center(section[0]).distance(section_center(section[1])))
            .collect();
        let total: f32 = lengths.iter().sum();
        let at_corner = ribbon_mesh(&path, lengths[0] / total, |_| egui::Color32::WHITE).unwrap();
        let past_corner =
            ribbon_mesh(&path, (lengths[0] + 1.0) / total, |_| egui::Color32::WHITE).unwrap();

        assert_eq!(at_corner.vertices, past_corner.vertices[..4]);
        assert_eq!(at_corner.indices, past_corner.indices[..6]);
    }

    #[test]
    fn fill_edge_stays_square_on_arms_and_blends_across_the_corner() {
        let path = [
            egui::pos2(0.0, 100.0),
            egui::pos2(100.0, 100.0),
            egui::pos2(130.0, 70.0),
        ];
        let outer_distances = [0.0, 100.0, 100.0 + 30.0 * 2.0_f32.sqrt()];
        let inner: Vec<_> = path
            .iter()
            .enumerate()
            .map(|(index, &point)| {
                point + inward_miter(&path, index) * thickness(outer_distances[index], 100.0)
            })
            .collect();
        let sections = ribbon_sections(&path, &inner);

        assert!(sections.len() >= 5);
        assert!((sections[0].0.x - sections[0].1.x).abs() < 1e-5);
        assert!((sections[1].0.x - sections[1].1.x).abs() < 1e-5);
        let diagonal = (path[2] - path[1]).normalized();
        assert!(
            diagonal
                .dot(sections[sections.len() - 2].0 - sections[sections.len() - 2].1)
                .abs()
                < 1e-5
        );
        assert!(
            diagonal
                .dot(sections[sections.len() - 1].0 - sections[sections.len() - 1].1)
                .abs()
                < 1e-5
        );
        assert!(sections[2].0.distance(sections[1].0) > 0.0);
        assert!(sections[2].1.distance(sections[1].1) > 0.0);
    }

    #[test]
    fn gauge_starts_vertical_and_finishes_horizontal() {
        let path = [
            egui::pos2(0.0, 100.0),
            egui::pos2(100.0, 100.0),
            egui::pos2(130.0, 70.0),
            egui::pos2(130.0, 0.0),
        ];
        let outer_distances = [
            0.0,
            100.0,
            100.0 + 30.0 * 2.0_f32.sqrt(),
            170.0 + 30.0 * 2.0_f32.sqrt(),
        ];
        let inner: Vec<_> = path
            .iter()
            .enumerate()
            .map(|(index, &point)| {
                point + inward_miter(&path, index) * thickness(outer_distances[index], 100.0)
            })
            .collect();
        let sections = ribbon_sections(&path, &inner);

        assert!((sections[0].0.x - sections[0].1.x).abs() < 1e-5);
        assert!((sections[1].0.x - sections[1].1.x).abs() < 1e-5);
        assert!((sections[sections.len() - 2].0.y - sections[sections.len() - 2].1.y).abs() < 1e-5);
        assert!((sections[sections.len() - 1].0.y - sections[sections.len() - 1].1.y).abs() < 1e-5);
    }

    #[test]
    fn gauge_keeps_its_content_area_clear_of_its_arms() {
        let gauge =
            egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(120.0, 120.0 * 3.0 / 5.0));
        let free = free_area(gauge);

        assert!(free.right() <= gauge.right() - INSET - HIGH_THICKNESS);
        assert!(free.bottom() <= gauge.bottom() - INSET - HIGH_THICKNESS);
    }
}
