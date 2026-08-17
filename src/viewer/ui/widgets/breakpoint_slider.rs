use bevy_egui::egui;

use super::stacked_slider;

/// A discrete slider with one evenly spaced position per allowed value.
pub(in crate::viewer::ui) fn show(
    ui: &mut egui::Ui,
    width: f32,
    value: &mut f32,
    breakpoints: &[f32],
    suffix: &str,
) -> egui::Response {
    assert!(!breakpoints.is_empty(), "breakpoint slider needs values");
    debug_assert!(breakpoints.windows(2).all(|pair| pair[0] < pair[1]));

    let mut index = nearest_index(*value, breakpoints);
    let label = format!("{:.0}{suffix}", breakpoints[index]);
    let response = stacked_slider::show(
        ui,
        width,
        label,
        egui::Slider::new(&mut index, 0..=breakpoints.len() - 1),
    );
    paint_breakpoints(ui, &response, breakpoints.len(), index);
    if response.changed() {
        *value = breakpoints[index];
    }
    response
}

fn paint_breakpoints(ui: &egui::Ui, response: &egui::Response, count: usize, selected: usize) {
    let rail_half_height = ui.spacing().slider_rail_height / 2.0;
    let handle_radius = response.rect.height() / 2.5;
    let handle_half_width = match ui.visuals().handle_shape {
        egui::style::HandleShape::Circle => handle_radius,
        egui::style::HandleShape::Rect { aspect_ratio } => handle_radius * aspect_ratio,
    };
    let x_range = response.rect.x_range().shrink(handle_half_width);
    let y_range =
        (response.rect.center().y - rail_half_height + 2.0)..=(response.rect.center().y + rail_half_height - 2.0);

    for breakpoint in 0..count {
        if breakpoint == selected {
            continue;
        }
        let position = breakpoint as f32 / (count - 1) as f32;
        let x = egui::lerp(x_range, position);
        let stroke = if breakpoint < selected {
            ui.visuals().selection.stroke
        } else {
            ui.visuals().widgets.inactive.fg_stroke
        };
        ui.painter()
            .line_segment([egui::pos2(x, *y_range.start()), egui::pos2(x, *y_range.end())], stroke);
    }
}

fn nearest_index(value: f32, breakpoints: &[f32]) -> usize {
    breakpoints
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| (*a - value).abs().total_cmp(&(*b - value).abs()))
        .map(|(index, _)| index)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    const BREAKPOINTS: [f32; 4] = [5.0, 20.0, 100.0, 500.0];

    #[test]
    fn values_snap_to_the_nearest_breakpoint() {
        assert_eq!(nearest_index(5.0, &BREAKPOINTS), 0);
        assert_eq!(nearest_index(99.0, &BREAKPOINTS), 2);
        assert_eq!(nearest_index(500.0, &BREAKPOINTS), 3);
    }
}
