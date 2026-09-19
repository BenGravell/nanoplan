use bevy_egui::egui;

/// Put the value above the track so narrow menus do not steal width from it.
pub(in crate::viewer::ui) fn show(
    ui: &mut egui::Ui,
    width: f32,
    value: impl Into<egui::WidgetText>,
    slider: egui::Slider<'_>,
) -> egui::Response {
    ui.add(egui::Label::new(value).wrap());
    let width = width.min(ui.available_width());
    ui.allocate_ui_with_layout(
        egui::vec2(width, ui.spacing().interact_size.y),
        egui::Layout::top_down(egui::Align::LEFT),
        |ui| {
            ui.spacing_mut().slider_width = width;
            ui.add(slider.show_value(false))
        },
    )
    .inner
}

pub(in crate::viewer::ui) fn paint_ticks(ui: &egui::Ui, response: &egui::Response, count: usize, selected: usize) {
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
