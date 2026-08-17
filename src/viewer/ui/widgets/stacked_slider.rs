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
