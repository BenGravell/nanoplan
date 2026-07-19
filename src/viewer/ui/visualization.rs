use bevy_egui::egui;

use super::hud;
use crate::viewer::colors::PANEL;
use crate::viewer::live::Live;

pub(super) fn visualization_rail(root: &mut egui::Ui, live: &Live, width: f32, compact: bool) {
    egui::Panel::right("visualization_rail")
        .exact_size(width)
        .resizable(false)
        .frame(
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(egui::Margin::same(if compact { 6 } else { 10 })),
        )
        .show(root, |ui| {
            hud::draw(ui, live, compact);
            ui.interact(
                ui.max_rect(),
                ui.id().with("accessibility"),
                egui::Sense::hover(),
            )
            .widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Visualization rail")
            });
        });
}
