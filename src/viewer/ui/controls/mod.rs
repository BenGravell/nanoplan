use bevy_egui::egui;

use crate::viewer::UiState;
use crate::viewer::live::Live;
use crate::viewer::ui::style::scroll_area;

mod camera;
pub(crate) mod metrics;
mod opponents;
mod planner;
mod timing;
mod visibility;

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum ControlTab {
    #[default]
    Planner,
    Opponents,
    Camera,
    Visibility,
    Metrics,
    Timing,
}

pub(super) fn control_deck(
    ui: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
    compact: bool,
    content_width: f32,
) {
    ui.set_max_width(content_width);
    let selector = ui.horizontal(|ui| {
        let gap = if compact { 2.0 } else { 4.0 };
        let side = (content_width - gap * 5.0) / 6.0;
        ui.spacing_mut().item_spacing.x = gap;
        ui.spacing_mut().button_padding = egui::Vec2::ZERO;
        ui.spacing_mut().interact_size = egui::Vec2::splat(side);
        for tab in ControlTab::ALL {
            let selected = *active_tab == tab;
            let image = egui::Image::new(tab.icon()).fit_to_exact_size(egui::Vec2::splat((side * 0.6).min(24.0)));
            let response = ui.add_sized(
                [side, side],
                egui::Button::image(image)
                    .image_tint_follows_text_color(true)
                    .selected(selected),
            );
            response.widget_info(|| {
                egui::WidgetInfo::selected(egui::WidgetType::Button, true, selected, format!("{} tab", tab.label()))
            });
            if response.on_hover_text(tab.label()).clicked() {
                *active_tab = tab;
            }
        }
    });
    selector
        .response
        .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "OPTIONS"));
    ui.add_space(if compact { 6.0 } else { 9.0 });

    scroll_area(ui, egui::ScrollArea::vertical().max_width(content_width), |ui| {
        ui.set_width(content_width);
        match *active_tab {
            ControlTab::Planner => planner::show(ui, state, content_width),
            ControlTab::Opponents => opponents::show(ui, state, live, content_width),
            ControlTab::Camera => camera::show(ui, live, compact, content_width),
            ControlTab::Visibility => visibility::show(ui, state, compact, content_width),
            ControlTab::Metrics => metrics::show(ui, live),
            ControlTab::Timing => timing::show(ui, live),
        }
    });
}

impl ControlTab {
    const ALL: [Self; 6] = [
        Self::Planner,
        Self::Opponents,
        Self::Camera,
        Self::Visibility,
        Self::Metrics,
        Self::Timing,
    ];

    fn icon(self) -> egui::ImageSource<'static> {
        match self {
            Self::Planner => egui::include_image!("../../../../assets/icons/lucide/route.svg"),
            Self::Opponents => egui::include_image!("../../../../assets/icons/lucide/car.svg"),
            Self::Camera => egui::include_image!("../../../../assets/icons/lucide/camera.svg"),
            Self::Visibility => egui::include_image!("../../../../assets/icons/lucide/eye.svg"),
            Self::Metrics => egui::include_image!("../../../../assets/icons/lucide/chart-no-axes-combined.svg"),
            Self::Timing => egui::include_image!("../../../../assets/icons/lucide/timer.svg"),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Planner => "PLANNER",
            Self::Opponents => "OPPONENTS",
            Self::Camera => "CAMERA",
            Self::Visibility => "VIZ",
            Self::Metrics => "METRICS",
            Self::Timing => "TIMING",
        }
    }
}
