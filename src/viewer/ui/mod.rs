use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use super::live::Live;
use super::{DrivingCanvas, UiState, viewport_supported};

pub(crate) mod controls;
mod hud;
mod landing;
mod portrait_prompt;
mod style;
mod tutorial;
mod visualization;
mod widgets;

use super::colors::{PANEL, SIDE_PANEL};
pub(crate) use controls::ControlTab;
use controls::control_deck;
use style::{configure, scale_to_viewport};
use visualization::visualization_rail;
pub(crate) use widgets::friction_box::FrictionBox;

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut driving_canvas: ResMut<DrivingCanvas>,
    mut live: NonSendMut<Live>,
    mut configured: Local<bool>,
    mut active_tab: Local<ControlTab>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    driving_canvas.rect = None;
    if !*configured {
        configure(ctx);
        *configured = true;
        ctx.request_repaint();
        return;
    }
    scale_to_viewport(ctx);
    let mut root = egui::Ui::new(
        ctx.clone(),
        "viewer_ui".into(),
        egui::UiBuilder::new().max_rect(ctx.content_rect()),
    );
    if !viewport_supported(root.max_rect().width(), root.max_rect().height()) {
        portrait_prompt::show(&mut root);
        ctx.request_repaint();
        return;
    }
    if !state.started {
        if state.tutorial {
            tutorial::show(&mut root, &mut state.tutorial);
        } else {
            let UiState {
                started, tutorial, ..
            } = &mut *state;
            if landing::show(&mut root, started, tutorial) {
                request_exit(&mut app_exit);
            }
        }
        return;
    }
    let (rect, exit_requested) = viewer_layout(&mut root, &mut state, &mut live, &mut active_tab);
    if exit_requested {
        request_exit(&mut app_exit);
    }
    let zoom = ctx.zoom_factor();
    driving_canvas.rect = Some(Rect::from_corners(
        Vec2::new(rect.min.x, rect.min.y) * zoom,
        Vec2::new(rect.max.x, rect.max.y) * zoom,
    ));
}

fn request_exit(app_exit: &mut MessageWriter<AppExit>) {
    #[cfg(target_family = "wasm")]
    if let Some(window) = web_sys::window() {
        let _ = window.close();
    }
    app_exit.write(AppExit::Success);
}

fn viewer_layout(
    root: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
) -> (egui::Rect, bool) {
    let canvas = root.max_rect();
    let viewport = canvas.size();
    let compact = compact_layout(viewport);
    let (left_width, right_width) = side_rail_widths(viewport);

    let mut right_overlay = overlay_root(root, "visualization_overlay");
    visualization_rail(&mut right_overlay, live, right_width, compact);

    let frame = egui::Frame::new()
        .fill(SIDE_PANEL)
        .inner_margin(egui::Margin::same(if compact { 10 } else { 16 }));
    let mut left_overlay = overlay_root(root, "control_overlay");
    egui::Panel::left("control_deck")
        .exact_size(left_width)
        .resizable(false)
        .frame(frame)
        .show(&mut left_overlay, |ui| {
            let rect = ui.max_rect();
            control_deck(ui, state, live, active_tab, compact);
            ui.interact(rect, ui.id().with("control_deck"), egui::Sense::hover())
                .widget_info(|| {
                    egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Control deck")
                });
        });

    let pause_rect = center_rail_rect(canvas, left_width, right_width);
    let mut pause_overlay = overlay_root_at(root, "pause_overlay", pause_rect);
    pause_rail(&mut pause_overlay, live, compact);
    let exit_requested = pause_modal(root.ctx(), state, live, compact);
    (canvas, exit_requested)
}

fn overlay_root(root: &egui::Ui, id: &'static str) -> egui::Ui {
    overlay_root_at(root, id, root.max_rect())
}

fn overlay_root_at(root: &egui::Ui, id: &'static str, rect: egui::Rect) -> egui::Ui {
    egui::Ui::new(
        root.ctx().clone(),
        id.into(),
        egui::UiBuilder::new().max_rect(rect),
    )
}

fn center_rail_rect(canvas: egui::Rect, left_width: f32, right_width: f32) -> egui::Rect {
    egui::Rect::from_min_max(
        egui::pos2(canvas.left() + left_width, canvas.top()),
        egui::pos2(canvas.right() - right_width, canvas.bottom()),
    )
}

fn pause_rail(root: &mut egui::Ui, live: &mut Live, compact: bool) {
    let margin = if compact { 6 } else { 10 };
    egui::Panel::top("pause_rail")
        .exact_size(if compact { 44.0 } else { 52.0 })
        .frame(
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(egui::Margin::same(margin)),
        )
        .show(root, |ui| {
            ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                if ui
                    .add_sized([120.0, ui.available_height()], egui::Button::new("PAUSE"))
                    .clicked()
                {
                    live.toggle_pause();
                }
            });
        });
}

fn pause_modal(ctx: &egui::Context, state: &mut UiState, live: &mut Live, compact: bool) -> bool {
    if !live.paused {
        return false;
    }

    let response = egui::Modal::new("pause_menu".into()).show(ctx, |ui| {
        ui.set_min_width(if compact { 220.0 } else { 280.0 });
        ui.vertical_centered(|ui| {
            ui.heading("PAUSED");
        });
        ui.add_space(8.0);
        let width = ui.available_width();
        let resume = ui.add_sized([width, 36.0], egui::Button::new("RESUME"));
        let start = ui.add_sized([width, 36.0], egui::Button::new("RETURN TO START MENU"));
        let exit = ui.add_sized([width, 36.0], egui::Button::new("EXIT"));
        (resume.clicked(), start.clicked(), exit.clicked())
    });

    let (resume, start, exit) = response.inner;
    if resume || response.should_close() {
        live.toggle_pause();
    } else if start {
        live.toggle_pause();
        state.started = false;
    }
    exit
}

fn side_rail_widths(viewport: egui::Vec2) -> (f32, f32) {
    let width = viewport.y * 0.375;
    (width, width)
}

fn compact_layout(viewport: egui::Vec2) -> bool {
    viewport.x < 900.0 || viewport.y < 600.0
}

#[cfg(test)]
mod tests;
