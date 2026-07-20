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

use super::colors::PANEL;
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
    let rect = viewer_layout(&mut root, &mut state, &mut live, &mut active_tab);
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
) -> egui::Rect {
    let viewport = root.max_rect().size();
    let compact = compact_layout(viewport);
    visualization_rail(root, live, right_rail_width(viewport, compact), compact);
    let frame = egui::Frame::new()
        .fill(PANEL)
        .inner_margin(egui::Margin::same(if compact { 10 } else { 16 }));
    let width = if compact {
        compact_rail_widths(viewport).0
    } else {
        desktop_rail_widths(viewport).0
    };
    egui::Panel::left("control_deck")
        .exact_size(width)
        .resizable(false)
        .frame(frame)
        .show(root, |ui| {
            control_deck(ui, state, live, active_tab, compact)
        });
    root.available_rect_before_wrap()
}

fn right_rail_width(viewport: egui::Vec2, compact: bool) -> f32 {
    if compact {
        compact_rail_widths(viewport).1
    } else {
        desktop_rail_widths(viewport).1
    }
}

fn desktop_rail_widths(viewport: egui::Vec2) -> (f32, f32) {
    // Scale against the viewport height so 1440p and 2160p layouts retain the
    // proportions of a 16:9 1080p display without ballooning on ultrawides.
    let reference_width = viewport.y * 16.0 / 9.0;
    (
        (reference_width * 0.2).max(372.0),
        (reference_width * 0.12).max(220.0),
    )
}

fn compact_layout(viewport: egui::Vec2) -> bool {
    viewport.x < 900.0 || viewport.y < 600.0
}

fn compact_rail_widths(viewport: egui::Vec2) -> (f32, f32) {
    (
        (viewport.x * 0.31).clamp(252.0, 280.0),
        (viewport.x * 0.17).clamp(132.0, 152.0),
    )
}

#[cfg(test)]
mod tests;
