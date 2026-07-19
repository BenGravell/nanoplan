use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use super::live::Live;
use super::{UiState, viewport_supported};

pub(crate) mod controls;
mod hud;
mod landing;
mod portrait_prompt;
mod style;
mod visualization;
mod widgets;

use super::colors::PANEL;
pub(crate) use controls::ControlTab;
use controls::control_deck;
use style::configure;
use visualization::visualization_rail;
pub(crate) use widgets::friction_box::FrictionBox;

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut live: NonSendMut<Live>,
    mut configured: Local<bool>,
    mut active_tab: Local<ControlTab>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    if !*configured {
        configure(ctx);
        *configured = true;
        ctx.request_repaint();
        return;
    }
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
        if landing::show(&mut root, &mut state.started) {
            request_exit(&mut app_exit);
        }
        return;
    }
    viewer_layout(&mut root, &mut state, &mut live, &mut active_tab);
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
) {
    let viewport = root.max_rect().size();
    let compact = compact_layout(viewport);
    visualization_rail(root, live, right_rail_width(viewport, compact), compact);
    let frame = egui::Frame::new()
        .fill(PANEL)
        .inner_margin(egui::Margin::same(if compact { 10 } else { 16 }));
    let width = if compact {
        compact_rail_widths(viewport).0
    } else {
        (viewport.x * 0.2).clamp(372.0, 440.0)
    };
    egui::Panel::left("control_deck")
        .exact_size(width)
        .resizable(false)
        .frame(frame)
        .show(root, |ui| {
            control_deck(ui, state, live, active_tab, compact)
        });
}

fn right_rail_width(viewport: egui::Vec2, compact: bool) -> f32 {
    if compact {
        compact_rail_widths(viewport).1
    } else {
        (viewport.x * 0.12).clamp(220.0, 260.0)
    }
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
