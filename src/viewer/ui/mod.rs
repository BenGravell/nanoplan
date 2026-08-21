use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use super::live::Live;
use super::{DrivingCanvas, UiState, viewport_constraints};

pub(crate) mod controls;
mod elements;
mod pages;
mod style;
mod widgets;

pub(crate) use controls::ControlTab;
#[cfg(test)]
use pages::Pages;
pub(crate) use pages::{Navigator, Page};
use style::{configure, scale_to_viewport};
pub(crate) use widgets::friction_box::FrictionBox;

#[allow(clippy::too_many_arguments)]
pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut navigator: ResMut<Navigator>,
    mut state: ResMut<UiState>,
    mut driving_canvas: ResMut<DrivingCanvas>,
    mut live: NonSendMut<Live>,
    mut configured: Local<bool>,
    mut active_tab: Local<ControlTab>,
    mut pages: Local<pages::Pages>,
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
    let viewport_constraints = viewport_constraints(root.max_rect().width(), root.max_rect().height());
    if !viewport_constraints.satisfied() {
        elements::portrait_prompt::show(&mut root, is_mobile_device(), viewport_constraints);
        ctx.request_repaint();
        return;
    }

    let Some(road) = navigator.show(&mut root, &mut pages, &mut state, &mut live, &mut active_tab) else {
        return;
    };
    let zoom = ctx.zoom_factor();
    driving_canvas.rect = Some(Rect::from_corners(
        Vec2::new(road.min.x, road.min.y) * zoom,
        Vec2::new(road.max.x, road.max.y) * zoom,
    ));
}

#[cfg(not(target_family = "wasm"))]
fn is_mobile_device() -> bool {
    false
}

#[cfg(target_family = "wasm")]
fn is_mobile_device() -> bool {
    let Some(window) = web_sys::window() else {
        return false;
    };
    let navigator = window.navigator();
    let user_agent = navigator.user_agent().unwrap_or_default().to_ascii_lowercase();
    user_agent.contains("android")
        || user_agent.contains("iphone")
        || user_agent.contains("ipad")
        || user_agent.contains("ipod")
        || user_agent.contains("mobile")
        || (user_agent.contains("macintosh") && navigator.max_touch_points() > 1)
}

#[cfg(test)]
use pages::driving::{center_rail_rect, compact_layout, side_panel_margin, side_rail_widths};
#[cfg(test)]
fn viewer_layout(
    root: &mut egui::Ui,
    navigator: &mut Navigator,
    pages: &mut pages::Pages,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
) -> egui::Rect {
    navigator.show_driving(
        pages,
        pages::PageContext {
            root,
            state,
            live,
            active_tab,
        },
    )
}

#[cfg(test)]
mod tests;
