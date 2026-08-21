use bevy_egui::egui;

use super::{PageContext, PageOutput, PageView, Route};
use crate::viewer::UiState;
use crate::viewer::colors::{GREY_152, ORANGE, PANEL, SIDE_PANEL, WHITE};
use crate::viewer::live::Live;
use crate::viewer::ui::controls::{ControlTab, control_deck};
use crate::viewer::ui::elements::visualization_rail::visualization_rail;
use crate::viewer::ui::widgets;

#[derive(Default)]
pub(crate) struct DrivingPage;

impl PageView for DrivingPage {
    fn show(&mut self, context: PageContext<'_>) -> PageOutput {
        let PageContext {
            root,
            state,
            live,
            active_tab,
        } = context;
        handle_keyboard_controls(root.ctx(), state, live);
        let (driving_rect, route) = draw(root, state, live, active_tab);
        PageOutput {
            driving_rect: Some(driving_rect),
            route,
        }
    }
}

pub(crate) fn draw(
    root: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
) -> (egui::Rect, Option<Route>) {
    let canvas = root.max_rect();
    let viewport = canvas.size();
    let compact = compact_layout(viewport);
    let (left_width, right_width) = side_rail_widths(viewport);
    let side_margin = side_panel_margin(viewport);

    let mut right_overlay = overlay_root(root, "visualization_overlay");
    visualization_rail(&mut right_overlay, live, right_width, compact);

    let frame = egui::Frame::new()
        .fill(SIDE_PANEL)
        .inner_margin(egui::Margin::same(side_margin));
    let control_width = (left_width - 2.0 * f32::from(side_margin)).max(0.0);
    let mut left_overlay = overlay_root(root, "control_overlay");
    egui::Panel::left("control_deck")
        .exact_size(left_width)
        .resizable(false)
        .frame(frame)
        .show(&mut left_overlay, |ui| {
            let rect = ui.max_rect();
            control_deck(ui, state, live, active_tab, compact, control_width);
            ui.interact(rect, ui.id().with("control_deck"), egui::Sense::hover())
                .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Control deck"));
        });

    let road_rect = center_rail_rect(canvas, left_width, right_width);
    let mut pause_overlay = overlay_root_at(root, "pause_overlay", road_rect);
    pause_rail(&mut pause_overlay, live, compact);
    planner_warning_overlay(root, live, road_rect, compact);
    if state.show_frame_time {
        frame_time_overlay(root, live, road_rect, compact);
    }
    live_minimap(root, state.track, live, road_rect, compact);
    let paused_before_escape = live.paused;
    if !root.ctx().egui_wants_keyboard_input()
        && root.input(|input| input.key_pressed(egui::Key::Escape))
        && !paused_before_escape
    {
        live.toggle_pause();
    }
    let route = pause_modal(root.ctx(), live, compact, paused_before_escape);
    (road_rect, route)
}

fn planner_warning_overlay(root: &egui::Ui, live: &Live, road_rect: egui::Rect, compact: bool) {
    if !live.world.planner_slow {
        return;
    }
    let top = road_rect.top() + if compact { 50.0 } else { 58.0 };
    let rect = egui::Rect::from_min_max(
        egui::pos2(road_rect.left(), top),
        egui::pos2(road_rect.right(), road_rect.bottom()),
    );
    let mut overlay = overlay_root_at(root, "planner_warning_overlay", rect);
    overlay.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
        egui::Frame::new()
            .fill(ORANGE)
            .inner_margin(egui::Margin::symmetric(10, 5))
            .show(ui, |ui| {
                ui.label(
                    egui::RichText::new("PLANNER TOO SLOW · REUSING LAST PLAN")
                        .color(WHITE)
                        .strong(),
                );
            });
    });
}

pub(crate) fn handle_keyboard_controls(ctx: &egui::Context, state: &mut UiState, live: &mut Live) {
    if ctx.egui_wants_keyboard_input() {
        return;
    }
    ctx.input(|input| {
        if input.key_pressed(egui::Key::Space) {
            live.toggle_pause();
        }
        if input.key_pressed(egui::Key::T) {
            state.show_frame_time = !state.show_frame_time;
        }
    });
}

fn frame_time_overlay(root: &egui::Ui, live: &Live, road_rect: egui::Rect, compact: bool) {
    let margin = if compact { 6.0 } else { 10.0 };
    let mut overlay = overlay_root_at(root, "frame_time_overlay", road_rect.shrink(margin));
    overlay.with_layout(egui::Layout::top_down(egui::Align::RIGHT), |ui| {
        egui::Frame::new()
            .fill(PANEL)
            .inner_margin(egui::Margin::symmetric(8, 4))
            .show(ui, |ui| {
                ui.label(egui::RichText::new(format!("FRAME {:.2} ms", live.frame_rate.milliseconds())).monospace());
            });
    });
}

fn live_minimap(root: &egui::Ui, track: usize, live: &Live, road_rect: egui::Rect, compact: bool) {
    let margin = if compact { 6.0 } else { 10.0 };
    let pause_height = if compact { 44.0 } else { 52.0 };
    let size = (road_rect.width() * 0.25).clamp(if compact { 88.0 } else { 120.0 }, 160.0);
    let rect = egui::Rect::from_min_size(
        egui::pos2(
            road_rect.right() - size - margin,
            road_rect.top() + pause_height + margin,
        ),
        egui::Vec2::splat(size),
    );
    let overlay = overlay_root_at(root, "live_minimap", rect);
    overlay.painter().rect_filled(rect, 1.0, PANEL);
    let opponents = live.world.actors.iter().map(|actor| actor.track_x).collect::<Vec<_>>();
    widgets::minimap::paint(
        &overlay,
        track,
        rect.shrink(if compact { 7.0 } else { 10.0 }),
        GREY_152,
        &opponents,
        Some(live.world.track_progress),
    );
    overlay
        .painter()
        .rect_stroke(rect, 1.0, egui::Stroke::new(1.0, GREY_152), egui::StrokeKind::Inside);
    overlay
        .interact(rect, overlay.id().with("accessibility"), egui::Sense::hover())
        .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Image, false, "Track minimap"));
}

fn overlay_root(root: &egui::Ui, id: &'static str) -> egui::Ui {
    overlay_root_at(root, id, root.max_rect())
}

fn overlay_root_at(root: &egui::Ui, id: &'static str, rect: egui::Rect) -> egui::Ui {
    egui::Ui::new(root.ctx().clone(), id.into(), egui::UiBuilder::new().max_rect(rect))
}

pub(crate) fn center_rail_rect(canvas: egui::Rect, left_width: f32, right_width: f32) -> egui::Rect {
    egui::Rect::from_min_max(
        egui::pos2(canvas.left() + left_width, canvas.top()),
        egui::pos2(canvas.right() - right_width, canvas.bottom()),
    )
}

fn pause_rail(root: &mut egui::Ui, live: &mut Live, compact: bool) {
    let margin = if compact { 6 } else { 10 };
    egui::Panel::top("pause_rail")
        .exact_size(if compact { 44.0 } else { 52.0 })
        .frame(egui::Frame::new().fill(PANEL).inner_margin(egui::Margin::same(margin)))
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

fn pause_modal(ctx: &egui::Context, live: &mut Live, compact: bool, allow_escape_close: bool) -> Option<Route> {
    if !live.paused {
        return None;
    }

    let response = egui::Modal::new("pause_menu".into()).show(ctx, |ui| {
        ui.set_min_width(if compact { 220.0 } else { 280.0 });
        ui.vertical_centered(|ui| {
            ui.heading("PAUSED");
        });
        ui.add_space(8.0);
        let width = ui.available_width();
        let resume = ui.add_sized([width, 36.0], egui::Button::new("RESUME"));
        let tracks = ui.add_sized([width, 36.0], egui::Button::new("RETURN TO TRACK SELECT"));
        let start = ui.add_sized([width, 36.0], egui::Button::new("RETURN TO START MENU"));
        (resume.clicked(), tracks.clicked(), start.clicked())
    });

    let (resume, tracks, start) = response.inner;
    if resume || (allow_escape_close && response.should_close()) {
        live.toggle_pause();
        None
    } else if tracks || start {
        live.toggle_pause();
        if tracks {
            Some(Route::TrackSelect)
        } else {
            Some(Route::StartMenu)
        }
    } else {
        None
    }
}

pub(crate) fn side_rail_widths(viewport: egui::Vec2) -> (f32, f32) {
    let width = viewport.y * 0.375;
    (width, width)
}

pub(crate) fn side_panel_margin(viewport: egui::Vec2) -> i8 {
    if !compact_layout(viewport) {
        16
    } else if viewport.y <= 320.0 {
        6
    } else {
        10
    }
}

pub(crate) fn compact_layout(viewport: egui::Vec2) -> bool {
    viewport.x < 900.0 || viewport.y < 600.0
}
