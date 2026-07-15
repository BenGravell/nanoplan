use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use nanoplan::PlannerKind;
use nanoplan::metrics::evaluate;
use nanoplan::planning::PLANNING_HORIZON_S;
use nanoplan::prediction::predict;
use nanoplan::simulation::curvature_limit;
use nanoplan::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};
use nanoplan::{Path, State};

use super::friction_box;
use super::live::{CameraTarget, Live, MAX_ZOOM, MIN_ZOOM};
use super::{UiState, is_portrait};

mod portrait_prompt;

const ORANGE: egui::Color32 = egui::Color32::from_rgb(255, 105, 0);
const BLUE: egui::Color32 = egui::Color32::from_rgb(45, 135, 160);
const RED: egui::Color32 = egui::Color32::from_rgb(255, 65, 80);
pub(super) const TEXT: egui::Color32 = egui::Color32::from_rgb(25, 29, 30);
pub(super) const DIM: egui::Color32 = egui::Color32::from_rgb(95, 108, 111);
const PANEL: egui::Color32 = egui::Color32::from_rgb(250, 250, 246);
pub(super) const SURFACE: egui::Color32 = egui::Color32::from_rgb(255, 255, 252);
const CONTROL: egui::Color32 = egui::Color32::from_rgb(232, 235, 229);
pub(super) const FAINT: egui::Color32 = egui::Color32::from_rgb(224, 229, 223);
pub(super) const STEEL: egui::Color32 = egui::Color32::from_rgb(147, 158, 156);

#[derive(Clone, Copy, Default, PartialEq)]
pub(crate) enum ControlTab {
    #[default]
    Planner,
    Camera,
    Visibility,
    Metrics,
}

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut live: NonSendMut<Live>,
    mut configured: Local<bool>,
    mut active_tab: Local<ControlTab>,
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
    if is_portrait(root.max_rect().width(), root.max_rect().height()) {
        portrait_prompt::show(&mut root);
        ctx.request_repaint();
        return;
    }
    viewer_layout(&mut root, &mut state, &mut live, &mut active_tab);
}

fn viewer_layout(
    root: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
) {
    let viewport = root.max_rect().size();
    let compact = compact_layout(viewport);
    if state.show_hud {
        if compact {
            compact_hud(root, live, compact_rail_widths(viewport).1);
        } else {
            hud(root, live, (viewport.x * 0.1).clamp(184.0, 220.0));
        }
    }
    let frame = egui::Frame::new()
        .fill(PANEL)
        .inner_margin(egui::Margin::same(if compact { 10 } else { 16 }));
    if compact {
        egui::Panel::left("control_deck")
            .exact_size(compact_rail_widths(viewport).0)
            .resizable(false)
            .frame(frame)
            .show(root, |ui| {
                control_deck(ui, state, live, active_tab, true);
            });
    } else {
        egui::Panel::left("control_deck")
            .exact_size((viewport.x * 0.2).clamp(372.0, 440.0))
            .resizable(false)
            .frame(frame)
            .show(root, |ui| {
                control_deck(ui, state, live, active_tab, false);
            });
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

fn control_deck(
    ui: &mut egui::Ui,
    state: &mut UiState,
    live: &mut Live,
    active_tab: &mut ControlTab,
    compact: bool,
) {
    brand_header(ui, compact);
    if compact {
        ui.add_space(4.0);
        transport_controls(ui, state, live);
        ui.add_space(6.0);
    } else {
        ui.add_space(12.0);
        transport_controls(ui, state, live);
        ui.add_space(9.0);
    }

    let tabs = [
        (ControlTab::Planner, "PLANNER"),
        (ControlTab::Camera, "CAMERA"),
        (ControlTab::Visibility, "VIZ"),
        (ControlTab::Metrics, "METRICS"),
    ];
    let columns = if compact { 2 } else { 4 };
    for row in tabs.chunks(columns) {
        let width = equal_button_width(ui, columns);
        ui.horizontal(|ui| {
            for &(tab, title) in row {
                let selected = *active_tab == tab;
                if ui
                    .add_sized(
                        [width, 32.0],
                        egui::Button::new(
                            egui::RichText::new(title)
                                .font(caps_font(12.0))
                                .color(if selected { egui::Color32::WHITE } else { TEXT }),
                        )
                        .selected(selected),
                    )
                    .clicked()
                {
                    *active_tab = tab;
                }
            }
        });
    }
    ui.add_space(if compact { 3.0 } else { 9.0 });

    egui::ScrollArea::vertical().show(ui, |ui| match *active_tab {
        ControlTab::Planner => {
            ui.label(
                egui::RichText::new("ACTIVE PLANNER")
                    .font(caps_font(11.0))
                    .color(DIM),
            );
            egui::ComboBox::from_id_salt("planner")
                .selected_text(state.planner.name())
                .width(ui.available_width())
                .show_ui(ui, |ui| {
                    for kind in PlannerKind::ALL {
                        ui.selectable_value(&mut state.planner, kind, kind.name());
                    }
                });
            ui.add_space(6.0);
            ui.add(
                egui::Slider::new(&mut state.preview_s, 0.0..=PLANNING_HORIZON_S as f32)
                    .step_by(0.5)
                    .text(if compact {
                        "preview"
                    } else {
                        "future preview [s]"
                    })
                    .trailing_fill(true),
            );
        }
        ControlTab::Camera => {
            ui.label(
                egui::RichText::new("FOLLOW TARGET")
                    .font(caps_font(11.0))
                    .color(DIM),
            );
            ui.checkbox(&mut live.camera.follow, "Follow camera");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut live.camera.target, CameraTarget::Ego, "Ego");
                ui.selectable_value(
                    &mut live.camera.target,
                    CameraTarget::Track,
                    "Track centerline",
                );
            });
            let heading = match live.camera.target {
                CameraTarget::Ego => "Align to ego heading",
                CameraTarget::Track => "Align to track heading",
            };
            ui.checkbox(&mut live.camera.align_heading, heading);
            ui.checkbox(&mut live.camera.smooth, "Smooth motion");
            ui.add(
                egui::Slider::new(&mut live.camera.zoom, MIN_ZOOM..=MAX_ZOOM)
                    .logarithmic(true)
                    .text("zoom")
                    .custom_formatter(|value, _| format!("{:.0}%", value * 100.0))
                    .trailing_fill(true),
            );
            ui.horizontal(|ui| {
                if ui.button("-15°").clicked() {
                    live.camera.rotation -= 15.0_f32.to_radians();
                    live.camera.align_heading = false;
                }
                if ui.button("NORTH UP").clicked() {
                    live.camera.rotation = 0.0;
                    live.camera.align_heading = false;
                }
                if ui.button("+15°").clicked() {
                    live.camera.rotation += 15.0_f32.to_radians();
                    live.camera.align_heading = false;
                }
                if ui.button("RESET").clicked() {
                    live.reset_camera();
                }
            });
            section_heading(ui, "CONTROLS");
            egui::Grid::new("camera_controls").show(ui, |ui| {
                for (input, action) in [
                    ("MMB / WASD", "PAN"),
                    ("RMB / Q E", "ROTATE"),
                    ("WHEEL", "ZOOM"),
                    ("F", "FOLLOW"),
                    ("R", "RESET"),
                ] {
                    metric(ui, input, action.into());
                }
            });
        }
        ControlTab::Visibility => {
            ui.checkbox(&mut state.show_grid, "Grid");
            ui.checkbox(&mut state.show_centerline, "Track centerline");
            ui.checkbox(&mut state.show_carpet, "Ego carpet");
            ui.checkbox(&mut state.show_plan, "Planned path");
            ui.checkbox(&mut state.show_hud, "Driving HUD");
            if state.planner.has_diagnostics() {
                ui.checkbox(&mut state.show_diag_points, "Search points");
                ui.checkbox(&mut state.show_diag_trajectories, "Candidate trajectories");
                if state.preview_s == 0.0
                    && (state.show_diag_points || state.show_diag_trajectories)
                {
                    ui.colored_label(RED, "Set future preview above zero to record diagnostics.");
                }
            }
        }
        ControlTab::Metrics => {
            let scores = preview_metric_scores(live);
            section_heading(ui, "PLANNER METRICS");
            egui::Grid::new("planner_metrics")
                .num_columns(2)
                .spacing(egui::vec2(28.0, 7.0))
                .show(ui, |ui| {
                    for (label, score) in ["SAFETY", "PROGRESS", "COMFORT"].into_iter().zip(scores)
                    {
                        metric(ui, label, format!("{:.1}%", score * 100.0));
                    }
                });
            section_heading(ui, "DRIVING");
            let actuation = live.world.actuation();
            egui::Grid::new("live_metrics")
                .num_columns(2)
                .spacing(egui::vec2(28.0, 7.0))
                .show(ui, |ui| {
                    metric(ui, "SPEED", format!("{:.1} m/s", live.world.ego.speed));
                    metric(
                        ui,
                        "ACCELERATION",
                        format!("{:+.2} m/s²", actuation.acceleration),
                    );
                    metric(ui, "CURVATURE", format!("{:+.4} m⁻¹", actuation.curvature));
                    metric(
                        ui,
                        "LATEST PLAN",
                        format!("{:.2} ms", live.world.last_plan_ms),
                    );
                });
            section_heading(ui, "PLANNER LATENCY SEAMS");
            egui::Grid::new("latency")
                .num_columns(2)
                .spacing(egui::vec2(28.0, 7.0))
                .show(ui, |ui| {
                    for seam in &live.latency.seams {
                        metric(
                            ui,
                            seam.name,
                            format!("mean {:.3} ms · max {:.3} ms", seam.mean_ms(), seam.max_ms),
                        );
                    }
                });
        }
    });
}

fn transport_controls(ui: &mut egui::Ui, state: &mut UiState, live: &mut Live) {
    let width = equal_button_width(ui, 2);
    ui.horizontal(|ui| {
        let pause_label = if live.paused { "RESUME" } else { "PAUSE" };
        if ui
            .add_sized(
                [width, 36.0],
                egui::Button::new(egui::RichText::new(pause_label).size(13.0)),
            )
            .clicked()
        {
            live.toggle_pause();
        }
        if ui
            .add_sized(
                [width, 36.0],
                egui::Button::new(egui::RichText::new("↻ NEW TRACK").size(13.0)),
            )
            .clicked()
        {
            live.regenerate(live.seed + 1, state.planner);
        }
    });
}

fn brand_header(ui: &mut egui::Ui, compact: bool) {
    let width = ui.available_width();
    egui::Frame::new()
        .fill(ORANGE)
        .inner_margin(egui::Margin::symmetric(8, 5))
        .show(ui, |ui| {
            ui.set_min_width(width - 16.0);
            ui.vertical_centered(|ui| {
                ui.label(
                    egui::RichText::new("NANOPLAN")
                        .font(caps_font(if compact { 20.0 } else { 24.0 }))
                        .color(egui::Color32::WHITE),
                );
            });
        });
}

fn equal_button_width(ui: &egui::Ui, count: usize) -> f32 {
    (ui.available_width() - ui.spacing().item_spacing.x * (count - 1) as f32) / count as f32
}

fn configure(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Light);
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "atkinson".into(),
        egui::FontData::from_static(include_bytes!(
            "../../assets/fonts/AtkinsonHyperlegibleNext.ttf"
        ))
        .into(),
    );
    fonts.font_data.insert(
        "atkinson_mono".into(),
        egui::FontData::from_static(include_bytes!(
            "../../assets/fonts/AtkinsonHyperlegibleMono.ttf"
        ))
        .into(),
    );
    let mut space_grotesk_bold =
        egui::FontData::from_static(include_bytes!("../../assets/fonts/SpaceGrotesk.ttf"));
    space_grotesk_bold.tweak.coords = egui::epaint::text::VariationCoords::new([(b"wght", 700.0)]);
    fonts
        .font_data
        .insert("space_grotesk_bold".into(), space_grotesk_bold.into());
    fonts
        .families
        .get_mut(&egui::FontFamily::Proportional)
        .unwrap()
        .insert(0, "atkinson".into());
    fonts
        .families
        .get_mut(&egui::FontFamily::Monospace)
        .unwrap()
        .insert(0, "atkinson_mono".into());
    fonts.families.insert(
        egui::FontFamily::Name("caps".into()),
        vec!["space_grotesk_bold".into(), "atkinson".into()],
    );
    ctx.set_fonts(fonts);

    let mut style = (*ctx.style_of(egui::Theme::Light)).clone();
    style.spacing.item_spacing = egui::vec2(10.0, 9.0);
    style.spacing.interact_size = egui::vec2(44.0, 32.0);
    style.spacing.button_padding = egui::vec2(12.0, 7.0);
    style.text_styles.insert(
        egui::TextStyle::Body,
        egui::FontId::new(16.0, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Button,
        egui::FontId::new(15.0, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Monospace,
        egui::FontId::new(14.0, egui::FontFamily::Monospace),
    );
    style.visuals.override_text_color = Some(TEXT);
    style.visuals.window_fill = PANEL;
    style.visuals.panel_fill = PANEL;
    style.visuals.window_stroke = egui::Stroke::NONE;
    style.visuals.window_corner_radius = 1.into();
    style.visuals.faint_bg_color = FAINT;
    style.visuals.extreme_bg_color = SURFACE;
    style.visuals.code_bg_color = CONTROL;
    style.visuals.hyperlink_color = ORANGE;
    style.visuals.selection.bg_fill = ORANGE;
    style.visuals.selection.stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    style.visuals.slider_trailing_fill = true;
    style.visuals.widgets.noninteractive.bg_fill = PANEL;
    style.visuals.widgets.noninteractive.weak_bg_fill = PANEL;
    style.visuals.widgets.noninteractive.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.inactive.bg_fill = CONTROL;
    style.visuals.widgets.inactive.weak_bg_fill = CONTROL;
    style.visuals.widgets.inactive.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.inactive.corner_radius = 1.into();
    style.visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(255, 210, 105);
    style.visuals.widgets.hovered.weak_bg_fill = egui::Color32::from_rgb(255, 210, 105);
    style.visuals.widgets.hovered.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.hovered.corner_radius = 1.into();
    style.visuals.widgets.open.bg_fill = SURFACE;
    style.visuals.widgets.open.weak_bg_fill = SURFACE;
    style.visuals.widgets.open.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.open.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.open.corner_radius = 1.into();
    style.visuals.widgets.active.bg_fill = ORANGE;
    style.visuals.widgets.active.weak_bg_fill = ORANGE;
    style.visuals.widgets.active.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.active.fg_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    style.visuals.widgets.active.corner_radius = 1.into();
    ctx.set_style_of(egui::Theme::Light, style);
}

pub(super) fn caps_font(size: f32) -> egui::FontId {
    egui::FontId::new(size, egui::FontFamily::Name("caps".into()))
}

fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(egui::RichText::new(label).font(caps_font(11.0)).color(DIM));
    ui.monospace(value);
    ui.end_row();
}

fn section_heading(ui: &mut egui::Ui, heading: &str) {
    ui.add_space(6.0);
    ui.label(
        egui::RichText::new(heading)
            .font(caps_font(12.0))
            .color(TEXT),
    );
}

fn preview_metric_scores(live: &Live) -> [f64; 3] {
    let ego: Vec<State> = std::iter::once(live.world.ego)
        .chain(live.world.plan.iter().skip(1).copied())
        .collect();
    let path = Path::new(&live.world.road.centerline);
    let actors: Vec<Vec<State>> = live
        .world
        .actors
        .iter()
        .map(|actor| {
            (0..ego.len())
                .map(|i| predict(&actor.state, Some(&path), i as f64 * live.world.dt))
                .collect()
        })
        .collect();
    evaluate(&ego, &actors, &live.world.road).aggregate
}

fn compact_hud(ui: &mut egui::Ui, live: &Live, width: f32) {
    let actuation = live.world.actuation();
    egui::Panel::right("driving_hud")
        .exact_size(width)
        .resizable(false)
        .frame(
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(egui::Margin::same(8)),
        )
        .show(ui, |ui| {
            ui.vertical_centered(|ui| {
                for (label, value) in [
                    ("SPEED", format!("{:.1} m/s", live.world.ego.speed)),
                    ("ACCEL", format!("{:+.1} m/s²", actuation.acceleration)),
                    ("CURV", format!("{:+.3} m⁻¹", actuation.curvature)),
                ] {
                    ui.label(egui::RichText::new(label).font(caps_font(10.0)).color(DIM));
                    ui.monospace(value);
                    ui.add_space(4.0);
                }
                let (plot, _) = ui.allocate_exact_size(
                    egui::vec2(ui.available_width(), ui.available_height()),
                    egui::Sense::hover(),
                );
                friction_box::draw(ui.painter(), plot, &live.friction_box, live.world.ego.speed);
            });
            hud_accessibility(ui);
        });
}

fn hud_accessibility(ui: &egui::Ui) {
    ui.interact(
        ui.max_rect(),
        ui.id().with("accessibility"),
        egui::Sense::hover(),
    )
    .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Driving HUD"));
}

fn hud(ui: &mut egui::Ui, live: &Live, width: f32) {
    const CONTENT_HEIGHT: f32 = 472.0;

    let speed = live.world.ego.speed;
    let actuation = live.world.actuation();
    egui::Panel::right("driving_hud")
        .exact_size(width)
        .resizable(false)
        .frame(egui::Frame::new().fill(PANEL))
        .show(ui, |ui| {
            let rect = ui.max_rect();
            let painter = ui.painter_at(rect);
            let center_x = rect.center().x;
            let top = (rect.center().y - CONTENT_HEIGHT / 2.0).max(rect.top());
            painter.text(
                egui::pos2(center_x, top + 10.0),
                egui::Align2::CENTER_TOP,
                "SPEED",
                caps_font(10.0),
                DIM,
            );
            painter.text(
                egui::pos2(center_x, top + 20.0),
                egui::Align2::CENTER_TOP,
                format!("{:04.1}", speed),
                egui::FontId::monospace(34.0),
                TEXT,
            );
            painter.text(
                egui::pos2(center_x, top + 55.0),
                egui::Align2::CENTER_TOP,
                "m/s",
                egui::FontId::monospace(10.0),
                DIM,
            );

            let accel_track = egui::Rect::from_min_max(
                egui::pos2(center_x - 6.0, top + 98.0),
                egui::pos2(center_x + 6.0, top + 178.0),
            );
            let accel_zero = accel_track.center().y;
            painter.rect_filled(accel_track, 2.0, FAINT);
            painter.line_segment(
                [
                    egui::pos2(accel_track.left() - 5.0, accel_zero),
                    egui::pos2(accel_track.right() + 5.0, accel_zero),
                ],
                egui::Stroke::new(1.0, TEXT),
            );
            let accel_fraction = signed_fraction(
                actuation.acceleration as f32,
                MAX_LON_ACCEL as f32,
                -MIN_LON_ACCEL as f32,
            );
            let accel_end = accel_zero - accel_fraction * accel_track.height() / 2.0;
            let accel_fill = egui::Rect::from_x_y_ranges(
                accel_track.x_range(),
                accel_end.min(accel_zero)..=accel_end.max(accel_zero),
            );
            painter.rect_filled(
                accel_fill,
                2.0,
                if accel_fraction >= 0.0 { BLUE } else { RED },
            );
            painter.text(
                egui::pos2(center_x + 18.0, accel_zero),
                egui::Align2::LEFT_CENTER,
                format!("A {:+.1}", actuation.acceleration),
                egui::FontId::monospace(11.0),
                TEXT,
            );

            let curve_track = egui::Rect::from_min_max(
                egui::pos2(center_x - 68.0, top + 224.0),
                egui::pos2(center_x + 68.0, top + 236.0),
            );
            painter.rect_filled(curve_track, 2.0, FAINT);
            painter.line_segment(
                [
                    egui::pos2(center_x, curve_track.top() - 5.0),
                    egui::pos2(center_x, curve_track.bottom() + 5.0),
                ],
                egui::Stroke::new(1.0, TEXT),
            );
            let curve_fraction = signed_fraction(
                actuation.curvature as f32,
                curvature_limit(speed) as f32,
                curvature_limit(speed) as f32,
            );
            let curve_end = center_x - curve_fraction * curve_track.width() / 2.0;
            let curve_fill = egui::Rect::from_x_y_ranges(
                curve_end.min(center_x)..=curve_end.max(center_x),
                curve_track.y_range(),
            );
            painter.rect_filled(curve_fill, 2.0, BLUE);
            painter.text(
                egui::pos2(center_x, top + 245.0),
                egui::Align2::CENTER_TOP,
                format!("CURV {:+.3}", actuation.curvature),
                egui::FontId::monospace(10.0),
                TEXT,
            );

            friction_box::draw(
                &painter,
                egui::Rect::from_min_size(
                    egui::pos2(rect.left(), top + 288.0),
                    egui::vec2(rect.width(), 184.0),
                ),
                &live.friction_box,
                speed,
            );
            hud_accessibility(ui);
        });
}

fn signed_fraction(value: f32, positive_max: f32, negative_max: f32) -> f32 {
    if value >= 0.0 {
        (value / positive_max).clamp(0.0, 1.0)
    } else {
        (value / negative_max).clamp(-1.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use bevy_egui::egui;
    use egui_kittest::{Harness, kittest::Queryable};

    use super::{
        ControlTab, UiState, compact_layout, compact_rail_widths, configure, portrait_prompt,
        preview_metric_scores, signed_fraction, viewer_layout,
    };
    use crate::viewer::{CANVAS_RGB, live::Live};

    const PHONE_LANDSCAPE_SIZES: [(&str, egui::Vec2); 4] = [
        ("phone-iphone-se-3", egui::vec2(667.0, 375.0)),
        ("phone-galaxy-s24", egui::vec2(780.0, 360.0)),
        ("phone-iphone-14-15-pro", egui::vec2(852.0, 393.0)),
        ("phone-galaxy-a55-wide", egui::vec2(1040.0, 480.0)),
    ];

    struct ViewerHarnessState {
        ui: UiState,
        live: Live,
        tab: ControlTab,
        configured: bool,
    }

    impl Default for ViewerHarnessState {
        fn default() -> Self {
            Self {
                ui: UiState::default(),
                live: Live::default(),
                tab: ControlTab::Planner,
                configured: false,
            }
        }
    }

    #[test]
    fn portrait_prompt_is_the_only_interactive_view() {
        let size = egui::vec2(390.0, 844.0);
        let mut harness = Harness::builder().with_size(size).build_ui_state(
            |ui, configured: &mut bool| {
                let ctx = ui.ctx().clone();
                if !*configured {
                    configure(&ctx);
                    *configured = true;
                    ctx.request_repaint();
                    return;
                }
                let mut root = egui::Ui::new(
                    ctx.clone(),
                    "portrait_render_test".into(),
                    egui::UiBuilder::new().max_rect(ctx.content_rect()),
                );
                portrait_prompt::show(&mut root);
            },
            false,
        );
        harness.run();

        assert!(harness.query_by_label("NANOPLAN").is_some());
        assert!(
            harness
                .query_by_label("TURN YOUR DEVICE SIDEWAYS")
                .is_some()
        );
        assert!(harness.query_by_label("PAUSE").is_none());
    }

    #[test]
    fn layout_only_compacts_for_phone_sized_viewports() {
        for (_, phone) in PHONE_LANDSCAPE_SIZES {
            assert!(compact_layout(phone));
            let (left, right) = compact_rail_widths(phone);
            assert!(phone.x - left - right >= phone.x * 0.4);
        }
        assert!(compact_layout(egui::vec2(960.0, 540.0)));
        assert!(!compact_layout(egui::vec2(1920.0, 1080.0)));
        assert!(!compact_layout(egui::vec2(3440.0, 1440.0)));
        assert!(!compact_layout(egui::vec2(3840.0, 2160.0)));
    }

    #[test]
    fn viewer_elements_fit_and_render_at_target_sizes() {
        let output_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("viewer-renders");
        std::fs::create_dir_all(&output_dir).unwrap();

        let target_sizes = [
            ("desktop-1080p", egui::vec2(1920.0, 1080.0), 1.0),
            ("desktop-ultrawide", egui::vec2(3440.0, 1440.0), 1.0),
            ("desktop-2160p", egui::vec2(1920.0, 1080.0), 2.0),
            (PHONE_LANDSCAPE_SIZES[0].0, PHONE_LANDSCAPE_SIZES[0].1, 1.0),
            (PHONE_LANDSCAPE_SIZES[1].0, PHONE_LANDSCAPE_SIZES[1].1, 1.0),
            (PHONE_LANDSCAPE_SIZES[2].0, PHONE_LANDSCAPE_SIZES[2].1, 1.0),
            (PHONE_LANDSCAPE_SIZES[3].0, PHONE_LANDSCAPE_SIZES[3].1, 1.0),
        ];
        for (name, size, pixels_per_point) in target_sizes {
            let mut harness = Harness::builder()
                .with_size(size)
                .with_pixels_per_point(pixels_per_point)
                .build_ui_state(
                    |ui, state: &mut ViewerHarnessState| {
                        let ctx = ui.ctx().clone();
                        if !state.configured {
                            configure(&ctx);
                            state.configured = true;
                            ctx.request_repaint();
                            return;
                        }
                        let mut root = egui::Ui::new(
                            ctx.clone(),
                            "viewer_render_test".into(),
                            egui::UiBuilder::new().max_rect(ctx.content_rect()),
                        );
                        root.painter().rect_filled(
                            root.max_rect(),
                            0.0,
                            egui::Color32::from_rgb(CANVAS_RGB.0, CANVAS_RGB.1, CANVAS_RGB.2),
                        );
                        viewer_layout(&mut root, &mut state.ui, &mut state.live, &mut state.tab);
                    },
                    ViewerHarnessState::default(),
                );
            harness.run();

            let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size * pixels_per_point);
            let compact = compact_layout(size);
            let (control_width, hud_width) = if compact {
                compact_rail_widths(size)
            } else {
                (
                    (size.x * 0.2).clamp(372.0, 440.0),
                    (size.x * 0.1).clamp(184.0, 220.0),
                )
            };
            for label in [
                "NANOPLAN",
                "PAUSE",
                "↻ NEW TRACK",
                "PLANNER",
                "CAMERA",
                "VIZ",
                "METRICS",
                "ACTIVE PLANNER",
                if compact {
                    "preview"
                } else {
                    "future preview [s]"
                },
            ] {
                for node in harness.get_all_by_label(label) {
                    let rect = node.rect();
                    assert!(
                        screen.contains_rect(rect)
                            && rect.max.x <= control_width * pixels_per_point
                            && rect.is_positive(),
                        "{label:?} is clipped at {name}: {rect:?} outside the control rail"
                    );
                }
            }
            let hud = harness.get_by_label("Driving HUD").rect();
            assert!(
                screen.contains_rect(hud) && hud.min.x >= (size.x - hud_width) * pixels_per_point,
                "HUD is clipped at {name}: {hud:?} outside the HUD rail"
            );

            let pause = harness.get_by_label("PAUSE").rect();
            let new_track = harness.get_by_label("↻ NEW TRACK").rect();
            let planner = harness.get_by_label("PLANNER").rect();
            assert!((pause.width() - new_track.width()).abs() <= 1.0);
            assert!((pause.left() - planner.left()).abs() <= 1.0);
            let last_tab = harness
                .get_by_label(if compact { "CAMERA" } else { "METRICS" })
                .rect();
            assert!((new_track.right() - last_tab.right()).abs() <= 1.0);
            for label in ["CAMERA", "VIZ", "METRICS"] {
                let tab = harness.get_by_label(label).rect();
                assert!(
                    (tab.width() - planner.width()).abs() <= 1.0,
                    "tab widths differ at {name}: PLANNER {planner:?}, {label} {tab:?}"
                );
            }

            harness
                .render()
                .unwrap_or_else(|error| panic!("failed to render {name}: {error}"))
                .save(output_dir.join(format!("{name}.png")))
                .unwrap();
        }
    }

    #[test]
    fn signed_hud_values_use_their_own_side_of_zero() {
        assert_eq!(signed_fraction(5.0, 10.0, 20.0), 0.5);
        assert_eq!(signed_fraction(-5.0, 10.0, 20.0), -0.25);
        assert_eq!(signed_fraction(30.0, 10.0, 20.0), 1.0);
    }

    #[test]
    fn preview_metrics_are_valid_scores() {
        let scores = preview_metric_scores(&Live::default());

        assert!(scores.into_iter().all(|score| (0.0..=1.0).contains(&score)));
    }
}
