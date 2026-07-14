use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use nanoplan::PlannerKind;
use nanoplan::planning::{PLANNING_HORIZON_S, model::curvature_limit};
use nanoplan::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

use super::UiState;
use super::live::{Live, MAX_ZOOM, MIN_ZOOM};

const PINK: egui::Color32 = egui::Color32::from_rgb(255, 58, 190);
const BLUE: egui::Color32 = egui::Color32::from_rgb(40, 160, 255);
const RED: egui::Color32 = egui::Color32::from_rgb(255, 65, 80);
const TEXT: egui::Color32 = egui::Color32::from_rgb(226, 241, 250);
const DIM: egui::Color32 = egui::Color32::from_rgb(105, 135, 153);
const PANEL: egui::Color32 = egui::Color32::from_rgba_premultiplied(10, 15, 24, 242);
const STEEL: egui::Color32 = egui::Color32::from_rgb(48, 70, 84);

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut live: NonSendMut<Live>,
    mut configured: Local<bool>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    if !*configured {
        configure(ctx);
        *configured = true;
        ctx.request_repaint();
        return;
    }
    if state.show_hud {
        hud(ctx, &live);
    }
    egui::Window::new("control_deck")
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(18.0, 18.0))
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .frame(
            egui::Frame::window(&ctx.style_of(egui::Theme::Dark))
                .fill(PANEL)
                .stroke(egui::Stroke::new(1.0, STEEL))
                .corner_radius(3.0)
                .inner_margin(egui::Margin::same(16)),
        )
        .show(ctx, |ui| {
            ui.set_min_width(340.0);
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new("NANOPLAN")
                        .font(heading_font(24.0))
                        .color(TEXT),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        egui::RichText::new("RACING SYSTEM / LIVE")
                            .monospace()
                            .size(10.0)
                            .color(PINK),
                    );
                });
            });
            let header_line = ui.available_rect_before_wrap();
            ui.painter().line_segment(
                [header_line.left_bottom(), header_line.right_bottom()],
                egui::Stroke::new(2.0, PINK),
            );
            ui.add_space(12.0);
            ui.horizontal(|ui| {
                let pause_label = if live.paused { "RESUME" } else { "PAUSE" };
                if ui
                    .add_sized([154.0, 36.0], egui::Button::new(pause_label))
                    .clicked()
                {
                    live.toggle_pause();
                }
                if ui
                    .add_sized([154.0, 36.0], egui::Button::new("↻ NEW TRACK"))
                    .clicked()
                {
                    let seed = live.seed + 1;
                    live.regenerate(seed, state.planner);
                }
            });

            let deck_height = (ctx.content_rect().height() - 130.0).max(300.0);
            egui::ScrollArea::vertical()
                .max_height(deck_height)
                .min_scrolled_height(deck_height)
                .auto_shrink([true, false])
                .show(ui, |ui| {
                    section_heading(ui, "PLANNER + CONFIG");
                    ui.label(egui::RichText::new("ACTIVE PLANNER").small().color(DIM));
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
                        egui::Slider::new(
                            &mut state.preview_s,
                            0.0..=PLANNING_HORIZON_S as f32,
                        )
                        .text("future preview [s]")
                        .trailing_fill(true),
                    );

                    section_heading(ui, "CAMERA");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut live.camera.follow_position, "Follow ego");
                        ui.checkbox(&mut live.camera.follow_heading, "Ego heading");
                    });
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut live.camera.align_track, "Track centerline pose");
                        ui.checkbox(&mut live.camera.smooth, "Smooth motion");
                    });
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
                            live.camera.follow_heading = false;
                            live.camera.align_track = false;
                        }
                        if ui.button("NORTH UP").clicked() {
                            live.camera.rotation = 0.0;
                            live.camera.follow_heading = false;
                            live.camera.align_track = false;
                        }
                        if ui.button("+15°").clicked() {
                            live.camera.rotation += 15.0_f32.to_radians();
                            live.camera.follow_heading = false;
                            live.camera.align_track = false;
                        }
                        if ui.button("RESET").clicked() {
                            live.reset_camera();
                        }
                    });
                    ui.label(
                        egui::RichText::new(
                            "MMB / WASD PAN   ·   RMB / Q E ROTATE   ·   WHEEL ZOOM   ·   F FOLLOW   ·   R RESET",
                        )
                        .monospace()
                        .size(10.0)
                        .color(DIM),
                    );

                    section_heading(ui, "VIEWER VISIBILITY");
                    ui.horizontal_wrapped(|ui| {
                        ui.checkbox(&mut state.show_grid, "Grid");
                        ui.checkbox(&mut state.show_plan, "Planned path");
                        ui.checkbox(&mut state.show_hud, "Driving HUD");
                    });

                    section_heading(ui, "METRICS");
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
                    egui::CollapsingHeader::new("Planner latency seams")
                        .default_open(false)
                        .show(ui, |ui| {
                            egui::Grid::new("latency").striped(true).show(ui, |ui| {
                                ui.weak("SEAM");
                                ui.weak("MEAN");
                                ui.weak("MAX");
                                ui.end_row();
                                for seam in &live.latency.seams {
                                    ui.label(seam.name);
                                    ui.monospace(format!("{:.3} ms", seam.mean_ms()));
                                    ui.monospace(format!("{:.3} ms", seam.max_ms));
                                    ui.end_row();
                                }
                            });
                        });

                    section_heading(ui, "DIAGNOSTICS");
                    if state.planner.has_diagnostics() {
                        ui.checkbox(&mut state.show_diag_points, "Search points");
                        ui.checkbox(
                            &mut state.show_diag_trajectories,
                            "Candidate trajectories",
                        );
                        if state.preview_s == 0.0
                            && (state.show_diag_points || state.show_diag_trajectories)
                        {
                            ui.colored_label(
                                RED,
                                "Set future preview above zero to record diagnostics.",
                            );
                        }
                    } else {
                        ui.label(
                            egui::RichText::new("No visual diagnostics for this planner.")
                                .color(DIM),
                        );
                    }
                });
        });
}

fn configure(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
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
    fonts.font_data.insert(
        "space_grotesk".into(),
        egui::FontData::from_static(include_bytes!("../../assets/fonts/SpaceGrotesk.ttf")).into(),
    );
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
        egui::FontFamily::Name("heading".into()),
        vec!["space_grotesk".into(), "atkinson".into()],
    );
    ctx.set_fonts(fonts);

    let mut style = (*ctx.style_of(egui::Theme::Dark)).clone();
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
    style.visuals.window_stroke = egui::Stroke::new(1.0, STEEL);
    style.visuals.window_corner_radius = 3.into();
    style.visuals.faint_bg_color = egui::Color32::from_rgb(15, 24, 35);
    style.visuals.extreme_bg_color = egui::Color32::from_rgb(7, 11, 18);
    style.visuals.selection.bg_fill = PINK;
    style.visuals.selection.stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.slider_trailing_fill = true;
    style.visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(18, 29, 42);
    style.visuals.widgets.inactive.weak_bg_fill = egui::Color32::from_rgb(18, 29, 42);
    style.visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, STEEL);
    style.visuals.widgets.inactive.corner_radius = 2.into();
    style.visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(25, 55, 76);
    style.visuals.widgets.hovered.weak_bg_fill = egui::Color32::from_rgb(25, 55, 76);
    style.visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.0, BLUE);
    style.visuals.widgets.hovered.corner_radius = 2.into();
    style.visuals.widgets.active.bg_fill = PINK;
    style.visuals.widgets.active.weak_bg_fill = PINK;
    style.visuals.widgets.active.bg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.active.corner_radius = 2.into();
    ctx.set_style_of(egui::Theme::Dark, style);
}

fn heading_font(size: f32) -> egui::FontId {
    egui::FontId::new(size, egui::FontFamily::Name("heading".into()))
}

fn section_heading(ui: &mut egui::Ui, title: &str) {
    ui.add_space(14.0);
    ui.separator();
    ui.add_space(3.0);
    ui.label(
        egui::RichText::new(title)
            .font(heading_font(15.0))
            .color(PINK),
    );
}

fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(egui::RichText::new(label).small().color(DIM));
    ui.monospace(value);
    ui.end_row();
}

fn hud(ctx: &egui::Context, live: &Live) {
    const SIZE: egui::Vec2 = egui::vec2(184.0, 224.0);

    let speed = live.world.ego.speed;
    let actuation = live.world.actuation();
    egui::Area::new("driving_hud".into())
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -24.0))
        .order(egui::Order::Foreground)
        .interactable(false)
        .show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(SIZE, egui::Sense::hover());
            let painter = ui.painter_at(rect);
            let center_x = rect.center().x;
            let top = rect.top();

            painter.rect_filled(rect, 3.0, PANEL);
            painter.rect_stroke(
                rect,
                3.0,
                egui::Stroke::new(1.0, STEEL),
                egui::StrokeKind::Inside,
            );
            painter.line_segment(
                [
                    egui::pos2(rect.left() + 1.0, rect.top() + 1.0),
                    egui::pos2(rect.right() - 1.0, rect.top() + 1.0),
                ],
                egui::Stroke::new(2.0, PINK),
            );
            painter.text(
                egui::pos2(center_x, top + 10.0),
                egui::Align2::CENTER_TOP,
                "SPEED",
                egui::FontId::monospace(10.0),
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
                egui::pos2(center_x - 6.0, top + 78.0),
                egui::pos2(center_x + 6.0, top + 158.0),
            );
            let accel_zero = accel_track.center().y;
            painter.rect_filled(accel_track, 2.0, egui::Color32::from_white_alpha(20));
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
                egui::pos2(center_x - 68.0, top + 184.0),
                egui::pos2(center_x + 68.0, top + 196.0),
            );
            painter.rect_filled(curve_track, 2.0, egui::Color32::from_white_alpha(20));
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
                egui::pos2(center_x, top + 205.0),
                egui::Align2::CENTER_TOP,
                format!("CURV {:+.3}", actuation.curvature),
                egui::FontId::monospace(10.0),
                TEXT,
            );
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
    use super::signed_fraction;

    #[test]
    fn signed_hud_values_use_their_own_side_of_zero() {
        assert_eq!(signed_fraction(5.0, 10.0, 20.0), 0.5);
        assert_eq!(signed_fraction(-5.0, 10.0, 20.0), -0.25);
        assert_eq!(signed_fraction(30.0, 10.0, 20.0), 1.0);
    }
}
