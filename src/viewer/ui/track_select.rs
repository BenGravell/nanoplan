use std::sync::OnceLock;

use bevy_egui::egui;

use super::super::UiState;
use super::super::colors::{CONTROL, DIM_TEXT, FAINT, ORANGE, PANEL, SURFACE, TEXT, WHITE};
use super::super::live::Live;
use super::style::{caps_font, scroll_area};
use super::widgets::minimap;
use crate::track::{TRACK_CATALOG, TRACK_PRESETS, Track};

const GALLERY_FRACTION: f32 = 0.2;
const STATS_STEP_M: f64 = 5.0;
const CORNER_CURVATURE_THRESHOLD: f64 = 0.004;
const CORNER_RESET_M: f64 = 20.0;
const CORNER_SMOOTHING_M: f64 = 20.0;

struct TrackPreview {
    name: &'static str,
    length_m: f64,
    corners: usize,
    average_radius_m: f64,
    minimum_radius_m: f64,
}

static PREVIEWS: OnceLock<Vec<TrackPreview>> = OnceLock::new();

pub(super) fn show(root: &mut egui::Ui, state: &mut UiState, live: &mut Live) {
    root.style_mut().visuals.window_shadow = egui::epaint::Shadow::NONE;
    root.style_mut().visuals.popup_shadow = egui::epaint::Shadow::NONE;
    root.painter().rect_filled(root.max_rect(), 0.0, SURFACE);
    let previews = previews();
    let previous_track = state.track;

    if !root.ctx().egui_wants_keyboard_input() {
        root.input(|input| {
            if input.key_pressed(egui::Key::ArrowRight) {
                state.track = (state.track + 1) % previews.len();
            }
            if input.key_pressed(egui::Key::ArrowLeft) {
                state.track = (state.track + previews.len() - 1) % previews.len();
            }
            if input.key_pressed(egui::Key::Escape) {
                state.selecting_track = false;
            }
        });
    }

    let screen = root.max_rect();
    let gallery_height = screen.height() * GALLERY_FRACTION;
    let top = egui::Rect::from_min_max(screen.min, egui::pos2(screen.right(), screen.bottom() - gallery_height));
    let gallery = egui::Rect::from_min_max(egui::pos2(screen.left(), top.bottom()), screen.max);
    let start = top_section(root, top, state, &previews[state.track], state.track);
    let double_clicked = gallery_section(root, gallery, state, previews, state.track != previous_track);

    let keyboard_start = !root.ctx().egui_wants_keyboard_input()
        && root.input(|input| input.key_pressed(egui::Key::Enter) || input.key_pressed(egui::Key::Space));
    if start || double_clicked || keyboard_start {
        live.regenerate_with_actor_count(live.seed, state.planner, state.track, state.opponents);
        state.started = true;
        state.selecting_track = false;
    }
    root.ctx().request_repaint_after(std::time::Duration::from_millis(16));
}

fn top_section(
    root: &mut egui::Ui,
    rect: egui::Rect,
    state: &mut UiState,
    preview: &TrackPreview,
    track_index: usize,
) -> bool {
    let compact = rect.height() < 400.0;
    let margin = if compact { 12.0 } else { 24.0 };
    let inner = rect.shrink(margin);
    let heading_height = if compact { 38.0 } else { 56.0 };
    let heading = egui::Rect::from_min_max(inner.min, egui::pos2(inner.right(), inner.top() + heading_height));
    let back_size = egui::vec2(if compact { 76.0 } else { 96.0 }, if compact { 28.0 } else { 32.0 });
    let back = egui::Rect::from_min_size(egui::pos2(inner.right() - back_size.x, inner.top()), back_size);
    root.scope_builder(egui::UiBuilder::new().max_rect(heading), |ui| {
        ui.vertical_centered(|ui| {
            ui.label(
                egui::RichText::new(preview.name)
                    .font(caps_font(if compact { 30.0 } else { 44.0 }))
                    .color(TEXT),
            );
        });
    });
    if root
        .put(
            back,
            egui::Button::new(egui::RichText::new("BACK").font(caps_font(11.0))),
        )
        .clicked()
    {
        state.selecting_track = false;
    }

    let drive_size = egui::vec2(
        (inner.width() * 0.22).clamp(140.0, 240.0),
        if compact { 28.0 } else { 40.0 },
    );
    let drive = egui::Rect::from_min_size(
        egui::pos2(inner.center().x - drive_size.x * 0.5, inner.bottom() - drive_size.y),
        drive_size,
    );
    let content = egui::Rect::from_min_max(
        egui::pos2(inner.left(), heading.bottom()),
        egui::pos2(inner.right(), drive.top() - if compact { 6.0 } else { 12.0 }),
    );
    let details_width = (content.width() * if compact { 0.38 } else { 0.32 }).max(170.0);
    let map = egui::Rect::from_min_max(
        content.min,
        egui::pos2(content.right() - details_width - margin, content.bottom()),
    );
    let details = egui::Rect::from_min_max(egui::pos2(map.right() + margin, content.top()), content.max);

    root.painter().rect_filled(map, 1.0, PANEL);
    let time = root.input(|input| input.time);
    let lap_length = minimap::lap_length(track_index);
    let opponents: [f64; 9] =
        std::array::from_fn(|index| time * (55.0 + index as f64 * 2.0) + lap_length * (index + 1) as f64 / 10.0);
    minimap::paint(
        root,
        track_index,
        map.shrink(margin),
        DIM_TEXT,
        &opponents,
        Some(time * 65.0),
    );
    root.interact(map, root.make_persistent_id("selected_track_map"), egui::Sense::hover())
        .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Image, false, "Selected track map"));

    root.scope_builder(egui::UiBuilder::new().max_rect(details), |ui| {
        if compact {
            ui.spacing_mut().item_spacing.y = 3.0;
            ui.spacing_mut().button_padding.y = 3.0;
            ui.spacing_mut().interact_size.y = 26.0;
        }
        egui::Grid::new("track_stats")
            .num_columns(2)
            .spacing(egui::vec2(12.0, if compact { 3.0 } else { 8.0 }))
            .show(ui, |ui| {
                stat(ui, "LENGTH", format_length(preview.length_m));
                stat(ui, "CORNERS", preview.corners.to_string());
                stat(ui, "AVG RADIUS", format_radius(preview.average_radius_m));
                stat(ui, "MIN RADIUS", format_radius(preview.minimum_radius_m));
            });
    });
    root.put(
        drive,
        egui::Button::new(egui::RichText::new("DRIVE").font(caps_font(13.0))),
    )
    .clicked()
}

fn stat(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(egui::RichText::new(label).font(caps_font(10.0)).color(DIM_TEXT));
    ui.monospace(value);
    ui.end_row();
}

fn gallery_section(
    root: &mut egui::Ui,
    rect: egui::Rect,
    state: &mut UiState,
    previews: &[TrackPreview],
    scroll_to_selection: bool,
) -> bool {
    let compact = rect.height() < 100.0;
    let margin = if compact { 4 } else { 8 };
    let mut double_clicked = false;
    root.scope_builder(egui::UiBuilder::new().max_rect(rect), |ui| {
        egui::Frame::new()
            .fill(PANEL)
            .shadow(egui::epaint::Shadow::NONE)
            .inner_margin(egui::Margin::same(margin))
            .show(ui, |ui| {
                ui.style_mut().always_scroll_the_only_direction = true;
                let caption_height = if compact { 14.0 } else { 20.0 };
                let item_gap = 2.0;
                let square =
                    (ui.available_height() - caption_height - item_gap - ui.spacing().scroll.allocated_width())
                        .max(40.0);
                let button_width = if compact { 32.0 } else { 44.0 };
                let gallery_width =
                    (ui.available_width() - 2.0 * button_width - 2.0 * ui.spacing().item_spacing.x).max(40.0);
                let rail_height = ui.available_height();
                let (left, gallery, right) = ui
                    .horizontal_top(|ui| {
                        let left = gallery_button(ui, "‹", "Scroll tracks left", button_width, rail_height);
                        let gallery = scroll_area(
                            ui,
                            egui::ScrollArea::horizontal()
                                .id_salt("track_gallery")
                                .max_width(gallery_width)
                                .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysVisible),
                            |ui| {
                                ui.horizontal(|ui| {
                                    ui.spacing_mut().item_spacing.x = if compact { 6.0 } else { 10.0 };
                                    for (index, preview) in previews.iter().enumerate() {
                                        ui.vertical(|ui| {
                                            ui.spacing_mut().item_spacing.y = item_gap;
                                            let (thumbnail, response) = ui
                                                .allocate_exact_size(egui::vec2(square, square), egui::Sense::click());
                                            let selected = state.track == index;
                                            let fill = if selected {
                                                ORANGE
                                            } else if response.hovered() {
                                                FAINT
                                            } else {
                                                CONTROL
                                            };
                                            ui.painter().rect_filled(thumbnail, 1.0, fill);
                                            minimap::paint(
                                                ui,
                                                index,
                                                thumbnail.shrink(if compact { 5.0 } else { 8.0 }),
                                                if selected { WHITE } else { DIM_TEXT },
                                                &[],
                                                None,
                                            );
                                            response.widget_info(|| {
                                                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, preview.name)
                                            });
                                            if response.clicked() {
                                                state.track = index;
                                            }
                                            if response.double_clicked() {
                                                double_clicked = true;
                                            }
                                            if selected && scroll_to_selection {
                                                response.scroll_to_me(Some(egui::Align::Center));
                                            }
                                            ui.add_sized(
                                                [square, caption_height],
                                                egui::Label::new(
                                                    egui::RichText::new(preview.name)
                                                        .size(if compact { 9.0 } else { 11.0 })
                                                        .color(if selected { ORANGE } else { TEXT }),
                                                )
                                                .truncate(),
                                            );
                                        });
                                    }
                                });
                            },
                        );
                        let right = gallery_button(ui, "›", "Scroll tracks right", button_width, rail_height);
                        (left, gallery, right)
                    })
                    .inner;
                let direction = right.clicked() as i8 - left.clicked() as i8;
                if direction != 0 {
                    let mut scroll_state = gallery.state;
                    let maximum = (gallery.content_size.x - gallery.inner_rect.width()).max(0.0);
                    scroll_state.offset.x = (scroll_state.offset.x
                        + direction as f32 * gallery.inner_rect.width() * 0.4)
                        .clamp(0.0, maximum);
                    scroll_state.store(ui.ctx(), gallery.id);
                    ui.ctx().request_repaint();
                }
            });
    });
    double_clicked
}

fn gallery_button(ui: &mut egui::Ui, icon: &str, label: &'static str, width: f32, height: f32) -> egui::Response {
    let response = ui.add_sized([width, height], egui::Button::new(egui::RichText::new(icon).size(28.0)));
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label));
    response
}

fn previews() -> &'static [TrackPreview] {
    PREVIEWS.get_or_init(|| (0..track_count()).map(build_preview).collect())
}

fn track_count() -> usize {
    TRACK_PRESETS.len() + TRACK_CATALOG.len()
}

fn track_name(index: usize) -> &'static str {
    if index < TRACK_PRESETS.len() {
        TRACK_PRESETS[index].name
    } else {
        TRACK_CATALOG[index - TRACK_PRESETS.len()].name
    }
}

fn build_preview(index: usize) -> TrackPreview {
    let track = Track::from_catalog(index);
    let length_m = track.lap_length().expect("all selectable tracks are closed circuits");

    let sample_count = (length_m / STATS_STEP_M).ceil().max(3.0) as usize;
    let step = length_m / sample_count as f64;
    let signed_curvatures = (0..sample_count)
        .map(|sample| {
            let (_, before) = track.pose(sample as f64 * step);
            let (_, after) = track.pose((sample + 1) as f64 * step);
            (after - before).sin().atan2((after - before).cos()) / step
        })
        .collect::<Vec<_>>();
    let curvatures = signed_curvatures
        .iter()
        .map(|curvature| curvature.abs())
        .collect::<Vec<_>>();
    let maximum_curvature = curvatures.iter().copied().fold(0.0, f64::max);
    let average_curvature = curvatures.iter().sum::<f64>() / curvatures.len() as f64;

    TrackPreview {
        name: track_name(index),
        length_m,
        corners: count_corners(&signed_curvatures, step),
        average_radius_m: average_curvature.recip(),
        minimum_radius_m: maximum_curvature.recip(),
    }
}

fn count_corners(curvatures: &[f64], step: f64) -> usize {
    let smoothing_radius = ((CORNER_SMOOTHING_M / step).round() as usize / 2).max(1);
    let signs = (0..curvatures.len())
        .map(|curvature| {
            let curvature = (0..=2 * smoothing_radius)
                .map(|offset| curvatures[(curvature + curvatures.len() + offset - smoothing_radius) % curvatures.len()])
                .sum::<f64>()
                / (2 * smoothing_radius + 1) as f64;
            if curvature.abs() < CORNER_CURVATURE_THRESHOLD {
                0
            } else if curvature > 0.0 {
                1
            } else {
                -1
            }
        })
        .collect::<Vec<_>>();
    let reset_samples = (CORNER_RESET_M / step).ceil() as usize;
    let start = signs
        .iter()
        .enumerate()
        .filter(|(_, sign)| **sign == 0)
        .max_by_key(|(index, _)| {
            (1..signs.len())
                .take_while(|offset| signs[(index + offset) % signs.len()] == 0)
                .count()
        })
        .map(|(index, _)| {
            (index..index + signs.len())
                .find(|next| signs[next % signs.len()] != 0)
                .map_or(0, |next| next % signs.len())
        })
        .unwrap_or_else(|| {
            (0..signs.len())
                .find(|index| signs[*index] != signs[(index + signs.len() - 1) % signs.len()])
                .unwrap_or(0)
        });

    let mut corners = 0;
    let mut quiet = reset_samples;
    let mut previous_sign = 0;
    for offset in 0..signs.len() {
        let sign = signs[(start + offset) % signs.len()];
        if sign != 0 {
            if quiet >= reset_samples || sign != previous_sign {
                corners += 1;
            }
            previous_sign = sign;
            quiet = 0;
        } else {
            quiet += 1;
        }
    }
    corners
}

fn format_length(length_m: f64) -> String {
    if length_m >= 1000.0 {
        format!("{:.2} km", length_m / 1000.0)
    } else {
        format!("{length_m:.0} m")
    }
}

fn format_radius(radius_m: f64) -> String {
    format!("{radius_m:.0} m")
}

#[cfg(test)]
mod tests {
    #[test]
    fn corner_count_groups_bends_and_splits_direction_changes() {
        let mut curvatures = vec![0.0; 40];
        curvatures[5..13].fill(0.02);
        curvatures[18..26].fill(-0.02);

        assert_eq!(super::count_corners(&curvatures, 5.0), 2);
        assert_eq!(super::count_corners(&[0.0; 40], 5.0), 0);
    }
}
