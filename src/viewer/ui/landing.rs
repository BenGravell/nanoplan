use bevy_egui::egui;

use super::super::colors::{ORANGE, SURFACE, WHITE};

const BACKGROUND_ASPECT_RATIO: f32 = 16.0 / 9.0;
const TITLE_ASPECT_RATIO: f32 = 146.65262 / 23.909071;
const MENU_ITEMS: [&str; 2] = ["Start", "Exit"];
const MENU_INACTIVE: egui::Color32 = egui::Color32::from_rgb(171, 180, 193);
const MENU_ROW_SPACING: f32 = 0.1;
const ACTIVATION_DELAY_S: f64 = 0.2;

#[derive(Clone, Copy)]
struct MenuActivation {
    index: usize,
    started_at: f64,
}

pub(super) fn show(root: &mut egui::Ui, started: &mut bool) -> bool {
    root.painter().rect_filled(root.max_rect(), 0.0, SURFACE);
    background_graphics(root);
    title_graphic(root);
    let exit_requested = start_menu(root, started);
    root.ctx()
        .request_repaint_after(std::time::Duration::from_millis(16));
    exit_requested
}

pub(super) fn menu_row_rect(screen: egui::Rect, index: usize) -> egui::Rect {
    egui::Rect::from_min_size(
        normalized_pos(
            screen,
            0.057_291_668,
            0.324_074_06 + index as f32 * MENU_ROW_SPACING,
        ),
        normalized_size(screen, 0.229_166_67, 0.064_814_81),
    )
}

fn start_menu(root: &mut egui::Ui, started: &mut bool) -> bool {
    let screen = root.max_rect();
    let selection_id = egui::Id::new("landing_menu_selection");
    let activation_id = egui::Id::new("landing_menu_activation");
    let mut selected = root.data_mut(|data| data.get_temp::<usize>(selection_id).unwrap_or(0));
    let mut activation = root.data_mut(|data| data.get_temp::<MenuActivation>(activation_id));

    if root.input(|input| input.key_pressed(egui::Key::ArrowDown)) {
        selected = (selected + 1) % MENU_ITEMS.len();
    }
    if root.input(|input| input.key_pressed(egui::Key::ArrowUp)) {
        selected = (selected + MENU_ITEMS.len() - 1) % MENU_ITEMS.len();
    }

    let mut clicked = None;
    let now = root.input(|input| input.time);
    for (index, label) in MENU_ITEMS.iter().enumerate() {
        let response = root.interact(
            menu_row_rect(screen, index),
            root.make_persistent_id(("landing_menu_item", index)),
            egui::Sense::click(),
        );
        response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, *label));
        if response.hovered() {
            selected = index;
        }
        if response.clicked() {
            selected = index;
            clicked = Some(index);
        }
    }

    for (index, label) in MENU_ITEMS.iter().enumerate() {
        let color = if index == selected {
            ORANGE
        } else {
            MENU_INACTIVE
        };
        let row_y = 1.0 / 3.0 + index as f32 * MENU_ROW_SPACING;
        let center_y = row_y + 0.020_370_37;
        let painter = root.painter();
        let text_rect = painter.text(
            normalized_pos(screen, 0.080_729_164, center_y),
            egui::Align2::LEFT_CENTER,
            *label,
            egui::FontId::new(
                screen.height() * 0.044_444_446,
                egui::FontFamily::Proportional,
            ),
            color,
        );
        if index == selected {
            paint_chevron(root, screen, center_y, now);
        }
        let leader_y = (text_rect.bottom() - screen.top()) / screen.height() + 0.003_703_703_6;
        painter.add(egui::Shape::line(
            [
                normalized_pos(screen, 0.080_729_164, leader_y),
                normalized_pos(screen, 0.271_875, leader_y),
                normalized_pos(screen, 0.279_166_67, leader_y + 0.012_962_963),
            ]
            .to_vec(),
            egui::Stroke::new(screen.height() * 0.003, color),
        ));
    }

    let activate = clicked
        .or_else(|| {
            root.input(|input| input.key_pressed(egui::Key::Enter))
                .then_some(selected)
        })
        .or_else(|| {
            root.input(|input| input.key_pressed(egui::Key::Space))
                .then_some(0)
        });
    if let Some(index) = activate {
        activation = Some(MenuActivation {
            index,
            started_at: now,
        });
    }
    if let Some(active) = activation {
        if activation_ready(now - active.started_at) {
            root.data_mut(|data| data.remove::<MenuActivation>(activation_id));
            if active.index == 0 {
                *started = true;
            }
            return active.index == MENU_ITEMS.len() - 1;
        }
    }
    root.data_mut(|data| data.insert_temp(selection_id, selected));
    if let Some(active) = activation {
        root.data_mut(|data| data.insert_temp(activation_id, active));
    }
    false
}

fn paint_chevron(ui: &egui::Ui, screen: egui::Rect, center_y: f32, time: f64) {
    let (offset, scale) = chevron_animation(time);
    let center_x = 0.066_145_83 + offset;
    let half_width = 0.002_604_167 * scale;
    let half_height = 0.007_407_407_3 * scale;
    ui.painter().add(egui::Shape::line(
        [
            normalized_pos(screen, center_x - half_width, center_y - half_height),
            normalized_pos(screen, center_x + half_width, center_y),
            normalized_pos(screen, center_x - half_width, center_y + half_height),
        ]
        .to_vec(),
        egui::Stroke::new(screen.height() * 0.003_703_703_6 * scale, ORANGE),
    ));
}

pub(super) fn chevron_animation(time: f64) -> (f32, f32) {
    let sine = (time as f32 * std::f32::consts::TAU * 0.75).sin();
    let pulse = sine.signum() * (1.0 - (1.0 - sine.abs()).powi(2));
    (pulse * 0.003_5, 1.0 + pulse * 0.12)
}

pub(super) fn activation_ready(elapsed_s: f64) -> bool {
    elapsed_s >= ACTIVATION_DELAY_S
}

pub(super) fn background_rect(screen: egui::Rect, anchor: egui::Align2) -> egui::Rect {
    let size = egui::vec2(screen.height() * BACKGROUND_ASPECT_RATIO, screen.height());
    anchor.align_size_within_rect(size, screen)
}

fn background_graphics(ui: &egui::Ui) {
    for (image, anchor) in [
        (
            egui::Image::new(egui::include_image!(
                "../../../assets/landing/nanoplan_bkgd_bt_rt_corner.svg"
            )),
            egui::Align2::RIGHT_BOTTOM,
        ),
        (
            egui::Image::new(egui::include_image!(
                "../../../assets/landing/nanoplan_bkgd_bt_lf_corner.svg"
            )),
            egui::Align2::LEFT_BOTTOM,
        ),
        (
            egui::Image::new(egui::include_image!(
                "../../../assets/landing/nanoplan_bkgd_tp_lf_corner.svg"
            )),
            egui::Align2::LEFT_TOP,
        ),
    ] {
        paint_svg(ui, image, background_rect(ui.max_rect(), anchor));
    }
}

pub(super) fn title_rect(screen: egui::Rect) -> egui::Rect {
    let width = normalized_size(screen, 0.395_833_34, 0.0).x;
    egui::Rect::from_min_size(
        normalized_pos(screen, 0.041_666_668, 0.148_148_15),
        egui::vec2(width, width / TITLE_ASPECT_RATIO),
    )
}

fn normalized_pos(screen: egui::Rect, x: f32, y: f32) -> egui::Pos2 {
    screen.left_top() + normalized_size(screen, x, y)
}

fn normalized_size(screen: egui::Rect, x: f32, y: f32) -> egui::Vec2 {
    egui::vec2(
        screen.height() * BACKGROUND_ASPECT_RATIO * x,
        screen.height() * y,
    )
}

fn title_graphic(ui: &egui::Ui) {
    paint_svg(
        ui,
        egui::Image::new(egui::include_image!(
            "../../../assets/landing/nanoplan_title.svg"
        )),
        title_rect(ui.max_rect()),
    );
}

fn paint_svg(ui: &egui::Ui, image: egui::Image<'_>, rect: egui::Rect) {
    let raster_size = background_raster_size(
        rect.size(),
        ui.pixels_per_point(),
        ui.input(|input| input.max_texture_side),
    );
    if let Ok(egui::load::TexturePoll::Ready { texture }) =
        image.load_for_size(ui.ctx(), raster_size)
    {
        ui.painter().image(
            texture.id,
            rect,
            egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0)),
            WHITE,
        );
    }
}

pub(super) fn background_raster_size(
    display_size: egui::Vec2,
    pixels_per_point: f32,
    max_texture_side: usize,
) -> egui::Vec2 {
    let max_side_in_points = max_texture_side as f32 / pixels_per_point;
    display_size * (max_side_in_points / display_size.max_elem()).min(1.0)
}
