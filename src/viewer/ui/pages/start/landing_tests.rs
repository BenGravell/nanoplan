use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use crate::viewer::ui::Page;
use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::ViewerHarnessState;

#[test]
fn landing_starts_with_the_keyboard() {
    let mut harness = Harness::builder().with_size(egui::vec2(1280.0, 720.0)).build_ui_state(
        |ui, state: &mut ViewerHarnessState| {
            let ctx = ui.ctx().clone();
            if !state.configured {
                configure(&ctx);
                state.configured = true;
                ctx.request_repaint();
                return;
            }
            if state.navigator.page() == Page::Start {
                state
                    .navigator
                    .show(ui, &mut state.pages, &mut state.ui, &mut state.live, &mut state.tab);
            }
        },
        ViewerHarnessState::default(),
    );
    harness.run_steps(2);

    assert!(harness.query_by_label("Start").is_some());
    assert!(harness.query_by_label("Exit").is_none());
    harness.key_press(egui::Key::Enter);
    harness.run_steps(20);
    assert!(harness.state().pages.selecting_track());
    assert!(harness.query_by_label("CHOOSE A TRACK").is_none());
    assert!(harness.get_all_by_label("Test Track (large)").next().is_some());
    assert!(harness.query_by_label("Selected track map").is_some());
    for label in ["LENGTH", "CORNERS", "AVG RADIUS", "MIN RADIUS"] {
        assert!(
            harness.query_by_label(label).is_some(),
            "{label:?} missing from track select"
        );
    }

    harness.key_press(egui::Key::ArrowRight);
    harness.run_steps(1);
    assert_eq!(harness.state().ui.track, 1);
    harness.key_press(egui::Key::Enter);
    harness.run();
    assert_eq!(harness.state().navigator.page(), Page::Driving);
}

#[test]
fn landing_activation_waits_long_enough_to_show_feedback() {
    assert!(!super::activation_ready(0.199));
    assert!(super::activation_ready(0.2));
}

#[test]
fn landing_title_uses_normalized_reference_coordinates() {
    let screen = egui::Rect::from_min_size(egui::pos2(13.0, 17.0), egui::vec2(1000.0, 720.0));
    let title = super::title_rect(screen);
    let reference_width = screen.height() * 16.0 / 9.0;
    assert!(((title.left() - screen.left()) / reference_width - 0.041_666_668).abs() < f32::EPSILON);
    assert!(((title.top() - screen.top()) / screen.height() - 0.148_148_15).abs() < f32::EPSILON);
    assert!((title.width() / reference_width - 0.395_833_34).abs() < f32::EPSILON);
}

#[test]
fn landing_menu_uses_normalized_reference_coordinates() {
    let screen = egui::Rect::from_min_size(egui::pos2(13.0, 17.0), egui::vec2(1000.0, 720.0));
    let reference_width = screen.height() * 16.0 / 9.0;
    let first = super::menu_row_rect(screen, 0);
    let second = super::menu_row_rect(screen, 1);
    assert!(((first.left() - screen.left()) / reference_width - 0.057_291_668).abs() < f32::EPSILON);
    assert!(((first.top() - screen.top()) / screen.height() - 0.324_074_06).abs() < f32::EPSILON);
    assert!(((second.top() - first.top()) / screen.height() - 0.1).abs() < f32::EPSILON);
}

#[test]
fn landing_backgrounds_span_height_and_anchor_to_their_corners() {
    let screen = egui::Rect::from_min_size(egui::pos2(13.0, 17.0), egui::vec2(1000.0, 720.0));
    let bottom_right = super::background_rect(screen, egui::Align2::RIGHT_BOTTOM);
    let bottom_left = super::background_rect(screen, egui::Align2::LEFT_BOTTOM);
    let top_left = super::background_rect(screen, egui::Align2::LEFT_TOP);

    for background in [bottom_right, bottom_left, top_left] {
        assert_eq!(background.height(), screen.height());
        assert_eq!(background.width(), 1280.0);
    }
    assert_eq!(bottom_right.right_bottom(), screen.right_bottom());
    assert_eq!(bottom_left.left_bottom(), screen.left_bottom());
    assert_eq!(top_left.left_top(), screen.left_top());
}

#[test]
fn landing_background_respects_the_gpu_texture_limit() {
    let raster = super::background_raster_size(egui::vec2(2155.0, 1212.0), 1.0, 2048);
    assert_eq!(raster.x, 2048.0);
    assert!(raster.y <= 2048.0);
}

#[test]
fn landing_bottom_corner_visibility_threshold_prevents_svg_pixel_collisions() {
    assert!(super::show_bottom_left(egui::vec2(31.0, 20.0)));
    assert!(!super::show_bottom_left(egui::vec2(3.0, 2.0)));
    assert!(super::show_bottom_left(egui::vec2(16.0, 9.0)));

    for height in [360, 720, 1080] {
        let background_width = height * 16 / 9;
        let size = egui::vec2(background_width as f32, height as f32);
        let left = render_bottom_corner(super::BottomCorner::Left, size);
        let right = render_bottom_corner(super::BottomCorner::Right, size);
        let visible_width = (height as f32 * 31.0 / 20.0).ceil() as u32;
        let hidden_width = height * 3 / 2;

        assert!(!bottom_corners_collide(&left, &right, visible_width));
        assert!(bottom_corners_collide(&left, &right, hidden_width));
    }
}

fn render_bottom_corner(corner: super::BottomCorner, size: egui::Vec2) -> image::RgbaImage {
    let mut harness = Harness::builder().with_size(size).build_ui_state(
        move |ui, configured: &mut bool| {
            if !*configured {
                configure(ui.ctx());
                *configured = true;
                ui.ctx().request_repaint();
                return;
            }
            super::paint_bottom_corner(ui, corner);
        },
        false,
    );
    harness.run();
    harness.render().unwrap()
}

fn bottom_corners_collide(left: &image::RgbaImage, right: &image::RgbaImage, screen_width: u32) -> bool {
    let background_width = left.width();
    let right_offset = i64::from(screen_width) - i64::from(background_width);
    (0..screen_width).any(|screen_x| {
        let right_x = i64::from(screen_x) - right_offset;
        right_x >= 0
            && right_x < i64::from(background_width)
            && (0..left.height())
                .any(|y| left.get_pixel(screen_x, y).0[3] != 0 && right.get_pixel(right_x as u32, y).0[3] != 0)
    })
}
