use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use super::{center_rail_rect, compact_layout, side_rail_widths};
use crate::viewer::MIN_VIEWPORT_ASPECT_RATIO;
use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::PHONE_LANDSCAPE_SIZES;
use crate::viewer::ui::test_support::ViewerHarnessState;
use crate::viewer::ui::{Page, viewer_layout};

#[test]
fn pause_rail_opens_navigation_modal() {
    let mut harness = Harness::builder().with_size(egui::vec2(1280.0, 720.0)).build_ui_state(
        |ui, state: &mut ViewerHarnessState| {
            if !state.configured {
                configure(ui.ctx());
                state.configured = true;
                state.navigator.navigate(Page::Driving);
                ui.ctx().request_repaint();
                return;
            }
            viewer_layout(
                ui,
                &mut state.navigator,
                &mut state.pages,
                &mut state.ui,
                &mut state.live,
                &mut state.tab,
            );
        },
        ViewerHarnessState::default(),
    );
    harness.run_steps(2);

    harness.get_by_label("PAUSE").click();
    harness.run();
    for label in ["RESUME", "RETURN TO TRACK SELECT", "RETURN TO START MENU"] {
        assert!(harness.query_by_label(label).is_some());
    }
    assert!(harness.query_by_label("EXIT").is_none());
    let paused = harness.get_by_label("PAUSED").rect();
    let resume = harness.get_by_label("RESUME").rect();
    assert!((paused.center().x - resume.center().x).abs() <= 1.0);

    harness.get_by_label("RESUME").click();
    harness.run();
    assert!(!harness.state().live.paused);

    harness.get_by_label("PAUSE").click();
    harness.run();
    harness.get_by_label("RETURN TO TRACK SELECT").click();
    harness.run();
    assert_eq!(harness.state().navigator.page(), Page::Start);
    assert!(harness.state().pages.selecting_track());

    harness.state_mut().navigator.navigate(Page::Driving);
    harness.run();
    harness.get_by_label("PAUSE").click();
    harness.run();
    harness.get_by_label("RETURN TO START MENU").click();
    harness.run();
    assert_eq!(harness.state().navigator.page(), Page::Start);
    assert!(!harness.state().pages.selecting_track());
}

#[test]
fn planner_overrun_warning_is_visible_only_while_slow() {
    let mut harness = Harness::builder().with_size(egui::vec2(1280.0, 720.0)).build_ui_state(
        |ui, state: &mut ViewerHarnessState| {
            if !state.configured {
                configure(ui.ctx());
                state.configured = true;
                ui.ctx().request_repaint();
                return;
            }
            viewer_layout(
                ui,
                &mut state.navigator,
                &mut state.pages,
                &mut state.ui,
                &mut state.live,
                &mut state.tab,
            );
        },
        ViewerHarnessState::default(),
    );
    harness.run_steps(2);
    assert!(harness.query_by_label("PLANNER TOO SLOW · REUSING LAST PLAN").is_none());

    harness.state_mut().live.world.planner_slow = true;
    harness.run();
    assert!(harness.query_by_label("PLANNER TOO SLOW · REUSING LAST PLAN").is_some());
}

#[test]
fn keyboard_shortcuts_pause_and_toggle_frame_time() {
    let mut harness = Harness::builder().with_size(egui::vec2(1280.0, 720.0)).build_ui_state(
        |ui, state: &mut ViewerHarnessState| {
            if !state.configured {
                configure(ui.ctx());
                state.configured = true;
                ui.ctx().request_repaint();
                return;
            }
            viewer_layout(
                ui,
                &mut state.navigator,
                &mut state.pages,
                &mut state.ui,
                &mut state.live,
                &mut state.tab,
            );
        },
        ViewerHarnessState::default(),
    );
    harness.run_steps(2);

    harness.key_press(egui::Key::Space);
    harness.run();
    assert!(harness.state().live.paused);
    assert!(harness.query_by_label("PAUSED").is_some());

    harness.key_press(egui::Key::Space);
    harness.run();
    assert!(!harness.state().live.paused);

    harness.key_press(egui::Key::Escape);
    harness.run();
    assert!(harness.state().live.paused);

    harness.key_press(egui::Key::Escape);
    harness.run();
    assert!(!harness.state().live.paused);

    harness.key_press(egui::Key::T);
    harness.run();
    assert!(harness.state().ui.show_frame_time);
    assert!(harness.query_by_label("FRAME 0.00 ms").is_some());
}

#[test]
fn driving_canvas_excludes_side_rails() {
    let size = egui::vec2(1280.0, 720.0);
    let mut harness = Harness::builder().with_size(size).build_ui_state(
        |ui, state: &mut ViewerHarnessState| {
            if !state.configured {
                configure(ui.ctx());
                state.configured = true;
                ui.ctx().request_repaint();
                return;
            }
            let viewport = ui.max_rect();
            let (left, right) = side_rail_widths(viewport.size());
            let canvas = viewer_layout(
                ui,
                &mut state.navigator,
                &mut state.pages,
                &mut state.ui,
                &mut state.live,
                &mut state.tab,
            );
            assert_eq!(canvas, center_rail_rect(viewport, left, right));
        },
        ViewerHarnessState::default(),
    );

    harness.run_steps(2);

    let (left, right) = side_rail_widths(size);
    let road = center_rail_rect(egui::Rect::from_min_size(egui::Pos2::ZERO, size), left, right);
    let minimap = harness.get_by_label("Track minimap").rect();
    assert!(road.contains_rect(minimap));
    assert!(minimap.center().x > road.center().x && minimap.top() > road.top());
}

#[test]
fn pause_rail_fits_exactly_between_the_side_overlays() {
    let canvas = egui::Rect::from_min_size(egui::pos2(12.0, 8.0), egui::vec2(1280.0, 720.0));
    let pause = center_rail_rect(canvas, 372.0, 384.0);

    assert_eq!(pause.left(), canvas.left() + 372.0);
    assert_eq!(pause.right(), canvas.right() - 384.0);
    assert_eq!(pause.y_range(), canvas.y_range());
}

#[test]
fn layout_only_compacts_for_phone_sized_viewports() {
    for (_, phone) in PHONE_LANDSCAPE_SIZES {
        assert!(compact_layout(phone));
    }
    assert!(compact_layout(egui::vec2(960.0, 540.0)));
    assert!(!compact_layout(egui::vec2(1920.0, 1080.0)));
    assert!(!compact_layout(egui::vec2(3440.0, 1440.0)));
    assert!(!compact_layout(egui::vec2(3840.0, 2160.0)));
}

#[test]
fn side_menus_are_each_three_eighths_of_the_viewport_height() {
    for viewport in [
        egui::vec2(1920.0, 1080.0),
        egui::vec2(3440.0, 1440.0),
        egui::vec2(3840.0, 2160.0),
    ]
    .into_iter()
    .chain(PHONE_LANDSCAPE_SIZES.map(|(_, size)| size))
    {
        let expected = viewport.y * 0.375;
        assert_eq!(side_rail_widths(viewport), (expected, expected));
    }
}

#[test]
fn supported_viewport_aspect_ratio_keeps_a_positive_center_canvas() {
    let height = 1000.0;
    let viewport = egui::vec2(height * MIN_VIEWPORT_ASPECT_RATIO, height);
    let (left, right) = side_rail_widths(viewport);
    let center_width = viewport.x - left - right;

    assert!(center_width > 0.0);
}
