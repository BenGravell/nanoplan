use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::ViewerHarnessState;

#[test]
fn track_select_keeps_preview_details_and_gallery_in_their_panes() {
    for size in [egui::vec2(520.0, 390.0), egui::vec2(1280.0, 720.0)] {
        let mut harness = Harness::builder().with_size(size).build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    state.pages.select_track();
                    ui.ctx().request_repaint();
                    return;
                }
                state
                    .navigator
                    .show(ui, &mut state.pages, &mut state.ui, &mut state.live, &mut state.tab);
            },
            ViewerHarnessState::default(),
        );
        harness.run_steps(2);

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
        let top = egui::Rect::from_min_max(screen.min, egui::pos2(screen.right(), screen.top() + size.y * 0.8));
        let gallery = egui::Rect::from_min_max(egui::pos2(screen.left(), top.bottom()), screen.max);
        for label in ["LENGTH", "CORNERS", "AVG RADIUS", "MIN RADIUS", "DRIVE", "BACK"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                top.contains_rect(rect),
                "{label:?} is outside the top pane at {size:?}: {rect:?}"
            );
        }
        let map = harness.get_by_label("Selected track map").rect();
        assert!(top.contains_rect(map) && map.is_positive());
        let drive = harness.get_by_label("DRIVE").rect();
        let back = harness.get_by_label("BACK").rect();
        assert!(drive.top() >= map.bottom());
        assert!((drive.center().x - screen.center().x).abs() <= 1.0);
        assert!(back.center().x > screen.center().x && back.center().y < map.top());
        assert!(
            harness
                .get_all_by_label("Test Track (large)")
                .any(|node| top.contains_rect(node.rect())),
            "track title is missing from the top pane at {size:?}"
        );

        let thumbnail = harness
            .get_by_role_and_label(egui::accesskit::Role::Button, "Test Track (large)")
            .rect();
        assert!(
            gallery.expand(1.0).contains_rect(thumbnail),
            "thumbnail is outside the gallery at {size:?}: {thumbnail:?}"
        );
        assert!((thumbnail.width() - thumbnail.height()).abs() <= 1.0);

        let left_control = harness.get_by_label("Scroll tracks left").rect();
        let right_control = harness.get_by_label("Scroll tracks right").rect();
        assert!(
            gallery.expand(1.0).contains_rect(left_control) && gallery.expand(1.0).contains_rect(right_control),
            "rail controls outside gallery at {size:?}: {left_control:?}, {right_control:?}"
        );
        assert!((thumbnail.top() - left_control.top()).abs() <= 1.0);
        assert_eq!(left_control.height(), right_control.height());
        harness.get_by_label("Scroll tracks right").click();
        harness.run_steps(2);
        let button_scrolled = harness
            .get_by_role_and_label(egui::accesskit::Role::Button, "Test Track (large)")
            .rect();
        assert!(button_scrolled.left() < thumbnail.left());
        harness.get_by_label("Scroll tracks left").click();
        harness.run_steps(2);

        harness.hover_at(gallery.center());
        harness.event(egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            delta: egui::vec2(0.0, -100.0),
            phase: egui::TouchPhase::Move,
            modifiers: egui::Modifiers::NONE,
        });
        harness.run_steps(4);
        let scrolled = harness
            .get_by_role_and_label(egui::accesskit::Role::Button, "Test Track (large)")
            .rect();
        assert!(
            scrolled.left() < thumbnail.left(),
            "mouse wheel did not move the gallery at {size:?}"
        );
    }
}

#[test]
fn corner_count_groups_bends_and_splits_direction_changes() {
    let mut curvatures = vec![0.0; 40];
    curvatures[5..13].fill(0.02);
    curvatures[18..26].fill(-0.02);

    assert_eq!(super::count_corners(&curvatures, 5.0), 2);
    assert_eq!(super::count_corners(&[0.0; 40], 5.0), 0);
}
