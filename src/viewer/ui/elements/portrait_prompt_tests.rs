use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use super::{prompt_copy, show};
use crate::viewer::ui::style::configure;
use crate::viewer::{MIN_VIEWPORT_WIDTH, viewport_constraints, viewport_supported};

#[test]
fn portrait_prompt_is_the_only_interactive_view() {
    for size in [egui::vec2(390.0, 844.0), egui::vec2(180.0, 320.0)] {
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
                show(&mut root, true, viewport_constraints(size.x, size.y));
            },
            false,
        );
        harness.run();

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
        for label in ["TURN YOUR DEVICE SIDEWAYS", "Nanoplan requires landscape orientation."] {
            assert!(screen.contains_rect(harness.get_by_label(label).rect()));
        }
        assert!(harness.query_by_label("NANOPLAN").is_none());
        assert!(harness.query_by_label("PAUSE").is_none());
    }
}

#[test]
fn undersized_landscape_asks_for_a_wider_window() {
    let size = egui::vec2(MIN_VIEWPORT_WIDTH - 1.0, 269.0);
    assert!(!viewport_supported(size.x, size.y));
    let mut harness = Harness::builder().with_size(size).build_ui_state(
        |ui, configured: &mut bool| {
            if !*configured {
                configure(ui.ctx());
                *configured = true;
                ui.ctx().request_repaint();
                return;
            }
            show(ui, false, viewport_constraints(size.x, size.y));
        },
        false,
    );
    harness.run();

    assert!(harness.query_by_label("MAKE YOUR WINDOW WIDER").is_some());
    assert!(
        harness
            .query_by_label("Nanoplan requires a viewport at least 520 px wide.")
            .is_some()
    );
    assert!(harness.query_by_label("PAUSE").is_none());
}

#[test]
fn bumper_copy_matches_the_violated_viewport_constraint() {
    assert_eq!(
        prompt_copy(viewport_constraints(390.0, 844.0), true),
        (
            "TURN YOUR DEVICE SIDEWAYS",
            "Nanoplan requires landscape orientation.".to_owned()
        )
    );
    assert_eq!(
        prompt_copy(viewport_constraints(390.0, 844.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport at least 520 px wide with a 4:3 aspect ratio.".to_owned()
        )
    );
    assert_eq!(
        prompt_copy(viewport_constraints(700.0, 844.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport with at least a 4:3 aspect ratio.".to_owned()
        )
    );
    assert_eq!(
        prompt_copy(viewport_constraints(519.0, 320.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport at least 520 px wide.".to_owned()
        )
    );
    assert_eq!(
        prompt_copy(viewport_constraints(600.0, 500.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport with at least a 4:3 aspect ratio.".to_owned()
        )
    );
}
