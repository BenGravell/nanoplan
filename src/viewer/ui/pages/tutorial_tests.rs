use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use crate::viewer::ui::Page;
use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::PHONE_LANDSCAPE_SIZES;
use crate::viewer::ui::test_support::ViewerHarnessState;

#[test]
fn landing_tutorial_opens_the_introduction_and_camera_keymap_and_returns() {
    let mut harness = Harness::builder().with_size(egui::vec2(1280.0, 720.0)).build_ui_state(
        |ui, state: &mut ViewerHarnessState| {
            let ctx = ui.ctx().clone();
            if !state.configured {
                configure(&ctx);
                state.configured = true;
                ctx.request_repaint();
                return;
            }
            state
                .navigator
                .show(ui, &mut state.pages, &mut state.ui, &mut state.live, &mut state.tab);
        },
        ViewerHarnessState::default(),
    );
    harness.run_steps(2);

    harness.get_by_label("Tutorial").click();
    harness.run_steps(30);
    for label in [
        "TUTORIAL",
        "01 / 02  ·  INTRODUCTION",
        "The ego and traffic race on various circuits.",
        "TRACK",
        "PLANNER",
        "FUTURE PREVIEW",
        "DIAGNOSTIC POINTS / TRAJECTORIES",
        "PAUSE",
        "SCROLL",
    ] {
        assert!(
            harness.query_by_label(label).is_some(),
            "{label:?} missing from the Tutorial introduction"
        );
    }

    harness.get_by_label("CONTROLS  →").click();
    harness.run_steps(2);
    for label in [
        "TUTORIAL",
        "02 / 02  ·  CONTROLS",
        "LMB / WASD",
        "PAN",
        "RMB / Q E",
        "ROTATE",
        "WHEEL",
        "ZOOM",
        "F",
        "FOLLOW",
        "R",
        "RESET",
        "SPACE / ESC",
        "PAUSE",
        "T",
        "FRAME TIME",
    ] {
        assert!(
            harness.query_by_label(label).is_some(),
            "{label:?} missing from the Tutorial page"
        );
    }

    harness.get_by_label("BACK").click();
    harness.run_steps(2);
    assert!(harness.query_by_label("Start").is_some());
    assert!(harness.query_by_label("02 / 02  ·  CONTROLS").is_none());
}

#[test]
fn tutorial_pages_fit_supported_phone_viewports() {
    for (_, size) in PHONE_LANDSCAPE_SIZES {
        let mut harness = Harness::builder().with_size(size).build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    state.navigator.navigate(Page::Tutorial);
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
        for label in ["TUTORIAL", "TRACK", "PAUSE", "BACK", "CONTROLS  →"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                screen.contains_rect(rect),
                "{label:?} at {rect:?} is clipped at {size:?}"
            );
        }

        harness.get_by_label("CONTROLS  →").click();
        harness.run_steps(2);
        for label in ["TUTORIAL", "LMB / WASD", "RESET", "BACK", "←  INTRODUCTION"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                screen.contains_rect(rect),
                "{label:?} at {rect:?} is clipped at {size:?}"
            );
        }
    }
}
