use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::ViewerHarnessState;
use crate::viewer::ui::{ControlTab, viewer_layout};

#[test]
fn ego_carpet_selector_lives_in_the_viz_menu() {
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
    harness.state_mut().tab = ControlTab::Visibility;
    harness.run();

    assert!(harness.get_by_label("Ego carpet color").rect().right() <= 440.0);
    assert!(harness.query_by_label("EGO CARPET COLOR").is_none());
}

#[test]
fn future_controls_live_together_in_the_viz_menu() {
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

    assert!(harness.query_by_label("Future preview").is_none());
    harness.state_mut().tab = ControlTab::Visibility;
    harness.run();

    for label in [
        "FUTURE PREVIEW [S]",
        "Future preview",
        "Ego carpet",
        "Planned path",
        "Search points",
        "Candidate trajectories",
    ] {
        assert!(
            harness.get_all_by_label(label).next().is_some(),
            "{label:?} missing from the Viz menu"
        );
    }
}
