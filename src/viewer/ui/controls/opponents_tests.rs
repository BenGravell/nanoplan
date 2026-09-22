use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::ViewerHarnessState;
use crate::viewer::ui::{ControlTab, viewer_layout};

#[test]
fn opponents_menu_controls_the_opponent_count_from_zero_to_fifteen() {
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
    harness.state_mut().live.camera.zoom = 2.0;
    let ego = harness.state().live.world.ego();
    harness.get_by_label("OPPONENTS tab").click();
    harness.run();
    assert!(harness.state().tab == ControlTab::Opponents);

    assert!(harness.query_by_label("OPPONENTS").is_some());
    assert_eq!(harness.state().live.world.ego(), ego);
    assert_eq!(harness.state().live.camera.zoom, 2.0);
    let slider = harness.get_by_role(egui::accesskit::Role::Slider).rect();
    let left = slider.left_center() + egui::vec2(1.0, 0.0);
    let right = slider.right_center() - egui::vec2(1.0, 0.0);
    harness.drag_at(left);
    harness.hover_at(right);
    harness.drop_at(right);
    harness.run();
    assert_eq!(harness.state().ui.opponents, 15);
    assert_eq!(harness.state().live.world.actors.len(), 15);

    harness.state_mut().ui.opponents = 0;
    harness.run();
    assert_eq!(harness.state().ui.opponents, 0);
    assert!(harness.state().live.world.actors.is_empty());
}
