use std::path::Path;

use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use super::pages::driving::{compact_layout, side_panel_margin, side_rail_widths};
use super::style::desktop_zoom;
use super::test_support::PHONE_LANDSCAPE_SIZES;
use super::{ControlTab, viewer_layout};
use crate::viewer::ui::style::configure;
use crate::viewer::ui::test_support::ViewerHarnessState;
use crate::viewer::{CANVAS_RGB, MIN_VIEWPORT_ASPECT_RATIO, MIN_VIEWPORT_WIDTH};

#[test]
fn viewer_elements_fit_and_render_at_target_sizes() {
    let output_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("viewer-renders");
    std::fs::create_dir_all(&output_dir).unwrap();

    let target_sizes = [
        (
            "minimum-supported",
            egui::vec2(MIN_VIEWPORT_WIDTH, MIN_VIEWPORT_WIDTH / MIN_VIEWPORT_ASPECT_RATIO),
            1.0,
        ),
        ("desktop-1080p", egui::vec2(1920.0, 1080.0), 1.0),
        ("desktop-ultrawide", egui::vec2(3440.0, 1440.0), 1.0),
        ("desktop-2160p", egui::vec2(1920.0, 1080.0), 2.0),
        ("desktop-crt-svga", egui::vec2(800.0, 600.0), 1.0),
        (PHONE_LANDSCAPE_SIZES[0].0, PHONE_LANDSCAPE_SIZES[0].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[1].0, PHONE_LANDSCAPE_SIZES[1].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[2].0, PHONE_LANDSCAPE_SIZES[2].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[3].0, PHONE_LANDSCAPE_SIZES[3].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[4].0, PHONE_LANDSCAPE_SIZES[4].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[5].0, PHONE_LANDSCAPE_SIZES[5].1, 1.0),
    ];
    for (name, size, pixels_per_point) in target_sizes {
        let mut harness = Harness::builder()
            .with_size(size)
            .with_pixels_per_point(pixels_per_point)
            .build_ui_state(
                |ui, state: &mut ViewerHarnessState| {
                    let ctx = ui.ctx().clone();
                    if !state.configured {
                        configure(&ctx);
                        state.configured = true;
                        ctx.request_repaint();
                        return;
                    }
                    let mut root = egui::Ui::new(
                        ctx.clone(),
                        "viewer_render_test".into(),
                        egui::UiBuilder::new().max_rect(ctx.content_rect()),
                    );
                    root.painter().rect_filled(
                        root.max_rect(),
                        0.0,
                        egui::Color32::from_rgb(CANVAS_RGB.0, CANVAS_RGB.1, CANVAS_RGB.2),
                    );
                    viewer_layout(
                        &mut root,
                        &mut state.navigator,
                        &mut state.pages,
                        &mut state.ui,
                        &mut state.live,
                        &mut state.tab,
                    );
                },
                ViewerHarnessState::default(),
            );
        harness.run();

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size * pixels_per_point);
        let compact = compact_layout(size);
        let (control_width, rail_width) = side_rail_widths(size);
        for label in ["OPTIONS", "ACTIVE PLANNER"] {
            let nodes: Vec<_> = harness
                .get_all_by_label(label)
                .filter(|node| node.rect().left() < control_width * pixels_per_point)
                .collect();
            assert!(!nodes.is_empty(), "{label:?} missing from the control rail at {name}");
            for node in nodes {
                let rect = node.rect();
                assert!(
                    screen.contains_rect(rect) && rect.max.x <= control_width * pixels_per_point && rect.is_positive(),
                    "{label:?} is clipped at {name}: {rect:?} outside the control rail"
                );
            }
        }
        assert!(harness.query_by_label("NANOPLAN").is_none());
        let rail = harness.get_by_label("Visualization rail").rect();
        assert!(
            screen.contains_rect(rail) && rail.min.x >= (size.x - rail_width) * pixels_per_point,
            "visualization rail is clipped at {name}: {rail:?}"
        );
        let hud = harness.get_by_label("Driving HUD").rect();
        assert!(
            rail.contains_rect(hud),
            "HUD is outside the right rail at {name}: {hud:?}"
        );
        let sections: Vec<_> = ["Lap stats", "Friction box", "Speed gauge"]
            .map(|label| harness.get_by_label(label).rect())
            .into_iter()
            .collect();
        for section in &sections {
            assert!(
                hud.contains_rect(*section) && section.is_positive(),
                "HUD section spills outside its container at {name}: {section:?} outside {hud:?}"
            );
        }
        assert!(
            sections[0].bottom() < sections[1].top() && sections[1].bottom() < sections[2].top(),
            "HUD sections have no gutters at {name}: {sections:?}"
        );
        assert!(
            sections[0].center().y < sections[1].center().y && sections[1].center().y < sections[2].center().y,
            "HUD sections lost top/middle/bottom alignment at {name}: {sections:?}"
        );

        let selector = harness.get_by_label("OPTIONS").rect();
        let mut previous_right = selector.left();
        for label in ["PLANNER", "OPPONENTS", "CAMERA", "VIZ", "METRICS", "TIMING"] {
            let rect = harness.get_by_label(&format!("{label} tab")).rect();
            assert!(
                selector.contains_rect(rect),
                "{label} tab spills outside selector at {name}"
            );
            assert!(
                (rect.width() - rect.height()).abs() < 1.0,
                "{label} tab is not square at {name}"
            );
            assert!(
                rect.left() >= previous_right - 1.0,
                "{label} tab overlaps its neighbor at {name}"
            );
            previous_right = rect.right();
        }
        let pause = harness.get_by_label("PAUSE").rect();
        assert!(
            pause.left() >= control_width * pixels_per_point
                && pause.right() <= screen.right() - rail_width * pixels_per_point,
            "pause button is outside the center rail at {name}: {pause:?}"
        );
        assert!(
            (pause.center().x - screen.center().x).abs() <= 1.0,
            "pause button is not centered at {name}: {pause:?}"
        );
        let margin = f32::from(side_panel_margin(size)) * desktop_zoom(size.y * pixels_per_point);
        assert!(
            (selector.left() - margin).abs() <= 1.0,
            "selector does not start at the left menu margin at {name}: {selector:?}"
        );
        assert!(
            (selector.right() - (control_width * pixels_per_point - margin)).abs() <= 1.0,
            "selector does not end at the right menu margin at {name}: {selector:?}"
        );
        let control_rail = egui::Rect::from_min_max(
            screen.left_top(),
            egui::pos2(control_width * pixels_per_point, screen.bottom()),
        );
        let budget = harness
            .get_by_role_and_label(egui::accesskit::Role::Slider, "Compute budget")
            .rect();
        assert!(
            (budget.left() - selector.left()).abs() <= 2.0 && (budget.right() - selector.right()).abs() <= 2.0,
            "compute budget slider does not use the full menu width at {name}: {budget:?}"
        );

        harness.state_mut().tab = ControlTab::Opponents;
        harness.run();
        for label in ["OPPONENTS", "5"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                control_rail.contains_rect(rect) && rect.is_positive(),
                "opponent control {label:?} spills outside the control rail at {name}: {rect:?}"
            );
        }
        let opponent_count = harness
            .get_by_role_and_label(egui::accesskit::Role::Slider, "Opponent count")
            .rect();
        assert!(
            (opponent_count.left() - selector.left()).abs() <= 2.0
                && (opponent_count.right() - selector.right()).abs() <= 2.0,
            "opponent slider does not use the full menu width at {name}: {opponent_count:?}, menu {selector:?}"
        );

        harness.state_mut().tab = ControlTab::Camera;
        harness.run();
        let control_deck = harness.get_by_label("Control deck").rect();
        assert!(
            control_rail.contains_rect(control_deck) && control_deck.is_positive(),
            "control deck spills outside the viewport at {name}: {control_deck:?}"
        );
        let camera_labels = if compact {
            [
                "FOLLOW",
                "Follow",
                "Align heading",
                "Smooth",
                "ZOOM",
                "Zoom control",
                "-15°",
                "NORTH",
                "+15°",
                "RESET",
            ]
        } else {
            [
                "FOLLOW",
                "Follow ego",
                "Align to ego heading",
                "Smooth motion",
                "ZOOM",
                "Zoom control",
                "-15°",
                "NORTH UP",
                "+15°",
                "RESET",
            ]
        };
        for label in camera_labels {
            let nodes: Vec<_> = harness.get_all_by_label(label).collect();
            assert!(!nodes.is_empty(), "camera control {label:?} missing at {name}");
            for node in nodes {
                let rect = node.rect();
                assert!(
                    rect.left() >= control_rail.left() && rect.right() <= control_rail.right() && rect.is_positive(),
                    "camera control {label:?} spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
                );
            }
        }
        let zoom = harness
            .get_by_role_and_label(egui::accesskit::Role::Slider, "Zoom control")
            .rect();
        assert!(
            (zoom.left() - selector.left()).abs() <= 2.0 && (zoom.right() - selector.right()).abs() <= 2.0,
            "zoom slider does not use the full menu width at {name}: {zoom:?}"
        );

        harness.state_mut().tab = ControlTab::Visibility;
        harness.run();
        let visibility_labels = if compact {
            [
                "FUTURE PREVIEW [S]",
                "Future preview",
                "Stations",
                "Centerline",
                "Carpet",
                "Ego carpet color",
                "Path",
            ]
        } else {
            [
                "FUTURE PREVIEW [S]",
                "Future preview",
                "Track stations",
                "Track centerline",
                "Ego carpet",
                "Ego carpet color",
                "Planned path",
            ]
        };
        for label in visibility_labels {
            let nodes: Vec<_> = harness.get_all_by_label(label).collect();
            assert!(!nodes.is_empty(), "visibility control {label:?} missing at {name}");
            for node in nodes {
                let rect = node.rect();
                assert!(
                    rect.left() >= control_rail.left() && rect.right() <= control_rail.right() && rect.width() > 0.0,
                    "visibility control {label:?} spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
                );
            }
        }
        let preview = harness
            .get_by_role_and_label(egui::accesskit::Role::Slider, "Future preview")
            .rect();
        assert!(
            (preview.left() - selector.left()).abs() <= 2.0 && (preview.right() - selector.right()).abs() <= 2.0,
            "future preview slider does not use the full menu width at {name}: {preview:?}"
        );

        harness.state_mut().tab = ControlTab::Metrics;
        harness.run();
        for label in ["PLANNER METRICS", "SAFETY", "PROGRESS", "COMFORT", "OVERALL"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                rect.left() >= control_rail.left() && rect.right() <= control_rail.right() && rect.width() > 0.0,
                "metric text {label:?} spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
            );
        }
        for label in [
            "DRIVING",
            "SPEED",
            "ACCELERATION",
            "CURVATURE",
            "LATEST PLAN",
            "FRAME",
            "WHOLE FRAME",
            "LATENCY SEAMS",
        ] {
            assert!(
                harness.query_by_label(label).is_none(),
                "timing field {label:?} leaked into Metrics at {name}"
            );
        }

        harness.state_mut().tab = ControlTab::Timing;
        harness.run();
        for label in ["PLANNING", "LATEST PLAN", "FRAME", "WHOLE FRAME", "LATENCY SEAMS"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                rect.left() >= control_rail.left() && rect.right() <= control_rail.right() && rect.width() > 0.0,
                "metric text {label:?} spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
            );
        }
        let metric_text: Vec<_> = harness
            .get_all_by_role(egui::accesskit::Role::Label)
            .filter(|node| node.rect().left() < control_rail.right())
            .collect();
        assert!(!metric_text.is_empty(), "metric text missing at {name}");
        for node in metric_text {
            let rect = node.rect();
            assert!(
                rect.left() >= control_rail.left() && rect.right() <= control_rail.right() && rect.width() > 0.0,
                "metric text spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
            );
        }

        harness
            .render()
            .unwrap_or_else(|error| panic!("failed to render {name}: {error}"))
            .save(output_dir.join(format!("{name}.png")))
            .unwrap();
    }
}
