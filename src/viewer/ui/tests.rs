use std::path::Path;

use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use super::controls::metrics::preview_metrics;
use super::{
    ControlTab, UiState, compact_layout, compact_rail_widths, configure, portrait_prompt,
    right_rail_width, viewer_layout,
};
use crate::planning::Latency;
use crate::viewer::{CANVAS_RGB, live::Live};

const PHONE_LANDSCAPE_SIZES: [(&str, egui::Vec2); 4] = [
    ("phone-iphone-se-3", egui::vec2(667.0, 375.0)),
    ("phone-galaxy-s24", egui::vec2(780.0, 360.0)),
    ("phone-iphone-14-15-pro", egui::vec2(852.0, 393.0)),
    ("phone-galaxy-a55-wide", egui::vec2(1040.0, 480.0)),
];

struct ViewerHarnessState {
    ui: UiState,
    live: Live,
    tab: ControlTab,
    configured: bool,
}

impl Default for ViewerHarnessState {
    fn default() -> Self {
        let mut live = Live::default();
        live.world.tick_recording_latency(&Latency::default());
        Self {
            ui: UiState::default(),
            live,
            tab: ControlTab::Planner,
            configured: false,
        }
    }
}

#[test]
fn visualization_defaults_show_only_track_stations() {
    let state = UiState::default();
    assert!(state.show_stations);
    assert!(!state.show_centerline);
    assert!(!state.show_plan);
}

#[test]
fn portrait_prompt_is_the_only_interactive_view() {
    let size = egui::vec2(390.0, 844.0);
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
            portrait_prompt::show(&mut root);
        },
        false,
    );
    harness.run();

    assert!(harness.query_by_label("NANOPLAN").is_some());
    assert!(
        harness
            .query_by_label("TURN YOUR DEVICE SIDEWAYS")
            .is_some()
    );
    assert!(harness.query_by_label("PAUSE").is_none());
}

#[test]
fn layout_only_compacts_for_phone_sized_viewports() {
    for (_, phone) in PHONE_LANDSCAPE_SIZES {
        assert!(compact_layout(phone));
        let (left, right) = compact_rail_widths(phone);
        assert!(phone.x - left - right >= phone.x * 0.4);
    }
    assert!(compact_layout(egui::vec2(960.0, 540.0)));
    assert!(!compact_layout(egui::vec2(1920.0, 1080.0)));
    assert!(!compact_layout(egui::vec2(3440.0, 1440.0)));
    assert!(!compact_layout(egui::vec2(3840.0, 2160.0)));
}

#[test]
fn viewer_elements_fit_and_render_at_target_sizes() {
    let output_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("viewer-renders");
    std::fs::create_dir_all(&output_dir).unwrap();

    let target_sizes = [
        ("desktop-1080p", egui::vec2(1920.0, 1080.0), 1.0),
        ("desktop-ultrawide", egui::vec2(3440.0, 1440.0), 1.0),
        ("desktop-2160p", egui::vec2(1920.0, 1080.0), 2.0),
        (PHONE_LANDSCAPE_SIZES[0].0, PHONE_LANDSCAPE_SIZES[0].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[1].0, PHONE_LANDSCAPE_SIZES[1].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[2].0, PHONE_LANDSCAPE_SIZES[2].1, 1.0),
        (PHONE_LANDSCAPE_SIZES[3].0, PHONE_LANDSCAPE_SIZES[3].1, 1.0),
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
                    viewer_layout(&mut root, &mut state.ui, &mut state.live, &mut state.tab);
                },
                ViewerHarnessState::default(),
            );
        harness.run();

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size * pixels_per_point);
        let compact = compact_layout(size);
        let (control_width, rail_width) = if compact {
            compact_rail_widths(size)
        } else {
            (
                (size.x * 0.2).clamp(372.0, 440.0),
                right_rail_width(size, compact),
            )
        };
        for label in [
            "NANOPLAN",
            "PAUSE",
            "↻ NEW TRACK",
            "PLANNER",
            "CAMERA",
            "VIZ",
            "METRICS",
            "ACTIVE PLANNER",
            if compact {
                "preview"
            } else {
                "future preview [s]"
            },
        ] {
            let nodes: Vec<_> = harness
                .get_all_by_label(label)
                .into_iter()
                .filter(|node| node.rect().left() < control_width * pixels_per_point)
                .collect();
            assert!(
                !nodes.is_empty(),
                "{label:?} missing from the control rail at {name}"
            );
            for node in nodes {
                let rect = node.rect();
                assert!(
                    screen.contains_rect(rect)
                        && rect.max.x <= control_width * pixels_per_point
                        && rect.is_positive(),
                    "{label:?} is clipped at {name}: {rect:?} outside the control rail"
                );
            }
        }
        let rail = harness.get_by_label("Timeseries rail").rect();
        assert!(
            screen.contains_rect(rail) && rail.min.x >= (size.x - rail_width) * pixels_per_point,
            "timeseries rail is clipped at {name}: {rail:?}"
        );
        let hud = harness.get_by_label("Driving HUD").rect();
        assert!(
            rail.contains_rect(hud),
            "HUD is outside the right rail at {name}: {hud:?}"
        );

        let pause = harness.get_by_label("PAUSE").rect();
        let new_track = harness.get_by_label("↻ NEW TRACK").rect();
        let planner = harness.get_by_label("PLANNER").rect();
        assert!((pause.width() - new_track.width()).abs() <= 1.0);
        assert!((pause.left() - planner.left()).abs() <= 1.0);
        let control_rect = |label| {
            harness
                .get_all_by_label(label)
                .into_iter()
                .map(|node| node.rect())
                .find(|rect| rect.left() < control_width * pixels_per_point)
                .unwrap()
        };
        let last_tab = control_rect(if compact { "CAMERA" } else { "METRICS" });
        assert!((new_track.right() - last_tab.right()).abs() <= 1.0);
        for label in ["CAMERA", "VIZ", "METRICS"] {
            let tab = control_rect(label);
            assert!(
                (tab.width() - planner.width()).abs() <= 1.0,
                "tab widths differ at {name}: PLANNER {planner:?}, {label} {tab:?}"
            );
        }

        harness
            .render()
            .unwrap_or_else(|error| panic!("failed to render {name}: {error}"))
            .save(output_dir.join(format!("{name}.png")))
            .unwrap();
    }
}

#[test]
fn preview_metrics_are_valid_scores() {
    let metrics = preview_metrics(&Live::default());
    assert!(
        metrics
            .aggregate
            .into_iter()
            .chain([metrics.score])
            .all(|score| (0.0..=1.0).contains(&score))
    );
}
