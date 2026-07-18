use std::path::Path;

use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use super::controls::metrics::preview_metrics;
use super::{
    ControlTab, UiState, compact_layout, compact_rail_widths, configure, landing, portrait_prompt,
    right_rail_width, viewer_layout,
};
use crate::planning::{Latency, PlannerKind};
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
    exit_requested: bool,
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
            exit_requested: false,
        }
    }
}

#[test]
fn visualization_defaults_show_only_track_stations() {
    let state = UiState::default();
    assert!(!state.started);
    assert_eq!(state.planner, PlannerKind::Basic);
    assert!(state.show_stations);
    assert!(!state.show_centerline);
    assert!(!state.show_plan);
}

#[test]
fn landing_starts_with_the_keyboard() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                let ctx = ui.ctx().clone();
                if !state.configured {
                    configure(&ctx);
                    state.configured = true;
                    ctx.request_repaint();
                    return;
                }
                state.exit_requested = landing::show(ui, &mut state.ui.started);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    assert!(harness.query_by_label("Start").is_some());
    harness.key_press(egui::Key::Enter);
    harness.step();
    assert!(harness.state().ui.started);
}

#[test]
fn landing_exit_selection_requests_app_shutdown() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                let ctx = ui.ctx().clone();
                if !state.configured {
                    configure(&ctx);
                    state.configured = true;
                    ctx.request_repaint();
                    return;
                }
                state.exit_requested = landing::show(ui, &mut state.ui.started);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    harness.get_by_label("Exit").click();
    harness.step();
    assert!(harness.state().exit_requested);
}

#[test]
fn landing_title_uses_normalized_reference_coordinates() {
    let screen = egui::Rect::from_min_size(egui::pos2(13.0, 17.0), egui::vec2(1000.0, 720.0));
    let title = landing::title_rect(screen);
    let reference_width = screen.height() * 16.0 / 9.0;
    assert!(
        ((title.left() - screen.left()) / reference_width - 0.041_666_668).abs() < f32::EPSILON
    );
    assert!(((title.top() - screen.top()) / screen.height() - 0.148_148_15).abs() < f32::EPSILON);
    assert!((title.width() / reference_width - 0.395_833_34).abs() < f32::EPSILON);
}

#[test]
fn landing_menu_uses_normalized_reference_coordinates() {
    let screen = egui::Rect::from_min_size(egui::pos2(13.0, 17.0), egui::vec2(1000.0, 720.0));
    let reference_width = screen.height() * 16.0 / 9.0;
    let first = landing::menu_row_rect(screen, 0);
    let second = landing::menu_row_rect(screen, 1);
    assert!(
        ((first.left() - screen.left()) / reference_width - 0.057_291_668).abs() < f32::EPSILON
    );
    assert!(((first.top() - screen.top()) / screen.height() - 0.324_074_06).abs() < f32::EPSILON);
    assert!(((second.top() - first.top()) / screen.height() - 0.1).abs() < f32::EPSILON);
}

#[test]
fn landing_backgrounds_span_height_and_anchor_to_their_corners() {
    let screen = egui::Rect::from_min_size(egui::pos2(13.0, 17.0), egui::vec2(1000.0, 720.0));
    let bottom_right = landing::background_rect(screen, egui::Align2::RIGHT_BOTTOM);
    let bottom_left = landing::background_rect(screen, egui::Align2::LEFT_BOTTOM);
    let top_left = landing::background_rect(screen, egui::Align2::LEFT_TOP);

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
    let raster = landing::background_raster_size(egui::vec2(2155.0, 1212.0), 1.0, 2048);
    assert_eq!(raster.x, 2048.0);
    assert!(raster.y <= 2048.0);
}

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
                portrait_prompt::show(&mut root);
            },
            false,
        );
        harness.run();

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
        for label in [
            "NANOPLAN",
            "TURN YOUR DEVICE SIDEWAYS",
            "Nanoplan requires landscape orientation.",
        ] {
            assert!(screen.contains_rect(harness.get_by_label(label).rect()));
        }
        assert!(harness.query_by_label("PAUSE").is_none());
    }
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
        assert!(harness.query_by_label("NANOPLAN").is_none());
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
