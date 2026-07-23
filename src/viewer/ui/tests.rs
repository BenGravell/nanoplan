use std::path::Path;

use bevy_egui::egui;
use egui_kittest::{Harness, kittest::Queryable};

use super::controls::metrics::preview_metrics;
use super::style::desktop_zoom;
use super::{
    ControlTab, UiState, center_rail_rect, compact_layout, configure, handle_keyboard_controls,
    landing, portrait_prompt, side_panel_margin, side_rail_widths, tutorial, viewer_layout,
};
use crate::planning::{Latency, PlannerKind};
use crate::viewer::{
    CANVAS_RGB, MIN_VIEWPORT_ASPECT_RATIO, MIN_VIEWPORT_WIDTH, ResizeDebounce, live::Live,
    viewport_constraints, viewport_supported,
};

const PHONE_LANDSCAPE_SIZES: [(&str, egui::Vec2); 6] = [
    ("phone-galaxy-s", egui::vec2(800.0, 480.0)),
    ("phone-iphone-se-2016", egui::vec2(568.0, 320.0)),
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
                state.exit_requested |=
                    landing::show(ui, &mut state.ui.started, &mut state.ui.tutorial);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    assert!(harness.query_by_label("Start").is_some());
    harness.key_press(egui::Key::Enter);
    harness.run_steps(20);
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
                state.exit_requested |=
                    landing::show(ui, &mut state.ui.started, &mut state.ui.tutorial);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    harness.get_by_label("Exit").click();
    harness.run_steps(30);
    assert!(harness.state().exit_requested);
}

#[test]
fn landing_tutorial_opens_the_camera_keymap_and_returns() {
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
                if state.ui.tutorial {
                    tutorial::show(ui, &mut state.ui.tutorial);
                } else {
                    state.exit_requested |=
                        landing::show(ui, &mut state.ui.started, &mut state.ui.tutorial);
                }
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    harness.get_by_label("Tutorial").click();
    harness.run_steps(30);
    for label in [
        "TUTORIAL",
        "CAMERA CONTROLS",
        "MMB / WASD",
        "PAN",
        "RMB / Q E",
        "ROTATE",
        "WHEEL",
        "ZOOM",
        "F",
        "FOLLOW",
        "R",
        "RESET",
        "P / ESC",
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
    assert!(harness.query_by_label("CAMERA CONTROLS").is_none());
}

#[test]
fn tutorial_keymap_fits_supported_phone_viewports() {
    for (_, size) in PHONE_LANDSCAPE_SIZES {
        let mut harness = Harness::builder().with_size(size).build_ui_state(
            |ui, configured: &mut bool| {
                if !*configured {
                    configure(ui.ctx());
                    *configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                let mut open = true;
                tutorial::show(ui, &mut open);
            },
            false,
        );
        harness.run_steps(2);

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
        for label in ["TUTORIAL", "CAMERA CONTROLS", "MMB / WASD", "RESET", "BACK"] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                screen.contains_rect(rect),
                "{label:?} at {rect:?} is clipped at {size:?}"
            );
        }
    }
}

#[test]
fn landing_activation_waits_long_enough_to_show_feedback() {
    assert!(!landing::activation_ready(0.199));
    assert!(landing::activation_ready(0.2));
}

#[test]
fn landing_chevron_pulses_and_bounces_horizontally() {
    let (center, normal) = landing::chevron_animation(0.0);
    let (shoulder, _) = landing::chevron_animation(1.0 / 6.0);
    let (right, large) = landing::chevron_animation(1.0 / 3.0);
    let (left, small) = landing::chevron_animation(1.0);

    assert!(large > normal && small < normal);
    assert!(right > center && left < center);
    assert!(shoulder > right * 0.9);
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

fn render_bottom_corner(corner: landing::BottomCorner, size: egui::Vec2) -> image::RgbaImage {
    let mut harness = Harness::builder().with_size(size).build_ui_state(
        move |ui, configured: &mut bool| {
            if !*configured {
                configure(ui.ctx());
                *configured = true;
                ui.ctx().request_repaint();
                return;
            }
            landing::paint_bottom_corner(ui, corner);
        },
        false,
    );
    harness.run();
    harness.render().unwrap()
}

fn bottom_corners_collide(
    left: &image::RgbaImage,
    right: &image::RgbaImage,
    screen_width: u32,
) -> bool {
    let background_width = left.width();
    let right_offset = i64::from(screen_width) - i64::from(background_width);
    (0..screen_width).any(|screen_x| {
        let right_x = i64::from(screen_x) - right_offset;
        right_x >= 0
            && right_x < i64::from(background_width)
            && (0..left.height()).any(|y| {
                left.get_pixel(screen_x, y).0[3] != 0
                    && right.get_pixel(right_x as u32, y).0[3] != 0
            })
    })
}

#[test]
fn landing_bottom_corner_visibility_threshold_prevents_svg_pixel_collisions() {
    assert!(landing::show_bottom_left(egui::vec2(31.0, 20.0)));
    assert!(!landing::show_bottom_left(egui::vec2(3.0, 2.0)));
    assert!(landing::show_bottom_left(egui::vec2(16.0, 9.0)));

    for height in [360, 720, 1080] {
        let background_width = height * 16 / 9;
        let size = egui::vec2(background_width as f32, height as f32);
        let left = render_bottom_corner(landing::BottomCorner::Left, size);
        let right = render_bottom_corner(landing::BottomCorner::Right, size);
        let visible_width = (height as f32 * 31.0 / 20.0).ceil() as u32;
        let hidden_width = height * 3 / 2;

        assert!(!bottom_corners_collide(&left, &right, visible_width));
        assert!(bottom_corners_collide(&left, &right, hidden_width));
    }
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
                portrait_prompt::show(&mut root, true, viewport_constraints(size.x, size.y));
            },
            false,
        );
        harness.run();

        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, size);
        for label in [
            "TURN YOUR DEVICE SIDEWAYS",
            "Nanoplan requires landscape orientation.",
        ] {
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
            portrait_prompt::show(ui, false, viewport_constraints(size.x, size.y));
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
        portrait_prompt::prompt_copy(viewport_constraints(390.0, 844.0), true),
        (
            "TURN YOUR DEVICE SIDEWAYS",
            "Nanoplan requires landscape orientation.".to_owned()
        )
    );
    assert_eq!(
        portrait_prompt::prompt_copy(viewport_constraints(390.0, 844.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport at least 520 px wide with a 4:3 aspect ratio.".to_owned()
        )
    );
    assert_eq!(
        portrait_prompt::prompt_copy(viewport_constraints(700.0, 844.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport with at least a 4:3 aspect ratio.".to_owned()
        )
    );
    assert_eq!(
        portrait_prompt::prompt_copy(viewport_constraints(519.0, 320.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport at least 520 px wide.".to_owned()
        )
    );
    assert_eq!(
        portrait_prompt::prompt_copy(viewport_constraints(600.0, 500.0), false),
        (
            "MAKE YOUR WINDOW WIDER",
            "Nanoplan requires a viewport with at least a 4:3 aspect ratio.".to_owned()
        )
    );
}

#[test]
fn viewport_requires_minimum_width_and_safe_background_aspect_ratio() {
    assert!(viewport_supported(568.0, 320.0));
    assert!(viewport_supported(
        MIN_VIEWPORT_WIDTH,
        MIN_VIEWPORT_WIDTH / MIN_VIEWPORT_ASPECT_RATIO
    ));
    assert!(!viewport_supported(
        MIN_VIEWPORT_WIDTH - 1.0,
        (MIN_VIEWPORT_WIDTH - 1.0) / MIN_VIEWPORT_ASPECT_RATIO
    ));
    assert!(!viewport_supported(MIN_VIEWPORT_WIDTH, 391.0));
}

#[test]
fn resize_waits_for_a_quiet_period_then_accepts_native_4k() {
    let mut resize = ResizeDebounce::default();
    assert!(!resize.observe(bevy::math::UVec2::new(1280, 720), 0.0));
    assert!(resize.observe(bevy::math::UVec2::new(3840, 2160), 0.0));
    assert!(resize.observe(bevy::math::UVec2::new(3840, 2160), 0.19));
    assert!(!resize.observe(bevy::math::UVec2::new(3840, 2160), 0.02));
    assert_eq!(resize.displayed, bevy::math::UVec2::new(3840, 2160));
    assert_eq!(resize.rollback(), Some(bevy::math::UVec2::new(1280, 720)));
}

#[test]
fn ego_carpet_selector_lives_in_the_viz_menu() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
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
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
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

#[test]
fn opponents_menu_controls_the_opponent_count_from_zero_to_fifteen() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);
    harness.state_mut().live.camera.zoom = 2.0;
    let ego = harness.state().live.world.ego();
    harness.state_mut().tab = ControlTab::Opponents;
    harness.run();

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

#[test]
fn section_selector_opens_track_options() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    harness
        .get_by_role_and_label(egui::accesskit::Role::ComboBox, "OPTIONS")
        .click();
    harness.run();
    harness.get_by_label("TRACK").click();
    harness.run();

    assert!(harness.state().tab == ControlTab::Track);
    assert!(harness.query_by_label("↻ NEW TRACK").is_some());

    harness.state_mut().ui.track = 1;
    harness.run();
    assert!(harness.query_by_label("↻ NEW TRACK").is_none());
}

#[test]
fn pause_rail_opens_navigation_modal() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    state.ui.started = true;
                    ui.ctx().request_repaint();
                    return;
                }
                let (_, exit_requested) =
                    viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
                state.exit_requested |= exit_requested;
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    harness.get_by_label("PAUSE").click();
    harness.run();
    for label in ["RESUME", "RETURN TO START MENU", "EXIT"] {
        assert!(harness.query_by_label(label).is_some());
    }
    let paused = harness.get_by_label("PAUSED").rect();
    let resume = harness.get_by_label("RESUME").rect();
    assert!((paused.center().x - resume.center().x).abs() <= 1.0);

    harness.get_by_label("RESUME").click();
    harness.run();
    assert!(!harness.state().live.paused);

    harness.get_by_label("PAUSE").click();
    harness.run();
    harness.get_by_label("RETURN TO START MENU").click();
    harness.run();
    assert!(!harness.state().ui.started);

    harness.state_mut().ui.started = true;
    harness.get_by_label("PAUSE").click();
    harness.run();
    harness.get_by_label("EXIT").click();
    harness.run();
    assert!(harness.state().exit_requested);
}

#[test]
fn keyboard_shortcuts_pause_and_toggle_frame_time() {
    let mut harness = Harness::builder()
        .with_size(egui::vec2(1280.0, 720.0))
        .build_ui_state(
            |ui, state: &mut ViewerHarnessState| {
                if !state.configured {
                    configure(ui.ctx());
                    state.configured = true;
                    ui.ctx().request_repaint();
                    return;
                }
                handle_keyboard_controls(ui.ctx(), &mut state.ui, &mut state.live);
                viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
            },
            ViewerHarnessState::default(),
        );
    harness.run_steps(2);

    harness.key_press(egui::Key::P);
    harness.run();
    assert!(harness.state().live.paused);
    assert!(harness.query_by_label("PAUSED").is_some());

    harness.key_press(egui::Key::P);
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
            let (canvas, _) = viewer_layout(ui, &mut state.ui, &mut state.live, &mut state.tab);
            assert_eq!(canvas, center_rail_rect(viewport, left, right));
        },
        ViewerHarnessState::default(),
    );

    harness.run_steps(2);
}

#[test]
fn active_scroll_handles_use_the_orange_widget_fill() {
    let ctx = egui::Context::default();
    configure(&ctx);
    let style = ctx.style_of(egui::Theme::Light);

    assert!(!style.spacing.scroll.foreground_color);
    assert_eq!(
        style.visuals.widgets.active.bg_fill,
        crate::viewer::colors::ORANGE
    );
    assert_ne!(
        style.visuals.widgets.hovered.bg_fill,
        style.visuals.widgets.active.bg_fill
    );
}

#[test]
fn orange_ui_states_always_use_white_foregrounds() {
    let ctx = egui::Context::default();
    configure(&ctx);
    let style = ctx.style_of(egui::Theme::Light);

    assert_eq!(
        style.visuals.widgets.active.bg_fill,
        crate::viewer::colors::ORANGE
    );
    assert_eq!(
        style.visuals.widgets.active.fg_stroke.color,
        egui::Color32::WHITE
    );
    assert_eq!(
        style.visuals.selection.bg_fill,
        crate::viewer::colors::ORANGE
    );
    assert_eq!(style.visuals.selection.stroke.color, egui::Color32::WHITE);
    assert_eq!(style.visuals.override_text_color, None);
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

#[test]
fn desktop_ui_zoom_scales_smoothly_from_1080p_through_2160p() {
    assert_eq!(desktop_zoom(720.0), 1.0);
    assert_eq!(desktop_zoom(1080.0), 1.0);
    assert!((desktop_zoom(1440.0) - 4.0 / 3.0).abs() < f32::EPSILON);
    assert_eq!(desktop_zoom(2160.0), 2.0);
    assert_eq!(desktop_zoom(2880.0), 2.0);
}

#[test]
fn viewer_elements_fit_and_render_at_target_sizes() {
    let output_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("viewer-renders");
    std::fs::create_dir_all(&output_dir).unwrap();

    let target_sizes = [
        (
            "minimum-supported",
            egui::vec2(
                MIN_VIEWPORT_WIDTH,
                MIN_VIEWPORT_WIDTH / MIN_VIEWPORT_ASPECT_RATIO,
            ),
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
                    viewer_layout(&mut root, &mut state.ui, &mut state.live, &mut state.tab);
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
            sections[0].center().y < sections[1].center().y
                && sections[1].center().y < sections[2].center().y,
            "HUD sections lost top/middle/bottom alignment at {name}: {sections:?}"
        );

        let selector = harness
            .get_by_role_and_label(egui::accesskit::Role::ComboBox, "OPTIONS")
            .rect();
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

        harness.state_mut().tab = ControlTab::Camera;
        harness.run();
        let control_deck = harness.get_by_label("Control deck").rect();
        let control_rail = egui::Rect::from_min_max(
            screen.left_top(),
            egui::pos2(control_width * pixels_per_point, screen.bottom()),
        );
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
                "Follow camera",
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
            assert!(
                !nodes.is_empty(),
                "camera control {label:?} missing at {name}"
            );
            for node in nodes {
                let rect = node.rect();
                assert!(
                    rect.left() >= control_rail.left()
                        && rect.right() <= control_rail.right()
                        && rect.is_positive(),
                    "camera control {label:?} spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
                );
            }
        }

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
            assert!(
                !nodes.is_empty(),
                "visibility control {label:?} missing at {name}"
            );
            for node in nodes {
                let rect = node.rect();
                assert!(
                    rect.left() >= control_rail.left()
                        && rect.right() <= control_rail.right()
                        && rect.width() > 0.0,
                    "visibility control {label:?} spills horizontally outside the control rail at {name}: {rect:?} outside {control_rail:?}"
                );
            }
        }

        harness.state_mut().tab = ControlTab::Metrics;
        harness.run();
        for label in [
            "PLANNER METRICS",
            "SAFETY",
            "PROGRESS",
            "COMFORT",
            "OVERALL",
        ] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                rect.left() >= control_rail.left()
                    && rect.right() <= control_rail.right()
                    && rect.width() > 0.0,
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
        for label in [
            "PLANNING",
            "LATEST PLAN",
            "FRAME",
            "WHOLE FRAME",
            "LATENCY SEAMS",
        ] {
            let rect = harness.get_by_label(label).rect();
            assert!(
                rect.left() >= control_rail.left()
                    && rect.right() <= control_rail.right()
                    && rect.width() > 0.0,
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
                rect.left() >= control_rail.left()
                    && rect.right() <= control_rail.right()
                    && rect.width() > 0.0,
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
