use super::{MIN_VIEWPORT_ASPECT_RATIO, MIN_VIEWPORT_WIDTH, ResizeDebounce, UiState, viewport_supported};
use crate::planning::PlannerKind;
use crate::viewer::ui::{Navigator, Page};

#[test]
fn visualization_defaults_show_only_track_stations() {
    let state = UiState::default();
    assert_eq!(Navigator::default().page(), Page::Start);
    assert_eq!(state.planner, PlannerKind::Basic);
    assert!(state.show_stations);
    assert!(!state.show_centerline);
    assert!(!state.show_plan);
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
fn resize_coalesces_changes_then_accepts_native_4k() {
    let mut resize = ResizeDebounce::default();
    assert!(!resize.observe(bevy::math::UVec2::new(1280, 720), 0.0));
    assert!(resize.observe(bevy::math::UVec2::new(3840, 2160), 0.0));
    assert!(resize.observe(bevy::math::UVec2::new(3840, 2160), 0.19));
    assert!(!resize.observe(bevy::math::UVec2::new(3840, 2160), 0.02));
    assert_eq!(resize.displayed, bevy::math::UVec2::new(3840, 2160));
    assert_eq!(resize.rollback(), Some(bevy::math::UVec2::new(1280, 720)));
}

#[test]
fn resize_churn_cannot_disable_rendering_forever() {
    let mut resize = ResizeDebounce::default();
    assert!(!resize.observe(bevy::math::UVec2::new(1280, 720), 0.0));
    assert!(resize.observe(bevy::math::UVec2::new(1280, 719), 0.07));
    assert!(resize.observe(bevy::math::UVec2::new(1280, 718), 0.07));
    assert!(!resize.observe(bevy::math::UVec2::new(1280, 717), 0.07));
    assert_eq!(resize.displayed, bevy::math::UVec2::new(1280, 717));
}
