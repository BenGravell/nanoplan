use crate::geometry::CAR_FOOTPRINT;
use crate::planning::PlannerKind;
use crate::simulation::State;
use bevy::prelude::*;

use super::camera::{
    CAMERA_BOTTOM_PADDING_PX, CameraState, CameraTarget, DEFAULT_ZOOM, MAX_ZOOM, MIN_ZOOM,
    followed_camera_center, smooth_angle,
};
use super::rendering::interpolate_state;
use super::screen::PX_PER_M;
use super::*;

#[test]
fn planner_change_resets_latency_stats() {
    let mut live = Live::default();
    live.latency.absorb(vec![("total", 1.0)]);
    live.set_planner(PlannerKind::Lattice);
    assert!(live.latency.seams.is_empty());
}

#[test]
fn camera_reset_restores_the_smooth_centerline_follow_view() {
    let mut camera = CameraState {
        center: Vec2::splat(99.0),
        zoom: 2.0,
        rotation: 1.0,
        follow: false,
        align_heading: false,
        target: CameraTarget::Ego,
        smooth: false,
    };

    camera.reset(Vec2::new(3.0, 4.0));

    assert_eq!(camera.center, Vec2::new(3.0, 4.0));
    assert_eq!(camera.zoom, DEFAULT_ZOOM);
    assert_eq!(camera.rotation, 0.0);
    assert!(camera.follow);
    assert!(camera.align_heading);
    assert!(matches!(camera.target, CameraTarget::Track));
    assert!(camera.smooth);
}

#[test]
fn camera_smoothing_takes_the_short_way_across_pi() {
    let almost_pi = std::f32::consts::PI - 0.1;
    let almost_negative_pi = -std::f32::consts::PI + 0.1;
    assert!(smooth_angle(almost_pi, almost_negative_pi, 0.5) > almost_pi);
}

#[test]
fn followed_camera_keeps_fixed_padding_behind_ego_at_every_zoom() {
    let viewport_height = 720.0;
    for (zoom, ego_yaw) in [(MIN_ZOOM, -0.4), (DEFAULT_ZOOM, 0.7), (MAX_ZOOM, 1.2)] {
        let camera = CameraState {
            center: Vec2::splat(50.0),
            zoom,
            rotation: 0.7,
            ..Default::default()
        };
        let ego = State {
            x: 3.0,
            y: 4.0,
            yaw: ego_yaw,
            ..Default::default()
        };
        let center = followed_camera_center(camera, ego, viewport_height);
        let up = Rot2::radians(camera.rotation) * Vec2::Y;
        let ego_in_view = (screen::px(&ego) - center).dot(up);
        let rear_extent =
            CAR_FOOTPRINT.support_radius(ego.yaw, [up.x as f64, up.y as f64]) as f32 * PX_PER_M;
        let rear_screen_y = (ego_in_view - rear_extent) * zoom;

        assert!((rear_screen_y + viewport_height / 2.0 - CAMERA_BOTTOM_PADDING_PX).abs() < 1e-3);
    }
}

#[test]
fn render_interpolation_blends_pose_and_wraps_yaw() {
    let previous = State {
        x: 2.0,
        yaw: std::f64::consts::PI - 0.2,
        speed: 4.0,
        ..Default::default()
    };
    let current = State {
        x: 6.0,
        y: 2.0,
        yaw: -std::f64::consts::PI + 0.2,
        speed: 8.0,
    };

    let rendered = interpolate_state(previous, current, 0.5);

    assert_eq!(rendered.x, 4.0);
    assert_eq!(rendered.y, 1.0);
    assert_eq!(rendered.speed, 6.0);
    assert!((rendered.yaw - std::f64::consts::PI).abs() < 1e-9);
}

#[test]
fn new_track_starts_with_ego_aligned_to_its_tangent() {
    let mut live = Live::default();
    live.acc = DT as f32 * 0.9;

    live.regenerate(2, PlannerKind::BezierToppra, 0);

    let (_, track_yaw) = live.world.track.pose(live.world.track_progress);
    assert!((live.world.ego.yaw - track_yaw).abs() < 1e-12);
    assert_eq!(live.previous.ego.yaw, track_yaw);
    assert_eq!(live.acc, 0.0);
}
