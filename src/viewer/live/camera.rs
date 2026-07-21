use crate::geometry::CAR_FOOTPRINT;
use crate::simulation::State;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;

use super::Live;
use super::screen::{PX_PER_M, px};
use crate::viewer::DrivingCanvas;

pub(super) const DEFAULT_ZOOM: f32 = 1.0;
pub(crate) const MIN_ZOOM: f32 = 0.01;
pub(crate) const MAX_ZOOM: f32 = 6.0;
pub(super) const CAMERA_BOTTOM_PADDING_PX: f32 = 48.0;

#[derive(Clone, Copy)]
pub(crate) struct CameraState {
    pub(crate) center: Vec2,
    pub(crate) zoom: f32,
    pub(crate) rotation: f32,
    pub(crate) follow: bool,
    pub(crate) align_heading: bool,
    pub(crate) smooth: bool,
}

impl CameraState {
    pub(super) fn reset(&mut self, ego: Vec2) {
        *self = Self {
            center: ego,
            zoom: DEFAULT_ZOOM,
            rotation: 0.0,
            follow: true,
            align_heading: true,
            smooth: true,
        };
    }
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            center: Vec2::ZERO,
            zoom: DEFAULT_ZOOM,
            rotation: 0.0,
            follow: true,
            align_heading: true,
            smooth: true,
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn camera_input(
    mut live: NonSendMut<Live>,
    mouse: Res<ButtonInput<MouseButton>>,
    touches: Res<Touches>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    egui_input: Res<EguiWantsInput>,
    driving_canvas: Res<DrivingCanvas>,
    window: Single<&Window>,
    time: Res<Time>,
) {
    if !egui_input.wants_any_pointer_input() {
        if cursor_over_driving_canvas(window.cursor_position(), driving_canvas.rect) {
            let scroll_steps = match scroll.unit {
                MouseScrollUnit::Line => scroll.delta.y,
                MouseScrollUnit::Pixel => scroll.delta.y / 50.0,
            };
            live.camera.zoom =
                (live.camera.zoom * 1.1f32.powf(scroll_steps)).clamp(MIN_ZOOM, MAX_ZOOM);
        }

        let touches: Vec<_> = touches.iter().take(2).collect();
        if let [first, second] = touches.as_slice() {
            let previous = first
                .previous_position()
                .distance(second.previous_position());
            let current = first.position().distance(second.position());
            live.camera.zoom =
                (live.camera.zoom * pinch_scale(previous, current)).clamp(MIN_ZOOM, MAX_ZOOM);
        }

        if mouse.pressed(MouseButton::Middle) && motion.delta != Vec2::ZERO {
            let drag = Rot2::radians(live.camera.rotation)
                * Vec2::new(motion.delta.x, -motion.delta.y)
                / live.camera.zoom;
            live.camera.center -= drag;
            live.camera.follow = false;
        }
        if mouse.pressed(MouseButton::Right) && motion.delta.x != 0.0 {
            live.camera.rotation += motion.delta.x * 0.005;
            live.camera.align_heading = false;
        }
    }

    if egui_input.wants_any_keyboard_input() {
        return;
    }
    if keys.just_pressed(KeyCode::KeyF) {
        live.camera.follow = !live.camera.follow;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        live.reset_camera();
    }

    let mut pan = Vec2::ZERO;
    for (key, direction) in [
        (KeyCode::KeyA, -Vec2::X),
        (KeyCode::ArrowLeft, -Vec2::X),
        (KeyCode::KeyD, Vec2::X),
        (KeyCode::ArrowRight, Vec2::X),
        (KeyCode::KeyW, Vec2::Y),
        (KeyCode::ArrowUp, Vec2::Y),
        (KeyCode::KeyS, -Vec2::Y),
        (KeyCode::ArrowDown, -Vec2::Y),
    ] {
        if keys.pressed(key) {
            pan += direction;
        }
    }
    if pan != Vec2::ZERO {
        let camera = live.camera;
        live.camera.center +=
            Rot2::radians(camera.rotation) * pan.normalize() * 500.0 * time.delta_secs()
                / camera.zoom;
        live.camera.follow = false;
    }
    let rotation_input = keys.pressed(KeyCode::KeyE) as i8 - keys.pressed(KeyCode::KeyQ) as i8;
    if rotation_input != 0 {
        live.camera.rotation += rotation_input as f32 * time.delta_secs();
        live.camera.align_heading = false;
    }
}

pub(super) fn cursor_over_driving_canvas(cursor: Option<Vec2>, canvas: Option<Rect>) -> bool {
    cursor
        .zip(canvas)
        .is_some_and(|(cursor, canvas)| canvas.contains(cursor))
}

pub(super) fn pinch_scale(previous_distance: f32, current_distance: f32) -> f32 {
    if previous_distance > f32::EPSILON {
        current_distance / previous_distance
    } else {
        1.0
    }
}

pub(super) fn smooth_angle(current: f32, target: f32, blend: f32) -> f32 {
    let delta = (target - current + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    current + delta * blend
}

pub(super) fn followed_camera_center(
    camera: CameraState,
    ego: State,
    viewport_height: f32,
) -> Vec2 {
    let up = Rot2::radians(camera.rotation) * Vec2::Y;
    let rear_extent =
        CAR_FOOTPRINT.support(ego.yaw, [-up.x as f64, -up.y as f64]) as f32 * PX_PER_M;
    let ego_y = -(viewport_height / 2.0 - CAMERA_BOTTOM_PADDING_PX) / camera.zoom + rear_extent;
    camera.center + up * ((px(&ego) - camera.center).dot(up) - ego_y)
}
