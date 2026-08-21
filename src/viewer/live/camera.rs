use crate::simulation::State;
use bevy::input::mouse::{AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;

use super::Live;
use super::rendering::rendered_ego;
use super::screen::px;
use crate::viewer::DrivingCanvas;

pub(super) const DEFAULT_ZOOM: f32 = 1.0;
pub(crate) const MIN_ZOOM: f32 = 0.02;
pub(crate) const MAX_ZOOM: f32 = 10.0;
pub(super) const CAMERA_BOTTOM_PADDING_PX: f32 = 48.0;
const PIXELS_PER_SCROLL_STEP: f32 = 50.0;
const ZOOM_PER_SCROLL_STEP: f32 = 1.1;
const MOUSE_ROTATION_RADIANS_PER_PIXEL: f32 = 0.005;
const KEYBOARD_PAN_SPEED_PX_PER_SECOND: f32 = 500.0;
const MAX_GESTURE_TOUCHES: usize = 2;
const ZERO_ROTATION_RADIANS: f32 = 0.0;
const NO_ROTATION_INPUT: i8 = 0;
const UNCHANGED_ZOOM_SCALE: f32 = 1.0;
const VIEWPORT_CENTER_DIVISOR: f32 = 2.0;

#[derive(Clone, Copy, PartialEq)]
pub(super) enum MouseGestureKind {
    Pan,
    Rotate,
}

#[derive(Clone, Copy)]
pub(crate) struct MouseGesture {
    pub(super) kind: MouseGestureKind,
    pub(super) anchor: Vec2,
    pub(super) follows_ego: bool,
    pub(super) previous_cursor: Vec2,
}

#[derive(Clone, Copy)]
pub(crate) struct CameraState {
    pub(crate) center: Vec2,
    pub(crate) follow_offset: Vec2,
    pub(crate) gesture_active: bool,
    pub(crate) zoom: f32,
    pub(crate) rotation: f32,
    pub(crate) follow: bool,
    pub(crate) align_heading: bool,
    pub(crate) smooth: bool,
}

impl CameraState {
    pub(super) fn reset(&mut self, ego: State) {
        *self = Self {
            center: px(&ego),
            follow_offset: Vec2::ZERO,
            gesture_active: false,
            zoom: DEFAULT_ZOOM,
            rotation: ego.pose.yaw as f32 - std::f32::consts::FRAC_PI_2,
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
            follow_offset: Vec2::ZERO,
            gesture_active: false,
            zoom: DEFAULT_ZOOM,
            rotation: ZERO_ROTATION_RADIANS,
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
    scroll: Res<AccumulatedMouseScroll>,
    egui_input: Res<EguiWantsInput>,
    driving_canvas: Res<DrivingCanvas>,
    window: Single<&Window>,
    camera_transform: Single<&Transform, With<Camera2d>>,
    time: Res<Time>,
    mut gesture: Local<Option<MouseGesture>>,
) {
    if live.paused {
        *gesture = None;
        live.camera.gesture_active = false;
        return;
    }

    let cursor = window.cursor_position();
    let mut gesture_applied = false;
    if gesture.as_ref().is_some_and(|gesture| {
        let button = match gesture.kind {
            MouseGestureKind::Pan => MouseButton::Left,
            MouseGestureKind::Rotate => MouseButton::Right,
        };
        !mouse.pressed(button) && !mouse.just_released(button)
    }) {
        *gesture = None;
    }

    if gesture.is_some() || !egui_input.wants_any_pointer_input() {
        if cursor_over_driving_canvas(window.cursor_position(), driving_canvas.rect) {
            let scroll_steps = match scroll.unit {
                MouseScrollUnit::Line => scroll.delta.y,
                MouseScrollUnit::Pixel => scroll.delta.y / PIXELS_PER_SCROLL_STEP,
            };
            zoom_camera(&mut live.camera, ZOOM_PER_SCROLL_STEP.powf(scroll_steps));
        }

        apply_touch_controls(&mut live.camera, &touches);

        let displayed_center = camera_transform.translation.truncate();
        if gesture.is_none()
            && let Some(cursor) = cursor.filter(|cursor| cursor_over_driving_canvas(Some(*cursor), driving_canvas.rect))
        {
            let kind = if mouse.just_pressed(MouseButton::Left) {
                Some(MouseGestureKind::Pan)
            } else if mouse.just_pressed(MouseButton::Right) {
                Some(MouseGestureKind::Rotate)
            } else {
                None
            };
            if let Some(kind) = kind {
                let mut displayed_camera = live.camera;
                displayed_camera.center = displayed_center;
                let follows_ego = live.camera.follow;
                let ego = rendered_ego(&live);
                let anchor = match (kind, follows_ego) {
                    (MouseGestureKind::Rotate, true) => px(&ego),
                    (MouseGestureKind::Rotate, false) => displayed_center,
                    (MouseGestureKind::Pan, _) => world_under_cursor(displayed_camera, cursor, window.size()),
                };
                *gesture = Some(MouseGesture {
                    kind,
                    anchor: if kind == MouseGestureKind::Pan && follows_ego {
                        anchor - px(&ego)
                    } else {
                        anchor
                    },
                    follows_ego,
                    previous_cursor: cursor,
                });
            }
        }
        if let (Some(gesture), Some(cursor)) = (gesture.as_mut(), cursor) {
            let ego = rendered_ego(&live);
            apply_mouse_gesture(&mut live.camera, gesture, cursor, window.size(), ego);
            gesture_applied = true;
        }
        if gesture.as_ref().is_some_and(|gesture| {
            mouse.just_released(match gesture.kind {
                MouseGestureKind::Pan => MouseButton::Left,
                MouseGestureKind::Rotate => MouseButton::Right,
            })
        }) {
            *gesture = None;
        }
    }
    live.camera.gesture_active = gesture.is_some() || gesture_applied;

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
            Rot2::radians(camera.rotation) * pan.normalize() * KEYBOARD_PAN_SPEED_PX_PER_SECOND * time.delta_secs()
                / camera.zoom;
        live.camera.follow = false;
    }
    let rotation_input = keys.pressed(KeyCode::KeyE) as i8 - keys.pressed(KeyCode::KeyQ) as i8;
    if rotation_input != NO_ROTATION_INPUT {
        rotate_camera(&mut live.camera, rotation_input as f32 * time.delta_secs());
    }
}

fn apply_touch_controls(camera: &mut CameraState, touches: &Touches) {
    let touches: Vec<_> = touches.iter().take(MAX_GESTURE_TOUCHES).collect();
    match touches.as_slice() {
        [touch] if touch.delta() != Vec2::ZERO => pan_camera(camera, touch.delta()),
        [first, second] => {
            let previous = second.previous_position() - first.previous_position();
            let current = second.position() - first.position();
            zoom_camera(camera, pinch_scale(previous.length(), current.length()));
            rotate_camera(camera, twist_angle(previous, current));
        }
        _ => {}
    }
}

fn pan_camera(camera: &mut CameraState, screen_delta: Vec2) {
    camera.center -= screen_drag(screen_delta, camera.rotation, camera.zoom);
    camera.follow = false;
}

fn zoom_camera(camera: &mut CameraState, scale: f32) {
    camera.zoom = (camera.zoom * scale).clamp(MIN_ZOOM, MAX_ZOOM);
}

fn rotate_camera(camera: &mut CameraState, delta: f32) {
    if delta != ZERO_ROTATION_RADIANS {
        camera.rotation += delta;
        camera.align_heading = false;
    }
}

pub(super) fn apply_mouse_gesture(
    camera: &mut CameraState,
    gesture: &mut MouseGesture,
    cursor: Vec2,
    viewport: Vec2,
    ego: State,
) {
    if gesture.kind == MouseGestureKind::Rotate {
        rotate_camera(
            camera,
            (cursor.x - gesture.previous_cursor.x) * MOUSE_ROTATION_RADIANS_PER_PIXEL,
        );
    }
    if camera.follow && camera.align_heading {
        camera.rotation = ego.pose.yaw as f32 - std::f32::consts::FRAC_PI_2;
    }
    let ego_position = px(&ego);
    if gesture.kind == MouseGestureKind::Rotate && gesture.follows_ego {
        camera.center = ego_position;
        gesture.previous_cursor = cursor;
        return;
    }
    let anchor = gesture.anchor + if gesture.follows_ego { ego_position } else { Vec2::ZERO };
    let anchor_screen = if gesture.kind == MouseGestureKind::Rotate && !gesture.follows_ego {
        viewport / 2.0
    } else {
        cursor
    };
    let desired_center = anchor - screen_drag(anchor_screen - viewport / 2.0, camera.rotation, camera.zoom);
    if gesture.follows_ego {
        camera.center = ego_position;
        let offset = Rot2::radians(camera.rotation) * camera.follow_offset;
        let base = followed_camera_center(*camera, ego, viewport.y) - offset;
        camera.follow_offset = Rot2::radians(-camera.rotation) * (desired_center - base);
    } else {
        camera.center = desired_center;
    }
    gesture.previous_cursor = cursor;
}

pub(super) fn world_under_cursor(camera: CameraState, cursor: Vec2, viewport: Vec2) -> Vec2 {
    camera.center + screen_drag(cursor - viewport / 2.0, camera.rotation, camera.zoom)
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
        UNCHANGED_ZOOM_SCALE
    }
}

pub(super) fn twist_angle(previous: Vec2, current: Vec2) -> f32 {
    if previous.length_squared() > f32::EPSILON && current.length_squared() > f32::EPSILON {
        previous.angle_to(current)
    } else {
        ZERO_ROTATION_RADIANS
    }
}

pub(super) fn screen_drag(delta: Vec2, rotation: f32, zoom: f32) -> Vec2 {
    Rot2::radians(rotation) * Vec2::new(delta.x, -delta.y) / zoom
}

pub(super) fn smooth_angle(current: f32, target: f32, blend: f32) -> f32 {
    let delta = (target - current + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
    current + delta * blend
}

pub(super) fn followed_camera_center(camera: CameraState, ego: State, viewport_height: f32) -> Vec2 {
    let up = Rot2::radians(camera.rotation) * Vec2::Y;
    px(&ego)
        + up * (viewport_height / VIEWPORT_CENTER_DIVISOR - CAMERA_BOTTOM_PADDING_PX) / camera.zoom
        + Rot2::radians(camera.rotation) * camera.follow_offset
}
