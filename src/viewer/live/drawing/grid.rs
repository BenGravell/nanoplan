use bevy::prelude::*;

use super::super::camera::CameraState;
use super::super::screen::PX_PER_M;

pub(in crate::viewer::live) fn draw(gizmos: &mut Gizmos, camera: CameraState, window: &Window) {
    let extent = window.width().hypot(window.height()) / camera.zoom;
    let wide_grid = camera.zoom < 0.25;
    let spacing = if wide_grid { 50.0 } else { 10.0 } * PX_PER_M;
    let major_every = if wide_grid { 2 } else { 5 };
    let min_x = ((camera.center.x - extent) / spacing).floor() as i64;
    let max_x = ((camera.center.x + extent) / spacing).ceil() as i64;
    let min_y = ((camera.center.y - extent) / spacing).floor() as i64;
    let max_y = ((camera.center.y + extent) / spacing).ceil() as i64;
    for x in min_x..=max_x {
        let color = if x.rem_euclid(major_every) == 0 {
            Color::srgba(0.2, 0.48, 0.68, 0.11)
        } else {
            Color::srgba(0.2, 0.4, 0.55, 0.035)
        };
        let x = x as f32 * spacing;
        gizmos.line_2d(
            Vec2::new(x, camera.center.y - extent),
            Vec2::new(x, camera.center.y + extent),
            color,
        );
    }
    for y in min_y..=max_y {
        let color = if y.rem_euclid(major_every) == 0 {
            Color::srgba(0.2, 0.48, 0.68, 0.11)
        } else {
            Color::srgba(0.2, 0.4, 0.55, 0.035)
        };
        let y = y as f32 * spacing;
        gizmos.line_2d(
            Vec2::new(camera.center.x - extent, y),
            Vec2::new(camera.center.x + extent, y),
            color,
        );
    }
}
