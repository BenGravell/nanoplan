use bevy::prelude::*;
use nanoplan::{CAR_FOOTPRINT, Footprint, State};

pub(crate) const ACCENT: Color = Color::srgb(1.0, 0.25, 0.85);

use super::screen::{PX_PER_M, px};

pub(crate) fn draw_car(gizmos: &mut Gizmos, s: &State, color: Color) {
    draw_agent(gizmos, s, CAR_FOOTPRINT, color);
}

pub(crate) fn draw_agent(gizmos: &mut Gizmos, s: &State, footprint: Footprint, color: Color) {
    let size = Vec2::new(footprint.length as f32, footprint.width as f32);
    let iso = Isometry2d::new(px(s), Rot2::radians(s.yaw as f32));
    gizmos.rect_2d(iso, size * PX_PER_M, color);
    gizmos.line_2d(
        iso * Vec2::ZERO,
        iso * Vec2::new(size.x * PX_PER_M / 2.0, 0.0),
        color,
    );
}
