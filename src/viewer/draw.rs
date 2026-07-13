use bevy::prelude::*;
use nanoplan::{CAR_FOOTPRINT, Footprint, State};

pub(crate) const PX_PER_M: f32 = 6.0;
pub(crate) const ACCENT: Color = Color::srgb(1.0, 0.25, 0.85);

pub(crate) fn px(s: &State) -> Vec2 {
    Vec2::new(s.x as f32, s.y as f32) * PX_PER_M
}

pub(crate) fn ppx(p: [f64; 2]) -> Vec2 {
    Vec2::new(p[0] as f32, p[1] as f32) * PX_PER_M
}

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
