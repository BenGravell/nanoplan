use bevy::prelude::*;
use nanoplan::Track;

use super::super::screen::ppx;

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos,
    track: &Track,
    ego_x: f64,
    show_centerline: bool,
) {
    let xs = ((ego_x - 250.0) / 5.0).floor() as i64..=((ego_x + 750.0) / 5.0).ceil() as i64;
    let samples: Vec<_> = xs
        .map(|i| {
            let x = i as f64 * 5.0;
            let (position, yaw) = track.pose(x);
            (position, yaw, track.half_width(x))
        })
        .collect();

    for sign in [-1.0, 1.0] {
        gizmos.linestrip_2d(
            samples.iter().map(|&(position, yaw, width)| {
                ppx([
                    position[0] - sign * width * yaw.sin(),
                    position[1] + sign * width * yaw.cos(),
                ])
            }),
            Color::srgb(0.6, 0.6, 0.6),
        );
    }
    if show_centerline {
        gizmos.linestrip_2d(
            samples.iter().map(|&(position, _, _)| ppx(position)),
            Color::srgb(0.25, 0.5, 0.35),
        );
    }
    for &(position, yaw, width) in &samples {
        let offset = [width * yaw.sin(), -width * yaw.cos()];
        gizmos.line_2d(
            ppx([position[0] - offset[0], position[1] - offset[1]]),
            ppx([position[0] + offset[0], position[1] + offset[1]]),
            Color::srgba(0.6, 0.6, 0.6, 0.2),
        );
    }
}
