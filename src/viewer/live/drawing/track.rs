use crate::track::Track;
use bevy::prelude::*;

use super::super::screen::ppx;

const SAMPLE_STEP_M: f64 = 5.0;
const EDGE: Color = Color::srgb(0.6, 0.6, 0.6);
const CENTERLINE: Color = Color::srgb(0.25, 0.5, 0.35);
const SUBDUED_EDGE: Color = Color::srgba(0.6, 0.6, 0.6, 0.18);
const SUBDUED_CENTERLINE: Color = Color::srgba(0.25, 0.5, 0.35, 0.14);

type TrackSample = ([f64; 2], f64, f64, f64);

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos,
    track: &Track,
    progress: f64,
    show_centerline: bool,
) {
    if let Some(length) = track.lap_length() {
        let mut samples: Vec<_> = (0..(length / SAMPLE_STEP_M).ceil() as usize)
            .map(|i| sample(track, i as f64 * SAMPLE_STEP_M))
            .collect();
        samples.push(sample(track, length));
        draw_lines(
            gizmos,
            &samples,
            show_centerline,
            SUBDUED_EDGE,
            SUBDUED_CENTERLINE,
        );
    }

    let samples: Vec<_> = (((progress - 250.0) / SAMPLE_STEP_M).floor() as i64
        ..=((progress + 750.0) / SAMPLE_STEP_M).ceil() as i64)
        .map(|i| sample(track, i as f64 * SAMPLE_STEP_M))
        .collect();

    draw_lines(gizmos, &samples, show_centerline, EDGE, CENTERLINE);

    for &(position, yaw, right, left) in &samples {
        let normal = [-yaw.sin(), yaw.cos()];
        gizmos.line_2d(
            ppx([
                position[0] - right * normal[0],
                position[1] - right * normal[1],
            ]),
            ppx([
                position[0] + left * normal[0],
                position[1] + left * normal[1],
            ]),
            Color::srgba(0.6, 0.6, 0.6, 0.2),
        );
    }
}

fn sample(track: &Track, progress: f64) -> TrackSample {
    let (position, yaw) = track.pose(progress);
    let (right, left) = track.widths(progress);
    (position, yaw, right, left)
}

fn draw_lines(
    gizmos: &mut Gizmos,
    samples: &[TrackSample],
    show_centerline: bool,
    edge: Color,
    centerline: Color,
) {
    for (sign, left) in [(-1.0, false), (1.0, true)] {
        gizmos.linestrip_2d(
            samples.iter().map(|sample| {
                let (position, yaw) = (sample.0, sample.1);
                let width = if left { sample.3 } else { sample.2 };
                ppx([
                    position[0] - sign * width * yaw.sin(),
                    position[1] + sign * width * yaw.cos(),
                ])
            }),
            edge,
        );
    }
    if show_centerline {
        gizmos.linestrip_2d(
            samples.iter().map(|&(position, _, _, _)| ppx(position)),
            centerline,
        );
    }
}
