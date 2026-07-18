use crate::track::{Track, road_edges};
use bevy::prelude::*;

use super::super::screen::ppx;

const SAMPLE_STEP_M: f64 = 5.0;
const EDGE: Color = Color::srgb(0.6, 0.6, 0.6);
const CENTERLINE: Color = Color::srgb(0.25, 0.5, 0.35);
const SUBDUED_EDGE: Color = Color::srgba(0.6, 0.6, 0.6, 0.18);
const SUBDUED_CENTERLINE: Color = Color::srgba(0.25, 0.5, 0.35, 0.14);

type TrackSample = ([f64; 2], f64, f64);

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos,
    track: &Track,
    progress: f64,
    show_stations: bool,
    show_centerline: bool,
) {
    if let Some(length) = track.lap_length() {
        let samples: Vec<_> = (0..(length / SAMPLE_STEP_M).ceil() as usize)
            .map(|i| sample(track, i as f64 * SAMPLE_STEP_M))
            .collect();
        draw_lines(
            gizmos,
            &samples,
            true,
            show_centerline,
            SUBDUED_EDGE,
            SUBDUED_CENTERLINE,
        );
    }

    let samples: Vec<_> = (((progress - 250.0) / SAMPLE_STEP_M).floor() as i64
        ..=((progress + 750.0) / SAMPLE_STEP_M).ceil() as i64)
        .map(|i| sample(track, i as f64 * SAMPLE_STEP_M))
        .collect();

    draw_lines(gizmos, &samples, false, show_centerline, EDGE, CENTERLINE);

    if show_stations {
        if let Some(edges) = edges(&samples, false) {
            for (right, left) in edges {
                gizmos.line_2d(ppx(right), ppx(left), Color::srgba(0.6, 0.6, 0.6, 0.2));
            }
        }
    }
}

fn sample(track: &Track, progress: f64) -> TrackSample {
    let position = track.point(progress);
    let (right, left) = track.widths(progress);
    (position, right, left)
}

fn edges(samples: &[TrackSample], closed: bool) -> Option<Vec<([f64; 2], [f64; 2])>> {
    road_edges(
        &samples.iter().map(|sample| sample.0).collect::<Vec<_>>(),
        &samples.iter().map(|sample| sample.1).collect::<Vec<_>>(),
        &samples.iter().map(|sample| sample.2).collect::<Vec<_>>(),
        closed,
    )
}

fn draw_lines(
    gizmos: &mut Gizmos,
    samples: &[TrackSample],
    closed: bool,
    show_centerline: bool,
    edge: Color,
    centerline: Color,
) {
    let Some(edges) = edges(samples, closed) else {
        return;
    };
    for side in 0..2 {
        let mut line = edges
            .iter()
            .map(|edge| ppx(if side == 0 { edge.0 } else { edge.1 }))
            .collect::<Vec<_>>();
        if closed {
            line.push(line[0]);
        }
        gizmos.linestrip_2d(line, edge);
    }
    if show_centerline {
        let mut line = samples
            .iter()
            .map(|sample| ppx(sample.0))
            .collect::<Vec<_>>();
        if closed {
            line.push(line[0]);
        }
        gizmos.linestrip_2d(line, centerline);
    }
}
