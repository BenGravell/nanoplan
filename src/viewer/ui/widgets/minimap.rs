use std::sync::OnceLock;

use bevy_egui::egui;

use crate::simulation::Position;
use crate::track::{TRACK_CATALOG, TRACK_PRESETS, Track};
use crate::viewer::colors::{GREY_048, ORANGE};

const SAMPLE_STEP_M: f64 = 5.0;

struct MapTrack {
    points: Vec<Position>,
    bounds: (Position, Position),
    length: f64,
}

static TRACKS: OnceLock<Vec<MapTrack>> = OnceLock::new();

pub(in crate::viewer::ui) fn paint(
    ui: &egui::Ui,
    track_index: usize,
    rect: egui::Rect,
    track_color: egui::Color32,
    opponents: &[f64],
    ego: Option<f64>,
) {
    let track = track(track_index);
    let map_point = |point| map_point(track, rect, point);
    let points = track.points.iter().copied().map(map_point).collect();
    let scale = rect.width().min(rect.height());
    ui.painter().add(egui::Shape::closed_line(
        points,
        egui::Stroke::new((scale * 0.025).clamp(1.25, 4.0), track_color),
    ));

    let radius = (scale * 0.04).clamp(3.0, 7.0);
    for progress in opponents {
        ui.painter()
            .circle_filled(map_point(point_at(track, *progress)), radius, GREY_048);
    }
    if let Some(progress) = ego {
        ui.painter()
            .circle_filled(map_point(point_at(track, progress)), radius, ORANGE);
    }
}

pub(in crate::viewer::ui) fn lap_length(track_index: usize) -> f64 {
    track(track_index).length
}

fn track(index: usize) -> &'static MapTrack {
    &TRACKS.get_or_init(|| {
        (0..TRACK_PRESETS.len() + TRACK_CATALOG.len())
            .map(build_track)
            .collect()
    })[index]
}

fn build_track(index: usize) -> MapTrack {
    let track = Track::from_catalog(index);
    let length = track.lap_length().expect("all selectable tracks are closed circuits");
    let sample_count = (length / SAMPLE_STEP_M).ceil().max(3.0) as usize;
    let mut points = (0..sample_count)
        .map(|sample| track.point(length * sample as f64 / sample_count as f64))
        .collect::<Vec<_>>();
    orient_horizontally(&mut points);
    let mut min = Position::new(f64::INFINITY, f64::INFINITY);
    let mut max = Position::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in &points {
        min.x = min.x.min(point.x);
        min.y = min.y.min(point.y);
        max.x = max.x.max(point.x);
        max.y = max.y.max(point.y);
    }
    MapTrack {
        points,
        bounds: (min, max),
        length,
    }
}

fn orient_horizontally(points: &mut [Position]) {
    let center = points
        .iter()
        .copied()
        .fold(Position::default(), |sum, point| sum + point)
        * (1.0 / points.len() as f64);
    let (xx, yy, xy) = points.iter().fold((0.0, 0.0, 0.0), |(xx, yy, xy), point| {
        let x = point.x - center.x;
        let y = point.y - center.y;
        (xx + x * x, yy + y * y, xy + x * y)
    });
    let angle = 0.5 * (2.0 * xy).atan2(xx - yy);
    let (sin, cos) = angle.sin_cos();
    for point in points {
        let x = point.x - center.x;
        let y = point.y - center.y;
        *point = Position::new(x * cos + y * sin, -x * sin + y * cos);
    }
}

fn point_at(track: &MapTrack, progress: f64) -> Position {
    let station = progress.rem_euclid(track.length) / track.length * track.points.len() as f64;
    let a = station.floor() as usize % track.points.len();
    let b = (a + 1) % track.points.len();
    let u = station.fract();
    track.points[a] + (track.points[b] - track.points[a]) * u
}

fn map_point(track: &MapTrack, rect: egui::Rect, point: Position) -> egui::Pos2 {
    let (min, max) = track.bounds;
    let span = Position::new((max.x - min.x).max(1.0), (max.y - min.y).max(1.0));
    let scale = (rect.width() / span.x as f32).min(rect.height() / span.y as f32);
    let center = (min + max) * 0.5;
    egui::pos2(
        rect.center().x + (point.x - center.x) as f32 * scale,
        rect.center().y - (point.y - center.y) as f32 * scale,
    )
}

#[cfg(test)]
mod tests {
    #[test]
    fn cached_tracks_use_at_most_five_metre_centerline_steps() {
        for index in 0..crate::track::TRACK_PRESETS.len() + crate::track::TRACK_CATALOG.len() {
            let track = super::track(index);
            assert!(track.length / track.points.len() as f64 <= super::SAMPLE_STEP_M);
            assert_eq!(super::point_at(track, 0.0), super::point_at(track, track.length));
            let (variance_x, variance_y) = track.points.iter().fold((0.0, 0.0), |sum, point| {
                (sum.0 + point.x * point.x, sum.1 + point.y * point.y)
            });
            assert!(variance_x >= variance_y, "track {index} is not horizontally oriented");
        }
    }
}
