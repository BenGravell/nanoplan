//! Public baked track selection and lap geometry.

use std::sync::Arc;

use super::catalog::{self, PRESET_TRACKS};
use super::circuit::Circuit;
use super::presets::TRACK_PRESETS;
use crate::geometry::RoadPolygon;
use crate::simulation::Position;

#[derive(Debug, Clone)]
pub(crate) struct Track {
    pub(super) geometry: TrackGeometry,
}

#[derive(Debug, Clone)]
pub(super) enum TrackGeometry {
    Circuit(Arc<Circuit>),
}

impl Track {
    pub(crate) fn from_catalog(index: usize) -> Self {
        if index < TRACK_PRESETS.len() {
            return Self {
                geometry: TrackGeometry::Circuit(Arc::new(Circuit::baked(PRESET_TRACKS[index]))),
            };
        }
        Self {
            geometry: TrackGeometry::Circuit(
                catalog::circuit(index - TRACK_PRESETS.len()).expect("selected track is invalid"),
            ),
        }
    }

    pub(crate) fn point(&self, progress: f64) -> Position {
        self.pose(progress).0
    }

    pub(crate) fn pose(&self, progress: f64) -> (Position, f64) {
        match &self.geometry {
            TrackGeometry::Circuit(circuit) => circuit.pose(progress),
        }
    }

    pub(crate) fn widths(&self, progress: f64) -> (f64, f64) {
        match &self.geometry {
            TrackGeometry::Circuit(circuit) => circuit.widths(progress),
        }
    }

    pub(crate) fn half_width(&self, progress: f64) -> f64 {
        let (right, left) = self.widths(progress);
        right.min(left)
    }

    #[cfg(test)]
    pub(crate) fn centerline(&self, from: f64, to: f64, step: f64) -> Vec<Position> {
        let first = (from / step).floor() as i64;
        let last = (to / step).ceil() as i64;
        (first..=last).map(|i| self.point(i as f64 * step)).collect()
    }

    pub(crate) fn road_polygon(&self, from: f64, to: f64, step: f64, closed: bool) -> Option<RoadPolygon> {
        let progress = if closed {
            let count = ((to - from) / step).ceil().max(2.0) as usize;
            (0..count).map(|i| from + i as f64 * step).collect::<Vec<_>>()
        } else {
            let first = (from / step).floor() as i64;
            let last = (to / step).ceil() as i64;
            (first..=last).map(|i| i as f64 * step).collect::<Vec<_>>()
        };
        let centerline = progress.iter().map(|&s| self.point(s)).collect();
        let (right_widths, left_widths) = progress.iter().map(|&s| self.widths(s)).unzip();
        RoadPolygon::new(centerline, right_widths, left_widths, closed)
    }

    pub(crate) fn lap_length(&self) -> Option<f64> {
        match &self.geometry {
            TrackGeometry::Circuit(circuit) => Some(circuit.length),
        }
    }

    pub(crate) fn project_progress(&self, point: Position, hint: f64) -> f64 {
        match &self.geometry {
            TrackGeometry::Circuit(circuit) => circuit.project(point, hint),
        }
    }
}
