//! Public baked track selection and lap geometry.

use std::sync::{Arc, OnceLock};

use super::catalog::{self, PRESET_TRACKS};
use super::geometry::TrackGeometry;
use super::path::ReferenceGeometry;
use super::presets::TRACK_PRESETS;
use crate::simulation::Position;

#[derive(Debug, Clone)]
pub(crate) struct Track {
    pub(super) geometry: Arc<TrackGeometry>,
    prepared: Arc<OnceLock<super::prepared::PreparedRoads>>,
}

impl Track {
    pub(crate) fn from_catalog(index: usize) -> Self {
        if index < TRACK_PRESETS.len() {
            static PRESETS: [OnceLock<Arc<TrackGeometry>>; TRACK_PRESETS.len()] =
                [const { OnceLock::new() }; TRACK_PRESETS.len()];
            return Self {
                geometry: PRESETS[index]
                    .get_or_init(|| Arc::new(TrackGeometry::baked(PRESET_TRACKS[index])))
                    .clone(),
                prepared: Default::default(),
            };
        }
        Self {
            geometry: catalog::geometry(index - TRACK_PRESETS.len()).expect("selected track is invalid"),
            prepared: Default::default(),
        }
    }

    pub(crate) fn prepared(&self) -> &super::prepared::PreparedRoads {
        self.prepare(
            *crate::simulation::MAX_TERMINAL_SPEED_MPS,
            crate::planning::PLANNING_DT_S,
        )
    }

    pub(crate) fn prepare(&self, initial_speed: f64, dt: f64) -> &super::prepared::PreparedRoads {
        self.prepared
            .get_or_init(|| super::prepared::PreparedRoads::new(self, initial_speed, dt))
    }

    pub(crate) fn point(&self, progress: f64) -> Position {
        self.pose(progress).0
    }

    pub(crate) fn pose(&self, progress: f64) -> (Position, f64) {
        self.geometry.pose(progress)
    }

    pub(crate) fn widths(&self, progress: f64) -> (f64, f64) {
        self.geometry.widths(progress)
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

    fn road_stations(from: f64, to: f64, step: f64, closed: bool) -> Vec<f64> {
        if closed {
            let count = ((to - from) / step).ceil().max(2.0) as usize;
            (0..count).map(|i| from + i as f64 * step).collect::<Vec<_>>()
        } else {
            let first = (from / step).floor() as i64;
            let last = (to / step).ceil() as i64;
            (first..=last).map(|i| i as f64 * step).collect::<Vec<_>>()
        }
    }

    pub(crate) fn reference_geometry(&self, from: f64, to: f64, step: f64, closed: bool) -> Vec<ReferenceGeometry> {
        Self::road_stations(from, to, step, closed)
            .into_iter()
            .map(|s| ReferenceGeometry {
                heading: self.geometry.heading(s),
                curvature: self.geometry.curvature(s),
            })
            .collect()
    }

    pub(crate) fn lap_length(&self) -> Option<f64> {
        Some(self.geometry.length)
    }

    pub(crate) fn project_progress(&self, point: Position, hint: f64) -> f64 {
        self.geometry.project(point, hint)
    }
}
