//! Public generated/downloaded track selection and lap geometry.

use std::sync::Arc;

use super::catalog::loaded_catalog;
use super::circuit::Circuit;
use super::presets::{self, TRACK_PRESETS};
use crate::geometry::RoadPolygon;

pub(crate) const GENERATED_TRACK_NAME: &str = "Generated Circuit";

#[derive(Debug, Clone)]
pub(crate) struct Track {
    pub(super) geometry: TrackGeometry,
}

#[derive(Debug, Clone)]
pub(super) enum TrackGeometry {
    Circuit(Arc<Circuit>),
}

impl Track {
    pub(crate) fn new(seed: u64) -> Self {
        #[cfg(test)]
        super::loader::install_test_catalog();
        let circuit = Circuit::generated(
            loaded_catalog()
                .expect("track catalog not loaded at startup")
                .model
                .generate(seed)
                .expect("spectral track model could not generate a simple circuit"),
        );
        Self {
            geometry: TrackGeometry::Circuit(Arc::new(circuit)),
        }
    }

    pub(crate) fn from_catalog(index: usize, seed: u64) -> Self {
        if index == 0 {
            return Self::new(seed);
        }
        if index <= TRACK_PRESETS.len() {
            return Self {
                geometry: TrackGeometry::Circuit(Arc::new(Circuit::generated(presets::generate(
                    index - 1,
                )))),
            };
        }
        Self {
            geometry: TrackGeometry::Circuit(
                loaded_catalog()
                    .expect("track catalog not loaded at startup")
                    .circuits
                    .get(index - TRACK_PRESETS.len() - 1)
                    .expect("track catalog index out of bounds")
                    .clone(),
            ),
        }
    }

    pub(crate) fn point(&self, progress: f64) -> [f64; 2] {
        self.pose(progress).0
    }

    pub(crate) fn pose(&self, progress: f64) -> ([f64; 2], f64) {
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
    pub(crate) fn centerline(&self, from: f64, to: f64, step: f64) -> Vec<[f64; 2]> {
        let first = (from / step).floor() as i64;
        let last = (to / step).ceil() as i64;
        (first..=last)
            .map(|i| self.point(i as f64 * step))
            .collect()
    }

    pub(crate) fn road_polygon(
        &self,
        from: f64,
        to: f64,
        step: f64,
        closed: bool,
    ) -> Option<RoadPolygon> {
        let progress = if closed {
            let count = ((to - from) / step).ceil().max(2.0) as usize;
            (0..count)
                .map(|i| from + i as f64 * step)
                .collect::<Vec<_>>()
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

    pub(crate) fn project_progress(&self, point: [f64; 2], hint: f64) -> f64 {
        match &self.geometry {
            TrackGeometry::Circuit(circuit) => circuit.project(point, hint),
        }
    }
}
