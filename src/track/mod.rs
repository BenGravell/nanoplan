//! Geometry shared by tracks and the planners.

mod catalog;
mod circuit;
pub(crate) mod loader;
mod model;
mod path;
mod presets;
mod road;
mod track;

pub(crate) use catalog::TRACK_CATALOG;
pub(crate) use path::Path;
pub(crate) use presets::TRACK_PRESETS;
pub(crate) use road::Road;
pub(crate) use track::{GENERATED_TRACK_NAME, Track};

/// Shared sampling grid for rendered and physical road boundaries.
pub(crate) const ROAD_SAMPLE_STEP_M: f64 = 5.0;

#[cfg(test)]
mod tests {
    use super::loader::install_test_catalog;
    use super::model::is_simple;
    use super::track::TrackGeometry;
    use super::*;
    use crate::common::measure::dist;

    #[test]
    fn generated_track_is_simple_and_closed() {
        install_test_catalog();
        let track = Track::new(1);
        let length = track.lap_length().unwrap();
        assert!(dist(track.point(0.0), track.point(length)) < 1e-9);
        let TrackGeometry::Circuit(circuit) = &track.geometry;
        let points = circuit
            .samples
            .iter()
            .map(|sample| sample.point)
            .collect::<Vec<_>>();
        assert!(is_simple(&points));
    }

    #[test]
    fn downloaded_track_wraps_and_projects_progress_across_the_finish_line() {
        install_test_catalog();
        let track = Track::from_catalog(TRACK_PRESETS.len() + 1, 0);
        let length = track.lap_length().unwrap();
        assert!(dist(track.point(0.0), track.point(length)) < 1e-9);
        let progress = length + 10.0;
        assert!((track.project_progress(track.point(progress), progress) - progress).abs() < 1e-6);
        assert!(Track::new(0).lap_length().is_some());
    }

    #[test]
    fn every_catalog_track_has_finite_geometry_and_widths() {
        install_test_catalog();
        for index in 1..=TRACK_CATALOG.len() {
            let track = Track::from_catalog(TRACK_PRESETS.len() + index, 0);
            let TrackGeometry::Circuit(circuit) = &track.geometry;
            let points = circuit
                .samples
                .iter()
                .map(|sample| sample.point)
                .collect::<Vec<_>>();
            assert!(
                is_simple(&points),
                "{} intersects itself",
                TRACK_CATALOG[index - 1].name
            );
            let (point, yaw) = track.pose(100.0);
            let widths = track.widths(100.0);
            assert!(
                point
                    .into_iter()
                    .chain([yaw, widths.0, widths.1])
                    .all(f64::is_finite)
            );
            assert!(widths.0 > 0.0 && widths.1 > 0.0);
        }
    }

    #[test]
    fn every_preset_track_is_simple_closed_and_finite() {
        for index in 1..=TRACK_PRESETS.len() {
            let track = Track::from_catalog(index, 0);
            let length = track.lap_length().unwrap();
            assert!(dist(track.point(0.0), track.point(length)) < 1e-9);
            let TrackGeometry::Circuit(circuit) = &track.geometry;
            assert!(
                circuit.is_simple(),
                "{} intersects itself",
                TRACK_PRESETS[index - 1].name
            );
            for i in 0..circuit.samples.len() {
                let progress = length * i as f64 / circuit.samples.len() as f64;
                let (point, yaw) = track.pose(progress);
                let (right, left) = track.widths(progress);
                assert!(
                    point
                        .into_iter()
                        .chain([yaw, right, left])
                        .all(f64::is_finite)
                );
                assert!(right > 0.0 && left > 0.0);
            }
        }
    }
}
