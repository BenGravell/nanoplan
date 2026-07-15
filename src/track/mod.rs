//! Geometry shared by tracks and the planners.

mod catalog;
mod circuit;
pub mod loader;
mod model;
mod path;
mod road;
mod track;

pub use catalog::TRACK_CATALOG;
pub use path::Path;
pub use road::Road;
pub use track::{GENERATED_TRACK_NAME, Track};

#[cfg(test)]
mod tests {
    use super::catalog::install_test_catalog;
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
        let track = Track::from_catalog(1, 0);
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
            let track = Track::from_catalog(index, 0);
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
}
