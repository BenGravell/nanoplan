//! Geometry shared by tracks and the planners.

mod catalog;
mod circuit;
mod path;
pub(crate) mod pregenerate;
#[cfg_attr(not(any(test, feature = "track-pregeneration")), allow(dead_code))]
mod presets;
mod road;
#[allow(clippy::module_inception)]
mod track;

pub(crate) use catalog::TRACK_CATALOG;
pub(crate) use path::Path;
pub(crate) use presets::TRACK_PRESETS;
pub(crate) use road::Road;
pub(crate) use track::Track;

/// Shared sampling grid for rendered and physical road boundaries.
pub(crate) const ROAD_SAMPLE_STEP_M: f64 = 1.0;

#[cfg(test)]
mod tests {
    use super::track::TrackGeometry;
    use super::*;
    use crate::geometry::distance::dist;

    #[test]
    fn baked_track_wraps_and_projects_progress_across_the_finish_line() {
        let track = Track::from_catalog(TRACK_PRESETS.len());
        let length = track.lap_length().unwrap();
        assert!(dist(track.point(0.0), track.point(length)) < 1e-9);
        let progress = length + 10.0;
        assert!((track.project_progress(track.point(progress), progress) - progress).abs() < 1e-6);
    }

    #[test]
    fn every_catalog_track_has_finite_geometry_and_widths() {
        for (index, info) in TRACK_CATALOG.iter().enumerate() {
            let track = Track::from_catalog(TRACK_PRESETS.len() + index);
            let TrackGeometry::Circuit(circuit) = &track.geometry;
            assert!(circuit.is_simple(), "{} intersects itself", info.name);
            let (point, yaw) = track.pose(100.0);
            let widths = track.widths(100.0);
            assert!(point.is_finite() && [yaw, widths.0, widths.1].into_iter().all(f64::is_finite));
            assert!(widths.0 > 0.0 && widths.1 > 0.0);
        }
    }

    #[test]
    fn every_preset_track_is_simple_closed_and_finite() {
        for (index, info) in TRACK_PRESETS.iter().enumerate() {
            let track = Track::from_catalog(index);
            let length = track.lap_length().unwrap();
            assert!(dist(track.point(0.0), track.point(length)) < 1e-9);
            let TrackGeometry::Circuit(circuit) = &track.geometry;
            assert!(circuit.is_simple(), "{} intersects itself", info.name);
            for i in 0..circuit.samples.len() {
                let progress = length * i as f64 / circuit.samples.len() as f64;
                let (point, yaw) = track.pose(progress);
                let (right, left) = track.widths(progress);
                assert!(point.is_finite() && [yaw, right, left].into_iter().all(f64::is_finite));
                assert!(right > 0.0 && left > 0.0);
            }
        }
    }
}
