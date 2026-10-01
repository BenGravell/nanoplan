//! Finite road windows consumed by planners and simulation.

use super::path::ReferenceGeometry;
use crate::common::geometry::RoadPolygon;
use crate::common::geometry::barrier::{Barrier, road_side_barriers};
use crate::common::interp::lerp;
use crate::simulation::Position;

/// The finite planning window sampled from the active track.
#[derive(Debug, Clone)]
pub(crate) struct Road {
    polygon: RoadPolygon,
    segments: std::sync::Arc<std::sync::OnceLock<crate::common::geometry::segment_index::SegmentIndex>>,
    stations: std::sync::Arc<[f64]>,
    range: std::ops::Range<usize>,
    path: std::sync::Arc<std::sync::OnceLock<std::sync::Arc<super::path::PathGeometry>>>,
    reference_geometry: Option<std::sync::Arc<[ReferenceGeometry]>>,
    pub(crate) target_speed: f64,
    pub(crate) half_width: f64,
    barriers: std::sync::Arc<[Barrier]>,
    pub(crate) dt: f64,
    /// Anchor station and search radius for ego in a rolling road window.
    /// Ordinary finite roads do not restrict ego projection.
    pub(crate) ego_projection_window: Option<(f64, f64)>,
}

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone)]
pub(crate) struct RoadView {
    pub(crate) range: std::ops::Range<usize>,
    pub(crate) ego_projection_window: Option<(f64, f64)>,
}

impl Road {
    #[cfg(test)]
    pub(crate) fn new<P: Into<Position>>(centerline: Vec<P>, target_speed: f64, half_width: f64, dt: f64) -> Self {
        let polygon = RoadPolygon::uniform(centerline.into_iter().map(Into::into).collect(), half_width)
            .expect("road needs a finite positive width and at least two distinct stations");
        Self::from_polygon(polygon, target_speed, dt)
    }

    pub(crate) fn from_polygon(polygon: RoadPolygon, target_speed: f64, dt: f64) -> Self {
        let mut stations = Vec::with_capacity(polygon.centerline().len());
        stations.push(0.0);
        for pair in polygon.centerline().windows(2) {
            stations.push(stations.last().copied().unwrap() + pair[0].distance(pair[1]));
        }
        let half_width = polygon
            .right_widths()
            .iter()
            .chain(polygon.left_widths())
            .copied()
            .reduce(f64::min)
            .expect("road polygon needs widths");
        let barriers = road_side_barriers(&polygon);
        crate::planning::latency::geometry_build_work(stations.len() as u64);
        Self {
            range: 0..stations.len(),
            path: Default::default(),
            polygon,
            segments: Default::default(),
            stations: stations.into(),
            reference_geometry: None,
            target_speed,
            half_width,
            barriers: barriers.into(),
            dt,
            ego_projection_window: None,
        }
    }

    pub(crate) fn prepare(&self) {
        self.segments.get_or_init(|| {
            crate::common::geometry::segment_index::SegmentIndex::new(self.centerline(), self.polygon.is_closed(), 1e-9)
        });
        self.path.get_or_init(|| {
            super::Path::with_geometry(self.centerline(), self.reference_geometry()).prepared_geometry()
        });
    }

    pub(crate) fn view(&self) -> RoadView {
        RoadView {
            range: self.range.clone(),
            ego_projection_window: self.ego_projection_window,
        }
    }

    pub(crate) fn with_view(&self, view: RoadView) -> Self {
        assert!(view.range.len() >= 2 && view.range.end <= self.stations.len());
        self.prepare();
        let polygon = self.polygon.window(view.range.clone());
        let half_width = polygon
            .right_widths()
            .iter()
            .chain(polygon.left_widths())
            .copied()
            .reduce(f64::min)
            .unwrap();
        Self {
            polygon,
            half_width,
            range: view.range,
            ego_projection_window: view.ego_projection_window,
            ..self.clone()
        }
    }

    pub(crate) fn path(&self) -> super::Path {
        self.prepare();
        super::Path::from_geometry(self.path.get().unwrap().clone(), self.range.clone())
    }

    pub(crate) fn station_at_sample(&self, index: f64) -> f64 {
        let i = (index as usize).min(self.stations.len() - 2);
        lerp(
            self.stations[i],
            self.stations[i + 1],
            (index - i as f64).clamp(0.0, 1.0),
        )
    }

    pub(crate) fn closest_centerline_segment(&self, p: Position) -> Option<(usize, f64)> {
        self.prepare();
        let end = self.range.end - usize::from(!self.polygon.is_closed());
        self.segments
            .get()
            .unwrap()
            .nearest_in_range(p, self.range.start..end)
            .map(|(i, u)| (i - self.range.start, u))
    }

    pub(crate) fn set_reference_geometry(&mut self, geometry: Vec<ReferenceGeometry>) {
        assert_eq!(geometry.len(), self.stations.len());
        self.reference_geometry = Some(geometry.into());
        self.path = Default::default();
    }

    pub(crate) fn reference_geometry(&self) -> Option<&[ReferenceGeometry]> {
        self.reference_geometry.as_deref()
    }

    pub(crate) fn centerline(&self) -> &[Position] {
        self.polygon.centerline()
    }

    pub(crate) fn length(&self) -> f64 {
        self.stations[self.range.end - 1] - self.stations[self.range.start]
    }

    pub(crate) fn polygon(&self) -> &RoadPolygon {
        &self.polygon
    }

    pub(crate) fn barriers(&self) -> &[Barrier] {
        &self.barriers[2 * self.range.start..2 * (self.range.start + self.polygon.segment_count())]
    }

    /// Signed usable road bounds at centerline station `s`: right is
    /// negative and left is positive. Unlike [`Road::half_width`], this
    /// preserves local and asymmetric source widths.
    pub(crate) fn lateral_bounds_at(&self, s: f64) -> (f64, f64) {
        let stations = &self.stations[self.range.clone()];
        let s = s.clamp(0.0, self.length()) + stations[0];
        let i = stations
            .partition_point(|&station| station < s)
            .clamp(1, stations.len() - 1);
        let ds = stations[i] - stations[i - 1];
        let u = (s - stations[i - 1]) / ds.max(1e-9);
        let at = |values: &[f64]| lerp(values[i - 1], values[i], u);
        (-at(self.polygon.right_widths()), at(self.polygon.left_widths()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn barrier_projection_latency_clocks_bound_long_roads_and_clones() {
        use crate::planning::Latency;
        for segments in [256, 4096] {
            let road = Road::new((0..=segments).map(|i| [i as f64, 0.0]).collect(), 10.0, 3.5, 0.1);
            let lat = Latency::default();
            lat.time("build", || road.closest_centerline_segment([0.5, 1.0].into()));
            let build = lat.take()[0].clocks;
            assert!((2 * segments as u64..2 * segments as u64 + 64).contains(&build));
            let cloned = road.clone();
            lat.time("queries", || {
                for i in 0..128 {
                    let x = (segments - 1) as f64 * i as f64 / 128.0 + 0.25;
                    let (segment, u) = cloned.closest_centerline_segment([x, 1.0].into()).unwrap();
                    assert_eq!(segment as f64 + u, x);
                }
            });
            let clocks = lat.take()[0].clocks;
            assert!(
                (128..128 * 64).contains(&clocks),
                "{segments} segments: {clocks} clocks"
            );
        }
    }

    #[test]
    fn local_lateral_bounds_interpolate_each_side() {
        let polygon = RoadPolygon::new(
            vec![Position::new(0.0, 0.0), Position::new(10.0, 0.0)],
            vec![2.0, 4.0],
            vec![3.0, 7.0],
            false,
        )
        .unwrap();
        let road = Road::from_polygon(polygon, 10.0, 0.1);
        assert_eq!(road.lateral_bounds_at(5.0), (-3.0, 5.0));
    }
}
