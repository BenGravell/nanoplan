//! Arc-length polyline lookup and Frenet projection.

use crate::common::geometry::{menger_curvature, vertex_heading};
use crate::common::{
    differencing::forward_difference,
    interp::{lerp, lerp_angle},
    measure::dot,
    types::{FrenetPosition, Position},
};
use crate::simulation::State;

use crate::common::geometry::segment_index::SegmentIndex;

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy)]
pub(crate) struct ReferenceGeometry {
    pub(crate) heading: f64,
    pub(crate) curvature: f64,
}

#[derive(Debug)]
pub(crate) struct PathGeometry {
    pts: Vec<Position>,
    segments: std::sync::OnceLock<SegmentIndex>,
    s: Vec<f64>,
    geometry: Vec<ReferenceGeometry>,
}

/// A cheap view of prepared geometry with a per-plan actor cache.
pub(crate) struct Path {
    data: std::sync::Arc<PathGeometry>,
    range: std::ops::Range<usize>,
    actor_projections: std::cell::RefCell<Vec<(State, FrenetPosition)>>,
}

impl Path {
    #[cfg(test)]
    pub(crate) fn new(pts: &[Position]) -> Self {
        Self::with_geometry(pts, None)
    }

    pub(crate) fn with_geometry(pts: &[Position], geometry: Option<&[ReferenceGeometry]>) -> Self {
        assert!(pts.len() >= 2);
        let mut s = vec![0.0];
        for w in pts.windows(2) {
            let distance = w[0].distance(w[1]);
            let next = s.last().unwrap() + distance;
            assert!(
                distance > f64::EPSILON && next.is_finite() && next > *s.last().unwrap(),
                "path requires finite, resolvable sample spacing"
            );
            s.push(next);
        }
        let geometry = if let Some(geometry) = geometry {
            assert_eq!(geometry.len(), pts.len());
            geometry.to_vec()
        } else {
            (0..pts.len())
                .map(|i| {
                    let a = pts[i.saturating_sub(1)];
                    let b = pts[i];
                    let c = pts[(i + 1).min(pts.len() - 1)];
                    let heading = if i == 0 {
                        (c - b).angle()
                    } else if i + 1 == pts.len() {
                        (b - a).angle()
                    } else {
                        vertex_heading(a, b, c)
                    };
                    let curvature = if i == 0 || i + 1 == pts.len() {
                        0.0
                    } else {
                        menger_curvature(a, b, c)
                    };
                    ReferenceGeometry { heading, curvature }
                })
                .collect()
        };
        crate::planning::latency::geometry_build_work(pts.len() as u64);
        Self::from_geometry(
            std::sync::Arc::new(PathGeometry {
                pts: pts.to_vec(),
                segments: Default::default(),
                s,
                geometry,
            }),
            0..pts.len(),
        )
    }

    pub(crate) fn from_geometry(data: std::sync::Arc<PathGeometry>, range: std::ops::Range<usize>) -> Self {
        assert!(range.len() >= 2 && range.end <= data.pts.len());
        Self {
            data,
            range,
            actor_projections: Default::default(),
        }
    }

    pub(crate) fn prepared_geometry(&self) -> std::sync::Arc<PathGeometry> {
        self.data
            .segments
            .get_or_init(|| SegmentIndex::new(&self.data.pts, false, 0.0));
        self.data.clone()
    }

    fn pts(&self) -> &[Position] {
        &self.data.pts[self.range.clone()]
    }
    fn stations(&self) -> &[f64] {
        &self.data.s[self.range.clone()]
    }
    fn geometry(&self) -> &[ReferenceGeometry] {
        &self.data.geometry[self.range.clone()]
    }
    fn origin(&self) -> f64 {
        self.data.s[self.range.start]
    }

    pub(crate) fn length(&self) -> f64 {
        self.stations().last().unwrap() - self.origin()
    }

    fn segment(&self, s: f64) -> (usize, f64) {
        let s = s.clamp(0.0, self.length());
        let i = self
            .stations()
            .partition_point(|&x| x < s + self.origin())
            .clamp(1, self.pts().len() - 1);
        (
            i,
            forward_difference(
                self.stations()[i - 1] - self.origin(),
                s,
                self.stations()[i] - self.stations()[i - 1],
            ),
        )
    }

    pub(crate) fn pose_at(&self, s: f64) -> (Position, f64) {
        let (i, u) = self.segment(s);
        let (a, b) = (self.pts()[i - 1], self.pts()[i]);
        (lerp(a, b, u), (b - a).angle())
    }

    pub(crate) fn heading_at(&self, s: f64) -> f64 {
        let (i, u) = self.segment(s);
        let (a, b) = (self.geometry()[i - 1].heading, self.geometry()[i].heading);
        lerp_angle(a, b, u)
    }

    pub(crate) fn curvature_at(&self, s: f64) -> f64 {
        let (i, u) = self.segment(s);
        let (a, b) = (self.geometry()[i - 1].curvature, self.geometry()[i].curvature);
        lerp(a, b, u)
    }

    pub(crate) fn sharpness_at(&self, s: f64) -> f64 {
        let (i, _) = self.segment(s);
        forward_difference(
            self.geometry()[i - 1].curvature,
            self.geometry()[i].curvature,
            self.stations()[i] - self.stations()[i - 1],
        )
    }

    pub(crate) fn project(&self, p: impl Into<Position>) -> FrenetPosition {
        let p = p.into();
        let index = self
            .data
            .segments
            .get_or_init(|| SegmentIndex::new(&self.data.pts, false, 0.0));
        let (i, _) = index
            .nearest_in_range(p, self.range.start..self.range.end - 1)
            .expect("path has at least one segment");
        let i = i - self.range.start;
        self.project_range(p, i, i + 1)
    }

    /// Projection cached for unchanged actor states that
    /// are predicted repeatedly during one planner call.
    pub(crate) fn actor_projection(&self, state: State) -> FrenetPosition {
        if let Some((_, projection)) = self
            .actor_projections
            .borrow()
            .iter()
            .find(|(cached, _)| *cached == state)
        {
            return *projection;
        }
        let projection = self.project(state.position());
        self.actor_projections.borrow_mut().push((state, projection));
        projection
    }

    #[cfg(test)]
    pub(crate) fn cached_actor_count(&self) -> usize {
        self.actor_projections.borrow().len()
    }

    pub(crate) fn project_near(&self, p: impl Into<Position>, hint: f64, window: f64) -> FrenetPosition {
        let lo = self
            .stations()
            .partition_point(|&x| x < hint - window + self.origin())
            .saturating_sub(1);
        let hi = self
            .stations()
            .partition_point(|&x| x <= hint + window + self.origin())
            .max(lo + 1);
        self.project_range(p.into(), lo, hi)
    }

    fn project_range(&self, p: Position, lo: usize, hi: usize) -> FrenetPosition {
        let (mut best_s, mut best_d) = (0.0, f64::INFINITY);
        for i in lo..hi.min(self.pts().len() - 1) {
            crate::planning::latency::geometry_work(1);
            let (a, b) = (self.pts()[i], self.pts()[i + 1]);
            let ab = b - a;
            let len2 = ab.norm_squared();
            let u = (dot((p - a).xy(), ab.xy()) / len2).clamp(0.0, 1.0);
            let q = lerp(a, b, u);
            let offset = p - q;
            let d = offset.norm();
            if d < best_d.abs() {
                best_s = self.stations()[i] - self.origin() + len2.sqrt() * u;
                best_d = d.copysign(ab.cross(offset));
            }
        }
        FrenetPosition { s: best_s, d: best_d }
    }

    pub(crate) fn frenet_to_position(&self, s: f64, d: f64) -> Position {
        let (p, yaw) = self.pose_at(s);
        let left = Position::from_angle(yaw + std::f64::consts::FRAC_PI_2);
        p + left * d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexed_projection_matches_full_scan_on_tracks_and_repeated_laps() {
        use crate::track::{TRACK_CATALOG, TRACK_PRESETS, Track};
        for track_index in 0..TRACK_PRESETS.len() + TRACK_CATALOG.len() {
            let track = Track::from_catalog(track_index);
            let lap = track.lap_length().unwrap();
            let points = track.centerline(0.0, 2.0 * lap, 2.0);
            let path = Path::new(&points);
            for i in 0..64 {
                let (center, yaw) = track.pose(lap * i as f64 / 64.0);
                for offset in [-8.0, 0.0, 8.0] {
                    let p = center + Position::from_angle(yaw + std::f64::consts::FRAC_PI_2) * offset;
                    let expected = path.project_range(p, 0, points.len() - 1);
                    let actual = path.project(p);
                    assert!(
                        (actual.s - expected.s).abs() < 1e-8,
                        "track {track_index}: {actual:?} != {expected:?}"
                    );
                    assert!((actual.d - expected.d).abs() < 1e-8);
                }
            }
        }
        let path = Path::new(&[
            [0.0, 0.0].into(),
            [10.0, 0.0].into(),
            [10.0, 10.0].into(),
            [0.0, 10.0].into(),
            [0.0, 0.0].into(),
            [10.0, 0.0].into(),
        ]);
        assert_eq!(path.project([5.0, -1.0]), FrenetPosition { s: 5.0, d: -1.0 });
    }

    #[test]
    fn projection_latency_clocks_bound_long_road_queries() {
        use crate::planning::Latency;
        for segments in [256, 4096] {
            let points: Vec<_> = (0..=segments).map(|i| Position::new(i as f64, 0.0)).collect();
            let path = Path::new(&points);
            let lat = Latency::default();
            lat.time("build", || path.project([0.5, 1.0]));
            let build = lat.take()[0].clocks;
            assert!((segments as u64..segments as u64 + 64).contains(&build));
            lat.time("queries", || {
                for i in 0..128 {
                    let x = (segments - 1) as f64 * i as f64 / 128.0 + 0.25;
                    assert_eq!(path.project([x, 1.0]), FrenetPosition { s: x, d: 1.0 });
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
    fn projection_and_frenet_offset_use_the_same_side() {
        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(10.0, 0.0)]);
        let point = Position::new(3.0, 2.0);
        let FrenetPosition { s, d } = path.project(point);
        assert!((s - 3.0).abs() < 1e-12);
        assert!((d - 2.0).abs() < 1e-12);
        assert!(path.frenet_to_position(s, d).distance(point) < 1e-12);
        assert!((path.project(Position::new(3.0, -2.0)).d + 2.0).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "path requires finite, resolvable sample spacing")]
    fn rejects_samples_too_close_to_resolve() {
        Path::new(&[Position::default(), Position::new(f64::EPSILON / 2.0, 0.0)]);
    }

    #[test]
    fn supplied_geometry_interpolates_without_heading_wrap_spikes() {
        let points = [Position::new(0.0, 0.0), Position::new(1.0, 0.0)];
        let geometry = [
            ReferenceGeometry {
                heading: 3.0,
                curvature: 0.1,
            },
            ReferenceGeometry {
                heading: -3.0,
                curvature: 0.2,
            },
        ];
        let path = Path::with_geometry(&points, Some(&geometry));
        assert!((path.heading_at(0.5).abs() - std::f64::consts::PI).abs() < 1e-12);
        assert!((path.curvature_at(0.5) - 0.15).abs() < 1e-12);
        assert!((path.sharpness_at(0.5) - 0.1).abs() < 1e-12);
    }
}
