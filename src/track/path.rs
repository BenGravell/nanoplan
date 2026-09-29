//! Arc-length polyline lookup and Frenet projection.

use crate::common::geometry::{menger_curvature, vertex_heading};
use crate::common::{
    differencing::forward_difference,
    interp::{lerp, lerp_angle},
    measure::dot,
    types::position::Position,
};
use crate::simulation::State;

type Projection = (f64, f64, f64);

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy)]
pub(crate) struct ReferenceGeometry {
    pub(crate) heading: f64,
    pub(crate) curvature: f64,
}

/// A polyline with arc-length lookup and Frenet projection.
pub(crate) struct Path {
    pts: Vec<Position>,
    s: Vec<f64>,
    geometry: Vec<ReferenceGeometry>,
    actor_projections: std::cell::RefCell<Vec<(State, Projection)>>,
}

impl Path {
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
        Self {
            pts: pts.to_vec(),
            s,
            geometry,
            actor_projections: Default::default(),
        }
    }

    pub(crate) fn length(&self) -> f64 {
        *self.s.last().unwrap()
    }

    fn segment(&self, s: f64) -> (usize, f64) {
        let s = s.clamp(0.0, self.length());
        let i = self.s.partition_point(|&x| x < s).clamp(1, self.pts.len() - 1);
        (i, forward_difference(self.s[i - 1], s, self.s[i] - self.s[i - 1]))
    }

    pub(crate) fn pose_at(&self, s: f64) -> (Position, f64) {
        let (i, u) = self.segment(s);
        let (a, b) = (self.pts[i - 1], self.pts[i]);
        (lerp(a, b, u), (b - a).angle())
    }

    pub(crate) fn heading_at(&self, s: f64) -> f64 {
        let (i, u) = self.segment(s);
        let (a, b) = (self.geometry[i - 1].heading, self.geometry[i].heading);
        lerp_angle(a, b, u)
    }

    pub(crate) fn curvature_at(&self, s: f64) -> f64 {
        let (i, u) = self.segment(s);
        let (a, b) = (self.geometry[i - 1].curvature, self.geometry[i].curvature);
        lerp(a, b, u)
    }

    pub(crate) fn sharpness_at(&self, s: f64) -> f64 {
        let (i, _) = self.segment(s);
        forward_difference(
            self.geometry[i - 1].curvature,
            self.geometry[i].curvature,
            self.s[i] - self.s[i - 1],
        )
    }

    pub(crate) fn project(&self, p: impl Into<Position>) -> (f64, f64) {
        self.project_range(p.into(), 0, self.pts.len() - 1)
    }

    /// Projection and track heading cached for unchanged actor states that
    /// are predicted repeatedly during one planner call.
    pub(crate) fn actor_projection(&self, state: State) -> Projection {
        if let Some((_, projection)) = self
            .actor_projections
            .borrow()
            .iter()
            .find(|(cached, _)| *cached == state)
        {
            return *projection;
        }
        let (s, d) = self.project(state.position());
        let (_, heading) = self.pose_at(s);
        let projection = (s, d, heading);
        self.actor_projections.borrow_mut().push((state, projection));
        projection
    }

    #[cfg(test)]
    pub(crate) fn cached_actor_count(&self) -> usize {
        self.actor_projections.borrow().len()
    }

    pub(crate) fn project_near(&self, p: impl Into<Position>, hint: f64, window: f64) -> (f64, f64) {
        let lo = self.s.partition_point(|&x| x < hint - window).saturating_sub(1);
        let hi = self.s.partition_point(|&x| x <= hint + window).max(lo + 1);
        self.project_range(p.into(), lo, hi)
    }

    fn project_range(&self, p: Position, lo: usize, hi: usize) -> (f64, f64) {
        let (mut best_s, mut best_d) = (0.0, f64::INFINITY);
        for i in lo..hi.min(self.pts.len() - 1) {
            let (a, b) = (self.pts[i], self.pts[i + 1]);
            let ab = b - a;
            let len2 = ab.norm_squared();
            let u = (dot((p - a).xy(), ab.xy()) / len2).clamp(0.0, 1.0);
            let q = lerp(a, b, u);
            let offset = p - q;
            let d = offset.norm();
            if d < best_d.abs() {
                best_s = self.s[i] + len2.sqrt() * u;
                best_d = d.copysign(ab.cross(offset));
            }
        }
        (best_s, best_d)
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
    fn projection_and_frenet_offset_use_the_same_side() {
        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(10.0, 0.0)]);
        let point = Position::new(3.0, 2.0);
        let (s, d) = path.project(point);
        assert!((s - 3.0).abs() < 1e-12);
        assert!((d - 2.0).abs() < 1e-12);
        assert!(path.frenet_to_position(s, d).distance(point) < 1e-12);
        assert!((path.project(Position::new(3.0, -2.0)).1 + 2.0).abs() < 1e-12);
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
