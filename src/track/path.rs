//! Arc-length polyline lookup and Frenet projection.

use crate::common::{interp::lerp, types::position::Position};
use crate::simulation::State;

type Projection = (f64, f64, f64);

/// A polyline with arc-length lookup and Frenet projection.
pub(crate) struct Path {
    pts: Vec<Position>,
    s: Vec<f64>,
    actor_projections: std::cell::RefCell<Vec<(State, Projection)>>,
}

impl Path {
    pub(crate) fn new(pts: &[Position]) -> Self {
        assert!(pts.len() >= 2);
        let mut s = vec![0.0];
        for w in pts.windows(2) {
            s.push(s.last().unwrap() + w[0].distance(w[1]));
        }
        Self {
            pts: pts.to_vec(),
            s,
            actor_projections: Default::default(),
        }
    }

    pub(crate) fn length(&self) -> f64 {
        *self.s.last().unwrap()
    }

    pub(crate) fn pose_at(&self, s: f64) -> (Position, f64) {
        let s = s.clamp(0.0, self.length());
        let i = self.s.partition_point(|&x| x < s).clamp(1, self.pts.len() - 1);
        let (a, b) = (self.pts[i - 1], self.pts[i]);
        let u = (s - self.s[i - 1]) / (self.s[i] - self.s[i - 1]).max(1e-9);
        (lerp(a, b, u), (b.y - a.y).atan2(b.x - a.x))
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
        let projection = (s, d, self.pose_at(s).1);
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
        let mut best = (0.0, f64::INFINITY);
        for i in lo..hi.min(self.pts.len() - 1) {
            let (a, b) = (self.pts[i], self.pts[i + 1]);
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let len2 = (dx * dx + dy * dy).max(1e-12);
            let u = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
            let q = Position::new(a.x + dx * u, a.y + dy * u);
            let d = p.distance(q);
            if d < best.1.abs() {
                best = (
                    self.s[i] + len2.sqrt() * u,
                    d.copysign(dx * (p.y - q.y) - dy * (p.x - q.x)),
                );
            }
        }
        best
    }

    pub(crate) fn frenet_to_position(&self, s: f64, d: f64) -> Position {
        let (p, yaw) = self.pose_at(s);
        let left = Position::from_angle(yaw + std::f64::consts::FRAC_PI_2);
        Position::new(p.x + d * left.x, p.y + d * left.y)
    }
}
