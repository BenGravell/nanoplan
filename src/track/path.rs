//! Arc-length polyline lookup and Frenet projection.

use crate::simulation::{Position, State};

type Projection = (f64, f64, f64);

/// A polyline with arc-length lookup and Frenet projection.
pub(crate) struct Path {
    pts: Vec<[f64; 2]>,
    s: Vec<f64>,
    actor_projections: std::cell::RefCell<Vec<(State, Projection)>>,
}

impl Path {
    pub(crate) fn new(pts: &[[f64; 2]]) -> Self {
        assert!(pts.len() >= 2);
        let mut s = vec![0.0];
        for w in pts.windows(2) {
            s.push(s.last().unwrap() + distance(w[0], w[1]));
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

    pub(crate) fn pose_at(&self, s: f64) -> ([f64; 2], f64) {
        let s = s.clamp(0.0, self.length());
        let i = self
            .s
            .partition_point(|&x| x < s)
            .clamp(1, self.pts.len() - 1);
        let (a, b) = (self.pts[i - 1], self.pts[i]);
        let u = (s - self.s[i - 1]) / (self.s[i] - self.s[i - 1]).max(1e-9);
        (
            [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u],
            (b[1] - a[1]).atan2(b[0] - a[0]),
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
        let projection = (s, d, self.pose_at(s).1);
        self.actor_projections
            .borrow_mut()
            .push((state, projection));
        projection
    }

    #[cfg(test)]
    pub(crate) fn cached_actor_count(&self) -> usize {
        self.actor_projections.borrow().len()
    }

    pub(crate) fn project_near(
        &self,
        p: impl Into<Position>,
        hint: f64,
        window: f64,
    ) -> (f64, f64) {
        let lo = self
            .s
            .partition_point(|&x| x < hint - window)
            .saturating_sub(1);
        let hi = self.s.partition_point(|&x| x <= hint + window).max(lo + 1);
        self.project_range(p.into(), lo, hi)
    }

    fn project_range(&self, p: Position, lo: usize, hi: usize) -> (f64, f64) {
        let mut best = (0.0, f64::INFINITY);
        for i in lo..hi.min(self.pts.len() - 1) {
            let (a, b) = (self.pts[i], self.pts[i + 1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len2 = (dx * dx + dy * dy).max(1e-12);
            let u = (((p.x - a[0]) * dx + (p.y - a[1]) * dy) / len2).clamp(0.0, 1.0);
            let q = [a[0] + dx * u, a[1] + dy * u];
            let d = p.distance(q.into());
            if d < best.1.abs() {
                best = (
                    self.s[i] + len2.sqrt() * u,
                    d.copysign(dx * (p.y - q[1]) - dy * (p.x - q[0])),
                );
            }
        }
        best
    }

    pub(crate) fn frenet_to_xy(&self, s: f64, d: f64) -> [f64; 2] {
        let (p, yaw) = self.pose_at(s);
        [p[0] - d * yaw.sin(), p[1] + d * yaw.cos()]
    }
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
