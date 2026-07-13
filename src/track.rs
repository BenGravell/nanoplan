//! Geometry shared by the endless procedural track and the planners.

use crate::Barrier;
use crate::simulation::{Position, State, road_side_barriers};

/// A polyline with arc-length lookup and Frenet projection.
pub struct Path {
    pts: Vec<[f64; 2]>,
    s: Vec<f64>,
    actor_projections: std::cell::RefCell<Vec<(State, (f64, f64, f64))>>,
}

impl Path {
    pub fn new(pts: &[[f64; 2]]) -> Self {
        assert!(pts.len() >= 2);
        let mut s = vec![0.0];
        for w in pts.windows(2) {
            s.push(s.last().unwrap() + dist(w[0], w[1]));
        }
        Self {
            pts: pts.to_vec(),
            s,
            actor_projections: Default::default(),
        }
    }

    pub fn length(&self) -> f64 {
        *self.s.last().unwrap()
    }

    pub fn pose_at(&self, s: f64) -> ([f64; 2], f64) {
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

    pub fn project(&self, p: impl Into<Position>) -> (f64, f64) {
        self.project_range(p.into(), 0, self.pts.len() - 1)
    }

    /// Projection and track heading cached for the handful of unchanged
    /// actor states repeatedly predicted during one planner call.
    pub(crate) fn actor_projection(&self, state: State) -> (f64, f64, f64) {
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

    pub fn project_near(&self, p: impl Into<Position>, hint: f64, window: f64) -> (f64, f64) {
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

    pub fn frenet_to_xy(&self, s: f64, d: f64) -> [f64; 2] {
        let (p, yaw) = self.pose_at(s);
        [p[0] - d * yaw.sin(), p[1] + d * yaw.cos()]
    }
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The finite planning window sampled from the endless track.
#[derive(Debug, Clone)]
pub struct Road {
    pub centerline: Vec<[f64; 2]>,
    pub target_speed: f64,
    pub half_width: f64,
    pub barriers: Vec<Barrier>,
    pub dt: f64,
}

impl Road {
    pub fn new(centerline: Vec<[f64; 2]>, target_speed: f64, half_width: f64, dt: f64) -> Self {
        let barriers = road_side_barriers(&centerline, half_width);
        Self {
            centerline,
            target_speed,
            half_width,
            barriers,
            dt,
        }
    }
}

/// A deterministic, unbounded, single-lane track with changing curvature and width.
#[derive(Debug, Clone, Copy)]
pub struct Track {
    phase: f64,
}

impl Track {
    pub fn new(seed: u64) -> Self {
        Self {
            phase: (seed as f64 * 1.618_033_988_75).rem_euclid(std::f64::consts::TAU),
        }
    }

    pub fn point(&self, x: f64) -> [f64; 2] {
        [
            x,
            24.0 * (x / 180.0 + self.phase).sin() + 7.0 * (x / 67.0 - self.phase).sin(),
        ]
    }

    pub fn pose(&self, x: f64) -> ([f64; 2], f64) {
        let dy = 24.0 / 180.0 * (x / 180.0 + self.phase).cos()
            + 7.0 / 67.0 * (x / 67.0 - self.phase).cos();
        (self.point(x), dy.atan())
    }

    pub fn half_width(&self, x: f64) -> f64 {
        4.5 + 1.8 * (x / 140.0 + 0.7 * self.phase).sin() + 0.7 * (x / 47.0 - self.phase).sin()
    }

    pub fn centerline(&self, from_x: f64, to_x: f64, step: f64) -> Vec<[f64; 2]> {
        let first = (from_x / step).floor() as i64;
        let last = (to_x / step).ceil() as i64;
        (first..=last)
            .map(|i| self.point(i as f64 * step))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn track_varies_but_has_no_end() {
        let track = Track::new(1);
        assert_ne!(track.point(0.0)[1], track.point(10_000.0)[1]);
        assert_ne!(track.half_width(0.0), track.half_width(500.0));
        let widths: Vec<_> = (0..1000).map(|x| track.half_width(x as f64)).collect();
        assert!(
            widths.iter().copied().reduce(f64::max).unwrap()
                - widths.iter().copied().reduce(f64::min).unwrap()
                > 4.0
        );
        assert!(track.centerline(1_000_000.0, 1_000_100.0, 5.0).len() > 10);
    }
}
