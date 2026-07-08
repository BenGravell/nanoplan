//! Small shared mechanics for sampling-tree planners.
//!
//! The planners still own their edge costs and planner-specific sampling
//! policy. This module holds the boring tree-shaped plumbing they had each
//! been carrying: parent-chain extraction, search-queue ordering,
//! diagnostics, road-frame setup, and conversion from sampled geometry back
//! to simulator controls.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::planning::{Context, Diagnostics, PLANNING_HORIZON_S};
use crate::scenarios::Path;
use crate::simulation::{Control, State, action_toward, step};
use crate::wrap_angle;

pub(crate) struct RoadFrame {
    pub path: Path,
    pub s0: f64,
    pub d0: f64,
    pub speed: f64,
    pub horizon_m: f64,
}

impl RoadFrame {
    pub(crate) fn new(ego: State, ctx: &Context) -> Self {
        let path = Path::new(&ctx.road.centerline);
        let (s0, d0) = path.project([ego.x, ego.y]);
        let speed = ego.speed.clamp(2.0, ctx.road.target_speed.max(2.0));
        RoadFrame {
            path,
            s0,
            d0,
            speed,
            horizon_m: speed * PLANNING_HORIZON_S,
        }
    }
}

/// A best-first queue item where the lowest cost pops first.
pub(crate) struct QueueEntry {
    pub cost: f64,
    pub node: usize,
}

impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.node == other.node
    }
}

impl Eq for QueueEntry {}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.node.cmp(&self.node))
    }
}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub(crate) fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Whichever of `a`, `b` has the larger magnitude, keeping its sign.
pub(crate) fn signed_max(a: f64, b: f64) -> f64 {
    if a.abs() >= b.abs() { a } else { b }
}

/// Walk parent pointers from a leaf back to `root`, returning root-exclusive
/// node ids in root-to-leaf order.
pub(crate) fn parent_chain(
    mut node: usize,
    root: usize,
    mut parent: impl FnMut(usize) -> Option<usize>,
) -> Vec<usize> {
    let mut chain = Vec::new();
    while node != root {
        chain.push(node);
        node = parent(node).expect("node has no parent before reaching root");
    }
    chain.reverse();
    chain
}

pub(crate) struct BestFirstResult {
    pub goal: usize,
    pub parent: Vec<usize>,
}

pub(crate) fn best_first(
    n_nodes: usize,
    start: usize,
    mut is_goal: impl FnMut(usize) -> bool,
    mut successors: impl FnMut(usize) -> Vec<(usize, f64)>,
) -> Option<BestFirstResult> {
    let mut dist = vec![f64::INFINITY; n_nodes];
    let mut parent = vec![usize::MAX; n_nodes];
    let mut heap = BinaryHeap::new();
    dist[start] = 0.0;
    heap.push(QueueEntry {
        cost: 0.0,
        node: start,
    });

    while let Some(QueueEntry { cost: g, node }) = heap.pop() {
        if g > dist[node] {
            continue;
        }
        if is_goal(node) {
            return Some(BestFirstResult { goal: node, parent });
        }
        for (succ, edge_cost) in successors(node) {
            if !edge_cost.is_finite() {
                continue;
            }
            let nd = g + edge_cost;
            if nd < dist[succ] {
                dist[succ] = nd;
                parent[succ] = node;
                heap.push(QueueEntry {
                    cost: nd,
                    node: succ,
                });
            }
        }
    }
    None
}

pub(crate) fn record_diagnostics(
    diag: &Diagnostics,
    nodes: impl IntoIterator<Item = ([f64; 2], Vec<[f64; 2]>)>,
) {
    for (point, trajectory) in nodes {
        diag.record_point(point);
        diag.record_trajectory(trajectory);
    }
}

pub(crate) fn repeat_last_controls(controls: &[Control], horizon: usize) -> Vec<Control> {
    (0..horizon)
        .map(|t| controls[t.min(controls.len() - 1)])
        .collect()
}

pub(crate) fn brake_controls(ego: State, ctx: &Context, accel: f64) -> Vec<Control> {
    let mut x = ego;
    (0..ctx.horizon)
        .map(|_| {
            let u = action_toward(x, accel, 0.0, ctx.road.dt);
            x = step(x, u, ctx.road.dt);
            u
        })
        .collect()
}

// Pure-pursuit extraction for sampled tree geometry. The curvature gain is
// deliberately assertive because `action_toward` and `step` still clamp the
// request to the plant's curvature and curvature-rate limits.
const PATH_TRACK_LOOKAHEAD_TICKS: f64 = 8.0;
const PATH_TRACK_LOOKAHEAD_MIN_M: f64 = 3.0;
const PATH_TRACK_LOOKAHEAD_MAX_M: f64 = 10.0;
const PATH_TRACK_CURVATURE_GAIN: f64 = 10.0;

pub(crate) fn path_to_controls(ego: State, path: &Path, speed: f64, ctx: &Context) -> Vec<Control> {
    let total_len = path.length();
    let dt = ctx.road.dt;
    let lookahead = (speed * dt * PATH_TRACK_LOOKAHEAD_TICKS)
        .clamp(PATH_TRACK_LOOKAHEAD_MIN_M, PATH_TRACK_LOOKAHEAD_MAX_M);
    let mut x = ego;
    (0..ctx.horizon)
        .map(|i| {
            let s = (speed * dt * (i + 1) as f64 + lookahead).min(total_len);
            let (target, _) = path.pose_at(s);
            let dx = target[0] - x.x;
            let dy = target[1] - x.y;
            let local_y = -dx * x.yaw.sin() + dy * x.yaw.cos();
            let ld2 = (dx * dx + dy * dy).max(1e-6);
            let curvature = 2.0 * PATH_TRACK_CURVATURE_GAIN * local_y / ld2;
            let accel = (0.5 * (speed - x.speed)).clamp(-4.0, 2.0);
            let u = action_toward(x, accel, curvature, dt);
            x = step(x, u, dt);
            u
        })
        .collect()
}

/// Turn a sampled position trajectory (spaced `dt` apart, starting one tick
/// after the ego) into controls for the kinematic model.
pub(crate) fn xy_to_controls(ego: State, pts: &[[f64; 2]], dt: f64) -> Vec<Control> {
    let mut v = ego.speed;
    let mut yaw = ego.yaw;
    let mut prev = [ego.x, ego.y];
    let mut x = ego;
    pts.iter()
        .map(|&p| {
            let ds = dist(p, prev);
            let new_v = ds / dt;
            let new_yaw = if ds > 1e-6 {
                (p[1] - prev[1]).atan2(p[0] - prev[0])
            } else {
                yaw
            };
            let accel = (new_v - v) / dt;
            let curvature = if ds > 1e-6 {
                wrap_angle(new_yaw - yaw) / ds
            } else {
                0.0
            };
            let u = action_toward(x, accel, curvature, dt);
            x = step(x, u, dt);
            (v, yaw, prev) = (new_v, new_yaw, p);
            u
        })
        .collect()
}
