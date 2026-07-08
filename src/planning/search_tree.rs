//! Small shared mechanics for sampling-tree planners.
//!
//! The planners still own their sampling, steering, and edge costs. This
//! module only holds the boring tree-shaped plumbing they had each been
//! carrying: parent-chain extraction, search-queue ordering, diagnostics, and
//! conversion from sampled geometry back to simulator controls.

use std::cmp::Ordering;

use crate::planning::Diagnostics;
use crate::simulation::{Control, State, action_toward, step};
use crate::wrap_angle;

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

pub(crate) fn record_diagnostics(
    diag: &Diagnostics,
    nodes: impl IntoIterator<Item = ([f64; 2], Vec<[f64; 2]>)>,
) {
    for (point, trajectory) in nodes {
        diag.record_point(point);
        diag.record_trajectory(trajectory);
    }
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
