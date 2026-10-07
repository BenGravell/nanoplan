//! Small shared mechanics for sampling-tree planners.
//!
//! The planners still own their edge costs and planner-specific sampling
//! policy. This module holds the boring tree-shaped plumbing they had each
//! been carrying: parent-chain extraction, search-queue ordering,
//! and road-frame setup.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::common::types::FrenetPosition;
use crate::planning::{Context, PLANNING_HORIZON_S};
use crate::simulation::{Position, State};
use crate::track::Path;

pub(crate) struct RoadFrame<'a> {
    pub(crate) path: &'a Path,
    pub(crate) s0: f64,
    pub(crate) d0: f64,
    pub(crate) speed: f64,
    pub(crate) horizon_m: f64,
}

impl<'a> RoadFrame<'a> {
    pub(crate) fn new(ego: State, ctx: &'a Context) -> Self {
        let path = ctx.path();
        let FrenetPosition { s: s0, d: d0 } = path.project(ego.position());
        let speed = ego.speed.max(2.0);
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
    pub(crate) cost: f64,
    pub(crate) node: usize,
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

pub(crate) fn dist(a: Position, b: Position) -> f64 {
    a.distance(b)
}

/// Whichever of `a`, `b` has the larger magnitude, keeping its sign.
pub(crate) fn signed_max(a: f64, b: f64) -> f64 {
    if a.abs() >= b.abs() { a } else { b }
}

/// Walk parent pointers from a leaf back to `root`, returning root-exclusive
/// node ids in root-to-leaf order.
pub(crate) fn parent_chain(mut node: usize, root: usize, mut parent: impl FnMut(usize) -> Option<usize>) -> Vec<usize> {
    let mut chain = Vec::new();
    while node != root {
        chain.push(node);
        node = parent(node).expect("node has no parent before reaching root");
    }
    chain.reverse();
    chain
}

pub(crate) struct BestFirstResult {
    pub(crate) goal: usize,
    pub(crate) parent: Vec<usize>,
}

pub(crate) fn best_first(
    n_nodes: usize,
    start: usize,
    mut is_goal: impl FnMut(usize) -> bool,
    mut depth: impl FnMut(usize) -> usize,
    mut heuristic: impl FnMut(usize) -> f64,
    mut successors: impl FnMut(usize) -> Vec<(usize, f64)>,
    mut on_relax: impl FnMut(usize, usize),
) -> Option<BestFirstResult> {
    let mut dist = vec![f64::INFINITY; n_nodes];
    let mut parent = vec![usize::MAX; n_nodes];
    let mut heap = BinaryHeap::new();
    dist[start] = 0.0;
    heap.push(QueueEntry {
        cost: heuristic(start),
        node: start,
    });

    while let Some(QueueEntry { cost: priority, node }) = heap.pop() {
        if priority > dist[node] + heuristic(node) {
            continue;
        }
        if is_goal(node) {
            return Some(BestFirstResult { goal: node, parent });
        }
        let g = dist[node];
        for (succ, edge_cost) in successors(node) {
            if !edge_cost.is_finite() {
                continue;
            }
            let nd = g + edge_cost;
            if nd < dist[succ] {
                dist[succ] = nd;
                parent[succ] = node;
                on_relax(succ, node);
                heap.push(QueueEntry {
                    cost: nd + heuristic(succ),
                    node: succ,
                });
            }
        }
    }

    let goal = (0..n_nodes)
        .filter(|&node| node != start && dist[node].is_finite())
        .max_by(|&a, &b| depth(a).cmp(&depth(b)).then_with(|| dist[b].total_cmp(&dist[a])))?;
    Some(BestFirstResult { goal, parent })
}
