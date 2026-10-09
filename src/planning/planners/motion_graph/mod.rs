//! Shared time-layered motion graph for tree, lattice, and treetop planners.
//! Reuses the sampling envelopes and Cartesian transformations in
//! [`crate::planning::frenet`], and supplies initial guesses to
//! [`crate::planning::planners::treetop::TreetopPlanner`].

mod lattice;
mod nearest_zap;

use rstar::RTree;
use rstar::primitives::GeomWithData;

use crate::common::geometry::{EGO_FOOTPRINT, wrap_angle};
use crate::common::kinematics::{clamp_control, commanded_accel_for_net, commanded_accel_to_stop, zero_action_point};
use crate::common::polynomial::CubicPolynomial;
use crate::constraints::Constraints;
use crate::planning::controls::{repeat_last_controls, rollout_constrained, stop_controls};
use crate::planning::frenet::{Motion, frenet_boundary, lateral_targets, longitudinal_targets};
use crate::planning::planner_math;
use crate::planning::sampling::{Halton, QuasiMonteCarlo};
use crate::planning::take_warm;
use crate::planning::warm_start::shift_actions;
use crate::planning::{Context, PLANNING_TICKS as TICKS, Planner};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;

pub(crate) const STEER_TICKS: usize = 10;

/// Steering segments per trajectory — treetop's `NUM_STEER_SEGMENTS`; also
/// the number of tree layers past the root.
pub(crate) const SEGMENTS: usize = TICKS / STEER_TICKS;
const _: () = assert!(TICKS.is_multiple_of(STEER_TICKS));

// Warm samples use 20% of the draws when a previous solution is available;
// the rest explore the layer's reachable state space. Halton keeps the
// schedule deterministic.
const WARM_PROBA: f64 = 0.2;

/// Warm samples' perturbation half-widths around the previous solution's
/// Frenet state: ±2 m station/lateral position, ±30% of speed in lateral
/// velocity, and ±2 m/s station velocity.
const WARM_D_POS: f64 = 2.0;
const WARM_LATERAL_SPEED_FRACTION: f64 = 0.3;
const WARM_D_SPEED: f64 = 2.0;

/// The connection policy is the only planner-specific setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Connections {
    NearestZap,
    Lattice,
}

impl Connections {
    fn parents(self, index: &ZapIndex, target: State) -> Vec<usize> {
        match self {
            Self::NearestZap => nearest_zap::parents(index, target),
            Self::Lattice => lattice::parents(index, target),
        }
    }

    fn best_incoming(self, nodes: &[Node], incoming: &[Edge]) -> usize {
        match self {
            Self::NearestZap => nearest_zap::best_incoming(incoming),
            Self::Lattice => lattice::best_incoming(nodes, incoming),
        }
    }
}

/// One incoming edge, retaining its actual rollout rather than snapping to a sample.
struct Edge {
    parent: usize,
    controls: Vec<Control>,
    states: Vec<State>,
    eval: EdgeEval,
}

/// A sampled target's incoming edges and the best path reaching it.
/// The winning rollout endpoint is frozen before this node gains children.
/// This keeps extracted controls continuous despite steering endpoint error.
pub(crate) struct Node {
    pub(crate) state: State,
    incoming: Vec<Edge>,
    best: usize,
    cost_to_come: f64,
    collides: bool,
}

impl Node {
    fn edge(&self) -> &Edge {
        &self.incoming[self.best]
    }

    pub(crate) fn states(&self) -> &[State] {
        &self.edge().states
    }
}

/// Edge evaluation: the shared metric objective per rolled-out stage,
/// averaged over the segment. A hard violation is priced at the
/// shared depth-scaled penalty and flagged rather than propagated as an
/// infinity, because the fallback chains must be able to carry a cost.
struct EdgeEval {
    cost: f64,
    collides: bool,
}

/// A completed parent layer's coasting endpoints.
/// Spatial index with four coordinates instead of just position. Periodic yaw
/// copies preserve the wrapped-angle metric across the -π/π seam.
struct ZapIndex(RTree<GeomWithData<[f64; 4], usize>>);

impl ZapIndex {
    fn new(parents: impl Iterator<Item = (usize, State)>, ticks: usize, dt: f64) -> Self {
        Self(RTree::bulk_load(
            parents
                .flat_map(|(id, state)| {
                    let point = Self::point(zero_action_point(state, ticks, dt));
                    [-std::f64::consts::TAU, 0.0, std::f64::consts::TAU]
                        .map(|offset| GeomWithData::new([point[0], point[1], point[2] + offset, point[3]], id))
                })
                .collect(),
        ))
    }

    fn point(state: State) -> [f64; 4] {
        [
            state.position().x,
            state.position().y,
            wrap_angle(state.pose.yaw),
            state.speed,
        ]
    }
}

/// The layered motion graph. `layers[0]` holds only the root;
/// `layers[SEGMENTS]` holds the terminal nodes.
pub(crate) struct Graph {
    pub(crate) nodes: Vec<Node>,
    pub(crate) layers: [Vec<usize>; SEGMENTS + 1],
}

impl Graph {
    /// Grow a full-horizon graph: a zero-action fallback chain, a hot chain
    /// from warm-start actions (if any), then warm/cold samples distributed
    /// across every layer, including the terminal layer.
    pub(crate) fn grow(
        start: State,
        warm: Option<&[Control]>,
        samples: usize,
        path: &Path,
        ctx: &Context,
        connections: Connections,
    ) -> Graph {
        let mut tree = Graph {
            nodes: Vec::new(),
            layers: std::array::from_fn(|_| Vec::new()),
        };
        let g = Grower {
            path,
            ctx,
            initial_speed: start.speed,
            initial_station: ctx.project_ego(start).s,
        };
        let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, start.speed, g.initial_station);

        // Root node.
        tree.nodes.push(Node {
            state: start,
            incoming: Vec::new(),
            best: 0,
            cost_to_come: 0.0,
            collides: false,
        });
        tree.layers[0].push(0);

        // Zero-action fallback chain through every layer
        // (treetop `growZap`) — ignores collisions so a full parent chain
        // to the root always exists.
        let mut parent = 0usize;
        for layer in 1..=SEGMENTS {
            let from = tree.nodes[parent].state;
            let target = zero_action_point(from, STEER_TICKS, ctx.road.dt);
            let (us, xs, ee) = g.steer_edge(from, target, layer);
            parent = tree.add_node(parent, us, xs, layer, ee);
        }

        // Hot chain (treetop `growHot`): re-roll the warm-start actions
        // from the *current* start and split the result into one node per
        // segment, stopping at the first colliding segment.
        let warm_traj = warm.map(|actions| rollout_constrained(start, actions, ctx.road.dt));
        if let Some((wxs, wus)) = &warm_traj {
            let mut parent = 0usize;
            for layer in 1..=SEGMENTS {
                let lo = (layer - 1) * STEER_TICKS;
                let us = wus[lo..lo + STEER_TICKS].to_vec();
                let xs = wxs[lo..=lo + STEER_TICKS].to_vec();
                let ee = g.edge_eval(&xs, &us, layer);
                if ee.collides {
                    break;
                }
                parent = tree.add_node(parent, us, xs, layer, ee);
            }
        }

        // Layered sampling (treetop `growLayers`/`growSampleNode`), Halton
        // in place of the RNG. One global sample index keeps every draw —
        // category selector and state coordinates alike — deterministic.
        let per_layer = samples / SEGMENTS;
        let boundary = frenet_boundary(path, start, g.initial_station);
        let laterals = lateral_targets(
            path.project(start.position()).d,
            ctx.road.half_width,
            ctx.compute_budget,
        );
        let mut ix = 1usize;
        for layer in 1..=SEGMENTS {
            // A feasible edge cannot repair a collision earlier in its path.
            // Keep unsafe fallback nodes out of ordinary parent selection.
            if tree.layers[layer - 1].iter().all(|&id| tree.nodes[id].collides) {
                break;
            }
            // The previous layer is complete: cache its zero-action points
            // once and reuse the index for every sample in this layer.
            let parents = tree.zap_index(layer, ctx.road.dt);
            let layer_time = (layer * STEER_TICKS) as f64 * ctx.road.dt;
            for _ in 0..per_layer + usize::from(layer <= samples % SEGMENTS) {
                let selector = Halton::coordinate(ix, 4);
                let c: [f64; 4] = std::array::from_fn(|d| Halton::coordinate(ix, d));
                ix += 1;

                let target = if selector < WARM_PROBA
                    && let Some((wxs, _)) = &warm_traj
                {
                    let w = wxs[layer * STEER_TICKS];
                    let Some((mut lon, mut lat)) = frenet_boundary(path, w, path.project(w.position()).s) else {
                        continue;
                    };
                    lon[0] += (c[0] - 0.5) * 2.0 * WARM_D_POS;
                    lat[0] += (c[1] - 0.5) * 2.0 * WARM_D_POS;
                    lon[1] = (lon[1] + (c[3] - 0.5) * 2.0 * WARM_D_SPEED).max(0.0);
                    lat[1] += (c[2] - 0.5) * 2.0 * WARM_LATERAL_SPEED_FRACTION * w.speed;
                    let motion = Motion {
                        longitudinal: CubicPolynomial([lon[0], lon[1], 0.0, 0.0]),
                        lateral: CubicPolynomial([lat[0], lat[1], 0.0, 0.0]),
                    };
                    let Some((target, _)) = motion.at(path, 0.0) else {
                        continue;
                    };
                    target
                } else {
                    let Some((lon, lat)) = boundary else { continue };
                    let (d, dv) = laterals[(c[1] * laterals.len() as f64) as usize];
                    let lateral = CubicPolynomial::from_boundary(lat[0], lat[1], d, dv, layer_time);
                    let targets = longitudinal_targets(
                        start.speed,
                        ctx.road.dt,
                        ctx.compute_budget,
                        layer_time,
                        (path, lon[0], &lateral),
                    );
                    let (distance, speed) = targets[(c[0] * targets.len() as f64) as usize];
                    let motion = Motion {
                        longitudinal: CubicPolynomial::from_boundary(
                            lon[0],
                            lon[1],
                            lon[0] + distance,
                            speed,
                            layer_time,
                        ),
                        lateral,
                    };
                    let Some((target, _)) = motion.at(path, layer_time) else {
                        continue;
                    };
                    target
                };

                // Sampled state itself in collision → discard (treetop
                // checks its obstacles here; the metric objective's hard reject
                // is the equivalent).
                let (_, sample) = planner_math::state_sample(path, &target, layer_time, None);
                if !ctx.time("cost", || constraints.point_cost(&sample)).is_finite() {
                    continue;
                }

                let incoming = connections
                    .parents(&parents, target)
                    .into_iter()
                    .filter_map(|parent| {
                        let from = tree.nodes[parent].state;
                        let (controls, states, eval) = g.steer_edge(from, target, layer);
                        (!eval.collides).then_some(Edge {
                            parent,
                            controls,
                            states,
                            eval,
                        })
                    })
                    .collect::<Vec<_>>();
                if !incoming.is_empty() {
                    let best = connections.best_incoming(&tree.nodes, &incoming);
                    tree.add_incoming(incoming, layer, best);
                }
            }
        }

        tree
    }

    fn add_node(
        &mut self,
        parent: usize,
        controls: Vec<Control>,
        states: Vec<State>,
        layer: usize,
        ee: EdgeEval,
    ) -> usize {
        self.add_incoming(
            vec![Edge {
                parent,
                controls,
                states,
                eval: ee,
            }],
            layer,
            0,
        )
    }

    fn add_incoming(&mut self, incoming: Vec<Edge>, layer: usize, best: usize) -> usize {
        let edge = &incoming[best];
        let parent = &self.nodes[edge.parent];
        self.nodes.push(Node {
            state: *edge.states.last().unwrap(),
            cost_to_come: parent.cost_to_come + edge.eval.cost,
            collides: parent.collides || edge.eval.collides,
            incoming,
            best,
        });
        let id = self.nodes.len() - 1;
        self.layers[layer].push(id);
        id
    }

    fn zap_index(&self, layer: usize, dt: f64) -> ZapIndex {
        ZapIndex::new(
            self.layers[layer - 1]
                .iter()
                .copied()
                .filter(|&id| !self.nodes[id].collides)
                .map(|id| (id, self.nodes[id].state)),
            STEER_TICKS,
            dt,
        )
    }

    /// The best `k` full-length paths, preferring feasible paths, then progress cost.
    pub(crate) fn path_candidates(&self, k: usize) -> Vec<Vec<usize>> {
        let mut terminal_nodes = self.layers[SEGMENTS].clone();
        terminal_nodes.sort_by(|&a, &b| {
            let (na, nb) = (&self.nodes[a], &self.nodes[b]);
            na.collides
                .cmp(&nb.collides)
                .then(na.cost_to_come.total_cmp(&nb.cost_to_come))
        });
        terminal_nodes.truncate(k);
        terminal_nodes.iter().map(|&n| self.extract_path(n)).collect()
    }

    /// Walk parent pointers from a terminal node back to the root (treetop
    /// `extractPath`).
    fn extract_path(&self, mut node: usize) -> Vec<usize> {
        let mut path = Vec::new();
        while node != 0 {
            path.push(node);
            node = self.nodes[node].edge().parent;
        }
        path.reverse();
        path
    }

    /// Concatenate a path's edge actions into one full-horizon action
    /// sequence ([`TICKS`] controls) — treetop's
    /// `convertPathToActionSequence`, the tree→iLQR hand-off.
    pub(crate) fn actions_of(&self, path: &[usize]) -> Vec<Control> {
        let mut actions = Vec::with_capacity(TICKS);
        for &n in path {
            actions.extend_from_slice(&self.nodes[n].edge().controls);
        }
        actions
    }

    pub(crate) fn record_diagnostics(&self, diag: &crate::planning::Diagnostics) {
        for (layer, nodes) in self.layers.iter().enumerate().skip(1) {
            for &index in nodes {
                let node = &self.nodes[index];
                for edge in &node.incoming {
                    diag.record_point(edge.states.last().unwrap().position());
                    diag.record_timed_trajectory(
                        edge.states.iter().map(Into::into).collect(),
                        (0..edge.states.len())
                            .map(|tick| ((layer - 1) * STEER_TICKS + tick) as f64 * crate::planning::PLANNING_DT_S)
                            .collect(),
                    );
                }
            }
        }
    }
}

/// The per-grow context bundle: steering + edge pricing.
struct Grower<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
    initial_speed: f64,
    initial_station: f64,
}

impl Grower<'_, '_> {
    /// Steer from `from` toward `target` for [`STEER_TICKS`] ticks under
    /// the actuation limits, priced as the edge landing in `layer`.
    fn steer_edge(&self, from: State, target: State, layer: usize) -> (Vec<Control>, Vec<State>, EdgeEval) {
        let actions = steer_actions(self.path, &from, &target, self.ctx.road.dt);
        let invalid = actions.is_none();
        let actions = actions.unwrap_or_else(|| vec![Control::default(); STEER_TICKS]);
        let (xs, us) = rollout_constrained(from, &actions, self.ctx.road.dt);
        let mut ee = self.edge_eval(&xs, &us, layer);
        ee.collides |= invalid;
        (us, xs, ee)
    }

    /// Price one edge with the progress objective. `layer` fixes the
    /// absolute time of each stage, so actors are priced where they'll be.
    fn edge_eval(&self, xs: &[State], us: &[Control], layer: usize) -> EdgeEval {
        self.ctx.work(1);
        let dt = self.ctx.road.dt;
        let t0 = (layer - 1) as f64 * STEER_TICKS as f64 * dt;
        let mut total = 0.0;
        let mut collides = false;

        let constraints = Constraints::new(
            self.ctx.road.half_width,
            self.ctx.actors,
            self.path,
            self.initial_speed,
            self.initial_station,
        );
        for i in 0..us.len() {
            let x = &xs[i + 1];
            let (_, sample) = planner_math::state_sample(self.path, x, t0 + (i + 1) as f64 * dt, None);
            let mut sample = sample.with_control(us[i], xs[i].speed);
            sample.road_bounds = Some(self.ctx.road.lateral_bounds_at(sample.station));
            let shared = self.ctx.time("cost", || {
                if constraints.is_transition_violated(xs[i], *x, EGO_FOOTPRINT, self.ctx.road, &sample) {
                    f64::INFINITY
                } else {
                    constraints.point_cost(&sample)
                }
            });
            if shared.is_finite() {
                total += shared;
            } else {
                collides = true;
                total += constraints.violation_penalty(&sample);
            }
        }
        EdgeEval {
            cost: total / us.len().max(1) as f64,
            collides,
        }
    }
}

// ---- The steering function (treetop `steer.h`) --------------------------

/// Fit cubic station/lateral segments, transform their derivatives into
/// Cartesian commands, and realize the first segment under actuation limits.
fn steer_actions(path: &Path, start: &State, target: &State, dt: f64) -> Option<Vec<Control>> {
    let duration = STEER_TICKS as f64 * dt;
    let (lon, lat) = frenet_boundary(path, *start, path.project(start.position()).s)?;
    let (end_lon, end_lat) = frenet_boundary(path, *target, path.project(target.position()).s)?;
    let motion = Motion {
        longitudinal: CubicPolynomial::from_boundary(lon[0], lon[1], end_lon[0], end_lon[1], duration),
        lateral: CubicPolynomial::from_boundary(lat[0], lat[1], end_lat[0], end_lat[1], duration),
    };
    let mut state = *start;
    (0..STEER_TICKS)
        .map(|tick| {
            let (_, mut control) = motion.at(path, (tick as f64 + 0.5) * dt)?;
            control.acceleration = commanded_accel_for_net(control.acceleration, state.speed)
                .max(commanded_accel_to_stop(state.speed, dt));
            control = clamp_control(control, state.speed);
            state = world_step(state, control, dt);
            Some(control)
        })
        .collect()
}

/// The shared tree/lattice planner: grow, take the best path candidate, drive
/// it — no optimization pass. Warm-starts from its own previous plan the
/// same way the treetop planner feeds its optimized solution back in, so
/// consecutive replans refine one detour instead of rediscovering a
/// different one each tick.
pub(crate) struct GraphPlanner {
    connections: Connections,
    prev: Option<Vec<Control>>,
    expected_next: State,
}

/// Both modes sample the same targets. Lattice spends up to six edge
/// evaluations per target; tree spends one.
const SAMPLES: usize = 150;

impl GraphPlanner {
    pub(crate) fn new(connections: Connections) -> Self {
        Self {
            connections,
            prev: None,
            expected_next: State::default(),
        }
    }
}

impl Planner for GraphPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || ctx.path());
        let warm = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego).map(shift_actions)
        });

        // Retain the tree sampling allowance in both connection modes.
        let samples = ctx.compute_budget.scale(SAMPLES, 10);
        let tree = ctx.time("optimize", || {
            Graph::grow(ego, warm.as_deref(), samples, path, ctx, self.connections)
        });

        if let Some(diag) = ctx.diagnostics {
            tree.record_diagnostics(diag);
        }

        let controls = ctx.time("extract", || {
            let best = &tree.path_candidates(1)[0];
            if tree.nodes[*best.last().unwrap()].collides {
                // Keep the full fallback chain for treetop. For driving, use
                // a feasible prefix and brake beyond the verified portion.
                for layer in tree.layers.iter().skip(1).rev() {
                    if let Some(&node) = layer
                        .iter()
                        .filter(|&&id| !tree.nodes[id].collides)
                        .min_by(|&&a, &&b| tree.nodes[a].cost_to_come.total_cmp(&tree.nodes[b].cost_to_come))
                    {
                        let chain = tree.extract_path(node);
                        let mut controls = tree.actions_of(&chain);
                        controls.extend(stop_controls(tree.nodes[node].state, ctx, TICKS - controls.len()));
                        return controls;
                    }
                }
                stop_controls(ego, ctx, TICKS)
            } else {
                tree.actions_of(best)
            }
        });
        let out = repeat_last_controls(&controls, ctx.horizon);
        self.expected_next = world_step(ego, out[0], ctx.road.dt);
        self.prev = Some(controls);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connection_modes_keep_single_or_multiple_parents_and_continuous_paths() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let warm = vec![Control::default(); TICKS];
        for connections in [Connections::NearestZap, Connections::Lattice] {
            for warm in [None, Some(warm.as_slice())] {
                let graph = Graph::grow(ego, warm, SAMPLES, ctx.path(), &ctx, connections);
                let max_parents = graph.nodes.iter().map(|n| n.incoming.len()).max().unwrap();
                match connections {
                    Connections::NearestZap => assert_eq!(max_parents, 1),
                    Connections::Lattice => assert!(max_parents > 1 && max_parents <= lattice::MAX_PARENTS),
                }
                for node in graph.nodes.iter().skip(1) {
                    for edge in &node.incoming {
                        assert_eq!(edge.states[0], graph.nodes[edge.parent].state);
                        assert!(node.cost_to_come <= graph.nodes[edge.parent].cost_to_come + edge.eval.cost + 1e-10);
                    }
                }
                for chain in graph.path_candidates(3) {
                    let actions = graph.actions_of(&chain);
                    let (states, _) = rollout_constrained(ego, &actions, ctx.road.dt);
                    for (i, &id) in chain.iter().enumerate() {
                        assert_eq!(states[(i + 1) * STEER_TICKS], graph.nodes[id].state);
                    }
                }
            }
        }
    }

    #[test]
    fn steer_reaches_a_straight_ahead_target() {
        // Resistance is compensated; discrete integration still introduces endpoint error.
        let from = State {
            speed: 10.0,
            ..Default::default()
        };
        let dt = 0.1;
        let target = zero_action_point(from, STEER_TICKS, dt);
        let path = Path::new(&[[-20.0, 0.0].into(), [400.0, 0.0].into()]);
        let actions = steer_actions(&path, &from, &target, dt).unwrap();
        let (xs, _) = rollout_constrained(from, &actions, dt);
        let end = xs.last().unwrap();
        assert!(
            (end.position().x - target.position().x).abs() < 0.1,
            "x {} vs {}",
            end.position().x,
            target.position().x
        );
        assert!(end.position().y.abs() < 0.01);
        assert!(
            (end.speed - target.speed).abs() < 0.2,
            "speed {} vs {}",
            end.speed,
            target.speed
        );
    }

    #[test]
    fn steer_reaches_a_lateral_offset_target() {
        // 0.8 m of lateral over 1 s: the lateral acceleration stays inside
        // MAX_ABS_LAT_ACCEL so the projection
        // doesn't bite (a 2 m offset would demand 12 m/s² and get clamped
        // into an undershoot — that infeasible case is exactly what the
        // constrained rollout exists to prevent)
        let from = State {
            speed: 10.0,
            ..Default::default()
        };
        let dt = 0.1;
        let target = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(10.0, 0.8), 0.0),
            10.0,
        );
        let path = Path::new(&[[-20.0, 0.0].into(), [400.0, 0.0].into()]);
        let actions = steer_actions(&path, &from, &target, dt).unwrap();
        let (xs, _) = rollout_constrained(from, &actions, dt);
        let end = xs.last().unwrap();
        // the constrained rollout won't hit it exactly, but must get close
        assert!((end.position().x - 10.0).abs() < 1.0, "x {}", end.position().x);
        assert!((end.position().y - 0.8).abs() < 0.3, "y {}", end.position().y);
    }

    #[test]
    fn steering_follows_a_constant_frenet_offset_on_a_bend() {
        let points: Vec<_> = (0..1800)
            .map(|i| {
                let angle = i as f64 * 0.002;
                crate::simulation::Position::new(40.0 * angle.cos(), 40.0 * angle.sin())
            })
            .collect();
        let path = Path::new(&points);
        let motion = Motion {
            longitudinal: CubicPolynomial([10.0, 8.0, 0.0, 0.0]),
            lateral: CubicPolynomial([2.0, 0.0, 0.0, 0.0]),
        };
        let start = motion.at(&path, 0.0).unwrap().0;
        let goal = motion.at(&path, 1.0).unwrap().0;
        let actions = steer_actions(&path, &start, &goal, 0.1).unwrap();
        for action in &actions {
            // Projection uses road chords, so allow their finite resolution.
            assert!(
                (action.curvature - 1.0 / 38.0).abs() < 1e-4,
                "curvature {}",
                action.curvature
            );
        }
        let (states, _) = rollout_constrained(start, &actions, 0.1);
        assert!(states.last().unwrap().position().distance(goal.position()) < 0.1);
        // Invalid Frenet boundaries must reject steering, leaving fallback to the tree.
        let reverse = State { speed: -1.0, ..start };
        assert!(steer_actions(&path, &reverse, &goal, 0.1).is_none());
    }

    #[test]
    fn tree_always_offers_a_full_length_path() {
        // boxed in by actors, the zap fallback still yields a full path
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let actors: Vec<State> = (0..5)
            .map(|i| {
                State::new(
                    crate::simulation::Pose::new(
                        crate::simulation::Position::new(10.0 + 5.0 * i as f64, -2.0 + i as f64),
                        0.0,
                    ),
                    0.0,
                )
            })
            .collect();
        let ctx = crate::planning::test_ctx(&road, &actors);
        let path = Path::new(road.centerline());
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        for samples in [0, 1, 90] {
            let tree = Graph::grow(ego, None, samples, &path, &ctx, Connections::NearestZap);
            let cands = tree.path_candidates(2);
            assert!(!cands.is_empty());
            for cand in &cands {
                assert_eq!(cand.len(), SEGMENTS);
                assert_eq!(tree.actions_of(cand).len(), TICKS);
            }
            // Only the fallback chain may contain an unsafe ancestor.
            assert!(tree.nodes.iter().skip(SEGMENTS + 1).all(|node| !node.collides));
            if samples == 0 {
                assert!(tree.layers.iter().all(|layer| layer.len() == 1));
            }
        }
    }

    #[test]
    fn terminal_layer_samples_diverse_states_with_and_without_warm_start() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let ego = State {
            speed: 8.0,
            ..State::default()
        };
        let actions = vec![Control::default(); TICKS];
        for warm in [None, Some(actions.as_slice())] {
            let tree = Graph::grow(ego, warm, 157, ctx.path(), &ctx, Connections::NearestZap);
            let terminals = &tree.layers[SEGMENTS];
            // More endpoints than the fallback and optional hot chain supply.
            assert!(terminals.len() > 2);
            let first = tree.nodes[terminals[0]].state;
            assert!(
                terminals
                    .iter()
                    .any(|&id| (tree.nodes[id].state.position().y - first.position().y).abs() > 0.5)
            );
            assert!(
                terminals
                    .iter()
                    .any(|&id| (tree.nodes[id].state.speed - first.speed).abs() > 1.0)
            );
            for layer in 1..=SEGMENTS {
                for &id in &tree.layers[layer] {
                    let node = &tree.nodes[id];
                    assert!(tree.layers[layer - 1].contains(&node.edge().parent));
                    assert_eq!(node.edge().controls.len(), STEER_TICKS);
                }
            }
            for path in tree.path_candidates(terminals.len()) {
                assert_eq!(tree.actions_of(&path).len(), TICKS);
            }
        }
    }

    #[test]
    fn stays_on_road_and_accelerates() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        );
        let trace = crate::planning::test_run(&mut GraphPlanner::new(Connections::NearestZap), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 4.5, "offset {}", end.position().y);
        assert!(end.speed > 10.0, "speed {}", end.speed);
    }

    #[test]
    fn avoids_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let trace = crate::planning::test_run(&mut GraphPlanner::new(Connections::NearestZap), ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.position().x - 40.0).hypot(s.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(
            trace.last().unwrap().position().x > 50.0,
            "did not pass, x {}",
            trace.last().unwrap().position().x
        );
    }

    #[test]
    fn plan_is_a_pure_function_of_state() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let actors = [obstacle];
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &actors);
        for connections in [Connections::NearestZap, Connections::Lattice] {
            let a = GraphPlanner::new(connections).plan(ego, &ctx);
            let b = GraphPlanner::new(connections).plan(ego, &ctx);
            assert_eq!(a, b);
        }
    }
}
