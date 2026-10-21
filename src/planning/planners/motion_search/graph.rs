//! Shared time-layered motion graph for tree and lattice search.
//! Reuses the sampling envelopes and Cartesian transformations in
//! [`crate::planning::frenet`]. Either search can use the shared iLQR post-process.

use super::{
    Candidate, SearchStrategy, initial_boundary, knot_boundary, lattice, nearest_zap, road_control, rollout_controls,
    select_candidates,
};

use rstar::RTree;
use rstar::primitives::GeomWithData;

use crate::common::geometry::{EGO_FOOTPRINT, wrap_angle};
use crate::common::kinematics::zero_action_point;
use crate::common::polynomial::CubicPolynomial;
use crate::common::steering::frenet::FrenetMotion;
use crate::common::steering::{ConstraintHandlingStrategy, motion_to_control};
use crate::constraints::Constraints;
use crate::planning::controls::rollout_constrained;
use crate::planning::frenet::{frenet_boundary, lateral_targets, longitudinal_targets};
use crate::planning::planner_math;
use crate::planning::sampling::{Halton, QuasiMonteCarlo};
use crate::planning::{Context, PLANNING_TICKS as TICKS};
use crate::simulation::{Control, State};
use crate::track::Path;

const STEER_TICKS: usize = 10;

/// Steering segments per trajectory — treetop's `NUM_STEER_SEGMENTS`; also
/// the number of tree layers past the root.
const SEGMENTS: usize = TICKS / STEER_TICKS;
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
pub(super) struct Edge {
    pub(super) parent: usize,
    controls: Vec<Control>,
    states: Vec<State>,
    pub(super) eval: EdgeEval,
}

/// A sampled target's incoming edges and the best path reaching it.
/// The winning rollout endpoint is frozen before this node gains children.
/// This keeps extracted controls continuous despite steering endpoint error.
pub(super) struct Node {
    pub(super) state: State,
    incoming: Vec<Edge>,
    best: usize,
    pub(super) cost_to_come: f64,
    pub(super) collides: bool,
}

impl Node {
    fn edge(&self) -> &Edge {
        &self.incoming[self.best]
    }
}

/// Edge evaluation: the shared metric objective per rolled-out stage,
/// averaged over the segment. A hard violation is priced at the
/// shared depth-scaled penalty and flagged rather than propagated as an
/// infinity, because the fallback chains must be able to carry a cost.
pub(super) struct EdgeEval {
    pub(super) cost: f64,
    pub(super) collides: bool,
}

/// A completed parent layer's coasting endpoints.
/// Spatial index with four coordinates instead of just position. Periodic yaw
/// copies preserve the wrapped-angle metric across the -π/π seam.
pub(super) struct ZapIndex(pub(super) RTree<GeomWithData<[f64; 4], usize>>);

impl ZapIndex {
    pub(super) fn new(parents: impl Iterator<Item = (usize, State)>, ticks: usize, dt: f64) -> Self {
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

    pub(super) fn point(state: State) -> [f64; 4] {
        [
            state.position().x,
            state.position().y,
            wrap_angle(state.pose.yaw),
            state.speed,
        ]
    }
}

/// The layered motion graph. `layers[0]` holds only the root;
/// The last layer holds the terminal nodes.
struct Graph {
    nodes: Vec<Node>,
    layers: Vec<Vec<usize>>,
    layer_ticks: Vec<usize>,
    dt: f64,
}

impl Graph {
    /// Grow a full-horizon graph: a zero-action fallback chain, a hot chain
    /// from warm-start actions (if any), then warm/cold samples distributed
    /// across every layer, including the terminal layer.
    fn grow(
        start: State,
        warm: Option<&[Control]>,
        samples: usize,
        path: &Path,
        ctx: &Context,
        connections: Connections,
        layer_count: usize,
    ) -> Graph {
        assert!((1..=TICKS).contains(&layer_count));
        let layer_ticks: Vec<_> = (0..=layer_count).map(|layer| layer * TICKS / layer_count).collect();
        let mut tree = Graph {
            nodes: Vec::new(),
            layers: vec![Vec::new(); layer_count + 1],
            layer_ticks: layer_ticks.clone(),
            dt: ctx.road.dt,
        };
        let g = Grower {
            path,
            ctx,
            initial_station: ctx.project_ego(start).s,
            layer_ticks: &layer_ticks,
        };
        let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, g.initial_station);

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
        for layer in 1..=layer_count {
            let from = tree.nodes[parent].state;
            let target = zero_action_point(from, layer_ticks[layer] - layer_ticks[layer - 1], ctx.road.dt);
            let (us, xs, ee) = g.steer_edge(from, target, layer);
            parent = tree.add_node(parent, us, xs, layer, ee);
        }

        // Hot chain (treetop `growHot`): re-roll the warm-start actions
        // from the *current* start and split the result into one node per
        // segment, stopping at the first colliding segment.
        let warm_traj = warm.map(|actions| {
            let mut actions = actions.to_vec();
            actions.resize(TICKS, Control::default());
            rollout_constrained(start, &actions, ctx.road.dt)
        });
        if let Some((wxs, wus)) = &warm_traj {
            let mut parent = 0usize;
            for layer in 1..=layer_count {
                let lo = layer_ticks[layer - 1];
                let hi = layer_ticks[layer];
                let us = wus[lo..hi].to_vec();
                let xs = wxs[lo..=hi].to_vec();
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
        let per_layer = samples / layer_count;
        let boundary = frenet_boundary(path, start, g.initial_station);
        let laterals = lateral_targets(
            path.project(start.position()).d,
            ctx.road.half_width,
            ctx.compute_budget,
        );
        let mut ix = 1usize;
        for layer in 1..=layer_count {
            // A feasible edge cannot repair a collision earlier in its path.
            // Keep unsafe fallback nodes out of ordinary parent selection.
            if tree.layers[layer - 1].iter().all(|&id| tree.nodes[id].collides) {
                break;
            }
            // The previous layer is complete: cache its zero-action points
            // once and reuse the index for every sample in this layer.
            let parents = tree.zap_index(layer, ctx.road.dt);
            let layer_time = layer_ticks[layer] as f64 * ctx.road.dt;
            for _ in 0..per_layer + usize::from(layer <= samples % layer_count) {
                let selector = Halton::coordinate(ix, 4);
                let c: [f64; 4] = std::array::from_fn(|d| Halton::coordinate(ix, d));
                ix += 1;

                let target = if selector < WARM_PROBA
                    && let Some((wxs, _)) = &warm_traj
                {
                    let w = wxs[layer_ticks[layer]];
                    let Some((mut lon, mut lat)) = frenet_boundary(path, w, path.project(w.position()).s) else {
                        continue;
                    };
                    lon[0] += (c[0] - 0.5) * 2.0 * WARM_D_POS;
                    lat[0] += (c[1] - 0.5) * 2.0 * WARM_D_POS;
                    lon[1] = (lon[1] + (c[3] - 0.5) * 2.0 * WARM_D_SPEED).max(0.0);
                    lat[1] += (c[2] - 0.5) * 2.0 * WARM_LATERAL_SPEED_FRACTION * w.speed;
                    let motion = FrenetMotion {
                        s: CubicPolynomial([lon[0], lon[1], 0.0, 0.0]),
                        d: CubicPolynomial([lat[0], lat[1], 0.0, 0.0]),
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
                    let motion = FrenetMotion {
                        s: CubicPolynomial::from_boundary(lon[0], lon[1], lon[0] + distance, speed, layer_time),
                        d: lateral,
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
            self.layer_ticks[layer] - self.layer_ticks[layer - 1],
            dt,
        )
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
    /// `convertPathToActionSequence`.
    fn actions_of(&self, path: &[usize]) -> Vec<Control> {
        let mut actions = Vec::with_capacity(TICKS);
        for &n in path {
            actions.extend_from_slice(&self.nodes[n].edge().controls);
        }
        actions
    }

    fn record_diagnostics(&self, diag: &crate::planning::Diagnostics) {
        for (layer, nodes) in self.layers.iter().enumerate().skip(1) {
            for &index in nodes {
                let node = &self.nodes[index];
                for edge in &node.incoming {
                    diag.record_point(edge.states.last().unwrap().position());
                    diag.record_timed_trajectory(
                        edge.states.iter().map(Into::into).collect(),
                        (0..edge.states.len())
                            .map(|tick| (self.layer_ticks[layer - 1] + tick) as f64 * self.dt)
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
    initial_station: f64,
    layer_ticks: &'a [usize],
}

impl Grower<'_, '_> {
    /// Steer from `from` toward `target` for this layer's duration under
    /// the actuation limits, priced as the edge landing in `layer`.
    fn steer_edge(&self, from: State, target: State, layer: usize) -> (Vec<Control>, Vec<State>, EdgeEval) {
        let ticks = self.layer_ticks[layer] - self.layer_ticks[layer - 1];
        let trajectory = self.ctx.time("kinodynamic", || {
            steer_trajectory(self.path, &from, &target, ticks, self.ctx.road.dt)
        });
        let invalid = trajectory.is_none();
        let (xs, us) = trajectory
            .map(|trajectory| (trajectory.states, trajectory.controls))
            .unwrap_or_else(|| rollout_constrained(from, &vec![Control::default(); ticks], self.ctx.road.dt));
        let mut ee = self.edge_eval(&xs, &us, layer);
        ee.collides |= invalid;
        (us, xs, ee)
    }

    /// Price one edge with the progress objective. `layer` fixes the
    /// absolute time of each stage, so actors are priced where they'll be.
    fn edge_eval(&self, xs: &[State], us: &[Control], layer: usize) -> EdgeEval {
        self.ctx.work(1);
        let dt = self.ctx.road.dt;
        let t0 = self.layer_ticks[layer - 1] as f64 * dt;
        let mut total = 0.0;
        let mut collides = false;

        let constraints = Constraints::new(
            self.ctx.road.half_width,
            self.ctx.actors,
            self.path,
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
fn steer_trajectory(
    path: &Path,
    start: &State,
    target: &State,
    ticks: usize,
    dt: f64,
) -> Option<crate::common::types::Trajectory> {
    let initial = initial_boundary(path, *start, path.project(start.position()).s)?;
    let end = initial_boundary(path, *target, path.project(target.position()).s)?;
    let motion = FrenetMotion::new(knot_boundary(initial), knot_boundary(end), ticks as f64 * dt);
    crate::simulation::rollout(*start, dt, ticks, |tick, actual| {
        motion_to_control(
            motion.at(path, (tick as f64 + 0.5) * dt)?,
            actual,
            dt,
            ConstraintHandlingStrategy::Clamp,
        )
    })
}

#[cfg(test)]
fn steer_actions(path: &Path, start: &State, target: &State, dt: f64) -> Option<Vec<Control>> {
    steer_trajectory(path, start, target, STEER_TICKS, dt).map(|trajectory| trajectory.controls)
}

/// Both modes draw the same targets; lattice evaluates up to six parents per target.
const SAMPLES: usize = 150;

impl SearchStrategy for Connections {
    const NAME: &'static str = "motion graph";

    fn search(&mut self, ego: State, warm: Option<&Candidate>, ctx: &Context) -> Option<Candidate> {
        let warm = warm.map(|previous| crate::planning::warm_start::shift_actions(previous.controls.clone()));
        let graph = ctx.time("optimize", || {
            Graph::grow(
                ego,
                warm.as_deref(),
                ctx.compute_budget.scale(SAMPLES, 10),
                ctx.path(),
                ctx,
                *self,
                SEGMENTS,
            )
        });
        if let Some(diag) = ctx.diagnostics {
            graph.record_diagnostics(diag);
        }
        // Full paths first, then progressively shorter safe prefixes completed with braking.
        // Final execution uses the same full-horizon validation as every other strategy.
        for layer in graph.layers.iter().skip(1).rev() {
            let mut nodes: Vec<_> = layer.iter().copied().filter(|&id| !graph.nodes[id].collides).collect();
            nodes.sort_by(|&a, &b| graph.nodes[a].cost_to_come.total_cmp(&graph.nodes[b].cost_to_come));
            for id in nodes {
                let actions = graph.actions_of(&graph.extract_path(id));
                let Some(mut candidate) = rollout_controls(ego, ctx, |tick, state| {
                    Some(
                        actions
                            .get(tick)
                            .copied()
                            .unwrap_or_else(|| road_control(state, ctx, crate::vehicle::MIN_LON_ACCEL)),
                    )
                }) else {
                    continue;
                };
                candidate.reward = -graph.nodes[id].cost_to_come;
                if let Some(candidate) = select_candidates(vec![candidate], 1, ctx).pop() {
                    return Some(candidate);
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{Planner, planners::motion_search::GraphPlanner};

    fn paths(graph: &Graph, count: usize) -> Vec<Vec<usize>> {
        graph
            .layers
            .last()
            .unwrap()
            .iter()
            .take(count)
            .map(|&id| graph.extract_path(id))
            .collect()
    }

    #[test]
    fn variable_layers_cover_the_horizon_with_continuous_paths() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        // Short warm starts can occur when the timestep or requested horizon changes.
        let warm = vec![Control::default(); TICKS / 2];
        for connections in [Connections::NearestZap, Connections::Lattice] {
            for layers in [1, 2, 7, 10] {
                for warm in [None, Some(warm.as_slice())] {
                    let graph = Graph::grow(ego, warm, 28, ctx.path(), &ctx, connections, layers);
                    assert_eq!(graph.layers.len(), layers + 1);
                    for layer in 1..=layers {
                        for &id in &graph.layers[layer] {
                            for edge in &graph.nodes[id].incoming {
                                assert!(graph.layers[layer - 1].contains(&edge.parent));
                                assert_eq!(
                                    edge.controls.len(),
                                    layer * TICKS / layers - (layer - 1) * TICKS / layers
                                );
                            }
                        }
                    }
                    for chain in paths(&graph, 3) {
                        assert_eq!(chain.len(), layers);
                        let actions = graph.actions_of(&chain);
                        assert_eq!(actions.len(), TICKS);
                        let (states, _) = rollout_constrained(ego, &actions, ctx.road.dt);
                        for (i, &id) in chain.iter().enumerate() {
                            assert_eq!(states[(i + 1) * TICKS / layers], graph.nodes[id].state);
                        }
                    }
                }
            }
        }
    }

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
                let graph = Graph::grow(ego, warm, SAMPLES, ctx.path(), &ctx, connections, SEGMENTS);
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
                for chain in paths(&graph, 3) {
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
        let motion = FrenetMotion {
            s: CubicPolynomial([10.0, 8.0, 0.0, 0.0]),
            d: CubicPolynomial([2.0, 0.0, 0.0, 0.0]),
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
            let tree = Graph::grow(ego, None, samples, &path, &ctx, Connections::NearestZap, SEGMENTS);
            let cands = paths(&tree, 2);
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
            let tree = Graph::grow(ego, warm, 157, ctx.path(), &ctx, Connections::NearestZap, SEGMENTS);
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
            for path in paths(&tree, terminals.len()) {
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
