//! Time-layered motion tree with Frenet sampling and cubic Frenet steering.
//! Reuses the sampling envelopes and Cartesian transformations in
//! [`crate::planning::frenet`], and supplies initial guesses to
//! [`crate::planning::planners::treetop::TreetopPlanner`].

use rstar::RTree;
use rstar::primitives::GeomWithData;

use crate::common::geometry::wrap_angle;
use crate::common::kinematics::{clamp_control, commanded_accel_for_net, commanded_accel_to_stop};
use crate::common::polynomial::CubicPolynomial;
use crate::constraints::Constraints;
use crate::planning::controls::{repeat_last_controls, rollout_constrained};
use crate::planning::frenet::{Motion, frenet_boundary, lateral_targets, longitudinal_targets};
use crate::planning::planner_math;
use crate::planning::planners::treetop::{SEGMENTS, STEER_TICKS, TICKS, goal_state, shift_actions, zero_action_point};
use crate::planning::sampling::{Halton, QuasiMonteCarlo};
use crate::planning::search_tree::parent_chain;
use crate::planning::take_warm;
use crate::planning::{Context, Planner};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;

// treetop's category probabilities (`sampling.h`): goal 0.1, warm 0.2,
// cold the rest. Drawn against a Halton coordinate instead of an RNG, so
// the schedule is a fixed interleaving rather than a random one.
const GOAL_PROBA: f64 = 0.1;
const WARM_PROBA: f64 = 0.2;

/// Warm samples' perturbation half-widths around the previous solution's
/// Frenet state: ±2 m station/lateral position, ±30% of speed in lateral
/// velocity, and ±2 m/s station velocity.
const WARM_D_POS: f64 = 2.0;
const WARM_LATERAL_SPEED_FRACTION: f64 = 0.3;
const WARM_D_SPEED: f64 = 2.0;

/// One tree node: a state, its parent, and the steering-segment edge that
/// reached it (treetop `node.h`, with `Rc` parent pointers flattened into
/// arena indices).
pub(crate) struct Node {
    pub(crate) state: State,
    pub(crate) parent: Option<usize>,
    /// Clamped actions of the edge from the parent ([`STEER_TICKS`] of
    /// them; empty for the root).
    pub(crate) controls: Vec<Control>,
    /// Rollout states of that edge, parent state included
    /// (`controls.len() + 1`; just the state for the root).
    pub(crate) states: Vec<State>,
    pub(crate) cost_to_come: f64,
    /// Whether any edge on the path to this node hard-violates the shared
    /// cost (collision / off-road) — set only by the fallback chains that
    /// deliberately ignore collisions to guarantee connectivity.
    pub(crate) collides: bool,
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
// Spatial index with four coordinates instead of just position. Periodic yaw
/// copies preserve the wrapped-angle metric across the -π/π seam.
struct ZapIndex(RTree<GeomWithData<[f64; 4], usize>>);

impl ZapIndex {
    fn new(parents: impl Iterator<Item = (usize, State)>, steer_duration: f64) -> Self {
        Self(RTree::bulk_load(
            parents
                .flat_map(|(id, state)| {
                    let point = Self::point(zero_action_point(state, steer_duration));
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

    fn nearest_parent(&self, target: State) -> usize {
        // Match the old scan's first-in-layer tie break, independent of
        // the spatial index's traversal order. Arena ids increase on insert.
        self.0
            .nearest_neighbors(&Self::point(target))
            .iter()
            .map(|point| point.data)
            .min()
            .expect("layers are never empty")
    }
}

/// The layered tree (treetop `tree.h`). `layers[0]` holds only the root;
/// `layers[SEGMENTS]` holds the goal nodes.
pub(crate) struct Tree {
    pub(crate) nodes: Vec<Node>,
    pub(crate) layers: [Vec<usize>; SEGMENTS + 1],
}

impl Tree {
    /// Grow a tree from `start` toward `goal`: root, zero-action fallback
    /// chain, hot chain from the warm-start actions (if any), `samples`
    /// goal/warm/cold samples spread over the intermediate layers, then
    /// goal nodes steered from every penultimate-layer parent — treetop's
    /// `Tree::grow`, in its exact phase order.
    pub(crate) fn grow(
        start: State,
        goal: State,
        warm: Option<&[Control]>,
        samples: usize,
        path: &Path,
        ctx: &Context,
    ) -> Tree {
        let mut tree = Tree {
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
            parent: None,
            controls: Vec::new(),
            states: vec![start],
            cost_to_come: 0.0,
            collides: false,
        });
        tree.layers[0].push(0);

        let steer_duration = STEER_TICKS as f64 * ctx.road.dt;

        // Zero-action fallback chain through the intermediate layers
        // (treetop `growZap`) — ignores collisions so a full parent chain
        // to the root always exists.
        let mut parent = 0usize;
        for layer in 1..SEGMENTS {
            let from = tree.nodes[parent].state;
            let target = zero_action_point(from, steer_duration);
            let (us, xs, ee) = g.steer_edge(from, target, steer_duration, layer);
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
        let per_layer = samples / (SEGMENTS - 1).max(1);
        let boundary = frenet_boundary(path, start, g.initial_station);
        let laterals = lateral_targets(
            path.project(start.position()).d,
            ctx.road.half_width,
            ctx.compute_budget,
        );
        let mut ix = 1usize;
        for layer in 1..SEGMENTS {
            // The previous layer is complete: cache its zero-action points
            // once and reuse the index for every sample in this layer.
            let parents = tree.zap_index(layer, steer_duration);
            for _ in 0..per_layer {
                let selector = Halton::coordinate(ix, 4);
                let c: [f64; 4] = std::array::from_fn(|d| Halton::coordinate(ix, d));
                ix += 1;

                let (target, reason) = if selector < GOAL_PROBA {
                    (goal, Reason::Goal)
                } else if selector < GOAL_PROBA + WARM_PROBA && warm_traj.is_some() {
                    let (wxs, _) = warm_traj.as_ref().unwrap();
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
                    (target, Reason::Sample)
                } else {
                    let Some((lon, lat)) = boundary else { continue };
                    let duration = layer as f64 * steer_duration;
                    let (d, dv) = laterals[(c[1] * laterals.len() as f64) as usize];
                    let lateral = CubicPolynomial::from_boundary(lat[0], lat[1], d, dv, duration);
                    let targets = longitudinal_targets(
                        start.speed,
                        ctx.road.dt,
                        ctx.compute_budget,
                        duration,
                        (path, lon[0], &lateral),
                    );
                    let (distance, speed) = targets[(c[0] * targets.len() as f64) as usize];
                    let motion = Motion {
                        longitudinal: CubicPolynomial::from_boundary(
                            lon[0],
                            lon[1],
                            lon[0] + distance,
                            speed,
                            duration,
                        ),
                        lateral,
                    };
                    let Some((target, _)) = motion.at(path, duration) else {
                        continue;
                    };
                    (target, Reason::Sample)
                };

                // Sampled state itself in collision → discard (treetop
                // checks its obstacles here; the metric objective's hard reject
                // is the equivalent).
                let t_s = layer as f64 * steer_duration;
                let (_, sample) = planner_math::state_sample(path, &target, t_s, None);
                if !ctx.time("cost", || constraints.point_cost(&sample)).is_finite() {
                    continue;
                }

                // Parent: goal samples take a rotating parent (treetop's
                // uniform-random one, made deterministic); the rest attach
                // to the nearest zero-action point.
                let prev = &tree.layers[layer - 1];
                let parent = match reason {
                    Reason::Goal => prev[ix % prev.len()],
                    Reason::Sample => parents.nearest_parent(target),
                };

                // Goal samples steer over the whole remaining horizon
                // (executing only this segment of the longer maneuver).
                let duration = match reason {
                    Reason::Goal => (SEGMENTS - layer) as f64 * steer_duration,
                    Reason::Sample => steer_duration,
                };
                let from = tree.nodes[parent].state;
                let (us, xs, ee) = g.steer_edge(from, target, duration.max(steer_duration), layer);
                if ee.collides {
                    continue;
                }
                tree.add_node(parent, us, xs, layer, ee);
            }
        }

        // Goal nodes (treetop `growGoalNodes`): steer to the goal from
        // every penultimate-layer parent; if every attempt collides, fall
        // back to the nearest-zap parent and accept the collision so the
        // goal layer is never empty.
        for i in 0..tree.layers[SEGMENTS - 1].len() {
            let parent = tree.layers[SEGMENTS - 1][i];
            let from = tree.nodes[parent].state;
            let (us, xs, ee) = g.steer_edge(from, goal, steer_duration, SEGMENTS);
            if ee.collides {
                continue;
            }
            tree.add_node(parent, us, xs, SEGMENTS, ee);
        }
        if tree.layers[SEGMENTS].is_empty() {
            let parent = tree.zap_index(SEGMENTS, steer_duration).nearest_parent(goal);
            let from = tree.nodes[parent].state;
            let (us, xs, ee) = g.steer_edge(from, goal, steer_duration, SEGMENTS);
            tree.add_node(parent, us, xs, SEGMENTS, ee);
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
        let p = &self.nodes[parent];
        let state = *states.last().unwrap();
        let node = Node {
            state,
            parent: Some(parent),
            cost_to_come: p.cost_to_come + ee.cost,
            collides: p.collides || ee.collides,
            controls,
            states,
        };
        self.nodes.push(node);
        let id = self.nodes.len() - 1;
        self.layers[layer].push(id);
        id
    }

    fn zap_index(&self, layer: usize, steer_duration: f64) -> ZapIndex {
        ZapIndex::new(
            self.layers[layer - 1].iter().map(|&id| (id, self.nodes[id].state)),
            steer_duration,
        )
    }

    /// The best `k` full-length paths, preferring feasible paths, then progress cost.
    pub(crate) fn path_candidates(&self, k: usize) -> Vec<Vec<usize>> {
        let mut goal_nodes = self.layers[SEGMENTS].clone();
        goal_nodes.sort_by(|&a, &b| {
            let (na, nb) = (&self.nodes[a], &self.nodes[b]);
            na.collides
                .cmp(&nb.collides)
                .then(na.cost_to_come.total_cmp(&nb.cost_to_come))
        });
        goal_nodes.truncate(k);
        goal_nodes.iter().map(|&n| self.extract_path(n)).collect()
    }

    /// Walk parent pointers from a goal node back to the root (treetop
    /// `extractPath`).
    fn extract_path(&self, node: usize) -> Vec<usize> {
        let path = parent_chain(node, 0, |n| self.nodes[n].parent);
        assert_eq!(path.len(), SEGMENTS);
        path
    }

    /// Concatenate a path's edge actions into one full-horizon action
    /// sequence ([`TICKS`] controls) — treetop's
    /// `convertPathToActionSequence`, the tree→iLQR hand-off.
    pub(crate) fn actions_of(&self, path: &[usize]) -> Vec<Control> {
        let mut actions = Vec::with_capacity(TICKS);
        for &n in path {
            actions.extend_from_slice(&self.nodes[n].controls);
        }
        actions
    }

    pub(crate) fn record_diagnostics(&self, diag: &crate::planning::Diagnostics) {
        for (layer, nodes) in self.layers.iter().enumerate().skip(1) {
            for &index in nodes {
                let node = &self.nodes[index];
                diag.record_point(node.state.position());
                diag.record_timed_trajectory(
                    node.states.iter().map(Into::into).collect(),
                    (0..node.states.len())
                        .map(|tick| ((layer - 1) * STEER_TICKS + tick) as f64 * crate::planning::PLANNING_DT_S)
                        .collect(),
                );
            }
        }
    }
}

enum Reason {
    Goal,
    Sample,
}

/// The per-grow context bundle: steering + edge pricing.
struct Grower<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
    initial_speed: f64,
    initial_station: f64,
}

impl Grower<'_, '_> {
    /// Steer from `from` toward `target` over `duration` and realize the
    /// first [`STEER_TICKS`] ticks of it under the actuation limits,
    /// priced as the edge landing in `layer`.
    fn steer_edge(
        &self,
        from: State,
        target: State,
        duration: f64,
        layer: usize,
    ) -> (Vec<Control>, Vec<State>, EdgeEval) {
        let actions = steer_actions(self.path, &from, &target, duration, self.ctx.road.dt);
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
            let sample = sample.with_control(us[i], xs[i].speed);
            let shared = self.ctx.time("cost", || constraints.point_cost(&sample));
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
fn steer_actions(path: &Path, start: &State, goal: &State, duration: f64, dt: f64) -> Option<Vec<Control>> {
    let (lon, lat) = frenet_boundary(path, *start, path.project(start.position()).s)?;
    let (end_lon, end_lat) = frenet_boundary(path, *goal, path.project(goal.position()).s)?;
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

/// The standalone tree planner: grow, take the best path candidate, drive
/// it — no optimization pass. Warm-starts from its own previous plan the
/// same way the treetop planner feeds its optimized solution back in, so
/// consecutive replans refine one detour instead of rediscovering a
/// different one each tick.
#[derive(Default)]
pub(crate) struct TreePlanner {
    prev: Option<Vec<Control>>,
    expected_next: State,
}

/// The standalone planner's sampling budget per plan — matches the treetop
/// planner's tree budget so the two search identically and differ only in
/// the optimization pass.
const SAMPLES: usize = 150;

impl Planner for TreePlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || ctx.path());
        let goal = goal_state(path, ego, ctx);
        let warm = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego).map(shift_actions)
        });

        // Offline calibration: 150 tree samples is about 100 ms.
        let samples = ctx.compute_budget.scale(SAMPLES, 20);
        let tree = ctx.time("optimize", || {
            Tree::grow(ego, goal, warm.as_deref(), samples, path, ctx)
        });

        if let Some(diag) = ctx.diagnostics {
            tree.record_diagnostics(diag);
        }

        let controls = ctx.time("extract", || {
            let best = &tree.path_candidates(1)[0];
            tree.actions_of(best)
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
    use crate::simulation::{Pose, Position};

    fn zap_dist2(zap: State, target: State) -> f64 {
        (zap.position().x - target.position().x).powi(2)
            + (zap.position().y - target.position().y).powi(2)
            + wrap_angle(zap.pose.yaw - target.pose.yaw).powi(2)
            + (zap.speed - target.speed).powi(2)
    }

    #[test]
    fn zap_index_matches_linear_scan() {
        use std::f64::consts::TAU;

        let state = |i| {
            State::new(
                Pose::new(
                    Position::new(80.0 * Halton::coordinate(i, 0), 10.0 * Halton::coordinate(i, 1)),
                    (Halton::coordinate(i, 2) - 0.5) * 4.0 * TAU,
                ),
                20.0 * Halton::coordinate(i, 3),
            )
        };
        // Sparse arena ids ensure the index returns node ids, not offsets
        // into its point cloud. Vary duration to catch hard-coded coasting.
        for count in [1, 16, 256] {
            let parents: Vec<_> = (1..=count).map(|i| (3 * i, state(i))).collect();
            for duration in [0.0, 0.4, 1.0, 2.0] {
                let index = ZapIndex::new(parents.iter().copied(), duration);
                for i in 257..513 {
                    let target = state(i);
                    let expected = parents
                        .iter()
                        .min_by(|(_, a), (_, b)| {
                            zap_dist2(zero_action_point(*a, duration), target)
                                .total_cmp(&zap_dist2(zero_action_point(*b, duration), target))
                        })
                        .unwrap()
                        .0;
                    assert_eq!(index.nearest_parent(target), expected);
                }
            }
        }
    }

    #[test]
    fn zap_index_wraps_yaw_and_breaks_ties_by_insertion_order() {
        use std::f64::consts::PI;

        let target = State::new(Pose::new(Position::new(0.0, 0.0), PI - 0.01), 0.0);
        let across_seam = State::new(Pose::new(target.position(), -PI + 0.01), 0.0);
        let farther = State::new(Pose::new(target.position(), PI - 0.1), 0.0);
        let faster = State { speed: 1.0, ..target };
        let parents = [(2, farther), (7, across_seam), (11, across_seam), (19, faster)];
        let index = ZapIndex::new(parents.into_iter(), 1.0);
        assert_eq!(index.nearest_parent(target), 7);

        // The nearest current position need not have the nearest coasting
        // endpoint. Match speed as well as position, even at zero duration.
        let target = State {
            speed: 8.0,
            ..State::default()
        };
        let moving = State::new(Pose::new(Position::new(-8.0, 0.0), 0.0), 8.0);
        let parents = [(4, State::default()), (9, moving)];
        let index = ZapIndex::new(parents.into_iter(), 1.0);
        assert_eq!(index.nearest_parent(zero_action_point(moving, 1.0)), 9);
        let index = ZapIndex::new([(4, State::default()), (9, target)].into_iter(), 0.0);
        assert_eq!(index.nearest_parent(target), 9);
    }

    #[test]
    fn steer_reaches_a_straight_ahead_target() {
        // Resistance is compensated; discrete integration still introduces endpoint error.
        let from = State {
            speed: 10.0,
            ..Default::default()
        };
        let dt = 0.1;
        let dur = STEER_TICKS as f64 * dt;
        let target = zero_action_point(from, dur);
        let path = Path::new(&[[-20.0, 0.0].into(), [400.0, 0.0].into()]);
        let actions = steer_actions(&path, &from, &target, dur, dt).unwrap();
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
        let dur = STEER_TICKS as f64 * dt;
        let target = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(10.0, 0.8), 0.0),
            10.0,
        );
        let path = Path::new(&[[-20.0, 0.0].into(), [400.0, 0.0].into()]);
        let actions = steer_actions(&path, &from, &target, dur, dt).unwrap();
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
        let actions = steer_actions(&path, &start, &goal, 1.0, 0.1).unwrap();
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
        assert!(steer_actions(&path, &reverse, &goal, 1.0, 0.1).is_none());
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
        let goal = goal_state(&path, ego, &ctx);
        let tree = Tree::grow(ego, goal, None, 90, &path, &ctx);
        let cands = tree.path_candidates(2);
        assert!(!cands.is_empty());
        for cand in &cands {
            assert_eq!(cand.len(), SEGMENTS);
            assert_eq!(tree.actions_of(cand).len(), TICKS);
        }
    }

    #[test]
    fn stays_on_road_and_accelerates() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        );
        let trace = crate::planning::test_run(&mut TreePlanner::default(), ego, &[], 150);
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
        let trace = crate::planning::test_run(&mut TreePlanner::default(), ego, &[obstacle], 150);
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
        let a = TreePlanner::default().plan(ego, &ctx);
        let b = TreePlanner::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }
}
