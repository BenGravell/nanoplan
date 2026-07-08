//! The treetop ego motion sampling tree (`tree/{tree,node,sampling,steer}.h`),
//! exposed standalone as [`RrtPlanner`] and reused by
//! [`TreetopPlanner`](super::TreetopPlanner) as its initial-guess engine.
//!
//! An RRT variant shaped by its job — feeding a trajectory optimizer —
//! rather than by asymptotic optimality (contrast [`crate::planning::rrt_star`]):
//!
//! - **Time-layered, fixed-depth growth.** The tree has exactly
//!   [`SEGMENTS`] layers past the root, each one steering segment
//!   ([`STEER_TICKS`] ticks) later in time, so *any* leaf in the final
//!   layer closes a full-horizon action sequence of exactly [`TICKS`]
//!   controls — precisely what the iLQR pass needs as input. Moving
//!   obstacles come free: a layer's states have a known absolute time, so
//!   collision checks price actors where they will be, not where they are.
//! - **Steering in action space.** [`steer_actions`] fits the shared
//!   quintic flat-output connector between two states' position, velocity,
//!   and acceleration boundary conditions, reads acceleration and curvature
//!   off the polynomial derivatives, then converts those
//!   targets into jerk/curvature-rate actions before rollout
//!   ([`rollout_constrained`]).
//! - **Zero-action-point parenting.** A sample attaches to the previous
//!   layer's node whose coasting endpoint ([`zero_action_point`]) is
//!   nearest in `(x, y, yaw, v)` — "who reaches me with the least effort",
//!   under simplifying kinematic assumptions. treetop builds a kd-tree
//!   (nanoflann) per layer for this; here a layer holds a few dozen nodes,
//!   so a linear scan is both simpler and faster than building the index.
//! - **A zero-action fallback chain** guarantees every layer is non-empty
//!   (so a full-length path always exists), *ignoring collisions* —
//!   treetop's `growZap`. Such nodes carry a `collides` flag and price
//!   their violating stages at the shared depth-scaled hard-violation
//!   penalty, so they lose to any genuine alternative and surface only as a
//!   better-than-nothing brace when the tree finds nothing else.
//! - **Layered sampling, three ways** (treetop `sampling.h`): *goal*
//!   samples steer gently toward the goal over the remaining horizon,
//!   *warm* samples perturb around the previous solution's trajectory, and
//!   *cold* samples cover the road-frame box — with treetop's RNG replaced
//!   by the shared Halton sequence (see the module doc in
//!   [`super`]), and treetop's axis-aligned `(x, y, yaw, v)` search-space
//!   box bent into the road frame: cold samples draw `(station, lateral,
//!   heading error, speed)` and map through the shared road-frame grid+QMC
//!   sampler, so the box follows a curved road instead of assuming the
//!   corridor is straight.
//!
//! **Seams**: `route`, `warm_start` (revalidate + shift the previous
//! solution), `optimize` (the whole grow), `extract`; `cost` (the shared
//! cost function, once per sampled point of every edge) nests inside
//! `optimize` and the warm-start replay alike.
//!
//! **Diagnostics**: every tree node as a point and every edge's rollout
//! polyline as a trajectory — the whole search considered, mirroring RRT*.

use super::{
    GOAL_HIT_TOL, SEGMENTS, STEER_TICKS, TICKS, goal_state, rollout_constrained, state_distance,
    take_warm, zero_action_point,
};
use crate::planning::planner_math;
use crate::planning::sampling::{self, Halton, QuasiMonteCarlo};
use crate::planning::search_tree::{parent_chain, record_diagnostics, repeat_last_controls};
use crate::planning::steering::QuinticSteer;
use crate::planning::{Context, Planner, cost};
use crate::scenarios::Path;
use crate::simulation::{Control, State, action_toward, step};
use crate::wrap_angle;

/// Lateral half-width cold samples span. Inside the shared cost's hard
/// off-road bound on a default-width road (`ROAD_HALF_WIDTH_M` = 5.5) so
/// samples are not born rejected there, and wide enough to let the optimizer
/// pull an aggressive detour back in. On a narrower road the shared cost's
/// per-plan `road_half_width` reject still discards any sample past the true
/// edge, so this fixed span only bounds where candidates are *drawn*, never
/// what counts as on-road.
const SAMPLE_LATERAL_M: f64 = 4.5;

/// Cold samples' heading spread around the lane direction (rad), and their
/// speed range as a multiple of the target speed. treetop samples yaw over
/// ±π/2 and speed over the full signed limit range; lane driving has no
/// use for near-perpendicular or reversing states, which would only steer
/// unreachable segments.
const SAMPLE_YAW_SPREAD: f64 = 0.5;
const SAMPLE_SPEED_FACTOR: f64 = 1.2;
const COLD_GRID_STATIONS: usize = SEGMENTS - 1;
const COLD_GRID_LATERALS: usize = 5;

// treetop's category probabilities (`sampling.h`): goal 0.1, warm 0.2,
// cold the rest. Drawn against a Halton coordinate instead of an RNG, so
// the schedule is a fixed interleaving rather than a random one.
const GOAL_PROBA: f64 = 0.1;
const WARM_PROBA: f64 = 0.2;

/// Warm samples' perturbation half-widths around the previous solution's
/// state: ±2 m position, ±0.3 rad heading, ±2 m/s speed (treetop's
/// `sampleNear`, with its ±π/2 yaw spread tightened for lane driving).
const WARM_D_POS: f64 = 2.0;
const WARM_D_YAW: f64 = 0.3;
const WARM_D_SPEED: f64 = 2.0;

/// One tree node: a state, its parent, and the steering-segment edge that
/// reached it (treetop `node.h`, with `Rc` parent pointers flattened into
/// arena indices).
pub(crate) struct Node {
    pub state: State,
    pub parent: Option<usize>,
    /// Clamped actions of the edge from the parent ([`STEER_TICKS`] of
    /// them; empty for the root).
    pub controls: Vec<Control>,
    /// Rollout states of that edge, parent state included
    /// (`controls.len() + 1`; just the state for the root).
    pub states: Vec<State>,
    pub cost_to_come: f64,
    pub dist_to_goal: f64,
    /// Whether any edge on the path to this node hard-violates the shared
    /// cost (collision / off-road) — set only by the fallback chains that
    /// deliberately ignore collisions to guarantee connectivity.
    pub collides: bool,
}

/// Edge evaluation: the shared cost function per rolled-out stage plus
/// treetop's `softLoss` integrand (the magnitude of total acceleration —
/// effort), averaged over the segment. A hard violation is priced at the
/// shared depth-scaled penalty and flagged rather than propagated as an
/// infinity, because the fallback chains must be able to carry a cost.
struct EdgeEval {
    cost: f64,
    collides: bool,
}

/// The layered tree (treetop `tree.h`). `layers[0]` holds only the root;
/// `layers[SEGMENTS]` holds the goal nodes.
pub(crate) struct Tree {
    pub nodes: Vec<Node>,
    pub layers: [Vec<usize>; SEGMENTS + 1],
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
        let g = Grower { path, ctx };
        let constraints = cost::HardConstraints::new(ctx.road.half_width, ctx.actors, Some(path));

        // Root node.
        tree.nodes.push(Node {
            state: start,
            parent: None,
            controls: Vec::new(),
            states: vec![start],
            cost_to_come: 0.0,
            dist_to_goal: state_distance(&start, &goal),
            collides: false,
        });
        tree.layers[0].push(0);

        let steer_dur = STEER_TICKS as f64 * ctx.road.dt;

        // Zero-action fallback chain through the intermediate layers
        // (treetop `growZap`) — ignores collisions so a full parent chain
        // to the root always exists.
        let mut parent = 0usize;
        for layer in 1..SEGMENTS {
            let from = tree.nodes[parent].state;
            let target = zero_action_point(from, steer_dur);
            let (us, xs, ee) = g.steer_edge(from, target, steer_dur, layer);
            parent = tree.add_node(parent, us, xs, layer, &goal, ee);
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
                parent = tree.add_node(parent, us, xs, layer, &goal, ee);
            }
        }

        // Layered sampling (treetop `growLayers`/`growSampleNode`), Halton
        // in place of the RNG. One global sample index keeps every draw —
        // category selector and state coordinates alike — deterministic.
        let per_layer = samples / (SEGMENTS - 1).max(1);
        let (s0, _) = path.project([start.x, start.y]);
        let (s_goal, _) = path.project([goal.x, goal.y]);
        let cold_samples = sampling::road_frame_samples::<Halton>(
            s0,
            (s_goal - s0).max(1.0),
            SAMPLE_LATERAL_M,
            COLD_GRID_STATIONS,
            COLD_GRID_LATERALS,
            samples,
        );
        let mut ix = 1usize;
        for layer in 1..SEGMENTS {
            for _ in 0..per_layer {
                let sample_id = ix;
                let selector = Halton::coordinate(ix, 4);
                let c: [f64; 4] = std::array::from_fn(|d| Halton::coordinate(ix, d));
                ix += 1;

                let (target, reason) = if selector < GOAL_PROBA {
                    (goal, Reason::Goal)
                } else if selector < GOAL_PROBA + WARM_PROBA && warm_traj.is_some() {
                    let (wxs, _) = warm_traj.as_ref().unwrap();
                    let w = wxs[layer * STEER_TICKS];
                    let target = State {
                        x: w.x + (c[0] - 0.5) * 2.0 * WARM_D_POS,
                        y: w.y + (c[1] - 0.5) * 2.0 * WARM_D_POS,
                        yaw: w.yaw + (c[2] - 0.5) * 2.0 * WARM_D_YAW,
                        speed: (w.speed + (c[3] - 0.5) * 2.0 * WARM_D_SPEED).max(0.0),
                        ..Default::default()
                    };
                    (target, Reason::Sample)
                } else {
                    // Cold: the shared road-frame grid+QMC box (see the
                    // module doc).
                    let (s, d) = cold_samples[(sample_id - 1) % cold_samples.len()];
                    let xy = path.frenet_to_xy(s, d);
                    let (_, lane_yaw) = path.pose_at(s);
                    let target = State {
                        x: xy[0],
                        y: xy[1],
                        yaw: lane_yaw + (2.0 * c[2] - 1.0) * SAMPLE_YAW_SPREAD,
                        speed: c[3] * SAMPLE_SPEED_FACTOR * ctx.road.target_speed,
                        ..Default::default()
                    };
                    (target, Reason::Sample)
                };

                // Sampled state itself in collision → discard (treetop
                // checks its obstacles here; the shared cost's hard reject
                // is the equivalent).
                let t_s = layer as f64 * steer_dur;
                let (_, sample) = planner_math::state_sample(path, &target, t_s, None);
                if !ctx
                    .time("cost", || {
                        constraints.point_cost(&sample, ctx.road.target_speed)
                    })
                    .is_finite()
                {
                    continue;
                }

                // Parent: goal samples take a rotating parent (treetop's
                // uniform-random one, made deterministic); the rest attach
                // to the nearest zero-action point.
                let prev = &tree.layers[layer - 1];
                let parent = match reason {
                    Reason::Goal => prev[ix % prev.len()],
                    Reason::Sample => tree.nearest_zap_parent(layer, &target, steer_dur),
                };

                // Goal samples steer over the whole remaining horizon
                // (executing only this segment of the longer maneuver).
                let duration = match reason {
                    Reason::Goal => (SEGMENTS - layer) as f64 * steer_dur,
                    Reason::Sample => steer_dur,
                };
                let from = tree.nodes[parent].state;
                let (us, xs, ee) = g.steer_edge(from, target, duration.max(steer_dur), layer);
                if ee.collides {
                    continue;
                }
                tree.add_node(parent, us, xs, layer, &goal, ee);
            }
        }

        // Goal nodes (treetop `growGoalNodes`): steer to the goal from
        // every penultimate-layer parent; if every attempt collides, fall
        // back to the nearest-zap parent and accept the collision so the
        // goal layer is never empty.
        for i in 0..tree.layers[SEGMENTS - 1].len() {
            let parent = tree.layers[SEGMENTS - 1][i];
            let from = tree.nodes[parent].state;
            let (us, xs, ee) = g.steer_edge(from, goal, steer_dur, SEGMENTS);
            if ee.collides {
                continue;
            }
            tree.add_node(parent, us, xs, SEGMENTS, &goal, ee);
        }
        if tree.layers[SEGMENTS].is_empty() {
            let parent = tree.nearest_zap_parent(SEGMENTS, &goal, steer_dur);
            let from = tree.nodes[parent].state;
            let (us, xs, ee) = g.steer_edge(from, goal, steer_dur, SEGMENTS);
            tree.add_node(parent, us, xs, SEGMENTS, &goal, ee);
        }

        tree
    }

    fn add_node(
        &mut self,
        parent: usize,
        controls: Vec<Control>,
        states: Vec<State>,
        layer: usize,
        goal: &State,
        ee: EdgeEval,
    ) -> usize {
        let p = &self.nodes[parent];
        let state = *states.last().unwrap();
        let node = Node {
            state,
            parent: Some(parent),
            cost_to_come: p.cost_to_come + ee.cost,
            dist_to_goal: state_distance(&state, goal),
            collides: p.collides || ee.collides,
            controls,
            states,
        };
        self.nodes.push(node);
        let id = self.nodes.len() - 1;
        self.layers[layer].push(id);
        id
    }

    /// The previous layer's node whose zero-action point is nearest the
    /// target in squared `(x, y, yaw, v)` distance — treetop's per-layer
    /// nanoflann kd-tree query, as a linear scan (see the module doc).
    fn nearest_zap_parent(&self, layer: usize, target: &State, steer_dur: f64) -> usize {
        *self.layers[layer - 1]
            .iter()
            .min_by(|&&a, &&b| {
                let da = zap_dist2(zero_action_point(self.nodes[a].state, steer_dur), target);
                let db = zap_dist2(zero_action_point(self.nodes[b].state, steer_dur), target);
                da.total_cmp(&db)
            })
            .expect("layers are never empty")
    }

    /// The best `k` full-length paths, each as `SEGMENTS` node ids from
    /// layer 1 to the goal layer — treetop's `getPathCandidates`, with the
    /// random alternates made deterministic: the best node is the
    /// cheapest goal-hitter (else the closest to the goal), and the
    /// alternates are the next-best goal nodes by the same ordering,
    /// instead of a shuffle.
    pub(crate) fn path_candidates(&self, k: usize) -> Vec<Vec<usize>> {
        let mut goal_nodes = self.layers[SEGMENTS].clone();
        goal_nodes.sort_by(|&a, &b| {
            let (na, nb) = (&self.nodes[a], &self.nodes[b]);
            let hit = |n: &Node| !(n.collides || n.dist_to_goal >= GOAL_HIT_TOL);
            // goal-hitters first, cheapest first among them; then by
            // distance to goal, collision-free before colliding
            hit(nb)
                .cmp(&hit(na))
                .then(na.collides.cmp(&nb.collides))
                .then(if hit(na) {
                    na.cost_to_come.total_cmp(&nb.cost_to_come)
                } else {
                    na.dist_to_goal.total_cmp(&nb.dist_to_goal)
                })
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
        record_diagnostics(
            diag,
            self.nodes.iter().skip(1).map(|node| {
                (
                    [node.state.x, node.state.y],
                    node.states.iter().map(|s| [s.x, s.y]).collect(),
                )
            }),
        );
    }
}

enum Reason {
    Goal,
    Sample,
}

fn zap_dist2(zap: State, target: &State) -> f64 {
    (zap.x - target.x).powi(2)
        + (zap.y - target.y).powi(2)
        + wrap_angle(zap.yaw - target.yaw).powi(2)
        + (zap.speed - target.speed).powi(2)
}

/// The per-grow context bundle: steering + edge pricing.
struct Grower<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
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
        let actions = steer_actions(&from, &target, duration, self.ctx.road.dt);
        let (xs, us) = rollout_constrained(from, &actions, self.ctx.road.dt);
        let ee = self.edge_eval(&xs, &us, layer);
        (us, xs, ee)
    }

    /// Price one edge: the shared cost function per stage (hard violations
    /// flagged and finitized) plus treetop's `softLoss` effort integrand
    /// and this planner's own centerline pull (the shared cost has no
    /// "hug the centerline" term by design — see the README's shared-cost
    /// section; without a pull of its own the tree happily settles ~2 m
    /// off-center, since a goal-hitting path there prices almost the same
    /// as a centered one), averaged over the segment. `layer` fixes the
    /// absolute time of each stage, so actors are priced where they'll be.
    fn edge_eval(&self, xs: &[State], us: &[Control], layer: usize) -> EdgeEval {
        const CENTER_W: f64 = 0.5;
        let dt = self.ctx.road.dt;
        let t0 = (layer - 1) as f64 * STEER_TICKS as f64 * dt;
        let mut total = 0.0;
        let mut collides = false;
        let constraints =
            cost::HardConstraints::new(self.ctx.road.half_width, self.ctx.actors, Some(self.path));
        for i in 0..us.len() {
            let x = &xs[i + 1];
            let (_, sample) =
                planner_math::state_sample(self.path, x, t0 + (i + 1) as f64 * dt, None);
            let shared = self.ctx.time("cost", || {
                constraints.point_cost(&sample, self.ctx.road.target_speed)
            });
            // treetop softLoss: magnitude of (lon, lat) acceleration
            let effort = x.accel.hypot(x.speed * x.speed * x.curvature);
            let pull = CENTER_W * sample.lateral * sample.lateral;
            let speed_err = x.speed - self.ctx.road.target_speed;
            let speed_cost = 0.5 * speed_err * speed_err;
            if shared.is_finite() {
                total += shared + effort + pull + speed_cost;
            } else {
                collides = true;
                total += constraints.violation_penalty(&sample) + effort + pull + speed_cost;
            }
        }
        EdgeEval {
            cost: total / us.len().max(1) as f64,
            collides,
        }
    }
}

// ---- The steering function (treetop `steer.h`) --------------------------

const STEER_ACCEL_TARGET_MAX: f64 = 3.0;

/// treetop's steering action generator: fit the shared quintic flat-output
/// connector matching both states' position, velocity, and acceleration
/// vector, then read the actions off the curve — acceleration is the
/// tangential derivative of speed, curvature is `(x'y'' - y'x'') / v^3` —
/// sampling each segment at its midpoint. A secant against the start
/// heading infers whether the curve is driven forward or in reverse,
/// flipping curvature accordingly.
/// Returns [`STEER_TICKS`] raw actions; the caller projects them onto the
/// actuation limits during rollout.
fn steer_actions(start: &State, goal: &State, duration: f64, dt: f64) -> Vec<Control> {
    let steer = QuinticSteer::from_states(start, goal, duration);
    let dir = steer.forward_sign(start.yaw, dt);

    let mut x = *start;
    (0..STEER_TICKS)
        .map(|i| {
            // midpoint of the segment: slightly more accurate than either end
            let t = (i as f64 + 0.5) * dt;
            let (_, accel, curvature) = steer.kinematics(t);
            let accel = accel.clamp(-STEER_ACCEL_TARGET_MAX, STEER_ACCEL_TARGET_MAX);
            let u = action_toward(x, accel, dir * curvature, dt);
            x = step(x, u, dt);
            u
        })
        .collect()
}

/// The standalone tree planner: grow, take the best path candidate, drive
/// it — no optimization pass. Warm-starts from its own previous plan the
/// same way the treetop planner feeds its optimized solution back in, so
/// consecutive replans refine one detour instead of rediscovering a
/// different one each tick.
#[derive(Default)]
pub struct RrtPlanner {
    prev: Option<Vec<Control>>,
    expected_next: State,
}

/// The standalone planner's sampling budget per plan — matches the treetop
/// planner's tree budget so the two search identically and differ only in
/// the optimization pass.
const SAMPLES: usize = 450;

impl Planner for RrtPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || Path::new(&ctx.road.centerline));
        let goal = goal_state(&path, ego, ctx);
        let warm = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego)
        });

        let tree = ctx.time("optimize", || {
            Tree::grow(ego, goal, warm.as_deref(), SAMPLES, &path, ctx)
        });

        if let Some(diag) = ctx.diagnostics {
            tree.record_diagnostics(diag);
        }

        let controls = ctx.time("extract", || {
            let best = &tree.path_candidates(1)[0];
            tree.actions_of(best)
        });
        let out = repeat_last_controls(&controls, ctx.horizon);
        self.expected_next = step(ego, out[0], ctx.road.dt);
        self.prev = Some(controls);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steer_reaches_a_straight_ahead_target() {
        // steering to the zero-action point should be nearly zero action
        let from = State {
            speed: 10.0,
            ..Default::default()
        };
        let dt = 0.1;
        let dur = STEER_TICKS as f64 * dt;
        let target = zero_action_point(from, dur);
        let actions = steer_actions(&from, &target, dur, dt);
        let (xs, _) = rollout_constrained(from, &actions, dt);
        let end = xs.last().unwrap();
        assert!(
            (end.x - target.x).abs() < 0.1,
            "x {} vs {}",
            end.x,
            target.x
        );
        assert!(end.y.abs() < 0.01);
        assert!((end.speed - 10.0).abs() < 0.1);
    }

    #[test]
    fn steer_reaches_a_lateral_offset_target() {
        // 0.8 m of lateral over 1 s: the lateral acceleration stays inside
        // ACCEL_LAT_MAX so the projection
        // doesn't bite (a 2 m offset would demand 12 m/s² and get clamped
        // into an undershoot — that infeasible case is exactly what the
        // constrained rollout exists to prevent)
        let from = State {
            speed: 10.0,
            ..Default::default()
        };
        let dt = 0.1;
        let dur = STEER_TICKS as f64 * dt;
        let target = State {
            x: 10.0,
            y: 0.8,
            yaw: 0.0,
            speed: 10.0,
            ..Default::default()
        };
        let actions = steer_actions(&from, &target, dur, dt);
        let (xs, _) = rollout_constrained(from, &actions, dt);
        let end = xs.last().unwrap();
        // the constrained rollout won't hit it exactly, but must get close
        assert!((end.x - 10.0).abs() < 1.0, "x {}", end.x);
        assert!((end.y - 0.8).abs() < 0.3, "y {}", end.y);
    }

    #[test]
    fn tree_always_offers_a_full_length_path() {
        // boxed in by actors, the zap fallback still yields a full path
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let actors: Vec<State> = (0..5)
            .map(|i| State {
                x: 10.0 + 5.0 * i as f64,
                y: -2.0 + i as f64,
                ..Default::default()
            })
            .collect();
        let ctx = crate::planning::test_ctx(&road, &actors);
        let path = Path::new(&road.centerline);
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
    fn tracks_centerline_and_speed() {
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let trace = crate::planning::test_run(&mut RrtPlanner::default(), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 1.5, "offset {}", end.y);
        assert!((end.speed - 10.0).abs() < 2.5, "speed {}", end.speed);
    }

    #[test]
    fn avoids_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = crate::planning::test_run(&mut RrtPlanner::default(), ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(
            trace.last().unwrap().x > 50.0,
            "did not pass, x {}",
            trace.last().unwrap().x
        );
    }

    #[test]
    fn plan_is_a_pure_function_of_state() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let actors = [obstacle];
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &actors);
        let a = RrtPlanner::default().plan(ego, &ctx);
        let b = RrtPlanner::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }
}
