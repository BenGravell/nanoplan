//! RRT* (rapidly-exploring random tree, asymptotically-optimal variant):
//! samples (station, lateral) targets in the road frame and grows a tree of
//! poses from the ego's current state, connecting each new node to its
//! cheapest collision-free nearby parent and rewiring existing nodes when a
//! cheaper path through the new node appears. Despite the name, nothing
//! about the sampling is actually random: targets come from a deterministic
//! road-geometry grid plus a low-discrepancy (Halton) sequence — see
//! `GRID_BUDGET`/`QMC_BUDGET`'s doc comments and the sampling comment in
//! `plan` — so `plan()` is a pure function of the ego state and scenario,
//! not a seeded-RNG rollout.
//!
//! The step connecting any two poses — the "steering function" — is a
//! cubic polynomial in each of `x` and `y`, chosen via differential
//! flatness: a unicycle/bicycle's heading (`atan2(y', x')`) and curvature
//! (`(x'y'' - y'x'') / |·|^3`) are both determined by the flat outputs
//! `(x, y)` and their derivatives alone, so matching position and heading
//! (via derivative *direction*) at both endpoints is enough to guarantee a
//! kinematically smooth connection, without solving for heading or
//! curvature directly.
//!
//! The three neighbor queries every new node runs — nearest node behind it,
//! candidate parents, rewire targets — go through an [`rstar`] R*-tree
//! spatial index grown alongside the node list, so each is `O(log n)` rather
//! than a linear scan, and only the `K_NEIGHBORS` nearest vertices are
//! considered (a *k*-nearest RRT*). See the `K_NEIGHBORS` doc comment and
//! the performance note in this module's README section.

use rstar::RTree;
use rstar::primitives::GeomWithData;

use crate::planning::cost::{self, Sample};
use crate::planning::sampling::{self, Halton};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State, action_toward, step};
use crate::wrap_angle;

/// A tree node's position tagged with its index in `nodes`, stored in the
/// [`RTree`] spatial index so nearest-neighbour and near-vertex queries run
/// in `O(log n)` instead of scanning every node.
type SpatialNode = GeomWithData<[f64; 2], usize>;
type Spatial = RTree<SpatialNode>;

// Sampling budget for the tree-growing loop in `plan`, split ~50/50
// between a deterministic road-geometry grid and a low-discrepancy
// quasi-Monte-Carlo sequence, both produced by
// `sampling::road_frame_samples` — the shared hybrid road-model + QMC
// sampler this planner and the judo optimizers (`sampling_mpc`) both draw
// from (see that module's parity note). Both are fully deterministic given
// the ego state, so two calls to `plan` from the same state grow the
// identical tree; no `Rng` is needed anywhere in this planner (unlike
// PI²-DDP, which still samples pseudo-randomly for its rollouts).
// road-geometry-informed grid, the same idea as the Frenet lattice's
// station-layers-by-laterals grid. An odd GRID_LATERALS keeps a station's
// centerline (d = 0) sample exactly on the grid, and the grid's last
// station lands exactly on the old goal-bias target (s0 + s_max, 0.0).
const GRID_STATIONS: usize = 10;
const GRID_LATERALS: usize = 9;
const GRID_BUDGET: usize = GRID_STATIONS * GRID_LATERALS;
// the rest of the budget: a 2D Halton sequence (`sampling::Halton`) over
// the same (station, lateral) domain, filling in what the fixed grid
// misses with well-distributed points — unlike pseudo-random sampling,
// which can leave gaps and clusters at this sample count. Set equal to
// GRID_BUDGET for the ~50/50 split.
const QMC_BUDGET: usize = GRID_BUDGET;
const STEP_MAX_M: f64 = 6.0;
// Warm-starting re-fits a fresh `CubicSteer` at every `STEER_SAMPLES`
// sub-point of the previous winning path, not just at tree-node boundaries
// (see `plan`'s warm-start block). `CubicSteer`'s tangent magnitude is
// `step_len / 3`, and curvature's denominator is that tangent magnitude
// cubed — for a sub-meter `step_len` between two adjacent sub-points, tiny
// positional noise in `p` (the executed trajectory rarely lands exactly on
// the previously planned polyline) turns into a wildly amplified, often
// spuriously huge curvature, breaking an otherwise perfectly good warm
// start over nothing. Skipping sub-points closer than this to the current
// parent — re-fitting only at strides comparable to a fresh `try_extend`
// hop — avoids the numerically fragile regime entirely.
const MIN_WARM_START_STEP_M: f64 = 1.5;
const NEIGHBOR_RADIUS_M: f64 = 12.0;
// Cap on how many near vertices (the closest ones) are considered as
// candidate parents, and separately as rewire targets, for each new node —
// i.e. the "k" of a k-nearest-neighbour RRT* rather than an every-node-in-
// radius RRT*. Both are asymptotically optimal variants; the k-nearest form
// is what keeps the cost bounded as the tree gets dense (without it, the
// count of vertices inside `NEIGHBOR_RADIUS_M` — and thus the steer +
// feasibility + edge-cost work per new node — grows with the tree, the
// O(n²) that dominated this planner's latency). The closest vertices are
// also the ones that matter: a near parent gives a short, cheap edge, and a
// far one almost never wins, so bounding to the nearest `k` barely changes
// the tree while cutting the work sharply. The spatial index makes pulling
// exactly the `k` nearest cheap.
const K_NEIGHBORS: usize = 24;
const LATERAL_BOUND_M: f64 = 4.5;
// curvature a steer is rejected past
const MAX_CURVATURE: f64 = 0.35;
const STEER_SAMPLES: usize = 8;
// Arc-length half-window for the Frenet projection of a steer segment's
// sampled points (`Path::project_near`). A segment is at most `STEP_MAX_M`
// of travel, roughly along the road, so a point's true station is within a
// few metres of the linear estimate from its endpoints' stations; this
// window is generous enough to always contain the nearest centerline
// segment while scanning a handful of them instead of the whole centerline
// (projection was the dominant cost once the spatial index removed the
// linear neighbour scans).
const PROJECT_WINDOW_M: f64 = 20.0;
const LATERAL_COST_WEIGHT: f64 = 0.5;
// planner-specific safety margin, a bit more than the shared cost
// function's hard-collision threshold (`cost::COLLISION_DIAMETER_M` = 2.5 m)
// to leave headroom for the discrete curve sampling below (the true closest
// approach between two sampled points can dip a little further than what
// gets checked)
const COLLISION_MARGIN_M: f64 = 3.0;
// see the goal-selection comment in `plan` for why this exists
const PROGRESS_TOLERANCE_M: f64 = 3.0;
// How far (in metres of Frenet station) the committed warm-started path may
// fall behind the furthest progress any leaf reaches this tick before goal
// selection abandons it for a fresh node. Deliberately several times the
// tick-to-tick jitter in the furthest fresh leaf's reach, so that jitter
// stops flipping the goal (the chattering the old one-bucket margin caused);
// only a genuine loss of progress — a stale or freshly-blocked committed
// path — crosses it. See the goal-selection comment in `plan`.
const WARM_VIABLE_BAND_M: f64 = 15.0;
// Weight on the goal-selection continuity bias, in effective *metres of
// progress* discounted per m² that a candidate's detour side
// (`Node::peak_lateral`) disagrees with the plan the ego is committed to
// (`committed_bias`). A full left↔right flip of a lane-width detour is a
// disagreement of ~8 m, whose square (~64 m²) times this weight is a ~10 m
// progress penalty — three PROGRESS_TOLERANCE_M buckets — so an opposite-side
// path has to reach implausibly much further to be preferred, and the
// coin-flip that made the goal chatter is gone. Not so large that it overrides
// a genuinely blocked side (that side simply has no feasible candidates to
// discount), and tuned against the synthetic batch: 0.15 both cut realized
// lateral-velocity reversals (151→128, worst 15→13 over 40 scenarios) and
// nudged mean score up (0.5549→0.5761), where a heavier 0.3 started trading
// score away. See the goal-selection comment.
const CONTINUITY_WEIGHT: f64 = 0.15;
// How fast `committed_bias` tracks the chosen path's side each tick (EMA
// weight on the new value). High enough to follow a real change of plan
// within a few ticks, low enough to smooth over single-tick sampling jitter.
const COMMIT_SMOOTHING: f64 = 0.5;
// How far inside the road's own drivable half-width ([`Road::half_width`],
// the bound the `drivable_area` metric and the shared cost score against)
// RRT* holds its detours, so a bypass never scores a "successful" avoidance
// by driving right up against the true edge. Subtracted from the road's
// actual half-width per plan (see `drivable_bound`) rather than a fixed
// 5.0 m, so the margin follows a narrow street in instead of letting the
// tree wander off it.
const DRIVABLE_MARGIN_M: f64 = 0.5;

/// The lateral bound RRT* holds its detours within on `ctx`'s road: the
/// road's drivable half-width less [`DRIVABLE_MARGIN_M`], floored so a very
/// narrow road still leaves the centerline itself reachable.
fn drivable_bound(ctx: &Context) -> f64 {
    (ctx.road.half_width - DRIVABLE_MARGIN_M).max(0.5)
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Whichever of `a`, `b` has the larger magnitude, keeping its sign — the
/// running "most lateral offset so far" for `Node::peak_lateral`.
fn signed_max(a: f64, b: f64) -> f64 {
    if a.abs() >= b.abs() { a } else { b }
}

/// Largest heading change worth attempting for a hop of length `step_len`,
/// so the resulting `CubicSteer` (tangent magnitude `step_len/3`, see
/// below) stays within `MAX_CURVATURE`. A Hermite curve whose start tangent
/// misses the chord direction by `dyaw` and whose end tangent matches it
/// exactly peaks at curvature ≈ `48 * dyaw / step_len` for that tangent
/// magnitude (measured empirically with this module's own diagnostic
/// instrumentation — see the git history for the throwaway script); solving
/// for `dyaw` at the curvature limit, with a safety factor for the
/// difference between this approximation and `feasible`'s actual discrete
/// sampling, and a sane upper cap so a very long hop can't claim an
/// unrealistically sharp turn is fine. Scaling with `step_len` matters: a
/// *short* hop can afford only a tiny heading change before curvature blows
/// up, exactly backwards from the fixed-angle-per-hop guess this module
/// started with, which mostly rejected long hops needlessly while still
/// letting short, sharp ones slip past the initial curvature check.
fn max_yaw_change(step_len: f64) -> f64 {
    (MAX_CURVATURE * step_len / 55.0).min(0.3)
}

/// Cubic-in-`s` connector between two oriented points: `x(s)` and `y(s)`
/// are each independently cubic, matching position and heading (tangent
/// direction) at `s=0` and `s=1` — the differential-flatness steering
/// function described in the module doc.
struct CubicSteer {
    cx: [f64; 4],
    cy: [f64; 4],
}

impl CubicSteer {
    fn new(p0: [f64; 2], yaw0: f64, p1: [f64; 2], yaw1: f64) -> Self {
        // tangent magnitude: a third of the chord length, the same
        // heuristic bezier_idm uses for its lane-return curve
        let k = (dist(p0, p1) / 3.0).max(1e-3);
        let hermite = |a0: f64, m0: f64, a1: f64, m1: f64| {
            [
                a0,
                m0,
                3.0 * (a1 - a0) - 2.0 * m0 - m1,
                2.0 * (a0 - a1) + m0 + m1,
            ]
        };
        CubicSteer {
            cx: hermite(p0[0], k * yaw0.cos(), p1[0], k * yaw1.cos()),
            cy: hermite(p0[1], k * yaw0.sin(), p1[1], k * yaw1.sin()),
        }
    }

    fn eval(c: &[f64; 4], s: f64) -> f64 {
        c[0] + s * (c[1] + s * (c[2] + s * c[3]))
    }

    fn eval_d1(c: &[f64; 4], s: f64) -> f64 {
        c[1] + s * (2.0 * c[2] + s * 3.0 * c[3])
    }

    fn eval_d2(c: &[f64; 4], s: f64) -> f64 {
        2.0 * c[2] + 6.0 * s * c[3]
    }

    fn point(&self, s: f64) -> [f64; 2] {
        [Self::eval(&self.cx, s), Self::eval(&self.cy, s)]
    }

    /// Curvature at `s`, via the flat-output formula
    /// `(x'y'' - y'x'') / (x'^2+y'^2)^1.5`.
    fn curvature(&self, s: f64) -> f64 {
        let (dx, dy) = (Self::eval_d1(&self.cx, s), Self::eval_d1(&self.cy, s));
        let (ddx, ddy) = (Self::eval_d2(&self.cx, s), Self::eval_d2(&self.cy, s));
        let speed = dx.hypot(dy).max(1e-6);
        (dx * ddy - dy * ddx) / speed.powi(3)
    }

    /// Sample `n` points from `s=0` to `s=1` inclusive.
    fn sample(&self, n: usize) -> Vec<[f64; 2]> {
        (0..n)
            .map(|i| self.point(i as f64 / (n - 1) as f64))
            .collect()
    }
}

struct Node {
    pos: [f64; 2],
    yaw: f64,
    /// Frenet station of `pos`, cached at creation. Used to keep every
    /// edge a step *forward* along the lane — see the module note on
    /// monotonic stations in `plan`.
    station: f64,
    /// Signed Frenet lateral offset of `pos` (positive = left of the lane).
    lateral: f64,
    /// The most lateral (largest `|lateral|`) offset, kept with its sign,
    /// anywhere on the path from the root to this node — i.e. which side the
    /// path swings out to and how far. Goal selection uses it to stay
    /// committed to one side of an obstacle across ticks instead of flipping
    /// (see the goal-selection comment in `plan`). Maintained incrementally
    /// (`signed_max` of the parent's value and this node's `lateral`); a
    /// rewire refreshes the rewired node but, like `cost`, doesn't propagate
    /// to its existing descendants — good enough as a side signal.
    peak_lateral: f64,
    cost: f64,
    parent: Option<usize>,
    /// Sampled polyline of the edge from `parent` to this node (empty for
    /// the root); kept for both the diagnostic overlay and final path
    /// extraction.
    segment: Vec<[f64; 2]>,
    /// Whether this node's *position* came from replaying last tick's
    /// winning path (see `plan`'s warm-start block), rather than from a
    /// sample drawn this tick. Used to prefer continuing an
    /// already-committed path over switching to a fresh, differently-
    /// shaped alternative purely because it's a hair cheaper — see the
    /// goal-selection comment for why bare cost/progress comparison
    /// chatters between ticks otherwise.
    warm_started: bool,
}

/// Feasibility check *and* edge cost in a single pass over the steer
/// segment, returning `Some(edge_cost)` for a drivable edge and `None` for
/// an infeasible one. Every sampled point is projected once
/// (`Path::project_near`) and priced once — previously a separate `feasible`
/// pass and `edge_cost` pass each projected and called `point_cost` on the
/// same points, doubling the hot-loop work; merging them halves it with no
/// change to which edges are feasible or what they cost.
///
/// Feasible means every point clears every actor's lane-aware predicted
/// position ([`crate::metrics::predict`], with this planner's own
/// `COLLISION_MARGIN_M` headroom on top of the shared cost's hard-collision
/// check — an actor driving the route is predicted along its curve), stays on
/// the drivable
/// road, and keeps the curve's curvature within what's actually drivable.
/// The edge cost is arc length (RRT*'s cost-to-come, what the "star"
/// rewiring minimizes) plus a lateral-offset pull toward the lane center,
/// plus the shared per-point cost (timed under the `cost` seam). Curvature
/// comes from the steering curve's own closed-form derivative — a geometric
/// fact about this already-fixed candidate, not a search gradient. `s0`/`v`
/// convert a point's station into a predicted time as the Frenet lattice
/// does; `sa`/`sb` are the segment endpoints' stations, hinting the
/// windowed projection.
fn steer_cost(
    curve: &CubicSteer,
    segment: &[[f64; 2]],
    path: &Path,
    s0: f64,
    v: f64,
    ctx: &Context,
    // stations of the segment's two endpoints, hinting the windowed projection
    [sa, sb]: [f64; 2],
) -> Option<f64> {
    let mut total = 0.0;
    let mut prev: Option<[f64; 2]> = None;
    for (i, &p) in segment.iter().enumerate() {
        let u = i as f64 / (segment.len() - 1) as f64;
        let curvature = curve.curvature(u);
        if curvature.abs() > MAX_CURVATURE {
            return None;
        }
        let (s, d) = path.project_near(p, sa + (sb - sa) * u, PROJECT_WINDOW_M);
        // Endpoints alone aren't enough: a Hermite curve whose tangent
        // directions don't line up well with its chord can bulge past
        // both endpoints' lateral offset before coming back — clamping
        // only the *target* d (see the bypass-seeding comment in `plan`)
        // still let some edges drift off-road mid-segment, caught the same
        // way as the other structural bugs here: running the batch runner
        // over general synthetic scenarios and finding `drivable_area`
        // scoring 0 despite every sampled *target* being in-bounds. This
        // is tighter than the shared cost function's own road-edge check
        // (the road's `half_width`), on purpose: a bypass should never count
        // as "successful" avoidance by driving right up against the true
        // edge.
        if d.abs() > drivable_bound(ctx) {
            return None;
        }
        let t = (s - s0) / v;
        for a in ctx.actors {
            let predicted = crate::metrics::predict(a, Some(path), t);
            if dist(p, [predicted.x, predicted.y]) < COLLISION_MARGIN_M {
                return None;
            }
        }
        let sample = Sample {
            xy: p,
            lateral: d,
            speed: v,
            curvature,
            t,
            ..Default::default()
        };
        let point = ctx.time("cost", || {
            cost::point_cost(
                &sample,
                ctx.road.target_speed,
                ctx.road.half_width,
                ctx.actors,
                Some(path),
            )
        });
        if point.is_infinite() {
            return None;
        }
        total += point + LATERAL_COST_WEIGHT * d * d / segment.len() as f64;
        if let Some(q) = prev {
            total += dist(q, p); // arc length
        }
        prev = Some(p);
    }
    Some(total)
}

/// Try to grow the tree one step toward `target`: find the nearest node
/// strictly behind its station, steer at most `max_yaw_change` away from
/// that node's own heading (never straight at `target` — see the comment
/// this replaced, in git history, for why), pick the cheapest
/// collision-free parent among nearby candidates, insert the new node, and
/// rewire any nearby nodes ahead of it that would now be cheaper through
/// it. Shared by both the deterministic actor-bypass seeding pass and the
/// random-sampling loop in `plan`, so both extend the tree exactly the
/// same way. Returns whether a node was actually added.
fn try_extend(
    nodes: &mut Vec<Node>,
    tree: &mut Spatial,
    path: &Path,
    s0: f64,
    v: f64,
    ctx: &Context,
    target: [f64; 2],
) -> bool {
    let target_s = path.project(target).0;
    // nearest existing node strictly behind the target's station: walk the
    // spatial index outward from the target (nearest first) and take the
    // first behind it — exact, and typically only a couple of steps.
    let Some(nearest_idx) = tree
        .nearest_neighbor_iter(&target)
        .map(|n| n.data)
        .find(|&i| nodes[i].station < target_s)
    else {
        return false;
    };

    let parent = &nodes[nearest_idx];
    let step_len = dist(parent.pos, target).min(STEP_MAX_M);
    let limit = max_yaw_change(step_len);
    let raw_dir = (target[1] - parent.pos[1]).atan2(target[0] - parent.pos[0]);
    let steer_dir = wrap_angle(parent.yaw + wrap_angle(raw_dir - parent.yaw).clamp(-limit, limit));
    let new_pos = [
        parent.pos[0] + step_len * steer_dir.cos(),
        parent.pos[1] + step_len * steer_dir.sin(),
    ];
    let new_yaw = steer_dir;
    let (new_s, new_d) = path.project(new_pos);
    if new_s <= nodes[nearest_idx].station {
        return false; // steering laterally lost all forward progress
    }

    // candidate parents: the `K_NEIGHBORS` nearest vertices to new_pos that
    // sit behind it in station (within NEIGHBOR_RADIUS_M), from the spatial
    // index. `nearest_idx` is always among them (it's `step_len ≤ STEP_MAX_M`
    // away, well inside the radius), but include it explicitly so the
    // straight steer-from edge is never dropped when the cap bites.
    let radius2 = NEIGHBOR_RADIUS_M * NEIGHBOR_RADIUS_M;
    let mut parent_candidates: Vec<usize> = tree
        .nearest_neighbor_iter_with_distance_2(&new_pos)
        .take_while(|(_, d2)| *d2 <= radius2)
        .filter_map(|(n, _)| (nodes[n.data].station < new_s).then_some(n.data))
        .take(K_NEIGHBORS)
        .collect();
    if !parent_candidates.contains(&nearest_idx) {
        parent_candidates.push(nearest_idx);
    }

    let best = parent_candidates
        .iter()
        .filter_map(|&j| {
            let curve = CubicSteer::new(nodes[j].pos, nodes[j].yaw, new_pos, new_yaw);
            let segment = curve.sample(STEER_SAMPLES);
            steer_cost(
                &curve,
                &segment,
                path,
                s0,
                v,
                ctx,
                [nodes[j].station, new_s],
            )
            .map(|ec| (j, nodes[j].cost + ec, segment))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1));
    let Some((parent_idx, cost, segment)) = best else {
        return false;
    };

    let new_idx = nodes.len();
    let peak_lateral = signed_max(nodes[parent_idx].peak_lateral, new_d);
    nodes.push(Node {
        pos: new_pos,
        yaw: new_yaw,
        station: new_s,
        lateral: new_d,
        peak_lateral,
        cost,
        parent: Some(parent_idx),
        segment,
        warm_started: false,
    });
    tree.insert(SpatialNode::new(new_pos, new_idx));

    // rewire: reconnect the `K_NEIGHBORS` nearest vertices strictly ahead of
    // new_pos through it when cheaper (ahead in station, so the reconnection
    // stays a forward edge; the new node itself has station == new_s, so the
    // `> new_s` filter excludes it even though it's now in the index).
    let rewire_candidates: Vec<usize> = tree
        .nearest_neighbor_iter_with_distance_2(&new_pos)
        .take_while(|(_, d2)| *d2 <= radius2)
        .filter_map(|(n, _)| (nodes[n.data].station > new_s).then_some(n.data))
        .take(K_NEIGHBORS)
        .collect();
    for j in rewire_candidates {
        let curve = CubicSteer::new(new_pos, new_yaw, nodes[j].pos, nodes[j].yaw);
        let segment = curve.sample(STEER_SAMPLES);
        let Some(ec) = steer_cost(
            &curve,
            &segment,
            path,
            s0,
            v,
            ctx,
            [new_s, nodes[j].station],
        ) else {
            continue;
        };
        let rewired_cost = cost + ec;
        if rewired_cost < nodes[j].cost {
            nodes[j].cost = rewired_cost;
            nodes[j].parent = Some(new_idx);
            nodes[j].segment = segment;
            nodes[j].peak_lateral = signed_max(peak_lateral, nodes[j].lateral);
            // ponytail: doesn't propagate the cheaper cost (or peak_lateral)
            // to j's existing descendants (would need child pointers, not
            // just parent ones) — harmless here since both are only used to
            // pick parents/the final leaf within this one plan() call, never
            // carried across ticks
        }
    }
    true
}

/// Turn a sampled position trajectory (spaced `dt` apart, starting one tick
/// after the ego) into controls for the kinematic model. Same technique as
/// the Frenet lattice's converter of the same name.
fn xy_to_controls(ego: State, pts: &[[f64; 2]], dt: f64) -> Vec<Control> {
    let mut v = ego.speed;
    let mut yaw = ego.yaw;
    let mut prev = [ego.x, ego.y];
    let mut x = ego;
    pts.iter()
        .map(|&p| {
            let ds = (p[0] - prev[0]).hypot(p[1] - prev[1]);
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

#[derive(Default)]
pub struct RrtStarPlanner {
    /// Last tick's winning polyline, in the same fixed world frame the ego
    /// is — reused to warm-start this tick's tree (see `plan`'s doc note).
    prev_path: Vec<[f64; 2]>,
    /// The side of the lane the committed plan has swung out to (signed peak
    /// lateral offset of the winning path), smoothed across ticks. Goal
    /// selection is biased toward candidates that stay on this side, so the
    /// planner commits to passing an obstacle on one side instead of
    /// flip-flopping (see the goal-selection comment in `plan`).
    committed_bias: f64,
}

impl Planner for RrtStarPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0, d0) = ctx.time("route", || {
            let path = Path::new(&ctx.road.centerline);
            let (s0, d0) = path.project([ego.x, ego.y]);
            (path, s0, d0)
        });
        let v = ego.speed.clamp(2.0, ctx.road.target_speed.max(2.0));
        let s_max = v * PLANNING_HORIZON_S;

        let mut nodes = vec![Node {
            pos: [ego.x, ego.y],
            yaw: ego.yaw,
            station: s0,
            lateral: d0,
            peak_lateral: d0,
            cost: 0.0,
            parent: None,
            segment: vec![],
            warm_started: false,
        }];
        // Spatial index over node positions, grown alongside `nodes` (root
        // first). Every place a node is pushed also inserts it here.
        let mut tree: Spatial = Spatial::new();
        tree.insert(SpatialNode::new([ego.x, ego.y], 0));

        // Warm start: replay whatever part of last tick's winning path is
        // still ahead of the ego and still collision-free against this
        // tick's (possibly moved) actors, as a ready-made chain of nodes.
        // Without this, the tree is rebuilt from independent random
        // samples every 0.1 s tick; since the simulator only ever executes
        // one control from each plan, the *realized* trajectory is
        // stitched from many differently-shaped one-tick plans and can
        // chatter much closer to an obstacle than any single plan
        // intended — found via this module's own closed-loop test, the
        // same way the lattice's initial-slope fix was.
        ctx.time("warm_start", || {
            let mut parent_idx = 0;
            for &p in &self.prev_path {
                let (station, lateral) = path.project(p);
                let parent = &nodes[parent_idx];
                if station <= parent.station {
                    continue; // behind the chain so far: drop, don't break the rest
                }
                let step_len = dist(parent.pos, p);
                if step_len < MIN_WARM_START_STEP_M {
                    continue; // too short to re-fit a curve to reliably; see the constant's doc comment
                }
                let limit = max_yaw_change(step_len);
                let chord_yaw = (p[1] - parent.pos[1]).atan2(p[0] - parent.pos[0]);
                let dyaw = wrap_angle(chord_yaw - parent.yaw).clamp(-limit, limit);
                let yaw = wrap_angle(parent.yaw + dyaw);
                let curve = CubicSteer::new(parent.pos, parent.yaw, p, yaw);
                let segment = curve.sample(STEER_SAMPLES);
                let Some(ec) = steer_cost(
                    &curve,
                    &segment,
                    &path,
                    s0,
                    v,
                    ctx,
                    [parent.station, station],
                ) else {
                    break; // stale from here on; random sampling takes over
                };
                let cost = parent.cost + ec;
                let peak_lateral = signed_max(parent.peak_lateral, lateral);
                let idx = nodes.len();
                nodes.push(Node {
                    pos: p,
                    yaw,
                    station,
                    lateral,
                    peak_lateral,
                    cost,
                    parent: Some(parent_idx),
                    segment,
                    warm_started: true,
                });
                tree.insert(SpatialNode::new(p, idx));
                parent_idx = idx;
            }
        });

        // Deterministic bypass seeding: for every actor, try extending the
        // tree toward a safe lateral offset on both sides, at a few
        // station offsets around it — every tick, unconditionally, not
        // just "with some probability" via the RNG. This is what makes
        // obstacle avoidance *consistent* tick to tick: randomized
        // informed sampling (try a random side, a random nearby station,
        // with some probability) found a wide bypass on some ticks and a
        // different, narrower one on others, and since the simulator only
        // ever executes each plan's first control, a closed-loop
        // trajectory stitched from differently-shaped detours doesn't
        // inherit any single one's safety margin — that's what the
        // swerves_around_stopped_obstacle test caught (min gaps well under
        // any individual plan's own COLLISION_MARGIN_M). Trying the same
        // candidates every time means the tree finds (and, via warm start
        // and rewiring, keeps refining) the *same* detour every tick.
        // Each side's ramp is seeded as a *chain*, not independent points:
        // try_extend always connects to the nearest existing node behind
        // it, so seeding in increasing-station order makes each waypoint
        // extend the previous one on the same side, gradually ramping the
        // offset up and back down rather than demanding one hop cover the
        // whole lateral distance (which max_yaw_change's steering-angle
        // limit would reject outright).
        let drivable = drivable_bound(ctx);
        for a in ctx.actors {
            let (a_s, a_d) = path.project([a.x, a.y]);
            for side in [-1.0, 1.0] {
                let bypass = (a_d + side * (COLLISION_MARGIN_M + 2.0)).clamp(-drivable, drivable);
                for (station_offset, lateral) in [
                    (-20.0, 0.25 * bypass),
                    (-10.0, 0.6 * bypass),
                    (-3.0, bypass),
                    (3.0, bypass),
                    (10.0, 0.6 * bypass),
                    (20.0, 0.0),
                ] {
                    let target = path.frenet_to_xy(a_s + station_offset, lateral);
                    try_extend(&mut nodes, &mut tree, &path, s0, v, ctx, target);
                }
            }
        }

        // Tree growth: a fixed road-geometry grid first, then a Halton
        // low-discrepancy sequence over the same domain — see the constants'
        // doc comments for why this split replaces plain pseudo-random
        // sampling. The (station, lateral) targets come from the shared
        // `sampling::road_frame_samples`, which lays the grid down in
        // ascending-station order (station-major, laterals inner) followed
        // by the QMC pass, so each layer's samples extend from parents the
        // previous, nearer layer already planted, the same layer-by-layer
        // growth the deterministic bypass seeding above relies on; this
        // builds a connected backbone across the full planning horizon
        // before the Halton pass runs, so its arbitrarily-ordered targets
        // (which don't respect station order) almost always land near an
        // existing node instead of failing for lack of one.
        ctx.time("optimize", || {
            for (s, d) in sampling::road_frame_samples::<Halton>(
                s0,
                s_max,
                LATERAL_BOUND_M,
                GRID_STATIONS,
                GRID_LATERALS,
                QMC_BUDGET,
            ) {
                try_extend(
                    &mut nodes,
                    &mut tree,
                    &path,
                    s0,
                    v,
                    ctx,
                    path.frenet_to_xy(s, d),
                );
            }
        });

        if let Some(diag) = ctx.diagnostics {
            for node in nodes.iter().skip(1) {
                diag.record_point(node.pos);
                diag.record_trajectory(node.segment.clone());
            }
        }

        // Goal selection. The simulator executes only the *first* segment of
        // the winning path each tick, so a smooth closed-loop trajectory needs
        // the goal — and above all which *side* of the lane it commits to — to
        // stay consistent tick to tick. The old logic ranked leaves by
        // progress (bucketed) then raw cost, preferring a warm-started node
        // only within one progress bucket of the best. Around an obstacle that
        // fails badly: a detour to the left and its mirror image to the right
        // reach near-identical progress at near-identical cost, so the tie
        // was effectively a coin flip that landed differently every tick as
        // the sampled tree jittered — the ego's steering chattered between the
        // two, never committing to a side (and the warm continuation couldn't
        // rescue it: replaying last tick's detour through the obstacle often
        // truncates, so it wasn't viable to continue). Two robust mechanisms:
        //
        // 1. A **continuity bias**. Each node carries `peak_lateral`, the
        //    furthest-out signed offset on its path from the root — i.e. which
        //    side it swings to and how far. `committed_bias` is that quantity
        //    for the plan already being executed, smoothed across ticks.
        //    Selection ranks by *effective* progress: a node's station minus
        //    `CONTINUITY_WEIGHT · (peak_lateral − committed_bias)²`. A path on
        //    the opposite side to the commitment loses a double-digit-metre
        //    chunk of effective progress, so it can't win by reaching a hair
        //    further — which is exactly how an opposite-side corner-cutter
        //    used to steal the goal (raw progress was the primary key, and the
        //    continuity/cost only broke ties within a bucket). On an open or
        //    gently curved lane every path has `peak_lateral ≈ 0`, so the term
        //    is inert and nothing changes. Effective progress is still
        //    bucketed to PROGRESS_TOLERANCE_M so that, among comparably-far
        //    nodes, the cheaper one wins rather than one a hair further along
        //    that squeezes the obstacle.
        // 2. **Warm continuation**. When the replay of last tick's winning
        //    path still reaches within WARM_VIABLE_BAND_M of the furthest
        //    progress any leaf makes, its deepest node is taken directly, so
        //    the executed first segment is literally last tick's. (This alone
        //    replaced the old one-bucket margin, which the per-tick progress
        //    jitter crossed constantly.)
        let committed_bias = self.committed_bias;
        let eff_bucket: Vec<f64> = nodes
            .iter()
            .map(|n| {
                let eff = n.station - CONTINUITY_WEIGHT * (n.peak_lateral - committed_bias).powi(2);
                (eff / PROGRESS_TOLERANCE_M).round()
            })
            .collect();
        let rank = |a: usize, b: usize| {
            eff_bucket[a]
                .total_cmp(&eff_bucket[b])
                .then(nodes[b].cost.total_cmp(&nodes[a].cost))
        };
        let overall_best = (1..nodes.len()).max_by(|&a, &b| rank(a, b));
        let warm_best = (1..nodes.len())
            .filter(|&i| nodes[i].warm_started)
            .max_by(|&a, &b| nodes[a].station.total_cmp(&nodes[b].station));
        let best_leaf = match (warm_best, overall_best) {
            (Some(w), Some(o)) if nodes[w].station >= nodes[o].station - WARM_VIABLE_BAND_M => {
                Some(w)
            }
            _ => overall_best,
        };

        let Some(mut idx) = best_leaf else {
            // every sample was infeasible (e.g. boxed in): brake straight,
            // and drop the stale warm start so next tick starts fresh.
            // Capped so one Euler step can't overshoot past zero speed —
            // the Simulator's kinematic step has no floor, so a *constant*
            // -4.0 accel held over several consecutive boxed-in ticks (this
            // whole Vec is returned every time, though only its first
            // control is ever applied) would eventually drive the ego
            // into reverse instead of holding it stopped. Found the same
            // way as this module's other structural bugs: running the
            // batch runner over general synthetic scenarios, not from
            // this module's own (single-obstacle) closed-loop tests.
            self.prev_path.clear();
            self.committed_bias = 0.0; // no plan to be committed to
            let accel = (-ego.speed / ctx.road.dt).max(-4.0);
            let mut x = ego;
            return (0..ctx.horizon)
                .map(|_| {
                    let u = action_toward(x, accel, 0.0, ctx.road.dt);
                    x = step(x, u, ctx.road.dt);
                    u
                })
                .collect();
        };

        // Smooth `committed_bias` toward the chosen path's side. As the ego
        // clears a detour and its path returns to the lane, `peak_lateral`
        // (measured from the advancing root) shrinks, so the bias decays back
        // to zero on its own.
        self.committed_bias = (1.0 - COMMIT_SMOOTHING) * self.committed_bias
            + COMMIT_SMOOTHING * nodes[idx].peak_lateral;

        let mut chain = vec![];
        while let Some(parent) = nodes[idx].parent {
            chain.push(idx);
            idx = parent;
        }
        chain.reverse();
        let mut winning_path = vec![nodes[0].pos];
        for i in chain {
            winning_path.extend(nodes[i].segment.iter().skip(1).copied());
        }

        let controls = ctx.time("extract", || {
            let final_path = Path::new(&winning_path);
            let total_len = final_path.length();
            let pts: Vec<[f64; 2]> = (1..=ctx.horizon)
                .map(|i| {
                    let s = (v * ctx.road.dt * i as f64).min(total_len);
                    final_path.pose_at(s).0
                })
                .collect();
            xy_to_controls(ego, &pts, ctx.road.dt)
        });
        self.prev_path = winning_path;
        controls
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::test_run;

    #[test]
    fn stays_on_empty_centerline() {
        let ego = State {
            y: 1.5,
            speed: 8.0,
            ..Default::default()
        };
        let trace = test_run(&mut RrtStarPlanner::default(), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 1.0, "offset {}", end.y);
    }

    /// The sampling is a fixed grid plus a Halton sequence, both pure
    /// functions of the ego state and scenario — no `Rng` advances between
    /// calls, unlike PI²-DDP. Two independent planners replanning from the
    /// identical state must therefore produce the identical plan, not just
    /// a reproducible-across-runs one.
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
        let a = RrtStarPlanner::default().plan(ego, &ctx);
        let b = RrtStarPlanner::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }

    #[test]
    fn swerves_around_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = test_run(&mut RrtStarPlanner::default(), ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        let end = trace.last().unwrap();
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(end.x > 60.0, "did not pass the obstacle, x {}", end.x);
    }

    /// Parity with the shared sampler: lifting RRT*'s old inline grid +
    /// van-der-Corput loop into `sampling::road_frame_samples` must produce
    /// the byte-identical target sequence it did before, so the compile-time
    /// interface share didn't quietly change where the tree grows. Pins the
    /// numeric contract on top of the structural (type-level) one.
    #[test]
    fn rrt_targets_match_shared_sampler() {
        let (s0, s_max) = (12.5, 80.0);
        let shared = sampling::road_frame_samples::<Halton>(
            s0,
            s_max,
            LATERAL_BOUND_M,
            GRID_STATIONS,
            GRID_LATERALS,
            QMC_BUDGET,
        );
        // reconstruct the historical inline loop and compare element-wise
        let mut expected = Vec::new();
        for gi in 0..GRID_STATIONS {
            let s = s0 + s_max * (gi + 1) as f64 / GRID_STATIONS as f64;
            for gj in 0..GRID_LATERALS {
                let d = -LATERAL_BOUND_M
                    + 2.0 * LATERAL_BOUND_M * gj as f64 / (GRID_LATERALS - 1) as f64;
                expected.push((s, d));
            }
        }
        for i in 1..=QMC_BUDGET {
            let s = s0 + sampling::van_der_corput(i, 2) * s_max;
            let d = -LATERAL_BOUND_M + sampling::van_der_corput(i, 3) * 2.0 * LATERAL_BOUND_M;
            expected.push((s, d));
        }
        assert_eq!(shared, expected);
    }

    #[test]
    fn records_diagnostics_when_requested() {
        use crate::planning::Diagnostics;

        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let diag = Diagnostics::default();
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let mut ctx = crate::planning::test_ctx(&road, &[]);
        ctx.diagnostics = Some(&diag);
        RrtStarPlanner::default().plan(ego, &ctx);
        let data = diag.take();
        assert!(!data.points.is_empty());
        assert_eq!(data.points.len(), data.trajectories.len());
    }
}
