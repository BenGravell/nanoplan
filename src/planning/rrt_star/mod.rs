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

use crate::planning::cost::{self, Sample};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State};
use crate::wrap_angle;

// Sampling budget for the tree-growing loop in `plan`, split ~50/50
// between a deterministic road-geometry grid and a low-discrepancy
// quasi-Monte-Carlo sequence — see the module doc and the sampling comment
// in `plan` for why. Both are fully deterministic given the ego state, so
// two calls to `plan` from the same state grow the identical tree; no `Rng`
// is needed anywhere in this planner (unlike PI²-DDP, which still samples
// pseudo-randomly for its rollouts).
// road-geometry-informed grid, the same idea as the Frenet lattice's
// station-layers-by-laterals grid. An odd GRID_LATERALS keeps a station's
// centerline (d = 0) sample exactly on the grid, and the grid's last
// station lands exactly on the old goal-bias target (s0 + s_max, 0.0).
const GRID_STATIONS: usize = 10;
const GRID_LATERALS: usize = 9;
const GRID_BUDGET: usize = GRID_STATIONS * GRID_LATERALS;
// the rest of the budget: a 2D Halton sequence (see `van_der_corput`) over
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
const LATERAL_BOUND_M: f64 = 4.5;
// curvature a steer is rejected past
const MAX_CURVATURE: f64 = 0.35;
const STEER_SAMPLES: usize = 8;
const LATERAL_COST_WEIGHT: f64 = 0.5;
// planner-specific safety margin, a bit more than the shared cost
// function's hard-collision threshold (`cost::COLLISION_DIAMETER_M` = 2.5 m)
// to leave headroom for the discrete curve sampling below (the true closest
// approach between two sampled points can dip a little further than what
// gets checked)
const COLLISION_MARGIN_M: f64 = 3.0;
// see the goal-selection comment in `plan` for why this exists
const PROGRESS_TOLERANCE_M: f64 = 3.0;
// how many progress buckets a fresh candidate must lead a warm-started one
// by before goal-selection abandons continuity for it (see the goal-
// selection comment in `plan`)
const WARM_START_PROGRESS_MARGIN: f64 = 1.0;
// a bit inside the drivable_area metric's own ROAD_HALF_WIDTH_M (5.5, see
// src/metrics/drivable_area) so a bypass never scores a "successful"
// avoidance by driving off the road instead
const DRIVABLE_HALF_WIDTH_M: f64 = 5.0;

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The van der Corput radical-inverse sequence in `base`: reverse the
/// base-`base` digits of `index` and put them after the radix point. The
/// building block of a Halton sequence — pairing two of these in different
/// (coprime) bases, one per dimension, gives a fully deterministic,
/// index-only low-discrepancy point set in 2D with no RNG state at all.
/// `index` should start at 1 — `index = 0` degenerates to `0.0` in every
/// base, which would stack a sample from every dimension onto one corner.
fn van_der_corput(mut index: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut fraction = 1.0 / base as f64;
    while index > 0 {
        result += fraction * (index % base) as f64;
        index /= base;
        fraction /= base as f64;
    }
    result
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

/// Whether every sampled point on `segment` clears every actor's
/// constant-velocity-predicted position (via the shared cost function's
/// hard-collision check, plus this planner's own `COLLISION_MARGIN_M`
/// headroom), stays on the drivable road, and keeps the curve's curvature
/// within what's actually drivable. `s0`/`v` convert a segment point's
/// station into a predicted time the same way the Frenet lattice does.
fn feasible(
    curve: &CubicSteer,
    segment: &[[f64; 2]],
    path: &Path,
    s0: f64,
    v: f64,
    ctx: &Context,
) -> bool {
    for (i, &p) in segment.iter().enumerate() {
        let u = i as f64 / (segment.len() - 1) as f64;
        let curvature = curve.curvature(u);
        if curvature.abs() > MAX_CURVATURE {
            return false;
        }
        let (s, d) = path.project(p);
        // Endpoints alone aren't enough: a Hermite curve whose tangent
        // directions don't line up well with its chord can bulge past
        // both endpoints' lateral offset before coming back — clamping
        // only the *target* d (see the bypass-seeding comment in `plan`)
        // still let some edges drift off-road mid-segment, caught the same
        // way as the other structural bugs here: running the batch runner
        // over general synthetic scenarios and finding `drivable_area`
        // scoring 0 despite every sampled *target* being in-bounds. This
        // is tighter than the shared cost function's own road-edge check
        // (`drivable_area::ROAD_HALF_WIDTH_M`, 5.5 m), on purpose: a bypass
        // should never count as "successful" avoidance by driving right up
        // against the true edge.
        if d.abs() > DRIVABLE_HALF_WIDTH_M {
            return false;
        }
        let t = (s - s0) / v;
        for a in ctx.actors {
            let predicted = crate::metrics::project(a, t);
            if dist(p, [predicted.x, predicted.y]) < COLLISION_MARGIN_M {
                return false;
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
        if ctx
            .time("cost", || {
                cost::point_cost(&sample, ctx.target_speed, ctx.actors)
            })
            .is_infinite()
        {
            return false;
        }
    }
    true
}

/// Cost of one edge: arc length (via the sampled polyline, RRT*'s own
/// cost-to-come — the "star" rewiring is built around minimizing exactly
/// this), a lateral-offset penalty pulling the tree toward the lane center
/// (both structural to how this search shapes its tree), plus the shared
/// cost of each sampled point, timed under the "cost" seam so it's
/// comparable across planners. Curvature comes from the steering curve's
/// own closed-form derivative (`CubicSteer::curvature`) — a geometric fact
/// about this already-fixed candidate, not a search gradient.
fn edge_cost(
    segment: &[[f64; 2]],
    curve: &CubicSteer,
    path: &Path,
    s0: f64,
    v: f64,
    ctx: &Context,
) -> f64 {
    let mut total = 0.0;
    for w in segment.windows(2) {
        total += dist(w[0], w[1]);
    }
    for (i, &p) in segment.iter().enumerate() {
        let u = i as f64 / (segment.len() - 1) as f64;
        let (s, d) = path.project(p);
        total += LATERAL_COST_WEIGHT * d * d / segment.len() as f64;
        let sample = Sample {
            xy: p,
            lateral: d,
            speed: v,
            curvature: curve.curvature(u),
            t: (s - s0) / v,
            ..Default::default()
        };
        total += ctx.time("cost", || {
            cost::point_cost(&sample, ctx.target_speed, ctx.actors)
        });
    }
    total
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
    path: &Path,
    s0: f64,
    v: f64,
    ctx: &Context,
    target: [f64; 2],
) -> bool {
    let target_s = path.project(target).0;
    let Some(nearest_idx) = (0..nodes.len())
        .filter(|&i| nodes[i].station < target_s)
        .min_by(|&a, &b| dist(nodes[a].pos, target).total_cmp(&dist(nodes[b].pos, target)))
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
    let new_s = path.project(new_pos).0;
    if new_s <= nodes[nearest_idx].station {
        return false; // steering laterally lost all forward progress
    }

    // candidate parents: nodes behind new_pos's station, close enough to
    // new_pos to be worth considering
    let parent_candidates: Vec<usize> = (0..nodes.len())
        .filter(|&i| {
            nodes[i].station < new_s
                && (i == nearest_idx || dist(nodes[i].pos, new_pos) < NEIGHBOR_RADIUS_M)
        })
        .collect();

    let best = parent_candidates
        .iter()
        .filter_map(|&j| {
            let curve = CubicSteer::new(nodes[j].pos, nodes[j].yaw, new_pos, new_yaw);
            let segment = curve.sample(STEER_SAMPLES);
            feasible(&curve, &segment, path, s0, v, ctx).then(|| {
                let cost = nodes[j].cost + edge_cost(&segment, &curve, path, s0, v, ctx);
                (j, cost, segment)
            })
        })
        .min_by(|a, b| a.1.total_cmp(&b.1));
    let Some((parent_idx, cost, segment)) = best else {
        return false;
    };

    let new_idx = nodes.len();
    nodes.push(Node {
        pos: new_pos,
        yaw: new_yaw,
        station: new_s,
        cost,
        parent: Some(parent_idx),
        segment,
        warm_started: false,
    });

    // rewire: reconnect nodes strictly ahead of new_pos through it when
    // cheaper (ahead in station, so the reconnection stays a forward edge)
    let rewire_candidates: Vec<usize> =
        (0..nodes.len() - 1) // exclude new_idx itself
            .filter(|&j| {
                nodes[j].station > new_s && dist(nodes[j].pos, new_pos) < NEIGHBOR_RADIUS_M
            })
            .collect();
    for j in rewire_candidates {
        let curve = CubicSteer::new(new_pos, new_yaw, nodes[j].pos, nodes[j].yaw);
        let segment = curve.sample(STEER_SAMPLES);
        if !feasible(&curve, &segment, path, s0, v, ctx) {
            continue;
        }
        let rewired_cost = cost + edge_cost(&segment, &curve, path, s0, v, ctx);
        if rewired_cost < nodes[j].cost {
            nodes[j].cost = rewired_cost;
            nodes[j].parent = Some(new_idx);
            nodes[j].segment = segment;
            // ponytail: doesn't propagate the cheaper cost to j's existing
            // descendants (would need child pointers, not just parent
            // ones) — harmless here since cost is only used to pick
            // parents/the final leaf within this one plan() call, never
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
    pts.iter()
        .map(|&p| {
            let ds = (p[0] - prev[0]).hypot(p[1] - prev[1]);
            let new_v = ds / dt;
            let new_yaw = if ds > 1e-6 {
                (p[1] - prev[1]).atan2(p[0] - prev[0])
            } else {
                yaw
            };
            let u = Control {
                accel: (new_v - v) / dt,
                curvature: if ds > 1e-6 {
                    wrap_angle(new_yaw - yaw) / ds
                } else {
                    0.0
                },
            };
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
}

impl Planner for RrtStarPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0) = ctx.time("route", || {
            let path = Path::new(ctx.centerline);
            let (s0, _) = path.project([ego.x, ego.y]);
            (path, s0)
        });
        let v = ego.speed.clamp(2.0, ctx.target_speed.max(2.0));
        let s_max = v * PLANNING_HORIZON_S;

        let mut nodes = vec![Node {
            pos: [ego.x, ego.y],
            yaw: ego.yaw,
            station: s0,
            cost: 0.0,
            parent: None,
            segment: vec![],
            warm_started: false,
        }];

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
                let station = path.project(p).0;
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
                if !feasible(&curve, &segment, &path, s0, v, ctx) {
                    break; // stale from here on; random sampling takes over
                }
                let cost = parent.cost + edge_cost(&segment, &curve, &path, s0, v, ctx);
                let idx = nodes.len();
                nodes.push(Node {
                    pos: p,
                    yaw,
                    station,
                    cost,
                    parent: Some(parent_idx),
                    segment,
                    warm_started: true,
                });
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
        for a in ctx.actors {
            let (a_s, a_d) = path.project([a.x, a.y]);
            for side in [-1.0, 1.0] {
                let bypass = (a_d + side * (COLLISION_MARGIN_M + 2.0))
                    .clamp(-DRIVABLE_HALF_WIDTH_M, DRIVABLE_HALF_WIDTH_M);
                for (station_offset, lateral) in [
                    (-20.0, 0.25 * bypass),
                    (-10.0, 0.6 * bypass),
                    (-3.0, bypass),
                    (3.0, bypass),
                    (10.0, 0.6 * bypass),
                    (20.0, 0.0),
                ] {
                    let target = path.frenet_to_xy(a_s + station_offset, lateral);
                    try_extend(&mut nodes, &path, s0, v, ctx, target);
                }
            }
        }

        // Tree growth: a fixed road-geometry grid first, then a Halton
        // low-discrepancy sequence over the same domain — see the constants'
        // doc comments for why this split replaces plain pseudo-random
        // sampling. The grid runs in ascending-station order (station-major,
        // laterals inner) so each layer's samples extend from parents the
        // previous, nearer layer already planted, the same layer-by-layer
        // growth the deterministic bypass seeding above relies on; this
        // builds a connected backbone across the full planning horizon
        // before the Halton pass runs, so its arbitrarily-ordered targets
        // (which don't respect station order) almost always land near an
        // existing node instead of failing for lack of one.
        ctx.time("optimize", || {
            for gi in 0..GRID_STATIONS {
                let s = s0 + s_max * (gi + 1) as f64 / GRID_STATIONS as f64;
                for gj in 0..GRID_LATERALS {
                    let d = -LATERAL_BOUND_M
                        + 2.0 * LATERAL_BOUND_M * gj as f64 / (GRID_LATERALS - 1) as f64;
                    try_extend(&mut nodes, &path, s0, v, ctx, path.frenet_to_xy(s, d));
                }
            }
            for i in 1..=QMC_BUDGET {
                let s = s0 + van_der_corput(i, 2) * s_max;
                let d = -LATERAL_BOUND_M + van_der_corput(i, 3) * 2.0 * LATERAL_BOUND_M;
                try_extend(&mut nodes, &path, s0, v, ctx, path.frenet_to_xy(s, d));
            }
        });

        if let Some(diag) = ctx.diagnostics {
            for node in nodes.iter().skip(1) {
                diag.record_point(node.pos);
                diag.record_trajectory(node.segment.clone());
            }
        }

        // goal: the node making the most progress along the lane, ties
        // broken by lower cost; the root itself never qualifies. Progress
        // is bucketed to PROGRESS_TOLERANCE_M rather than compared exactly
        // — without this, a node that's a hair's-breadth further along but
        // squeezes past an obstacle beats a node that's a few centimeters
        // short but gives it a much wider berth, every single time, since
        // station is compared before cost ever gets a say.
        let stations: Vec<f64> = nodes
            .iter()
            .map(|n| (n.station / PROGRESS_TOLERANCE_M).round())
            .collect();
        let rank = |a: usize, b: usize| {
            stations[a]
                .total_cmp(&stations[b])
                .then(nodes[b].cost.total_cmp(&nodes[a].cost))
        };
        let overall_best = (1..nodes.len()).max_by(|&a, &b| rank(a, b));
        // Prefer continuing whatever warm-started node makes it furthest,
        // even over a fresh alternative that's technically a hair cheaper
        // or a bucket further along: switching plans every tick for a
        // marginal gain is what caused the *realized*, closed-loop
        // trajectory to squeeze obstacles far closer than any single
        // plan's own clearance check ever allowed (each 0.1 s replan only
        // contributes its first control, so an ego trajectory stitched
        // from many independently-reshaped detours doesn't inherit any
        // one of their safety margins). Only fall back to a fresh node
        // when nothing warm-started gets within one progress bucket of the
        // best available progress — i.e. the old plan is genuinely stale.
        let warm_best = (1..nodes.len())
            .filter(|&i| nodes[i].warm_started)
            .max_by(|&a, &b| rank(a, b));
        let best_leaf = match (warm_best, overall_best) {
            (Some(w), Some(o)) if stations[w] >= stations[o] - WARM_START_PROGRESS_MARGIN => {
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
            let accel = (-ego.speed / ctx.dt).max(-4.0);
            return vec![
                Control {
                    accel,
                    curvature: 0.0,
                };
                ctx.horizon
            ];
        };

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
                    let s = (v * ctx.dt * i as f64).min(total_len);
                    final_path.pose_at(s).0
                })
                .collect();
            xy_to_controls(ego, &pts, ctx.dt)
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
        let ctx = crate::planning::test_ctx(&[[-20.0, 0.0], [400.0, 0.0]], &actors);
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

    #[test]
    fn records_diagnostics_when_requested() {
        use crate::planning::Diagnostics;

        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let diag = Diagnostics::default();
        let mut ctx = crate::planning::test_ctx(&[[-20.0, 0.0], [400.0, 0.0]], &[]);
        ctx.diagnostics = Some(&diag);
        RrtStarPlanner::default().plan(ego, &ctx);
        let data = diag.take();
        assert!(!data.points.is_empty());
        assert_eq!(data.points.len(), data.trajectories.len());
    }
}
