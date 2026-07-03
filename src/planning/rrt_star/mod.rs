//! RRT* (rapidly-exploring random tree, asymptotically-optimal variant):
//! samples random (station, lateral) targets in the road frame and grows a
//! tree of poses from the ego's current state, connecting each new node to
//! its cheapest collision-free nearby parent and rewiring existing nodes
//! when a cheaper path through the new node appears.
//!
//! The step connecting any two poses — the "steering function" — is a
//! cubic polynomial in each of `x` and `y`, chosen via differential
//! flatness: a unicycle/bicycle's heading (`atan2(y', x')`) and curvature
//! (`(x'y'' - y'x'') / |·|^3`) are both determined by the flat outputs
//! `(x, y)` and their derivatives alone, so matching position and heading
//! (via derivative *direction*) at both endpoints is enough to guarantee a
//! kinematically smooth connection, without solving for heading or
//! curvature directly.

use crate::Rng;
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State};
use crate::wrap_angle;

const MAX_ITERS: usize = 150;
const STEP_MAX_M: f64 = 6.0;
const NEIGHBOR_RADIUS_M: f64 = 12.0;
const LATERAL_BOUND_M: f64 = 4.5;
const GOAL_BIAS: f64 = 0.1;
// center-to-center; a bit more than the Frenet lattice's 2.5 m threshold to
// leave headroom for the discrete curve sampling below (the true closest
// approach between two sampled points can dip a little further than what
// gets checked)
const COLLISION_RADIUS_M: f64 = 3.0;
// curvature a steer is rejected past
const MAX_CURVATURE: f64 = 0.35;
const STEER_SAMPLES: usize = 8;
const LATERAL_COST_WEIGHT: f64 = 0.5;
// see the goal-selection comment in `plan` for why this exists
const PROGRESS_TOLERANCE_M: f64 = 3.0;
// a bit inside the drivable_area metric's own ROAD_HALF_WIDTH_M (5.5, see
// src/metrics/drivable_area) so a bypass never scores a "successful"
// avoidance by driving off the road instead
const DRIVABLE_HALF_WIDTH_M: f64 = 5.0;

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
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
/// constant-velocity-predicted position by `COLLISION_RADIUS_M`, and the
/// curve's curvature stays within what's actually drivable. `s0`/`v`
/// convert a segment point's station into a predicted time the same way
/// the Frenet lattice does.
fn feasible(
    curve: &CubicSteer,
    segment: &[[f64; 2]],
    path: &Path,
    s0: f64,
    v: f64,
    actors: &[State],
) -> bool {
    for (i, &p) in segment.iter().enumerate() {
        let u = i as f64 / (segment.len() - 1) as f64;
        if curve.curvature(u).abs() > MAX_CURVATURE {
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
        // scoring 0 despite every sampled *target* being in-bounds.
        if d.abs() > DRIVABLE_HALF_WIDTH_M {
            return false;
        }
        let t = (s - s0) / v;
        for a in actors {
            let q = [
                a.x + a.speed * a.yaw.cos() * t,
                a.y + a.speed * a.yaw.sin() * t,
            ];
            if dist(p, q) < COLLISION_RADIUS_M {
                return false;
            }
        }
    }
    true
}

/// Cost of one edge: arc length (via the sampled polyline), a
/// lateral-offset penalty pulling the tree toward the lane center, and an
/// inverse-square actor-proximity term — the same ingredients as the
/// lattice's edge cost, added incrementally rather than over a fixed grid.
fn edge_cost(segment: &[[f64; 2]], path: &Path, s0: f64, v: f64, actors: &[State]) -> f64 {
    let mut cost = 0.0;
    for w in segment.windows(2) {
        cost += dist(w[0], w[1]);
    }
    for &p in segment {
        let (s, d) = path.project(p);
        cost += LATERAL_COST_WEIGHT * d * d / segment.len() as f64;
        let t = (s - s0) / v;
        for a in actors {
            let q = [
                a.x + a.speed * a.yaw.cos() * t,
                a.y + a.speed * a.yaw.sin() * t,
            ];
            cost += 30.0 / dist(p, q).max(0.1).powi(2);
        }
    }
    cost
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
    actors: &[State],
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
            feasible(&curve, &segment, path, s0, v, actors).then(|| {
                let cost = nodes[j].cost + edge_cost(&segment, path, s0, v, actors);
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
        if !feasible(&curve, &segment, path, s0, v, actors) {
            continue;
        }
        let rewired_cost = cost + edge_cost(&segment, path, s0, v, actors);
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

pub struct RrtStarPlanner {
    rng: Rng,
    /// Last tick's winning polyline, in the same fixed world frame the ego
    /// is — reused to warm-start this tick's tree (see `plan`'s doc note).
    prev_path: Vec<[f64; 2]>,
}

impl Default for RrtStarPlanner {
    fn default() -> Self {
        RrtStarPlanner {
            rng: Rng(0xBF58476D1CE4E5B9),
            prev_path: Vec::new(),
        }
    }
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
                let limit = max_yaw_change(step_len);
                let chord_yaw = (p[1] - parent.pos[1]).atan2(p[0] - parent.pos[0]);
                let dyaw = wrap_angle(chord_yaw - parent.yaw).clamp(-limit, limit);
                let yaw = wrap_angle(parent.yaw + dyaw);
                let curve = CubicSteer::new(parent.pos, parent.yaw, p, yaw);
                let segment = curve.sample(STEER_SAMPLES);
                if !feasible(&curve, &segment, &path, s0, v, ctx.actors) {
                    break; // stale from here on; random sampling takes over
                }
                let cost = parent.cost + edge_cost(&segment, &path, s0, v, ctx.actors);
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
        // any individual plan's own COLLISION_RADIUS_M). Trying the same
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
                let bypass = (a_d + side * (COLLISION_RADIUS_M + 2.0))
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
                    try_extend(&mut nodes, &path, s0, v, ctx.actors, target);
                }
            }
        }

        ctx.time("optimize", || {
            for _ in 0..MAX_ITERS {
                let (s, d) = if self.rng.uniform() < GOAL_BIAS {
                    (s0 + s_max, 0.0)
                } else {
                    (
                        self.rng.range(s0, s0 + s_max),
                        self.rng.range(-LATERAL_BOUND_M, LATERAL_BOUND_M),
                    )
                };
                try_extend(
                    &mut nodes,
                    &path,
                    s0,
                    v,
                    ctx.actors,
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
            (Some(w), Some(o)) if stations[w] >= stations[o] - 1.0 => Some(w),
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
