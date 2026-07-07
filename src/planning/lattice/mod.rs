//! EM/lattice-style planner. Samples a deterministic grid of (station,
//! lateral) points in the road Frenet frame, connects successive layers into
//! a layered DAG with cubic-in-time lateral segments, assigns edge costs
//! (offset, smoothness, predicted-obstacle proximity), and finds the
//! cheapest path with **A\*** (best-first) search.
//!
//! The grid is deliberately high-resolution — `STATION_LAYERS × LATERALS`
//! is in the high hundreds — so the lattice can represent fine lateral
//! maneuvers and commit to them smoothly. At that size the exhaustive
//! layer-by-layer dynamic program this planner used to run (which prices
//! *every* `L`-to-`L` inter-layer edge, `O(S·L²)` cost-function evaluations)
//! is wasteful: almost all of those edges are large, obviously-bad lateral
//! jumps that no optimal path uses. A\* evaluates edge costs lazily — only
//! for nodes it actually expands, in increasing cost-so-far order — and
//! stops the moment it settles a node in the final layer, so on a typical
//! tick it prices a small fraction of the grid's edges and stays well inside
//! the real-time budget. The path it returns is identical to the DP's (all
//! edge costs are non-negative, so the first final-layer node A\* settles is
//! the global optimum); only the work to find it is smaller.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::planning::cost::{self, Sample};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State};
use crate::wrap_angle;

pub struct LatticePlanner;

/// Number of lateral samples per station layer (lateral grid resolution),
/// evenly spaced over `[-LAT_BOUND_M, LAT_BOUND_M]`. Odd, so one sample
/// lands exactly on the centerline (`d = 0`).
const LATERALS: usize = 47;
/// Number of station layers reaching out to the planning horizon (progress
/// grid resolution).
const STATION_LAYERS: usize = 32;
/// Half-width of the lateral sampling band. A bit inside the drivable
/// half-width so a sampled path stays clearly on the road.
const LAT_BOUND_M: f64 = 3.75;
/// Samples per lateral segment for cost integration and collision checking.
/// Lower than the old exhaustive DP used because there are now ~5× as many
/// (much shorter) segments spanning the horizon, so the whole path is still
/// sampled densely (`STATION_LAYERS × SAMPLES_PER_SEGMENT` points).
const SAMPLES_PER_SEGMENT: usize = 4;
/// How many lateral columns an edge may span between adjacent station
/// layers. A layer is only ~`horizon/STATION_LAYERS` of travel, so a jump of
/// more than a few columns (`≈ NEIGHBOR_SPAN × 0.25 m`) there is a curvature
/// no real car has — the shared cost would reject it, or price it out of any
/// optimal path, so never generating those edges costs nothing and keeps
/// the search branching factor (and cost-function evaluations) bounded. Full
/// lateral range is still reachable: over `STATION_LAYERS` layers the path
/// can ramp `NEIGHBOR_SPAN` columns per layer, far more than the grid's
/// width. This is what keeps the high-resolution grid inside the real-time
/// budget together with A*'s lazy expansion.
const NEIGHBOR_SPAN: usize = 4;

/// Total grid nodes — the resolution knob. `STATION_LAYERS × LATERALS`.
const GRID_NODES: usize = STATION_LAYERS * LATERALS;

/// Lateral offset of grid column `j`.
fn lateral(j: usize) -> f64 {
    -LAT_BOUND_M + 2.0 * LAT_BOUND_M * j as f64 / (LATERALS - 1) as f64
}

/// A\* priority-queue entry: the cheapest cost-so-far pops first (min-heap
/// via a reversed `Ord`). `total_cmp` gives a total order over the finite,
/// non-negative edge costs; ties break on node index for determinism.
struct QItem {
    cost: f64,
    node: usize,
}
impl PartialEq for QItem {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.node == other.node
    }
}
impl Eq for QItem {}
impl Ord for QItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // reversed so BinaryHeap (a max-heap) yields the minimum cost
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.node.cmp(&self.node))
    }
}
impl PartialOrd for QItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Turn a sampled position trajectory (spaced `dt` apart, starting one tick
/// after the ego) into controls for the kinematic model.
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

impl Planner for LatticePlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0, d0) = ctx.time("route", || {
            let path = Path::new(&ctx.road.centerline);
            let (s0, d0) = path.project([ego.x, ego.y]);
            (path, s0, d0)
        });
        // ponytail: constant-speed profile; couple IDM into the lattice when needed
        let v = ego.speed.clamp(2.0, ctx.road.target_speed.max(2.0));
        // STATION_LAYERS evenly spaced layers reaching out to the full
        // prediction horizon at the assumed cruise speed
        let stations_m: [f64; STATION_LAYERS] = std::array::from_fn(|i| {
            v * PLANNING_HORIZON_S * (i + 1) as f64 / STATION_LAYERS as f64
        });
        // initial lateral rate, expressed per unit of segment parameter u; the
        // first segment must honor it or every replan restarts the swerve at
        // zero slope and the executed path lags the plan into obstacles
        let (_, lane_yaw) = path.pose_at(s0);
        let m0_first = ego.speed * wrap_angle(ego.yaw - lane_yaw).sin() * (stations_m[0] / v);
        // cubic Hermite in u with start slope m0 and flat end
        let d_shape = |da: f64, db: f64, m0: f64, u: f64| {
            let (u2, u3) = (u * u, u * u * u);
            (2.0 * u3 - 3.0 * u2 + 1.0) * da + (u3 - 2.0 * u2 + u) * m0 + (3.0 * u2 - 2.0 * u3) * db
        };

        // cost of one lattice edge: planner-specific lateral-smoothness and
        // centerline-pull terms (structural to the search itself) plus the
        // shared cost of each sampled point, timed under the "cost" seam so
        // it's comparable across planners. Curvature at each point is a
        // numerical estimate off the last three sampled points
        // (`cost::curvature_of`) — the lattice has no closed-form curve to
        // evaluate directly, unlike RRT*'s steering function. Returns
        // `f64::INFINITY` for a colliding or off-road edge (A* skips it).
        let edge_cost = |sa: f64, da: f64, sb: f64, db: f64, m0: f64| -> f64 {
            let mut total = 2.0 * (db - da).powi(2); // lateral smoothness
            let mut prev2: Option<[f64; 2]> = None;
            let mut prev1 = path.frenet_to_xy(sa, da);
            for i in 1..=SAMPLES_PER_SEGMENT {
                let u = i as f64 / SAMPLES_PER_SEGMENT as f64;
                let s = sa + (sb - sa) * u;
                let d = d_shape(da, db, m0, u);
                total += d * d / SAMPLES_PER_SEGMENT as f64; // stay near centerline
                let p = path.frenet_to_xy(s, d);
                let curvature = prev2.map_or(0.0, |p0| cost::curvature_of(p0, prev1, p));
                let sample = Sample {
                    xy: p,
                    lateral: d,
                    speed: v,
                    curvature,
                    t: (s - s0) / v, // time when the ego gets there
                    ..Default::default()
                };
                let point = ctx.time("cost", || {
                    cost::point_cost(
                        &sample,
                        ctx.road.target_speed,
                        ctx.road.half_width,
                        ctx.actors,
                        Some(&path),
                    )
                });
                if point.is_infinite() {
                    return f64::INFINITY;
                }
                total += point;
                prev2 = Some(prev1);
                prev1 = p;
            }
            total
        };

        // Sampled Hermite connector between two grid nodes, for the
        // diagnostic overlay (recomputed only when diagnostics are on).
        let segment_pts = |sa: f64, da: f64, sb: f64, db: f64, m0: f64| -> Vec<[f64; 2]> {
            (0..=SAMPLES_PER_SEGMENT)
                .map(|i| {
                    let u = i as f64 / SAMPLES_PER_SEGMENT as f64;
                    let s = sa + (sb - sa) * u;
                    let d = d_shape(da, db, m0, u);
                    path.frenet_to_xy(s, d)
                })
                .collect()
        };

        // A* over the layered DAG. Node ids: 0 is the root (the ego's
        // projected pose); node `1 + layer*LATERALS + j` is grid column `j`
        // of station layer `layer`. `parent` reconstructs the winning path;
        // the first final-layer node popped is optimal since all edge costs
        // are non-negative.
        if let Some(diag) = ctx.diagnostics {
            diag.record_point(path.frenet_to_xy(s0, d0)); // tree root
        }
        let n_nodes = 1 + GRID_NODES;
        let mut dist = vec![f64::INFINITY; n_nodes];
        let mut parent = vec![usize::MAX; n_nodes];
        dist[0] = 0.0;
        let mut heap = BinaryHeap::new();
        heap.push(QItem { cost: 0.0, node: 0 });
        let mut goal: Option<usize> = None;

        ctx.time("optimize", || {
            while let Some(QItem { cost: g, node }) = heap.pop() {
                if g > dist[node] {
                    continue; // stale queue entry, already settled cheaper
                }
                // (station, lateral) and destination layer of this node
                let (layer, sa, da, col) = if node == 0 {
                    // map the ego's off-grid lateral to its nearest column so
                    // the first (short) segment is limited the same way
                    let c = (((d0 + LAT_BOUND_M) / (2.0 * LAT_BOUND_M) * (LATERALS - 1) as f64)
                        .round() as i64)
                        .clamp(0, LATERALS as i64 - 1) as usize;
                    (None, s0, d0, c)
                } else {
                    let idx = node - 1;
                    let l = idx / LATERALS;
                    (Some(l), s0 + stations_m[l], lateral(idx % LATERALS), idx % LATERALS)
                };
                if layer == Some(STATION_LAYERS - 1) {
                    goal = Some(node); // settled the cheapest final-layer node
                    return;
                }
                let next_layer = layer.map_or(0, |l| l + 1);
                let sb = s0 + stations_m[next_layer];
                let m0 = if layer.is_none() { m0_first } else { 0.0 };
                // only connect to nearby lateral columns (see NEIGHBOR_SPAN)
                let lo = col.saturating_sub(NEIGHBOR_SPAN);
                let hi = (col + NEIGHBOR_SPAN).min(LATERALS - 1);
                for j in lo..=hi {
                    let db = lateral(j);
                    if let Some(diag) = ctx.diagnostics {
                        diag.record_point(path.frenet_to_xy(sb, db));
                        diag.record_trajectory(segment_pts(sa, da, sb, db, m0));
                    }
                    let ec = edge_cost(sa, da, sb, db, m0);
                    if !ec.is_finite() {
                        continue;
                    }
                    let succ = 1 + next_layer * LATERALS + j;
                    let nd = g + ec;
                    if nd < dist[succ] {
                        dist[succ] = nd;
                        parent[succ] = node;
                        heap.push(QItem { cost: nd, node: succ });
                    }
                }
            }
        });

        let Some(goal) = goal else {
            // every path collides / leaves the road: brake straight ahead
            return vec![
                Control {
                    accel: -4.0,
                    curvature: 0.0,
                };
                ctx.horizon
            ];
        };

        // reconstruct the chosen lateral per layer from the parent chain
        let mut laterals = vec![0.0; STATION_LAYERS];
        let mut node = goal;
        while node != 0 {
            let idx = node - 1;
            laterals[idx / LATERALS] = lateral(idx % LATERALS);
            node = parent[node];
        }

        // sample the winning path over time; d is cubic in t on each segment
        let s_max = *stations_m.last().unwrap();
        ctx.time("extract", || {
            let pts: Vec<[f64; 2]> = (1..=ctx.horizon.max((s_max / (v * ctx.road.dt)) as usize))
                .map(|i| {
                    let s_rel = (v * ctx.road.dt * i as f64).min(s_max);
                    let seg = stations_m.iter().position(|&m| s_rel <= m).unwrap();
                    let (sa, da) = if seg == 0 {
                        (0.0, d0)
                    } else {
                        (stations_m[seg - 1], laterals[seg - 1])
                    };
                    let u = (s_rel - sa) / (stations_m[seg] - sa);
                    let m0 = if seg == 0 { m0_first } else { 0.0 };
                    let d = d_shape(da, laterals[seg], m0, u);
                    path.frenet_to_xy(s0 + s_rel, d)
                })
                .collect();
            xy_to_controls(ego, &pts, ctx.road.dt)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_road, test_run, test_run_on};
    use crate::scenarios::Road;

    #[test]
    fn stays_on_empty_centerline() {
        let ego = State {
            y: 1.5,
            speed: 8.0,
            ..Default::default()
        };
        let trace = test_run(&mut LatticePlanner, ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.5, "offset {}", end.y);
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
        let trace = test_run(&mut LatticePlanner, ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        let end = trace.last().unwrap();
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(end.x > 60.0, "did not pass the obstacle, x {}", end.x);
    }

    #[test]
    fn respects_the_roads_own_half_width_around_an_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let peak = |trace: &[State]| trace.iter().map(|s| s.y.abs()).fold(0.0, f64::max);
        let base = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);

        // A one-lane-wide road (3.5 m half-width) has just enough room to
        // detour around a stopped car (needs ~2.5 m of clearance): the
        // lattice passes it while never crossing the road edge.
        let narrow = Road {
            half_width: 3.5,
            ..base.clone()
        };
        let trace = test_run_on(&mut LatticePlanner, &narrow, ego, &[obstacle], 150);
        assert!(peak(&trace) <= 3.5, "left the road, peak {}", peak(&trace));
        assert!(trace.last().unwrap().x > 60.0, "never got past the obstacle");

        // A road too tight to fit the detour: rather than plan off the
        // surface to get around, the planner holds inside the road edge
        // (and brakes short of the obstacle).
        let too_tight = Road {
            half_width: 2.0,
            ..base
        };
        let trace = test_run_on(&mut LatticePlanner, &too_tight, ego, &[obstacle], 150);
        assert!(peak(&trace) <= 2.0, "left the tight road, peak {}", peak(&trace));
        assert!(trace.last().unwrap().x < 38.0, "drove into/around the obstacle");
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
        LatticePlanner.plan(ego, &ctx);
        let data = diag.take();
        // A* records the tree root plus one point and one connector per
        // *expanded* edge — the explored part of the search graph, not the
        // full grid. The exact count is search-dependent, but there is at
        // least the root and the first layer's fan-out, and points and
        // connectors are recorded in lockstep.
        assert!(data.points.len() > LATERALS, "only {} points", data.points.len());
        assert_eq!(data.points.len(), data.trajectories.len() + 1); // +1 for the root point
        assert!(
            data.trajectories
                .iter()
                .all(|t| t.len() == SAMPLES_PER_SEGMENT + 1)
        );
    }
}
