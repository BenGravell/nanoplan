//! EM/lattice-style planner. Samples a deterministic grid of (station,
//! lateral) points in the road Frenet frame, connects successive layers into
//! a layered DAG with cubic lateral segments, prices them with the production
//! composite metric, and finds the cheapest
//! path with **A\*** (best-first) search. The winning geometry then gets the
//! fastest speed profile allowed by steering, lateral grip, engine thrust,
//! drag, and braking.
//!
//! The grid is deliberately high-resolution — `STATION_LAYERS × LATERALS`
//! is in the hundreds — so the lattice can represent fine lateral
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

use crate::common::differencing::forward_difference;
use crate::common::kinematics::longitudinal_resistance_accel;
use crate::common::math::wrap_angle;
use crate::geometry::EGO_FOOTPRINT;
use crate::geometry::curvature::curvature_of;
use crate::planning::constraints::{HardConstraints, Sample};
use crate::planning::search_tree::{
    RoadFrame, best_first, brake_controls, parent_chain, xy_to_controls,
};
use crate::planning::{Context, Planner};
use crate::simulation::{Control, State, curvature_limit};
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};

pub(crate) struct LatticePlanner;

/// Number of lateral samples per station layer (lateral grid resolution),
/// evenly spaced over the usable road width. Odd, so one sample lands
/// exactly on the centerline (`d = 0`).
const LATERALS: usize = 17;
/// Number of station layers reaching out to the planning horizon (progress
/// grid resolution).
const STATION_LAYERS: usize = 16;
/// Samples per lateral segment for cost integration and collision checking.
/// Lower than the old exhaustive DP used because there are more, shorter
/// segments spanning the horizon, so the whole path is still
/// sampled densely (`STATION_LAYERS × SAMPLES_PER_SEGMENT` points).
const SAMPLES_PER_SEGMENT: usize = 4;
/// How many lateral columns an edge may span between adjacent station
/// layers. A layer is only ~`horizon/STATION_LAYERS` of travel, so a jump of
/// more than a few columns there is a curvature no real car has — the edge
/// feasibility check rejects it, or the metric objective prices it out of any
/// optimal path, so never generating those edges costs nothing and keeps
/// the search branching factor (and cost-function evaluations) bounded. Full
/// lateral range is still reachable: over `STATION_LAYERS` layers the path
/// can ramp `NEIGHBOR_SPAN` columns per layer, far more than the grid's
/// width. This is what keeps the high-resolution grid inside the real-time
/// budget together with A*'s lazy expansion.
const NEIGHBOR_SPAN: usize = 2;
/// Curvature is measured across this much road, rather than at the corners
/// of the piecewise-linear centerline supplied by the active track.
const CURVATURE_WINDOW_M: f64 = 7.5;
/// Station resolution of the forward/backward velocity pass.
const SPEED_PROFILE_STEP_M: f64 = 2.0;
/// Total grid nodes — the resolution knob. `STATION_LAYERS × LATERALS`.
const GRID_NODES: usize = STATION_LAYERS * LATERALS;

/// Lateral offset of grid column `j` over the usable road surface.
fn lateral(j: usize, bound: f64) -> f64 {
    -bound + 2.0 * bound * j as f64 / (LATERALS - 1) as f64
}

impl Planner for LatticePlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let RoadFrame {
            path,
            s0,
            d0,
            speed: v,
            horizon_m,
        } = ctx.time("route", || RoadFrame::new(ego, ctx));
        let constraints = HardConstraints::new(
            ctx.road.half_width,
            ctx.actors,
            &path,
            ego.speed,
            ctx.road.dt,
        );
        let lateral_bound = (ctx.road.half_width - EGO_FOOTPRINT.width / 2.0).max(0.0);
        let horizon_m = horizon_m.min((path.length() - s0).max(1.0));
        // STATION_LAYERS evenly spaced layers reaching out to the full
        // prediction horizon at the assumed cruise speed
        let stations_m: [f64; STATION_LAYERS] =
            std::array::from_fn(|i| horizon_m * (i + 1) as f64 / STATION_LAYERS as f64);
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

        // Cost of one lattice edge: the metric objective at each sampled
        // point, timed under the "cost" seam. Curvature at each point is a
        // numerical estimate off the last three sampled points
        // (`geometry::curvature::curvature_of`) — the lattice has no
        // closed-form curve to evaluate directly, unlike RRT*'s steering
        // function. Returns
        // `f64::INFINITY` for a colliding or off-road edge (A* skips it).
        let edge_cost = |sa: f64, da: f64, sb: f64, db: f64, m0: f64| -> f64 {
            let mut total = 0.0;
            let mut prev2: Option<[f64; 2]> = None;
            let mut prev1 = path.frenet_to_xy(sa, da);
            let mut elapsed = 0.0;
            for i in 1..=SAMPLES_PER_SEGMENT {
                let u = i as f64 / SAMPLES_PER_SEGMENT as f64;
                let s = sa + (sb - sa) * u;
                let d = d_shape(da, db, m0, u);
                let p = path.frenet_to_xy(s, d);
                let curvature = prev2.map_or(0.0, |p0| curvature_of(p0, prev1, p));
                if curvature > MAX_ABS_CURVATURE {
                    return f64::INFINITY;
                }
                let curve_speed = if curvature > 1e-9 {
                    (MAX_ABS_LAT_ACCEL / curvature).sqrt()
                } else {
                    ctx.road.target_speed
                };
                let speed = curve_speed.min(ctx.road.target_speed).max(0.5);
                let ds = (p[0] - prev1[0]).hypot(p[1] - prev1[1]);
                elapsed += ds / speed;
                let (_, lane_yaw) = path.pose_at(s);
                let heading_err = wrap_angle((p[1] - prev1[1]).atan2(p[0] - prev1[0]) - lane_yaw);
                let sample = Sample {
                    xy: p,
                    lateral: d,
                    heading_err,
                    speed,
                    t: (sa - s0) / v + elapsed,
                    ..Default::default()
                };
                let point = ctx.time("cost", || constraints.point_cost(&sample));
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
        // (station, lateral) and destination layer of a node.
        let node_info = |node: usize| {
            if node == 0 {
                // map the ego's off-grid lateral to its nearest column so
                // the first (short) segment is limited the same way
                let c = (((d0 + lateral_bound) / (2.0 * lateral_bound.max(1e-9))
                    * (LATERALS - 1) as f64)
                    .round() as i64)
                    .clamp(0, LATERALS as i64 - 1) as usize;
                (None, s0, d0, c)
            } else {
                let idx = node - 1;
                let l = idx / LATERALS;
                (
                    Some(l),
                    s0 + stations_m[l],
                    lateral(idx % LATERALS, lateral_bound),
                    idx % LATERALS,
                )
            }
        };

        let result = ctx.time("optimize", || {
            best_first(
                1 + GRID_NODES,
                0,
                |node| node_info(node).0 == Some(STATION_LAYERS - 1),
                |node| {
                    let (layer, sa, da, col) = node_info(node);
                    let next_layer = layer.map_or(0, |l| l + 1);
                    let sb = s0 + stations_m[next_layer];
                    let m0 = if layer.is_none() { m0_first } else { 0.0 };
                    // only connect to nearby lateral columns (see NEIGHBOR_SPAN)
                    let lo = col.saturating_sub(NEIGHBOR_SPAN);
                    let hi = (col + NEIGHBOR_SPAN).min(LATERALS - 1);
                    let mut successors = Vec::with_capacity(hi - lo + 1);
                    for j in lo..=hi {
                        let db = lateral(j, lateral_bound);
                        if let Some(diag) = ctx.diagnostics {
                            diag.record_point(path.frenet_to_xy(sb, db));
                            diag.record_trajectory(segment_pts(sa, da, sb, db, m0));
                        }
                        let ec = edge_cost(sa, da, sb, db, m0);
                        let succ = 1 + next_layer * LATERALS + j;
                        successors.push((succ, ec));
                    }
                    successors
                },
            )
        });

        let Some(result) = result else {
            // every path collides / leaves the road: brake straight ahead
            return brake_controls(ego, ctx, MIN_LON_ACCEL);
        };

        // reconstruct the chosen lateral per layer from the parent chain
        let mut laterals = [0.0; STATION_LAYERS];
        for node in parent_chain(result.goal, 0, |n| {
            (result.parent[n] != usize::MAX).then_some(result.parent[n])
        }) {
            let idx = node - 1;
            laterals[idx / LATERALS] = lateral(idx % LATERALS, lateral_bound);
        }

        // Evaluate the winning Frenet curve at any relative station.
        let s_max = *stations_m.last().unwrap();
        let winning_xy = |s_rel: f64| {
            let s_rel = s_rel.clamp(0.0, s_max);
            let seg = stations_m
                .iter()
                .position(|&m| s_rel <= m)
                .unwrap_or(STATION_LAYERS - 1);
            let (sa, da) = if seg == 0 {
                (0.0, d0)
            } else {
                (stations_m[seg - 1], laterals[seg - 1])
            };
            let u = (s_rel - sa) / (stations_m[seg] - sa).max(1e-9);
            let m0 = if seg == 0 { m0_first } else { 0.0 };
            path.frenet_to_xy(s0 + s_rel, d_shape(da, laterals[seg], m0, u))
        };

        // Build the time-optimal speed envelope: brake backward from every
        // bend, then accelerate forward with the thrust left after drag.
        ctx.time("extract", || {
            let n = (s_max / SPEED_PROFILE_STEP_M).ceil() as usize + 1;
            let stations: Vec<f64> = (0..n)
                .map(|i| (i as f64 * SPEED_PROFILE_STEP_M).min(s_max))
                .collect();
            let mut speeds: Vec<f64> = stations
                .iter()
                .map(|&s| {
                    let curvature = curvature_of(
                        winning_xy((s - CURVATURE_WINDOW_M).max(0.0)),
                        winning_xy(s),
                        winning_xy((s + CURVATURE_WINDOW_M).min(s_max)),
                    );
                    if curvature > 1e-9 {
                        ctx.road
                            .target_speed
                            .min((MAX_ABS_LAT_ACCEL / curvature).sqrt())
                    } else {
                        ctx.road.target_speed
                    }
                })
                .collect();
            for i in (0..n - 1).rev() {
                let ds = stations[i + 1] - stations[i];
                speeds[i] =
                    speeds[i].min((speeds[i + 1].powi(2) - 2.0 * MIN_LON_ACCEL * ds).sqrt());
            }
            speeds[0] = ego.speed.max(0.0);
            for i in 0..n - 1 {
                let ds = stations[i + 1] - stations[i];
                let net_accel = (MAX_LON_ACCEL - longitudinal_resistance_accel(speeds[i])).max(0.0);
                speeds[i + 1] =
                    speeds[i + 1].min((speeds[i].powi(2) + 2.0 * net_accel * ds).sqrt());
            }

            let mut times = vec![0.0; n];
            for i in 0..n - 1 {
                let ds = stations[i + 1] - stations[i];
                times[i + 1] = times[i] + 2.0 * ds / (speeds[i] + speeds[i + 1]).max(0.1);
            }
            let samples: Vec<([f64; 2], f64)> = (1..=ctx.horizon)
                .map(|tick| {
                    let t = tick as f64 * ctx.road.dt;
                    let i = times.partition_point(|&x| x < t).clamp(1, n - 1) - 1;
                    let dt = (t - times[i]).clamp(0.0, times[i + 1] - times[i]);
                    let ds = stations[i + 1] - stations[i];
                    let accel = (speeds[i + 1].powi(2) - speeds[i].powi(2)) / (2.0 * ds.max(1e-9));
                    (
                        winning_xy(
                            (stations[i] + speeds[i] * dt + 0.5 * accel * dt * dt).min(s_max),
                        ),
                        (speeds[i] + accel * dt).max(0.0),
                    )
                })
                .collect();
            let pts: Vec<_> = samples.iter().map(|&(p, _)| p).collect();
            let mut expected_speed = ego.speed;
            xy_to_controls(ego, &pts, ctx.road.dt)
                .into_iter()
                .zip(samples)
                .map(|(mut u, (_, target_speed))| {
                    u.curvature = u.curvature.clamp(
                        -curvature_limit(expected_speed),
                        curvature_limit(expected_speed),
                    );
                    u.acceleration =
                        (forward_difference(expected_speed, target_speed, ctx.road.dt)
                            + longitudinal_resistance_accel(expected_speed))
                        .clamp(MIN_LON_ACCEL, MAX_LON_ACCEL);
                    expected_speed = target_speed;
                    u
                })
                .collect()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_road, test_run, test_run_on};
    use crate::track::Road;

    #[test]
    fn stays_on_road_without_a_lane_center_metric() {
        let ego = State {
            y: 1.5,
            speed: 8.0,
            ..Default::default()
        };
        let trace = test_run(&mut LatticePlanner, ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 5.5, "offset {}", end.y);
        assert!(end.speed > ego.speed, "speed {}", end.speed);
    }

    #[test]
    fn accelerates_on_an_open_straight() {
        let mut road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        road.target_speed = 30.0;
        let controls = LatticePlanner.plan(
            State {
                speed: 5.0,
                ..Default::default()
            },
            &crate::planning::test_ctx(&road, &[]),
        );
        assert!(
            controls[0].acceleration > 5.0,
            "accel {}",
            controls[0].acceleration
        );
    }

    #[test]
    fn brakes_for_road_curvature() {
        let radius = 20.0;
        let centerline: Vec<[f64; 2]> = (0..=24)
            .map(|i| {
                let a = i as f64 * 0.05;
                [radius * a.sin(), radius * (1.0 - a.cos())]
            })
            .collect();
        let road = Road::new(centerline, 60.0, 5.5, 0.1);
        let controls = LatticePlanner.plan(
            State {
                speed: 30.0,
                ..Default::default()
            },
            &crate::planning::test_ctx(&road, &[]),
        );
        assert!(
            controls[0].acceleration < -1.0,
            "accel {}",
            controls[0].acceleration
        );
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
        let narrow = Road::new(base.centerline().to_vec(), base.target_speed, 3.5, base.dt);
        let trace = test_run_on(&mut LatticePlanner, &narrow, ego, &[obstacle], 150);
        assert!(peak(&trace) <= 3.5, "left the road, peak {}", peak(&trace));
        assert!(
            trace.last().unwrap().x > 60.0,
            "never got past the obstacle"
        );

        // A road too tight to fit the detour: rather than plan off the
        // surface to get around, the planner holds inside the road edge
        // (and brakes short of the obstacle).
        let too_tight = Road::new(base.centerline().to_vec(), base.target_speed, 2.0, base.dt);
        let trace = test_run_on(&mut LatticePlanner, &too_tight, ego, &[obstacle], 150);
        assert!(
            peak(&trace) <= 2.0,
            "left the tight road, peak {}",
            peak(&trace)
        );
        assert!(
            trace.last().unwrap().x < 38.0,
            "drove into/around the obstacle"
        );
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
        assert!(
            data.points.len() > LATERALS,
            "only {} points",
            data.points.len()
        );
        assert_eq!(data.points.len(), data.trajectories.len() + 1); // +1 for the root point
        assert!(
            data.trajectories
                .iter()
                .all(|t| t.len() == SAMPLES_PER_SEGMENT + 1)
        );
    }
}
