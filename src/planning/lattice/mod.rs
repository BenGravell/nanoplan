//! Sparse Frenet space-time lattice.
//!
//! Nodes sample `(station, lateral, speed)` at one-second time layers.  An
//! edge is already a timed ten-tick trajectory, so the winning A* chain
//! directly supplies acceleration and curvature controls; there is no
//! separate speed profiler.

use std::cell::{Cell, RefCell};

use crate::common::differencing::forward_difference;
use crate::common::kinematics::{
    LOW_SPEED_LIMIT_MPS, commanded_accel_for_net, curvature_limit, lateral_acceleration,
    net_longitudinal_accel,
};
use crate::common::math::wrap_angle;
use crate::geometry::EGO_FOOTPRINT;
use crate::geometry::barrier::collide_with_road_barriers;
use crate::planning::constraints::{HardConstraints, Sample};
use crate::planning::search_tree::{
    RoadFrame, best_first, brake_controls, parent_chain, repeat_last_controls,
};
use crate::planning::{Context, PLANNING_DT_S, PLANNING_TICKS, Planner};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

pub(crate) struct LatticePlanner;

const TICKS_PER_EDGE: usize = 10;
const _: () = assert!(PLANNING_TICKS.is_multiple_of(TICKS_PER_EDGE));
const TIME_LAYERS: usize = PLANNING_TICKS / TICKS_PER_EDGE;

const S_BREAKPOINTS: usize = 5;
const D_BREAKPOINTS: usize = 8;
const V_BREAKPOINTS: usize = 5;
const GRID_NODES: usize = TIME_LAYERS * S_BREAKPOINTS * D_BREAKPOINTS * V_BREAKPOINTS;
const MAX_EVALUATED_SEGMENTS: usize = 1_000;

const ENDPOINT_TOLERANCE_M: f64 = 0.75;
const CONTROL_FEASIBILITY_TOLERANCE: f64 = 0.01;

#[derive(Clone, Copy)]
struct Reach {
    min_s: f64,
    max_s: f64,
    min_v: f64,
    max_v: f64,
}

struct Segment {
    controls: Vec<Control>,
    samples: Vec<Sample>,
    points: Vec<[f64; 2]>,
}

fn level(i: usize, n: usize, lo: f64, hi: f64) -> f64 {
    if n == 1 {
        return 0.5 * (lo + hi);
    }
    lo + (hi - lo) * i as f64 / (n - 1) as f64
}

fn nearest_level(value: f64, n: usize, lo: f64, hi: f64) -> usize {
    if hi - lo <= 1e-9 {
        return 0;
    }
    (((value - lo) / (hi - lo) * (n - 1) as f64).round() as isize).clamp(0, n as isize - 1) as usize
}

fn neighborhood(i: usize, n: usize) -> std::ops::RangeInclusive<usize> {
    i.saturating_sub(1)..=(i + 1).min(n - 1)
}

fn hermite(a: f64, b: f64, da: f64, db: f64, u: f64) -> f64 {
    let (u2, u3) = (u * u, u * u * u);
    (2.0 * u3 - 3.0 * u2 + 1.0) * a
        + (u3 - 2.0 * u2 + u) * da
        + (-2.0 * u3 + 3.0 * u2) * b
        + (u3 - u2) * db
}

fn hermite_derivative(a: f64, b: f64, da: f64, db: f64, u: f64) -> f64 {
    let u2 = u * u;
    (6.0 * u2 - 6.0 * u) * a
        + (3.0 * u2 - 4.0 * u + 1.0) * da
        + (-6.0 * u2 + 6.0 * u) * b
        + (3.0 * u2 - 2.0 * u) * db
}

fn reachable(ego_speed: f64, dt: f64) -> [Reach; TIME_LAYERS] {
    let mut min_s = 0.0;
    let mut max_s = 0.0;
    let mut min_v = ego_speed.max(0.0);
    let mut max_v = min_v;
    std::array::from_fn(|_| {
        for _ in 0..TICKS_PER_EDGE {
            let next_min_v = (min_v + net_longitudinal_accel(MIN_LON_ACCEL, min_v) * dt).max(0.0);
            let next_max_v = (max_v + net_longitudinal_accel(MAX_LON_ACCEL, max_v) * dt).max(0.0);
            min_s += 0.5 * (min_v + next_min_v) * dt;
            max_s += 0.5 * (max_v + next_max_v) * dt;
            min_v = next_min_v;
            max_v = next_max_v;
        }
        Reach {
            min_s,
            max_s,
            min_v,
            max_v,
        }
    })
}

fn node_id(layer: usize, si: usize, di: usize, vi: usize) -> usize {
    1 + (((layer * S_BREAKPOINTS + si) * D_BREAKPOINTS + di) * V_BREAKPOINTS + vi)
}

fn decode_node(node: usize) -> (usize, usize, usize, usize) {
    let mut i = node - 1;
    let vi = i % V_BREAKPOINTS;
    i /= V_BREAKPOINTS;
    let di = i % D_BREAKPOINTS;
    i /= D_BREAKPOINTS;
    let si = i % S_BREAKPOINTS;
    (i / S_BREAKPOINTS, si, di, vi)
}

fn lateral(path_s: f64, di: usize, ctx: &Context) -> f64 {
    let (right, left) = ctx.road.lateral_bounds_at(path_s);
    let margin = EGO_FOOTPRINT.width / 2.0;
    let lo = (right + margin).min(0.0);
    let hi = (left - margin).max(0.0);
    // Keep an exact centerline sample even with eight lateral breakpoints.
    // The remaining positive-side samples use the otherwise duplicated zero
    // slot, retaining the finer spacing that makes high-speed setup feasible.
    if D_BREAKPOINTS == 8 && lo < 0.0 && hi > 0.0 {
        if di < 4 {
            level(di, 4, lo, 0.0)
        } else {
            hi * (di - 3) as f64 / 4.0
        }
    } else {
        level(di, D_BREAKPOINTS, lo, hi)
    }
}

#[allow(clippy::too_many_arguments)]
fn segment(
    path: &Path,
    ctx: &Context,
    start: State,
    s0: f64,
    d0: f64,
    v0: f64,
    s1: f64,
    d1: f64,
    v1: f64,
    start_time: f64,
) -> Option<Segment> {
    let dt = ctx.road.dt;
    let duration = TICKS_PER_EDGE as f64 * dt;
    let mut targets = Vec::with_capacity(TICKS_PER_EDGE);
    let mut previous_target = [start.x, start.y];

    for tick in 1..=TICKS_PER_EDGE {
        let u = tick as f64 / TICKS_PER_EDGE as f64;
        let s = hermite(s0, s1, v0 * duration, v1 * duration, u);
        let d = hermite(d0, d1, 0.0, 0.0, u);
        let s_dot = hermite_derivative(s0, s1, v0 * duration, v1 * duration, u) / duration;
        let d_dot = hermite_derivative(d0, d1, 0.0, 0.0, u) / duration;
        let xy = path.frenet_to_xy(s, d);
        let target_yaw = if (xy[0] - previous_target[0]).hypot(xy[1] - previous_target[1]) > 1e-6 {
            (xy[1] - previous_target[1]).atan2(xy[0] - previous_target[0])
        } else {
            start.yaw
        };
        targets.push((s, s_dot.hypot(d_dot), target_yaw, xy));
        previous_target = xy;
    }

    let mut controls = Vec::with_capacity(TICKS_PER_EDGE);
    let mut samples = Vec::with_capacity(TICKS_PER_EDGE);
    let mut points = Vec::with_capacity(TICKS_PER_EDGE);
    let mut actual = start;
    let mut previous_dynamics: Option<(Control, f64)> = None;

    // Kinematic feasibility is completed for the whole edge before any
    // metric call is made.
    for (tick, &(target_s, target_speed, target_yaw, _)) in targets.iter().enumerate() {
        let acceleration = commanded_accel_for_net(
            forward_difference(actual.speed, target_speed, dt),
            actual.speed,
        );
        let curvature = if actual.speed > LOW_SPEED_LIMIT_MPS {
            wrap_angle(target_yaw - actual.yaw) / (actual.speed * dt)
        } else {
            0.0
        };
        if !acceleration.is_finite()
            || !curvature.is_finite()
            || !(MIN_LON_ACCEL - CONTROL_FEASIBILITY_TOLERANCE
                ..=MAX_LON_ACCEL + CONTROL_FEASIBILITY_TOLERANCE)
                .contains(&acceleration)
            || curvature.abs() > curvature_limit(actual.speed) + CONTROL_FEASIBILITY_TOLERANCE
        {
            return None;
        }
        let control = Control {
            acceleration,
            curvature,
        };
        let before = actual;
        actual = world_step(actual, control, dt);
        let (actual_s, actual_d) = path.project_near(actual.position(), target_s, 30.0);
        let (right, left) = ctx.road.lateral_bounds_at(actual_s);
        let approximate_margin = EGO_FOOTPRINT.width / 2.0;
        let off_road = if start_time == 0.0 {
            collide_with_road_barriers(before, actual, EGO_FOOTPRINT, ctx.road) != actual
        } else {
            actual_d < right + approximate_margin || actual_d > left - approximate_margin
        };
        if off_road {
            return None;
        }
        let (s, d) = (actual_s, actual_d);
        let xy = [actual.x, actual.y];
        let lat_accel = lateral_acceleration(actual.speed, control.curvature);
        let (_, lane_yaw) = path.pose_at(s);
        let curvature_window = 2.0;
        let s_lo = (s - curvature_window).max(0.0);
        let s_hi = (s + curvature_window).min(path.length());
        let (_, yaw_lo) = path.pose_at(s_lo);
        let (_, yaw_hi) = path.pose_at(s_hi);
        let lane_curvature = wrap_angle(yaw_hi - yaw_lo) / (s_hi - s_lo).max(1e-9);
        let heading_err = wrap_angle(actual.yaw - lane_yaw);
        let station_speed = actual.speed * heading_err.cos() / (1.0 - lane_curvature * d).max(0.1);
        let (lon_jerk, lat_jerk) = previous_dynamics.map_or((0.0, 0.0), |(previous, lat)| {
            (
                forward_difference(previous.acceleration, acceleration, dt),
                forward_difference(lat, lat_accel, dt),
            )
        });
        samples.push(Sample {
            xy,
            lateral: d,
            road_bounds: Some((right, left)),
            heading_err,
            speed: actual.speed,
            station_speed: Some(station_speed),
            lon_jerk,
            lat_jerk,
            t: start_time + (tick + 1) as f64 * dt,
        });
        points.push(xy);
        controls.push(control);
        previous_dynamics = Some((control, lat_accel));
    }

    let last = points.last().copied()?;
    let target = path.frenet_to_xy(s1, d1);
    let endpoint_tolerance = ENDPOINT_TOLERANCE_M + v1 * dt;
    if (last[0] - target[0]).hypot(last[1] - target[1]) > endpoint_tolerance
        || (actual.speed - v1).abs() > 2.0
    {
        return None;
    }

    Some(Segment {
        controls,
        samples,
        points,
    })
}

impl Planner for LatticePlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        debug_assert!((ctx.road.dt - PLANNING_DT_S).abs() < 1e-9);
        let RoadFrame { path, s0, d0, .. } = ctx.time("route", || RoadFrame::new(ego, ctx));
        let reach = reachable(ego.speed, ctx.road.dt);
        let constraints = HardConstraints::new(
            ctx.road.half_width,
            ctx.actors,
            &path,
            ego.speed,
            ctx.road.dt,
        );
        let evaluated = Cell::new(0usize);
        let best_root_segment: RefCell<Option<(f64, Vec<Control>)>> = RefCell::new(None);

        let node_state = |node: usize| {
            let (layer, si, di, vi) = decode_node(node);
            let r = reach[layer];
            let s = (s0 + level(si, S_BREAKPOINTS, r.min_s, r.max_s)).min(path.length() - 1e-6);
            let d = lateral(s, di, ctx);
            let v = level(vi, V_BREAKPOINTS, r.min_v, r.max_v);
            (layer, si, di, vi, s, d, v)
        };

        let edge = |start: State,
                    sa: f64,
                    da: f64,
                    va: f64,
                    sb: f64,
                    db: f64,
                    vb: f64,
                    layer: usize|
         -> Option<(f64, Segment)> {
            if evaluated.get() >= MAX_EVALUATED_SEGMENTS {
                return None;
            }
            evaluated.set(evaluated.get() + 1);
            let segment = segment(&path, ctx, start, sa, da, va, sb, db, vb, layer as f64)?;
            let mut cost = 0.0;
            for sample in &segment.samples {
                let point = ctx.time("cost", || constraints.point_cost(sample));
                if !point.is_finite() {
                    return None;
                }
                cost += point * ctx.road.dt;
            }
            if layer == 0 {
                let mut best = best_root_segment.borrow_mut();
                if best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost) {
                    *best = Some((cost, segment.controls.clone()));
                }
            }
            Some((cost, segment))
        };

        let result = ctx.time("optimize", || {
            best_first(
                1 + GRID_NODES,
                0,
                |node| node != 0 && decode_node(node).0 == TIME_LAYERS - 1,
                |node| {
                    let (layer, _si, di, vi, sa, da, va, start) = if node == 0 {
                        (usize::MAX, 0, 0, 0, s0, d0, ego.speed.max(0.0), ego)
                    } else {
                        let (layer, si, di, vi, s, d, v) = node_state(node);
                        let (xy, yaw) = path.pose_at(s);
                        (
                            layer,
                            si,
                            di,
                            vi,
                            s,
                            d,
                            v,
                            State {
                                x: xy[0] - d * yaw.sin(),
                                y: xy[1] + d * yaw.cos(),
                                yaw,
                                speed: v,
                            },
                        )
                    };
                    let next_layer = if node == 0 { 0 } else { layer + 1 };
                    if next_layer >= TIME_LAYERS {
                        return Vec::new();
                    }
                    let r = reach[next_layer];
                    let v_indices: Vec<usize> = if node == 0 {
                        (0..V_BREAKPOINTS).collect()
                    } else {
                        neighborhood(vi, V_BREAKPOINTS).collect()
                    };
                    let d_indices: Vec<usize> = if node == 0 {
                        (0..D_BREAKPOINTS).collect()
                    } else {
                        neighborhood(di, D_BREAKPOINTS).collect()
                    };
                    let mut successors = Vec::new();
                    for next_vi in v_indices {
                        let vb = level(next_vi, V_BREAKPOINTS, r.min_v, r.max_v);
                        let predicted_s = sa + 0.5 * (va + vb);
                        let center_si =
                            nearest_level(predicted_s - s0, S_BREAKPOINTS, r.min_s, r.max_s);
                        for next_si in std::iter::once(center_si) {
                            let sb = (s0 + level(next_si, S_BREAKPOINTS, r.min_s, r.max_s))
                                .min(path.length() - 1e-6);
                            for &next_di in &d_indices {
                                let db = lateral(sb, next_di, ctx);
                                let Some((cost, segment)) =
                                    edge(start, sa, da, va, sb, db, vb, next_layer)
                                else {
                                    continue;
                                };
                                if let Some(diag) = ctx.diagnostics {
                                    diag.record_point(path.frenet_to_xy(sb, db));
                                    diag.record_trajectory(
                                        std::iter::once([start.x, start.y])
                                            .chain(segment.points.iter().copied())
                                            .collect(),
                                    );
                                }
                                successors
                                    .push((node_id(next_layer, next_si, next_di, next_vi), cost));
                            }
                        }
                    }
                    successors
                },
            )
        });
        ctx.work(evaluated.get() as u64);

        let Some(result) = result else {
            if let Some((_, controls)) = best_root_segment.into_inner() {
                return repeat_last_controls(&controls, ctx.horizon);
            }
            return brake_controls(ego, ctx, MIN_LON_ACCEL);
        };

        ctx.time("extract", || {
            let chain = parent_chain(result.goal, 0, |node| {
                (result.parent[node] != usize::MAX).then_some(result.parent[node])
            });
            let mut controls = Vec::with_capacity(PLANNING_TICKS);
            let mut previous = 0usize;
            for node in chain {
                let (_, _, _, _, sb, db, vb) = node_state(node);
                let (start, sa, da, va, layer) = if previous == 0 {
                    (ego, s0, d0, ego.speed.max(0.0), 0)
                } else {
                    let (pl, _, _, _, ps, pd, pv) = node_state(previous);
                    let (xy, yaw) = path.pose_at(ps);
                    (
                        State {
                            x: xy[0] - pd * yaw.sin(),
                            y: xy[1] + pd * yaw.cos(),
                            yaw,
                            speed: pv,
                        },
                        ps,
                        pd,
                        pv,
                        pl + 1,
                    )
                };
                let Some(segment) =
                    segment(&path, ctx, start, sa, da, va, sb, db, vb, layer as f64)
                else {
                    return brake_controls(ego, ctx, MIN_LON_ACCEL);
                };
                controls.extend(segment.controls);
                previous = node;
            }
            controls.truncate(ctx.horizon.min(PLANNING_TICKS));
            controls
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::barrier::collides_with_road_barrier;
    use crate::planning::{test_ctx, test_road, test_run, test_run_on};
    use crate::track::Road;

    #[test]
    fn reachable_bounds_include_drag_and_braking() {
        let r = reachable(20.0, 0.1);
        assert!(r[0].min_v < 20.0);
        assert!(r[0].max_v < 20.0 + MAX_LON_ACCEL);
        assert!(r.windows(2).all(|pair| pair[1].max_s > pair[0].max_s));
    }

    #[test]
    fn maximum_acceleration_segment_is_feasible() {
        let mut road = test_road(&[[-20.0, 0.0], [1_500.0, 0.0]]);
        road.target_speed = 100.0;
        let ctx = test_ctx(&road, &[]);
        let ego = State {
            speed: 5.0,
            ..Default::default()
        };
        let path = Path::new(road.centerline());
        let (s0, d0) = path.project(ego.position());
        let r = reachable(ego.speed, road.dt)[0];
        assert!(
            segment(
                &path,
                &ctx,
                ego,
                s0,
                d0,
                ego.speed,
                s0 + r.max_s,
                0.0,
                r.max_v,
                0.0,
            )
            .is_some()
        );
    }

    #[test]
    fn accelerates_on_an_open_straight_without_a_speed_profile() {
        let mut road = test_road(&[[-20.0, 0.0], [1_500.0, 0.0]]);
        road.target_speed = 100.0;
        let controls = LatticePlanner.plan(
            State {
                speed: 5.0,
                ..Default::default()
            },
            &test_ctx(&road, &[]),
        );
        assert!(!controls.is_empty());
        assert!(
            controls[0].acceleration > MAX_LON_ACCEL - 0.1,
            "accel {}",
            controls[0].acceleration
        );
    }

    #[test]
    fn brakes_for_road_curvature() {
        let radius = 20.0;
        let centerline: Vec<[f64; 2]> = (0..=80)
            .map(|i| {
                let a = i as f64 * 0.02;
                [radius * a.sin(), radius * (1.0 - a.cos())]
            })
            .collect();
        let road = Road::new(centerline, 60.0, 5.5, 0.1);
        let controls = LatticePlanner.plan(
            State {
                speed: 25.0,
                ..Default::default()
            },
            &test_ctx(&road, &[]),
        );
        assert!(controls[0].acceleration < 0.0);
    }

    #[test]
    fn stays_on_road_actor_free() {
        let trace = test_run(
            &mut LatticePlanner,
            State {
                y: 1.0,
                speed: 8.0,
                ..Default::default()
            },
            &[],
            80,
        );
        assert!(trace.iter().all(|state| state.y.abs() <= 5.5));
        assert!(
            trace.last().unwrap().speed > 8.0,
            "final state {:?}",
            trace.last().unwrap()
        );
    }

    #[test]
    fn uses_the_road_width_through_a_corner() {
        let radius = 25.0;
        let mut centerline: Vec<[f64; 2]> =
            (0..=25).map(|i| [-50.0 + 4.0 * i as f64, 0.0]).collect();
        centerline.extend((1..=32).map(|i| {
            let a = std::f64::consts::FRAC_PI_2 * i as f64 / 32.0;
            [50.0 + radius * a.sin(), radius * (1.0 - a.cos())]
        }));
        centerline.extend((1..=30).map(|i| [50.0 + radius, radius + 4.0 * i as f64]));
        let road = Road::new(centerline, 60.0, 7.0, 0.1);
        let trace = test_run_on(
            &mut LatticePlanner,
            &road,
            State {
                speed: 18.0,
                ..Default::default()
            },
            &[],
            130,
        );
        let path = Path::new(road.centerline());
        let lateral_peak = trace
            .iter()
            .map(|state| path.project(state.position()).1.abs())
            .fold(0.0, f64::max);
        assert!(
            lateral_peak > 0.5,
            "planner remained on the centerline instead of using road width; final {:?}",
            trace.last().unwrap()
        );
        assert!(
            trace
                .iter()
                .all(|state| !collides_with_road_barrier(*state, &road)),
            "racing trajectory contacted the road boundary"
        );
    }

    #[test]
    fn segment_budget_is_the_declared_realtime_bound() {
        assert_eq!(MAX_EVALUATED_SEGMENTS, 1_000);
    }

    #[test]
    fn records_no_more_than_the_segment_budget() {
        let road = test_road(&[[-20.0, 0.0], [1_500.0, 0.0]]);
        let diagnostics = crate::planning::Diagnostics::default();
        let mut ctx = test_ctx(&road, &[]);
        ctx.diagnostics = Some(&diagnostics);
        LatticePlanner.plan(
            State {
                speed: 8.0,
                ..Default::default()
            },
            &ctx,
        );
        let data = diagnostics.take();
        assert!(!data.trajectories.is_empty());
        assert!(data.trajectories.len() <= MAX_EVALUATED_SEGMENTS);
    }

    #[test]
    fn still_avoids_a_stopped_actor() {
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = test_run(
            &mut LatticePlanner,
            State {
                speed: 8.0,
                ..Default::default()
            },
            &[obstacle],
            120,
        );
        let min_gap = trace
            .iter()
            .map(|state| (state.x - obstacle.x).hypot(state.y - obstacle.y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "minimum actor gap {min_gap}");
    }
}
