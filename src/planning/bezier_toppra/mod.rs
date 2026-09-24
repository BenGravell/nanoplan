//! Road-following cubic Bezier candidates, each timed with scalar TOPP-RA.

use crate::common::kinematics::{longitudinal_resistance_accel, net_longitudinal_accel};
use crate::common::math::{smoothstep, wrap_angle};
use crate::geometry::barrier::{collide_with_road_barriers, collides_with_road_barrier};
use crate::geometry::{
    CAR_COLLISION_RADIUS_M, CAR_FOOTPRINT, EGO_COLLISION_RADIUS_M, EGO_FOOTPRINT, footprints_overlap,
};
use crate::planning::constraints::HardConstraints;
use crate::planning::planner_math::state_sample;
use crate::planning::policy::centerline_feedback;
use crate::planning::{ComputeBudget, Context, PLANNING_HORIZON_S, Planner};
use crate::prediction::predict;
use crate::simulation::{Control, Pose, Position, State, curvature_limit, world_step};
use crate::track::Path;
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL,
    ROLLING_RESISTANCE_ACCEL,
};

const GRID_STEPS: usize = 100;
// Browser-worker calibration leaves headroom below the 100 ms p99 allowance.
const NOMINAL_PATHS: usize = 9;
const MAX_REFINEMENTS: usize = 8;

pub(crate) struct BezierToppraPlanner;

impl Planner for BezierToppraPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let (path, s0) = ctx.time("route", || {
            let path = ctx.path();
            let (s0, _) = path.project(ego.position());
            ctx.work(ctx.road.centerline().len() as u64);
            (path, s0)
        });
        let Some(stations) = stations(ego.speed, s0, path.length(), ctx.road.dt) else {
            return brake(ego, ctx);
        };
        let count = lateral_count(ctx.compute_budget);
        let laterals = stations.map(|s| lateral_offsets(ctx.road.lateral_bounds_at(s), count));
        let ticks = ctx.horizon.max((PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize);
        let mut best: Option<(f64, Vec<Control>)> = None;
        for &d1 in &laterals[0] {
            for &d2 in &laterals[1] {
                let curve = ctx.time("bezier_fit", || {
                    ctx.work(GRID_STEPS as u64);
                    BezierPath::new(ego, path, stations, [d1, d2])
                });
                let controls = ctx.time("optimize", || parameterize(ego, ctx, &curve, ticks));
                let cost = ctx.time("cost", || candidate_cost(ego, ctx, &controls));
                if cost.is_finite() && best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost) {
                    best = Some((cost, controls));
                }
            }
        }
        best.map(|(_, mut controls)| {
            controls.truncate(ctx.horizon);
            controls
        })
        .unwrap_or_else(|| brake(ego, ctx))
    }
}

/// Keep a nearby steering station inside the reachable interval, but extend
/// the terminal station to the full maximum-acceleration reach.
fn stations(speed: f64, s0: f64, end: f64, dt: f64) -> Option<[f64; 2]> {
    let remaining = end - s0;
    if remaining <= 1e-3 {
        return None;
    }
    let steps = (PLANNING_HORIZON_S / (2.0 * dt)).ceil().max(1.0) as usize;
    let step = PLANNING_HORIZON_S / (2.0 * steps as f64);
    let mut speeds = [speed.max(0.0); 2];
    let mut distances = [0.0; 2];
    let mut targets = [0.0; 2];
    for (layer, target) in targets.iter_mut().enumerate() {
        for _ in 0..steps {
            for (i, accel) in [MIN_LON_ACCEL, MAX_LON_ACCEL].into_iter().enumerate() {
                distances[i] += speeds[i] * step;
                speeds[i] = (speeds[i] + net_longitudinal_accel(accel, speeds[i]) * step).max(0.0);
            }
        }
        *target = if layer == 0 {
            0.5 * (distances[0] + distances[1])
        } else {
            distances[1]
        };
    }
    // Preserve two distinct segments when the local road window is short.
    // One control-step margin reconciles the spatial profile with the
    // plant's Euler integration without capping acceleration on the last tick.
    targets[1] = (targets[1] + speeds[1] * dt).min(remaining);
    targets[0] = targets[0].min(0.5 * targets[1]);
    Some(targets.map(|distance| s0 + distance))
}

fn lateral_count(budget: ComputeBudget) -> usize {
    // Two independently sampled lateral axes: keep the product within budget.
    let count = (budget.scale(NOMINAL_PATHS, 1) as f64).sqrt() as usize;
    count.saturating_sub(1) / 2 * 2 + 1
}

fn lateral_offsets((right, left): (f64, f64), count: usize) -> Vec<f64> {
    // Leave room for the front corners while the rollout settles onto an
    // offset lane; rear-point width clearance alone hugs the barriers.
    let margin = EGO_FOOTPRINT.width / 2.0 + 0.5;
    let lo = (right + margin).min(0.0);
    let hi = (left - margin).max(0.0);
    let mut offsets = vec![0.0];
    for i in 1..=count / 2 {
        let fraction = i as f64 / (count / 2) as f64;
        for offset in [lo * fraction, hi * fraction] {
            if offset != 0.0 {
                offsets.push(offset);
            }
        }
    }
    offsets
}

struct BezierPath {
    segments: Vec<[Position; 4]>,
    /// Approximate arc length at each short cubic's endpoints.
    distance: Vec<f64>,
    reference: Path,
}

impl BezierPath {
    fn new(ego: State, path: &Path, stations: [f64; 2], offsets: [f64; 2]) -> Self {
        let (s0, d0) = path.project(ego.position());
        let heading = wrap_angle(ego.pose().yaw - path.pose_at(s0).1);
        // The two search stations control lateral motion, not road geometry.
        // Fit short cubics along the route so a distant endpoint cannot cut
        // across intervening bends and create an artificial braking obstacle.
        let points: Vec<_> = (0..=GRID_STEPS)
            .map(|i| {
                if i == 0 {
                    return ego.position();
                }
                let (layer, t) = segment_parameter(i as f64);
                let (start, d, slope) = if layer == 0 {
                    (s0, d0, heading.sin() / heading.cos().max(0.1))
                } else {
                    (stations[0], offsets[0], 0.0)
                };
                let span = stations[layer] - start;
                let blend = smoothstep(t);
                let offset = d + (offsets[layer] - d) * blend + span * slope * t * (1.0 - t).powi(2);
                path.frenet_to_position(start + span * t, offset)
            })
            .collect();
        let lengths: Vec<_> = points.windows(2).map(|p| p[0].distance(p[1])).collect();
        let poses: Vec<_> = points
            .iter()
            .enumerate()
            .map(|(i, &position)| {
                let yaw = if i == 0 {
                    ego.pose().yaw
                } else if i == GRID_STEPS {
                    path.pose_at(stations[1]).1
                } else {
                    let before = (position - points[i - 1]) * (1.0 / lengths[i - 1].max(1e-9));
                    let after = (points[i + 1] - position) * (1.0 / lengths[i].max(1e-9));
                    // Weight secants by the opposite interval: the two station
                    // spans can have very different sample spacing at their join.
                    let tangent = before * lengths[i] + after * lengths[i - 1];
                    tangent.y.atan2(tangent.x)
                };
                Pose::new(position, yaw)
            })
            .collect();
        let segments = (0..GRID_STEPS)
            .map(|i| {
                // Tangents share a direction; scale each handle by its own
                // interval so unequal spacing cannot amplify join curvature.
                fit_bezier(poses[i], poses[i + 1], lengths[i] / 3.0, lengths[i] / 3.0)
            })
            .collect();
        // ponytail: fixed-grid chord lengths approximate arc length; use
        // adaptive subdivision if tighter geometric accuracy is needed.
        let mut distance = vec![0.0];
        for pair in points.windows(2) {
            distance.push(distance.last().unwrap() + pair[0].distance(pair[1]).max(1e-9));
        }
        Self {
            segments,
            distance,
            reference: Path::new(&points),
        }
    }

    fn at(&self, distance: f64) -> (Pose, f64) {
        let i = self.index(distance);
        let t = ((distance - self.distance[i]) / self.ds(i)).clamp(0.0, 1.0);
        let b = &self.segments[i];
        let tangent = bezier_d1(b, t);
        (
            Pose::new(bezier_point(b, t), tangent[1].atan2(tangent[0])),
            bezier_curvature(b, t),
        )
    }

    fn index(&self, distance: f64) -> usize {
        self.distance
            .partition_point(|&s| s <= distance)
            .saturating_sub(1)
            .min(GRID_STEPS - 1)
    }

    fn ds(&self, i: usize) -> f64 {
        self.distance[i + 1] - self.distance[i]
    }
}

fn segment_parameter(index: f64) -> (usize, f64) {
    let parameter = index * 2.0 / GRID_STEPS as f64;
    let segment = (parameter as usize).min(1);
    (segment, parameter - segment as f64)
}

fn fit_bezier(start: Pose, end: Pose, start_handle: f64, end_handle: f64) -> [Position; 4] {
    [
        start.position,
        start.position + Position::from_angle(start.yaw) * start_handle,
        end.position - Position::from_angle(end.yaw) * end_handle,
        end.position,
    ]
}

fn actor_collision(pose: Pose, time: f64, ctx: &Context) -> bool {
    ctx.actors.iter().any(|actor| {
        ctx.work(1);
        // Live actors have already advanced when the current ego command is
        // applied. Check their supplied poses as well as the prediction.
        if time <= ctx.road.dt && footprints_overlap(pose, EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT) {
            return true;
        }
        let predicted = predict(actor, ctx.path(), time).pose();
        EGO_FOOTPRINT
            .center(pose)
            .position
            .distance(CAR_FOOTPRINT.center(predicted).position)
            < EGO_COLLISION_RADIUS_M + CAR_COLLISION_RADIUS_M
            && footprints_overlap(pose, EGO_FOOTPRINT, predicted, CAR_FOOTPRINT)
    })
}

fn parameterize(ego: State, ctx: &Context, curve: &BezierPath, ticks: usize) -> Vec<Control> {
    let mut limits: Vec<_> = curve
        .distance
        .iter()
        .map(|&s| {
            let (_, curvature) = curve.at(s);
            ctx.work(1);
            if curvature.abs() > MAX_ABS_CURVATURE {
                0.0
            } else {
                ctx.road
                    .target_speed
                    .powi(2)
                    .min(MAX_ABS_LAT_ACCEL / curvature.abs().max(1e-9))
            }
        })
        .collect();
    // The horizon endpoint is a preview boundary, not a destination. TOPP-RA
    // may leave it at speed; only genuine road/actor constraints require a stop.
    let ds: Vec<_> = (0..GRID_STEPS).map(|i| curve.ds(i)).collect();
    let mut controls = Vec::new();
    for _ in 0..MAX_REFINEMENTS {
        let speed2 = toppra_profile(ego.speed.max(0.0).powi(2), &ds, &limits);
        ctx.work(2 * GRID_STEPS as u64);
        let times = arrival_times(&ds, &speed2);
        let mut changed = false;
        for (i, &time) in times.iter().enumerate().skip(1) {
            if time > ticks as f64 * ctx.road.dt {
                break;
            }
            if actor_collision(curve.at(curve.distance[i]).0, time, ctx) && limits[i - 1] != 0.0 {
                limits[i - 1..].fill(0.0);
                changed = true;
            }
        }
        controls = ctx.time("extract", || extract_controls(ego, ctx, curve, &speed2, &times, ticks));
        // Close the loop around the actual plant, including steering feedback.
        let mut state = ego;
        let mut distance = 0.0;
        for (tick, &u) in controls.iter().enumerate() {
            let previous = state;
            distance += state.speed.max(0.0) * ctx.road.dt;
            state = world_step(state, u, ctx.road.dt);
            ctx.work(1);
            if collide_with_road_barriers(previous, state, EGO_FOOTPRINT, ctx.road) != state
                || actor_collision(state.pose(), (tick + 1) as f64 * ctx.road.dt, ctx)
            {
                let index = curve.index(distance).saturating_sub(1);
                if let Some(stop) = (0..=index).rev().find(|&i| limits[i] != 0.0) {
                    limits[stop..].fill(0.0);
                    changed = true;
                }
                break;
            }
        }
        if !changed {
            break;
        }
    }
    controls
}

/// Scalar TOPP-RA: backward controllable intervals, then maximum-control
/// forward propagation, using arc length rather than Bezier parameter distance.
fn toppra_profile(start_speed2: f64, ds: &[f64], max_speed2: &[f64]) -> Vec<f64> {
    let n = max_speed2.len() - 1;
    let mut controllable = max_speed2.to_vec();
    for i in (0..n).rev() {
        let next = predecessor_limit(controllable[i + 1], ds[i]);
        controllable[i] = controllable[i].min(next.max(0.0));
    }
    let mut x = vec![0.0; n + 1];
    x[0] = start_speed2;
    for i in 0..n {
        let resistance = longitudinal_resistance_accel(x[i].sqrt());
        let lo = (x[i] + 2.0 * ds[i] * (MIN_LON_ACCEL - resistance)).max(0.0);
        let hi = (x[i] + 2.0 * ds[i] * (MAX_LON_ACCEL - resistance)).max(0.0);
        x[i + 1] = hi.min(controllable[i + 1]).max(lo);
    }
    x
}

fn predecessor_limit(next_speed2: f64, ds: f64) -> f64 {
    (next_speed2 - 2.0 * ds * (MIN_LON_ACCEL - ROLLING_RESISTANCE_ACCEL))
        / (1.0 - 2.0 * ds * AERO_DRAG_ACCEL_COEFFICIENT)
}

fn arrival_times(ds: &[f64], speed2: &[f64]) -> Vec<f64> {
    let mut times = vec![0.0; speed2.len()];
    for i in 0..speed2.len() - 1 {
        let average = 0.5 * (speed2[i].sqrt() + speed2[i + 1].sqrt());
        times[i + 1] = times[i] + ds[i] / average.max(1e-3);
    }
    times
}

fn extract_controls(
    ego: State,
    ctx: &Context,
    curve: &BezierPath,
    speed2: &[f64],
    times: &[f64],
    ticks: usize,
) -> Vec<Control> {
    let mut state = ego;
    let mut distance = 0.0;
    (0..ticks)
        .map(|tick| {
            let time = (tick + 1) as f64 * ctx.road.dt;
            let i = times
                .partition_point(|&t| t <= time)
                .saturating_sub(1)
                .min(GRID_STEPS - 1);
            let fraction = ((time - times[i]) / (times[i + 1] - times[i])).clamp(0.0, 1.0);
            let target_speed = speed2[i].sqrt() + fraction * (speed2[i + 1].sqrt() - speed2[i].sqrt());
            // Track the timed TOPP-RA profile. Comparing speed against the
            // old Euler position cancels acceleration just after launch.
            let net_accel = (target_speed - state.speed) / ctx.road.dt;
            let feedback = centerline_feedback(&curve.reference, &state, target_speed);
            let u = Control {
                acceleration: (net_accel + longitudinal_resistance_accel(state.speed))
                    .clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
                curvature: (curve.at(distance).1 + feedback.curvature)
                    .clamp(-curvature_limit(state.speed), curvature_limit(state.speed)),
            };
            distance += state.speed.max(0.0) * ctx.road.dt;
            state = world_step(state, u, ctx.road.dt);
            ctx.work(1);
            u
        })
        .collect()
}

fn candidate_cost(ego: State, ctx: &Context, controls: &[Control]) -> f64 {
    let path = ctx.path();
    let constraints = HardConstraints::new(ctx.road.half_width, ctx.actors, path, ego.speed, ctx.road.dt);
    let mut state = ego;
    let mut total = 0.0;

    let mut points = ctx.diagnostics.map(|_| vec![ego.position()]);
    for (tick, &u) in controls.iter().enumerate() {
        let previous = state;
        state = world_step(state, u, ctx.road.dt);
        let time = (tick + 1) as f64 * ctx.road.dt;
        let (s, mut sample) = state_sample(path, &state, time, None);
        sample.road_bounds = Some(ctx.road.lateral_bounds_at(s));
        if !u.acceleration.is_finite()
            || !u.curvature.is_finite()
            || state.speed < -1e-9
            || collides_with_road_barrier(state, ctx.road)
            || collide_with_road_barriers(previous, state, EGO_FOOTPRINT, ctx.road) != state
            || actor_collision(state.pose(), time, ctx)
        {
            total = f64::INFINITY;
        }
        total += constraints.point_cost(&sample);
        ctx.work(1);
        if let Some(points) = &mut points {
            points.push(state.position());
        }
    }
    if let (Some(diag), Some(points)) = (ctx.diagnostics, points) {
        let times = (0..points.len()).map(|i| i as f64 * ctx.road.dt).collect();
        diag.record_timed_trajectory(points, times);
    }
    total
}

fn brake(ego: State, ctx: &Context) -> Vec<Control> {
    let mut state = ego;
    (0..ctx.horizon)
        .map(|_| {
            let mut u = centerline_feedback(ctx.path(), &state, 0.0);
            let (s, _) = ctx.path().project(state.position());
            let before = ctx.path().pose_at(s - 7.5).1;
            let after = ctx.path().pose_at(s + 7.5).1;
            u.curvature += wrap_angle(after - before) / 15.0;
            u.acceleration = (-state.speed / ctx.road.dt + longitudinal_resistance_accel(state.speed))
                .clamp(MIN_LON_ACCEL, MAX_LON_ACCEL);
            u.curvature = u
                .curvature
                .clamp(-curvature_limit(state.speed), curvature_limit(state.speed));
            state = world_step(state, u, ctx.road.dt);
            u
        })
        .collect()
}

fn bezier_point(p: &[Position; 4], t: f64) -> Position {
    let mt = 1.0 - t;
    let c = [mt.powi(3), 3.0 * mt * mt * t, 3.0 * mt * t * t, t.powi(3)];
    Position::new(
        c.iter().zip(p).map(|(c, p)| c * p.x).sum(),
        c.iter().zip(p).map(|(c, p)| c * p.y).sum(),
    )
}

fn bezier_d1(p: &[Position; 4], t: f64) -> [f64; 2] {
    let mt = 1.0 - t;
    let c = [3.0 * mt * mt, 6.0 * mt * t, 3.0 * t * t];
    [
        c[0] * (p[1].x - p[0].x) + c[1] * (p[2].x - p[1].x) + c[2] * (p[3].x - p[2].x),
        c[0] * (p[1].y - p[0].y) + c[1] * (p[2].y - p[1].y) + c[2] * (p[3].y - p[2].y),
    ]
}

fn bezier_d2(p: &[Position; 4], t: f64) -> [f64; 2] {
    let mt = 1.0 - t;
    [
        6.0 * mt * (p[2].x - 2.0 * p[1].x + p[0].x) + 6.0 * t * (p[3].x - 2.0 * p[2].x + p[1].x),
        6.0 * mt * (p[2].y - 2.0 * p[1].y + p[0].y) + 6.0 * t * (p[3].y - 2.0 * p[2].y + p[1].y),
    ]
}

fn bezier_curvature(p: &[Position; 4], t: f64) -> f64 {
    let d1 = bezier_d1(p, t);
    let d2 = bezier_d2(p, t);
    let speed = d1[0].hypot(d1[1]).max(1e-6);
    (d1[0] * d2[1] - d1[1] * d2[0]) / speed.powi(3)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{COMPUTE_BUDGET_BREAKPOINTS, Diagnostics, test_ctx, test_road, test_run_on};

    #[test]
    fn small_track_curves_follow_bends_and_previews_keep_moving() {
        use crate::planning::PlannerKind;
        use crate::world::{EgoStart, LiveWorld};
        let lap = crate::track::Track::from_catalog(1).lap_length().unwrap();
        for progress in (0..20).map(|i| lap * i as f64 / 20.0) {
            for speed in [0.0, 5.0, 20.0] {
                let world = LiveWorld::with_track_at(
                    1,
                    1,
                    PlannerKind::BezierToppra,
                    0,
                    0.1,
                    EgoStart {
                        progress,
                        speed,
                        ..Default::default()
                    },
                );
                let ego = world.ego();
                let ctx = Context::new(&world.road, &[], 100, ComputeBudget::NOMINAL, None, None);
                let path = ctx.path();
                let s0 = path.project(ego.position()).0;
                let targets = stations(speed, s0, path.length(), 0.1).unwrap();
                let curve = BezierPath::new(ego, path, targets, [0.0; 2]);
                let samples: Vec<_> = (0..=400)
                    .map(|i| curve.at(curve.distance[GRID_STEPS] * i as f64 / 400.0))
                    .collect();
                let max_d = samples
                    .iter()
                    .map(|(pose, _)| path.project(pose.position).1.abs())
                    .fold(0.0, f64::max);
                let max_k = samples.iter().map(|(_, k)| k.abs()).fold(0.0, f64::max);
                assert!(max_d < 1.0, "progress {progress}, speed {speed}: deviation {max_d}");
                assert!(
                    max_k < MAX_ABS_CURVATURE,
                    "progress {progress}, speed {speed}: curvature {max_k}"
                );
                assert!(
                    samples[320..].iter().all(|(_, k)| k.abs() < 0.1),
                    "progress {progress}, speed {speed}: terminal curvature spike"
                );
                let controls = BezierToppraPlanner.plan(ego, &ctx);
                let mut state = ego;
                let mut distance = 0.0;
                for u in controls {
                    distance += state.speed * 0.1;
                    state = world_step(state, u, 0.1);
                    assert!(!collides_with_road_barrier(state, &world.road));
                }
                assert!(distance > 100.0, "progress {progress}, speed {speed}: reach {distance}");
                assert!(
                    state.speed > 5.0,
                    "progress {progress}, speed {speed}: stopped prematurely"
                );
            }
        }
    }

    #[test]
    fn road_following_segments_interpolate_offsets_with_shared_tangent_directions() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let path = Path::new(road.centerline());
        let ego = State::from((Position::new(0.0, 1.0), 0.1, 8.0));
        let curve = BezierPath::new(ego, &path, [40.0, 80.0], [3.0, -2.0]);

        assert_eq!(curve.segments[0][0], ego.position());
        assert_eq!(
            curve.segments[GRID_STEPS / 2 - 1][3],
            path.frenet_to_position(40.0, 3.0)
        );
        assert_eq!(curve.segments[1][0], curve.segments[0][3]);
        assert_eq!(curve.segments[GRID_STEPS - 1][3], path.frenet_to_position(80.0, -2.0));
        for pair in curve.segments.windows(2) {
            let a = bezier_d1(&pair[0], 1.0);
            let b = bezier_d1(&pair[1], 0.0);
            assert!(wrap_angle(a[1].atan2(a[0]) - b[1].atan2(b[0])).abs() < 1e-10);
        }
        assert_eq!(
            curve.at(curve.distance[GRID_STEPS / 2]).0.position,
            curve.segments[GRID_STEPS / 2][0]
        );
        assert!(curve.distance.windows(2).all(|w| w[1] > w[0]));
        assert!(curve.distance[GRID_STEPS] > 60.0);
    }

    #[test]
    fn stations_expand_with_speed_and_fit_short_road_windows() {
        let s0 = 20.0;
        let mut previous = [s0; 2];
        for speed in [0.0_f64, 5.0, 20.0, 40.0] {
            let targets = stations(speed, s0, 2_000.0, 0.1).unwrap();
            for (i, &s) in targets.iter().enumerate() {
                let time = PLANNING_HORIZON_S * (i + 1) as f64 / 2.0;
                let acceleration_bound = speed * time + 0.5 * MAX_LON_ACCEL * time.powi(2);
                assert!(s > previous[i] && s <= s0 + acceleration_bound);
            }
            assert!(targets[1] > targets[0]);
            previous = targets;
        }
        let short = stations(20.0, s0, s0 + 1.0, 0.1).unwrap();
        assert!(s0 < short[0] && short[0] < short[1] && short[1] <= s0 + 1.0);
        assert!(stations(20.0, s0, s0, 0.1).is_none());
    }

    #[test]
    fn budget_scales_candidate_count_and_always_includes_centerline() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let mut previous = 0;
        for percent in COMPUTE_BUDGET_BREAKPOINTS {
            let budget = ComputeBudget::from_percent(percent);
            let count = lateral_count(budget);
            let offsets = lateral_offsets((-3.0, 5.0), count);
            assert_eq!(offsets[0], 0.0);
            assert_eq!(offsets.len(), count);
            assert!(count * count >= previous && count * count <= budget.scale(NOMINAL_PATHS, 1));
            assert!(
                offsets
                    .iter()
                    .all(|d| (-3.0 + EGO_FOOTPRINT.width / 2.0..=5.0 - EGO_FOOTPRINT.width / 2.0).contains(d))
            );
            let diagnostics = Diagnostics::default();
            let ctx = Context::new(&road, &[], 10, budget, None, Some(&diagnostics));
            let controls = BezierToppraPlanner.plan(State::default(), &ctx);
            let data = diagnostics.take();
            assert_eq!(controls.len(), ctx.horizon);
            assert_eq!(data.trajectories.len(), count * count);
            assert!(data.trajectories.iter().all(|points| points.len() == 101));
            assert!(data.trajectories[0].iter().all(|point| point.y.abs() < 1e-9));
            previous = count * count;
        }
        assert!(previous > NOMINAL_PATHS);
    }

    #[test]
    fn backward_pass_brakes_for_a_zero_speed_constraint() {
        let mut limits = vec![400.0; 11];
        limits[5] = 0.0;
        let x = toppra_profile(10.0_f64.powi(2), &[2.0; 10], &limits);
        assert!(x[5] < 1e-9, "speed² {}", x[5]);
        assert!(x[..5].windows(2).any(|w| w[1] < w[0]));
    }

    #[test]
    fn clear_straight_accelerates_through_the_entire_horizon() {
        let mut road = test_road(&[[-50.0, 0.0], [2_000.0, 0.0]]);
        road.target_speed = *crate::vehicle::MAX_TERMINAL_SPEED_MPS;
        for speed in [0.0, 20.0, 40.0, 70.0] {
            let ego = State {
                speed,
                ..Default::default()
            };
            let ctx = Context::new(&road, &[], 100, ComputeBudget::NOMINAL, None, None);
            let controls = BezierToppraPlanner.plan(ego, &ctx);
            for (tick, control) in controls.iter().enumerate() {
                assert!(
                    control.acceleration > MAX_LON_ACCEL - 0.05,
                    "initial speed {speed}, tick {tick}, control {control:?}"
                );
            }
        }
    }

    #[test]
    fn centerline_candidate_converges_to_centerline_and_target_speed() {
        let mut ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 3.0), 0.0),
            5.0,
        );
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let ctx = Context::new(&road, &[], 10, ComputeBudget::from_percent(5.0), None, None);
        for _ in 0..200 {
            let controls = BezierToppraPlanner.plan(ego, &ctx);
            ego = world_step(ego, controls[0], road.dt);
        }
        assert!(ego.position().y.abs() < 0.3, "offset {}", ego.position().y);
        assert!((ego.speed - 10.0).abs() < 0.5, "speed {}", ego.speed);
    }

    #[test]
    fn stops_behind_stopped_actor_when_road_is_too_narrow_to_pass() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let actor = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(50.0, 0.0), 0.0),
            0.0,
        );
        let road = crate::track::Road::new(vec![[-20.0, 0.0], [2_000.0, 0.0]], 10.0, 1.6, 0.1);
        let trace = test_run_on(&mut BezierToppraPlanner, &road, ego, &[actor], 300);
        let end = trace.last().unwrap();
        assert!(end.speed < 0.5, "speed {}", end.speed);
        assert!(
            end.position().x <= actor.position().x - crate::geometry::EGO_FOOTPRINT.length + 1e-9,
            "x {}",
            end.position().x
        );
    }

    #[test]
    fn selects_a_cheaper_feasible_detour_and_checks_beyond_requested_controls() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let actors = [State::from((Position::new(50.0, 0.0), 0.0, 0.0))];
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let ctx = Context::new(&road, &actors, 100, ComputeBudget::NOMINAL, None, None);
        let detour = BezierToppraPlanner.plan(ego, &ctx);
        let centerline = BezierToppraPlanner.plan(
            ego,
            &Context {
                compute_budget: ComputeBudget::from_percent(5.0),
                ..test_ctx(&road, &actors)
            },
        );
        let center_ctx = Context::new(&road, &actors, 100, ComputeBudget::from_percent(5.0), None, None);
        let centerline_full = BezierToppraPlanner.plan(ego, &center_ctx);
        assert_eq!(centerline, centerline_full[..centerline.len()]);
        let cost = candidate_cost(ego, &ctx, &detour);
        assert!(cost.is_finite() && cost < candidate_cost(ego, &ctx, &centerline_full));
        let end = detour.iter().fold(ego, |state, &u| world_step(state, u, road.dt));
        assert!(
            end.position().x > actors[0].position().x + CAR_FOOTPRINT.length,
            "end {end:?}"
        );
        let short = BezierToppraPlanner.plan(ego, &test_ctx(&road, &actors));
        assert_eq!(short, detour[..short.len()]);
        assert!(candidate_cost(ego, &ctx, &vec![Control::default(); 100]).is_infinite());
    }

    #[test]
    fn exhausted_route_brakes_to_standstill_without_reversing() {
        let road = test_road(&[[-20.0, 0.0], [0.0, 0.0]]);
        let ctx = Context::new(&road, &[], 100, ComputeBudget::NOMINAL, None, None);
        let mut ego = State {
            speed: 8.0,
            ..Default::default()
        };
        for u in BezierToppraPlanner.plan(ego, &ctx) {
            ego = world_step(ego, u, road.dt);
            assert!(ego.speed >= -1e-9);
        }
        assert!(ego.speed.abs() < 1e-9);
        assert!(BezierToppraPlanner.plan(ego, &Context { horizon: 0, ..ctx }).is_empty());
    }

    #[test]
    fn first_control_checks_supplied_actor_pose_as_well_as_prediction() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let actors = [State::from((Position::new(5.5, 0.0), 0.0, 10.0))];
        let ctx = test_ctx(&road, &actors);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let next = world_step(ego, Control::default(), road.dt);
        assert!(!footprints_overlap(
            next.pose(),
            EGO_FOOTPRINT,
            predict(&actors[0], ctx.path(), road.dt).pose(),
            CAR_FOOTPRINT
        ));
        assert!(candidate_cost(ego, &ctx, &[Control::default()]).is_infinite());
    }

    #[test]
    fn follows_a_slower_lead_without_contact() {
        use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};
        use crate::planning::{test_ctx, test_road};
        use crate::simulation::CommandLimiter;

        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let mut planner = BezierToppraPlanner;
        let mut limiter = CommandLimiter::new();
        let mut ego = State::default();
        let mut lead = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(55.0, 0.0), 0.0),
            7.0,
        );

        for tick in 0..300 {
            lead.pose.position.x += lead.speed * road.dt;
            let actors = [lead];
            let controls = planner.plan(ego, &test_ctx(&road, &actors));
            ego = limiter.step(ego, controls[0], road.dt);
            assert!(
                !footprints_overlap(ego.pose(), EGO_FOOTPRINT, lead.pose(), CAR_FOOTPRINT),
                "contact at tick {tick}: ego {ego:?}, lead {lead:?}"
            );
        }
    }

    #[test]
    fn baked_track_predictions_stay_inside_road_for_full_horizon() {
        use crate::geometry::barrier::collides_with_road_barrier;
        use crate::simulation::MAX_TERMINAL_SPEED_MPS;
        use crate::track::{Road, TRACK_PRESETS, Track};

        for track_index in 0..TRACK_PRESETS.len() {
            let track = Track::from_catalog(track_index);
            let lap = track.lap_length().unwrap();
            for n in 0..20 {
                let progress = lap * n as f64 / 20.0;
                let road = Road::new(
                    track.centerline(progress - 50.0, progress + 250.0, 15.0),
                    *MAX_TERMINAL_SPEED_MPS,
                    track.half_width(progress),
                    0.1,
                );
                let (p, yaw) = track.pose(progress);
                let ego = State::from((p, yaw, 20.0));
                let ctx = Context::new(&road, &[], 100, crate::planning::ComputeBudget::NOMINAL, None, None);
                let path = Path::new(road.centerline());
                let mut state = ego;
                for (tick, control) in BezierToppraPlanner.plan(ego, &ctx).into_iter().enumerate() {
                    state = world_step(state, control, road.dt);
                    let (s, d) = path.project(state.position());
                    assert!(
                        !collides_with_road_barrier(state, &road),
                        "track {track_index} progress {progress} width {} tick {tick} d {d} heading_err {} control {control:?} state {state:?}",
                        road.half_width,
                        wrap_angle(state.pose.yaw - path.pose_at(s).1)
                    );
                }
            }
        }
    }
}
