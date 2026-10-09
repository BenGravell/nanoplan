//! Frenet Bezier paths back to the centerline, timed in Cartesian arc length with TOPP-RA.

use crate::common::geometry::barrier::collide_with_road_barriers;
#[cfg(test)]
use crate::common::geometry::barrier::collides_with_road_barrier;
use crate::common::geometry::bezier::CubicBezier;
use crate::common::geometry::wrap_angle;
#[cfg(test)]
use crate::common::geometry::{CAR_FOOTPRINT, footprints_overlap};
use crate::common::geometry::{EGO_FOOTPRINT, Footprint};
use crate::common::kinematics::{commanded_accel_to_stop, longitudinal_resistance_accel, net_longitudinal_accel};
use crate::common::types::FrenetPosition;
use crate::constraints::collision::actor_collision;
use crate::metrics;
use crate::planning::feasibility::trajectory_is_feasible;
use crate::planning::frenet::cartesian_kinematics;
use crate::planning::policy::centerline_curvature;
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
#[cfg(test)]
use crate::prediction::predict;
use crate::simulation::{Control, Pose, Position, State, curvature_limit, world_step};
use crate::track::{Path, ReferenceGeometry};
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL,
    ROLLING_RESISTANCE_ACCEL,
};

const GRID_STEPS: usize = 100;
// Bound full TOPP-RA rollouts, independently of the number of search dimensions.
const NOMINAL_PATHS: usize = 9;
const MAX_REFINEMENTS: usize = 8;
// Rolling road windows and the closed circuit sample lap seams differently.
// Keep 10 cm of side clearance rather than accepting a grazing trajectory.
const ROAD_FOOTPRINT: Footprint = Footprint::new(EGO_FOOTPRINT.length + 0.1, EGO_FOOTPRINT.width + 0.2);

#[derive(Default)]
pub(crate) struct CenterlineFollower;

impl Planner for CenterlineFollower {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let (path, start) = ctx.time("route", || {
            let path = ctx.path();
            let start = ctx.project_ego(ego);
            ctx.work(1);
            (path, start)
        });
        let Some(stations) = stations(ego.speed, start.s, path.length(), ctx.road.dt) else {
            return brake(ego, ctx);
        };
        let ticks = ctx.horizon.max((PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize);
        let mut best: Option<(f64, Vec<Control>)> = None;
        for stations in path_targets(ctx, start.s, stations) {
            let curve = ctx.time("bezier_fit", || {
                ctx.work(GRID_STEPS as u64);
                CartesianPath::new(ego, path, start, stations)
            });
            let Some(curve) = curve else { continue };
            let controls = ctx.time("optimize", || parameterize(ego, ctx, &curve, ticks));
            let cost = ctx.time("cost", || candidate_cost(ego, ctx, &controls));
            if cost.is_finite() && best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost) {
                best = Some((cost, controls));
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
    Some(targets.map(|distance| (s0 + distance).min(end)))
}

/// Sample merge stations in a nested grid; every candidate ends on the centerline.
fn path_targets(ctx: &Context, s0: f64, stations: [f64; 2]) -> Vec<[f64; 2]> {
    (0..ctx.compute_budget.scale(NOMINAL_PATHS, 3))
        .map(|i| {
            let reach = if i == 0 {
                1.0
            } else {
                let denominator = (i + 1).next_power_of_two();
                (2 * (i - denominator / 2) + 1) as f64 / denominator as f64
            };
            [s0 + (stations[0] - s0) * reach, stations[1]]
        })
        .collect()
}

/// A merge cubic in (s, d), extended by a straight Bezier on d = 0.
fn frenet_segments(start: FrenetPosition, slope: f64, stations: [f64; 2]) -> [CubicBezier; 2] {
    let handle = (stations[0] - start.s) / 3.0;
    let extension = (stations[1] - stations[0]) / 3.0;
    [
        CubicBezier([
            Position::new(start.s, start.d),
            Position::new(start.s + handle, start.d + slope * handle),
            Position::new(stations[0] - handle, 0.0),
            Position::new(stations[0], 0.0),
        ]),
        CubicBezier([
            Position::new(stations[0], 0.0),
            Position::new(stations[0] + extension, 0.0),
            Position::new(stations[1] - extension, 0.0),
            Position::new(stations[1], 0.0),
        ]),
    ]
}

/// Cartesian samples and arc lengths, prepared before speed planning.
struct CartesianPath {
    distance: Vec<f64>,
    reference: Path,
}

impl CartesianPath {
    fn new(ego: State, path: &Path, start: FrenetPosition, stations: [f64; 2]) -> Option<Self> {
        let heading = wrap_angle(ego.pose.yaw - path.heading_at(start.s));
        let scale = 1.0 - path.curvature_at(start.s) * start.d;
        if heading.cos() <= 0.0 || scale <= 0.1 {
            return None;
        }
        let segments = frenet_segments(start, scale * heading.tan(), stations);
        let mut points = Vec::with_capacity(GRID_STEPS + 1);
        let mut geometry = Vec::with_capacity(GRID_STEPS + 1);
        let mut distance = vec![0.0];
        for i in 0..=GRID_STEPS {
            // Include the merge exactly and sample both cubics independently.
            let parameter = i as f64 * 2.0 / GRID_STEPS as f64;
            let segment = (parameter as usize).min(1);
            let [p, d1, d2] = segments[segment].at(parameter - segment as f64);
            let (state, control) = cartesian_kinematics(path, [p.x, d1.x, d2.x], [p.y, d1.y, d2.y])?;
            let point = if i == 0 { ego.position() } else { state.position() };
            if let Some(&previous) = points.last() {
                // ponytail: fixed-grid chords approximate Cartesian arc length;
                // use adaptive subdivision if tighter accuracy is needed.
                let ds = point.distance(previous);
                if ds <= f64::EPSILON || !ds.is_finite() {
                    return None;
                }
                distance.push(distance.last().unwrap() + ds);
            }
            points.push(point);
            geometry.push(ReferenceGeometry {
                heading: state.pose.yaw,
                curvature: control.curvature,
            });
        }
        Some(Self {
            distance,
            reference: Path::with_geometry(&points, Some(&geometry)),
        })
    }

    fn at(&self, distance: f64) -> (Pose, f64) {
        (
            Pose::new(self.reference.pose_at(distance).0, self.reference.heading_at(distance)),
            self.reference.curvature_at(distance),
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

fn parameterize(ego: State, ctx: &Context, curve: &CartesianPath, ticks: usize) -> Vec<Control> {
    let mut limits: Vec<_> = curve
        .distance
        .iter()
        .map(|&s| {
            let (_, curvature) = curve.at(s);
            ctx.work(1);
            if curvature.abs() > MAX_ABS_CURVATURE {
                0.0
            } else {
                crate::vehicle::MAX_TERMINAL_SPEED_MPS
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
        // Actors have already advanced before the next ego command is applied.
        // Keep the whole speed profile aligned with that first-step check.
        for (i, &time) in times.iter().enumerate().skip(1) {
            if time > ticks as f64 * ctx.road.dt {
                break;
            }
            if actor_collision(curve.at(curve.distance[i]).0, (time - ctx.road.dt).max(0.0), ctx)
                && limits[i - 1] != 0.0
            {
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
            let barrier = collide_with_road_barriers(previous, state, ROAD_FOOTPRINT, ctx.road) != state;
            let actor = actor_collision(state.pose(), tick as f64 * ctx.road.dt, ctx);
            if barrier || actor {
                let index = curve.index(distance).saturating_sub(1);
                if let Some(bound) = (0..=index).rev().find(|&i| limits[i] != 0.0) {
                    // A tracking error at a bend calls for a slower traversal,
                    // not an artificial destination. Actors may require a stop.
                    let cap = if actor {
                        0.0
                    } else {
                        limits[bound].min(state.speed.powi(2)) * 0.5
                    };
                    for limit in &mut limits[bound..] {
                        *limit = limit.min(cap);
                    }
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
    curve: &CartesianPath,
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
            let target_speed = crate::common::interp::lerp(speed2[i].sqrt(), speed2[i + 1].sqrt(), fraction);
            // Track the timed TOPP-RA profile. Comparing speed against the
            // old Euler position cancels acceleration just after launch.
            let net_accel = crate::common::differencing::forward_difference(state.speed, target_speed, ctx.road.dt);
            let feedback = centerline_curvature(&curve.reference, &state);
            let u = Control {
                acceleration: (net_accel + longitudinal_resistance_accel(state.speed))
                    .clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
                curvature: (curve.at(distance).1 + feedback)
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
    let mut states = Vec::with_capacity(controls.len() + 1);
    states.push(ego);
    for &u in controls {
        states.push(world_step(*states.last().unwrap(), u, ctx.road.dt));
        ctx.work(1);
    }
    if let Some(diag) = ctx.diagnostics {
        let points = states.iter().map(|state| state.position()).collect();
        let times = (0..states.len()).map(|i| i as f64 * ctx.road.dt).collect();
        diag.record_timed_trajectory(points, times);
    }
    if !trajectory_is_feasible(&states, controls, ctx) {
        return f64::INFINITY;
    }
    -metrics::evaluate(&states, ctx.road.dt, path, None)
}

fn brake(ego: State, ctx: &Context) -> Vec<Control> {
    let mut state = ego;
    (0..ctx.horizon)
        .map(|_| {
            let mut u = Control {
                acceleration: 0.0,
                curvature: centerline_curvature(ctx.path(), &state),
            };
            let s = ctx.path().project(state.position()).s;
            let before = ctx.path().pose_at(s - 7.5).1;
            let after = ctx.path().pose_at(s + 7.5).1;
            u.curvature += wrap_angle(after - before) / 15.0;
            u.acceleration = commanded_accel_to_stop(state.speed, ctx.road.dt).clamp(MIN_LON_ACCEL, MAX_LON_ACCEL);
            u.curvature = u
                .curvature
                .clamp(-curvature_limit(state.speed), curvature_limit(state.speed));
            state = world_step(state, u, ctx.road.dt);
            u
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{COMPUTE_BUDGET_BREAKPOINTS, ComputeBudget, Diagnostics, test_ctx, test_road, test_run_on};

    fn check_small_track_full_laps(percent: f32) {
        use crate::planning::{Latency, PlannerKind};
        use crate::world::LiveWorld;
        let mut world = LiveWorld::with_track(1, 1, PlannerKind::CenterlineFollower, 0, 0.1);
        world.preview_ticks = 100;
        world.compute_budget = ComputeBudget::from_percent(percent);
        let lap = world.track.lap_length().unwrap();
        for tick in 0..600 {
            world.tick_recording_latency(&Latency::default());
            assert_eq!(world.ego_collision_count, 0, "budget {percent}, tick {tick}");
            let end = world.trajectory.states.last().unwrap();
            assert!(
                end.speed > 0.5,
                "budget {percent}, tick {tick}, ego {:?}, preview end {end:?}",
                world.ego()
            );
            if world.track_progress > 2.0 * lap {
                break;
            }
        }
        assert!(
            world.track_progress > 2.0 * lap,
            "budget {percent}: failed to complete two laps"
        );
    }

    #[test]
    fn small_track_full_laps_minimum_budget() {
        check_small_track_full_laps(5.0);
    }

    #[test]
    fn small_track_full_laps_nominal_budget() {
        check_small_track_full_laps(100.0);
    }

    #[test]
    fn small_track_full_laps_maximum_budget() {
        check_small_track_full_laps(500.0);
    }

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
                    PlannerKind::CenterlineFollower,
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
                let start = ctx.project_ego(ego);
                let s0 = start.s;
                let targets = stations(speed, s0, path.length(), 0.1).unwrap();
                let curve = CartesianPath::new(ego, path, start, targets).unwrap();
                let samples: Vec<_> = (0..=400)
                    .map(|i| curve.at(curve.distance[GRID_STEPS] * i as f64 / 400.0))
                    .collect();
                let max_d = samples
                    .iter()
                    .map(|(pose, _)| path.project(pose.position).d.abs())
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
                let controls = CenterlineFollower.plan(ego, &ctx);
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
    fn frenet_beziers_merge_tangentially_then_follow_the_centerline() {
        let start = FrenetPosition { s: 20.0, d: 1.0 };
        let segments = frenet_segments(start, 0.1, [40.0, 80.0]);
        assert_eq!(segments[0].at(0.0)[0], Position::new(start.s, start.d));
        let initial_tangent = segments[0].at(0.0)[1];
        assert!((initial_tangent.y / initial_tangent.x - 0.1).abs() < 1e-12);
        assert_eq!(segments[0].at(1.0)[0], segments[1].at(0.0)[0]);
        assert_eq!(segments[0].at(1.0)[1].y, 0.0);
        for i in 0..=100 {
            let [p, d1, d2] = segments[1].at(i as f64 / 100.0);
            assert_eq!([p.y, d1.y, d2.y], [0.0; 3]);
            assert!(d1.x > 0.0);
        }
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let path = road.path();
        let ego = State::from((Position::new(0.0, 1.0), 0.1_f64.atan(), 8.0));
        let curve = CartesianPath::new(ego, &path, start, [40.0, 80.0]).unwrap();
        assert_eq!(curve.at(0.0).0.position, ego.position());
        assert!(wrap_angle(curve.at(0.0).0.yaw - ego.pose.yaw).abs() < 1e-12);
        assert_eq!(curve.at(curve.distance[50]).0.position, Position::new(20.0, 0.0));
        assert_eq!(curve.at(curve.distance[100]).0.position, Position::new(60.0, 0.0));
        assert!(curve.distance.windows(2).all(|w| w[1] > w[0]));
        assert!(curve.distance[100] > 60.0);
    }

    #[test]
    fn cartesian_samples_include_road_curvature_and_arc_length() {
        let radius = 40.0;
        let points: Vec<_> = (0..=1000)
            .map(|i| Position::from_angle(i as f64 * 0.0025) * radius)
            .collect();
        let path = Path::new(&points);
        let start = FrenetPosition { s: 10.0, d: 2.0 };
        let heading = path.heading_at(start.s);
        let ego = State::from((
            path.pose_at(start.s).0 + Position::from_angle(heading + std::f64::consts::FRAC_PI_2) * start.d,
            heading,
            5.0,
        ));
        let curve = CartesianPath::new(ego, &path, start, [40.0, 80.0]).unwrap();
        // The inner offset shortens the Cartesian distance despite the lateral merge.
        assert!(curve.distance[GRID_STEPS] < 70.0);
        for i in GRID_STEPS / 2..=GRID_STEPS {
            let (pose, curvature) = curve.at(curve.distance[i]);
            assert!((curvature - 1.0 / radius).abs() < 1e-8);
            assert!((pose.position.norm() - radius).abs() < 1e-4);
        }
        let backwards = State::from((ego.position(), heading + std::f64::consts::PI, 5.0));
        assert!(CartesianPath::new(backwards, &path, start, [40.0, 80.0]).is_none());
        assert!(CartesianPath::new(ego, &path, FrenetPosition { d: radius, ..start }, [40.0, 80.0]).is_none());
    }

    #[test]
    fn repeated_road_uses_one_ego_projection_for_stations_and_curve() {
        // Slightly different sampling of overlapping laps makes the later
        // copy geometrically closer, despite belonging to the wrong lap.
        let mut road = test_road(&[
            [-100.0, 0.01],
            [200.0, 0.01],
            [200.0, 100.0],
            [-100.0, 100.0],
            [-100.0, 0.0],
            [200.0, 0.0],
        ]);
        road.ego_projection_window = Some((160.0, 21.0));
        let ego = State::from((Position::new(60.0, 0.0), 0.0, 8.0));
        let ctx = test_ctx(&road, &[]);
        let path = ctx.path();

        let global = path.project(ego.position());
        let start = ctx.project_ego(ego);
        assert!(global.s > start.s + 500.0);
        assert!((start.s - 160.0).abs() < 1e-9);

        let curve = CartesianPath::new(ego, path, start, [start.s + 40.0, start.s + 80.0]).unwrap();
        let points: Vec<_> = (0..=100)
            .map(|i| curve.at(curve.distance[GRID_STEPS] * i as f64 / 100.0).0.position)
            .collect();
        assert!(points.windows(2).all(|pair| pair[1].x > pair[0].x));
        assert!((points.last().unwrap().x - 140.0).abs() < 1e-9);

        // A finite road must not inherit the live world's anchor assumption.
        let road = test_road(&[[-100.0, 0.0], [0.0, 0.0], [100.0, 0.0], [200.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        assert_eq!(ctx.project_ego(ego), ctx.path().project(ego.position()));
        assert!((ctx.project_ego(ego).s - 160.0).abs() < 1e-9);
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
        let end = 484.7325513507306;
        assert_eq!(stations(14.584, 66.85366782480057, end, 0.1).unwrap()[1], end);
    }

    #[test]
    fn budget_scales_centerline_merge_distances() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let mut previous = Vec::new();
        for percent in COMPUTE_BUDGET_BREAKPOINTS {
            let budget = ComputeBudget::from_percent(percent);
            let diagnostics = Diagnostics::default();
            let ctx = Context::new(&road, &[], 10, budget, None, Some(&diagnostics));
            let targets = path_targets(&ctx, 20.0, [80.0, 300.0]);
            assert_eq!(targets.len(), budget.scale(NOMINAL_PATHS, 3));
            assert!(targets.starts_with(&previous));
            assert_eq!(&targets[..3], &[[80.0, 300.0], [50.0, 300.0], [35.0, 300.0]]);
            for (i, target) in targets.iter().enumerate() {
                assert!(!targets[..i].contains(target));
                assert!(20.0 < target[0] && target[0] <= 80.0);
                assert_eq!(target[1], 300.0);
            }
            let controls = CenterlineFollower.plan(State::default(), &ctx);
            let data = diagnostics.take();
            assert_eq!(controls.len(), ctx.horizon);
            assert_eq!(data.trajectories.len(), targets.len());
            assert!(data.trajectories.iter().all(|points| points.len() == 101));
            assert!(data.trajectories.iter().flatten().all(|point| point.y.abs() < 1e-9));
            previous = targets;
        }
        assert_eq!(previous.len(), 45);
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
        let road = test_road(&[[-50.0, 0.0], [2_000.0, 0.0]]);
        for speed in [0.0, 20.0, 40.0, 70.0] {
            let ego = State {
                speed,
                ..Default::default()
            };
            let ctx = Context::new(&road, &[], 100, ComputeBudget::NOMINAL, None, None);
            let controls = CenterlineFollower.plan(ego, &ctx);
            for (tick, control) in controls.iter().enumerate() {
                assert!(
                    control.acceleration > MAX_LON_ACCEL - 0.05,
                    "initial speed {speed}, tick {tick}, control {control:?}"
                );
            }
        }
    }

    #[test]
    fn centerline_candidate_converges_to_centerline_and_accelerates() {
        let mut ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 3.0), 0.0),
            5.0,
        );
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let ctx = Context::new(&road, &[], 10, ComputeBudget::from_percent(5.0), None, None);
        for _ in 0..200 {
            let start = ctx.project_ego(ego);
            let targets = stations(ego.speed, start.s, ctx.path().length(), road.dt).unwrap();
            let curve = CartesianPath::new(ego, ctx.path(), start, targets).unwrap();
            let controls = parameterize(ego, &ctx, &curve, 100);
            ego = world_step(ego, controls[0], road.dt);
        }
        assert!(ego.position().y.abs() < 0.3, "offset {}", ego.position().y);
        assert!(ego.speed > 10.0, "speed {}", ego.speed);
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
        let road = crate::track::Road::new(vec![[-20.0, 0.0], [2_000.0, 0.0]], 1.6, 0.1);
        let trace = test_run_on(&mut CenterlineFollower, &road, ego, &[actor], 300);
        let end = trace.last().unwrap();
        assert!(end.speed < 0.5, "speed {}", end.speed);
        assert!(
            end.position().x <= actor.position().x - crate::common::geometry::EGO_FOOTPRINT.length + 1e-9,
            "x {}",
            end.position().x
        );
    }

    #[test]
    fn candidate_cost_uses_shared_metrics_and_rejects_off_road_rollouts() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let controls = vec![Control::default(); 100];
        let mut states = vec![ego];
        for &control in &controls {
            states.push(world_step(*states.last().unwrap(), control, road.dt));
        }
        let score = metrics::evaluate(&states, road.dt, &road.path(), None);
        assert_eq!(candidate_cost(ego, &ctx, &controls), -score);

        let off_road = State::from((Position::new(0.0, road.half_width + 1.0), 0.0, 8.0));
        assert!(candidate_cost(off_road, &ctx, &controls).is_infinite());
    }

    #[test]
    fn selects_lowest_cost_feasible_merge_and_checks_the_full_horizon() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let actors = [State::from((Position::new(50.0, 0.0), 0.0, 0.0))];
        let ego = State::from((Position::new(0.0, 2.0), 0.0, 8.0));
        let ctx = Context::new(&road, &actors, 100, ComputeBudget::NOMINAL, None, None);
        let start = ctx.project_ego(ego);
        let stations = stations(ego.speed, start.s, ctx.path().length(), road.dt).unwrap();
        let minimum = path_targets(&ctx, start.s, stations)
            .into_iter()
            .filter_map(|targets| {
                let curve = CartesianPath::new(ego, ctx.path(), start, targets)?;
                Some(candidate_cost(ego, &ctx, &parameterize(ego, &ctx, &curve, 100)))
            })
            .fold(f64::INFINITY, f64::min);
        assert!(minimum.is_finite());
        let selected = CenterlineFollower.plan(ego, &ctx);
        assert_eq!(candidate_cost(ego, &ctx, &selected), minimum);
        let short = CenterlineFollower.plan(ego, &test_ctx(&road, &actors));
        assert_eq!(short, selected[..short.len()]);
        assert!(candidate_cost(ego, &ctx, &vec![Control::default(); 100]).is_infinite());
    }

    #[test]
    fn replans_from_current_state_after_skipped_ticks_and_new_obstacles() {
        let road = crate::track::Road::new(vec![[-20.0, 0.0], [2_000.0, 0.0]], 1.6, 0.1);
        let mut planner = CenterlineFollower;
        let mut ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let ctx = Context::new(&road, &[], 100, ComputeBudget::NOMINAL, None, None);
        let clear = planner.plan(ego, &ctx);
        for &u in &clear[..3] {
            ego = world_step(ego, u, road.dt);
        }
        let actors = [State::from((Position::new(50.0, 0.0), 0.0, 0.0))];
        let diagnostics = Diagnostics::default();
        let blocked = Context::new(&road, &actors, 100, ComputeBudget::NOMINAL, None, Some(&diagnostics));
        assert!(candidate_cost(ego, &blocked, &clear[3..]).is_infinite());
        diagnostics.take();
        let controls = planner.plan(ego, &blocked);
        assert!(diagnostics.take().trajectories.len() == NOMINAL_PATHS);
        assert!(candidate_cost(ego, &blocked, &controls).is_finite());
        let end = controls.iter().fold(ego, |x, &u| world_step(x, u, road.dt));
        assert!(end.speed < 0.5 && end.position().x < 50.0 - EGO_FOOTPRINT.length);
    }

    #[test]
    fn exhausted_route_brakes_to_standstill_without_reversing() {
        let road = test_road(&[[-20.0, 0.0], [0.0, 0.0]]);
        let ctx = Context::new(&road, &[], 100, ComputeBudget::NOMINAL, None, None);
        let mut ego = State {
            speed: 8.0,
            ..Default::default()
        };
        for u in CenterlineFollower.plan(ego, &ctx) {
            ego = world_step(ego, u, road.dt);
            assert!(ego.speed >= -1e-9);
        }
        assert!(ego.speed.abs() < 1e-9);
        assert!(CenterlineFollower.plan(ego, &Context { horizon: 0, ..ctx }).is_empty());
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
        use crate::common::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};
        use crate::planning::{test_ctx, test_road};
        use crate::simulation::CommandLimiter;

        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let mut planner = CenterlineFollower;
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
        use crate::common::geometry::barrier::collides_with_road_barrier;
        use crate::track::{Road, TRACK_PRESETS, Track};

        for track_index in 0..TRACK_PRESETS.len() {
            let track = Track::from_catalog(track_index);
            let lap = track.lap_length().unwrap();
            for n in 0..20 {
                let progress = lap * n as f64 / 20.0;
                let road = Road::new(
                    track.centerline(progress - 50.0, progress + 250.0, 15.0),
                    track.half_width(progress),
                    0.1,
                );
                let (p, yaw) = track.pose(progress);
                let ego = State::from((p, yaw, 20.0));
                let ctx = Context::new(&road, &[], 100, crate::planning::ComputeBudget::NOMINAL, None, None);
                let path = Path::new(road.centerline());
                let mut state = ego;
                for (tick, control) in CenterlineFollower.plan(ego, &ctx).into_iter().enumerate() {
                    state = world_step(state, control, road.dt);
                    let FrenetPosition { s, d } = path.project(state.position());
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
