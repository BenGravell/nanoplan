//! Frenet cubic sampling with kinodynamic rejection, cost sorting, then collision checks.

use crate::common::geometry::barrier::collides_with_road_barrier;
use crate::common::geometry::{EGO_FOOTPRINT, Footprint, angle_delta};
use crate::common::interp::lerp;
use crate::common::kinematics::{commanded_accel_for_net, commanded_accel_to_stop};
use crate::common::measure::dot;
use crate::common::polynomial::CubicPolynomial;
use crate::common::types::{Control, Position, State};
use crate::constraints::collision::actor_collision;
use crate::constraints::{Constraint, Constraints, Kinodynamic, Sample};
use crate::metrics;
use crate::planning::planner_math::state_sample;
use crate::planning::policy::centerline_curvature;
use crate::planning::{ComputeBudget, Context, PLANNING_HORIZON_S, Planner};
use crate::simulation::{world_step, world_step_unclamped};
use crate::track::Path;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

const STATION_SAMPLES_PER_INTERVAL: usize = 9;
const SPEED_SAMPLES_PER_INTERVAL: usize = 5;
const LATERAL_SAMPLES: usize = 11;
const TERMINAL_LATERAL_SPEEDS_MPS: [f64; 3] = [-0.5, 0.0, 0.5];
// Below this speed, establish the initial heading with a straight acceleration prefix.
const CUBIC_LAUNCH_SPEED_MPS: f64 = 2.0;
// Match the existing Bezier planner's clearance for rolling-window/circuit seams.
const ROAD_FOOTPRINT: Footprint = Footprint::new(EGO_FOOTPRINT.length + 0.1, EGO_FOOTPRINT.width + 0.2);

#[derive(Default)]
pub(crate) struct FrenetSamplingPlanner {
    previous: Vec<(State, Control)>,
}

impl Planner for FrenetSamplingPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let (path, s0) = ctx.time("route", || route(ego, ctx));
        let mut controls = ctx.time("fit", || {
            best_candidate(ego, path, ctx, s0)
                .or_else(|| {
                    let braking = fallback_controls(ego, path, ctx);
                    if feasible_candidate_trajectory(ego, &braking, path, ctx).is_some() {
                        return Some(braking);
                    }
                    // A fresh grid can miss a safe continuation on the next tick.
                    // Align by state and revalidate against the current road and actors.
                    let (index, _) = self.previous.iter().enumerate().min_by(|(_, (a, _)), (_, (b, _))| {
                        let error = |state: &State| {
                            state.position().distance(ego.position()) + (state.speed - ego.speed).abs() * ctx.road.dt
                        };
                        error(a).total_cmp(&error(b))
                    })?;
                    let controls: Vec<_> = self.previous[index..].iter().map(|&(_, u)| u).collect();
                    feasible_candidate_trajectory(ego, &controls, path, ctx)?;
                    Some(controls)
                })
                .unwrap_or_else(|| fallback_controls(ego, path, ctx))
        });
        self.previous.clear();
        let mut state = ego;
        for &control in &controls {
            self.previous.push((state, control));
            state = world_step(state, control, ctx.road.dt);
        }
        controls.truncate(ctx.horizon);
        controls
    }
}

fn route<'a>(ego: State, ctx: &'a Context<'_>) -> (&'a Path, f64) {
    let path = ctx.path();
    let s0 = ctx.project_ego(ego).s;
    (path, s0)
}

/// Brake along the road to a stop.
fn fallback_controls(mut state: State, path: &Path, ctx: &Context) -> Vec<Control> {
    (0..ctx.horizon.max((PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize))
        .map(|_| {
            let control = Control {
                acceleration: MIN_LON_ACCEL.max(commanded_accel_to_stop(state.speed, ctx.road.dt)),
                curvature: centerline_curvature(path, &state),
            };
            state = world_step(state, control, ctx.road.dt);
            control
        })
        .collect()
}

struct Motion {
    longitudinal: CubicPolynomial,
    lateral: CubicPolynomial,
}

impl Motion {
    /// Transform Frenet position and derivatives into Cartesian state and net controls.
    fn at(&self, path: &Path, t: f64) -> Option<(State, Control)> {
        let [s, sv, sa] = self.longitudinal.at(t);
        let [d, dv, da] = self.lateral.at(t);
        if !(0.0..=path.length()).contains(&s) || sv < -1e-8 {
            return None;
        }
        let k = path.curvature_at(s);
        let dk = path.sharpness_at(s);
        let scale = 1.0 - k * d;
        if scale <= 0.1 {
            return None;
        }
        let vx = scale * sv;
        let ax = scale * sa - dk * d * sv * sv - 2.0 * k * sv * dv;
        let ay = da + k * scale * sv * sv;
        let velocity = Position::new(vx, dv);
        let acceleration = Position::new(ax, ay);
        let speed = velocity.norm();
        let (acceleration, curvature) = if speed > 1e-6 {
            (
                dot(velocity.xy(), acceleration.xy()) / speed,
                velocity.cross(acceleration) / speed.powi(3),
            )
        } else {
            (ax, 0.0)
        };
        let heading = path.heading_at(s);
        let left = Position::from_angle(heading + std::f64::consts::FRAC_PI_2);
        let state = State::from((path.pose_at(s).0 + left * d, heading + velocity.angle(), speed));
        let control = Control {
            acceleration,
            curvature,
        };
        (state.position().is_finite()
            && state.pose.yaw.is_finite()
            && speed.is_finite()
            && acceleration.is_finite()
            && curvature.is_finite())
        .then_some((state, control))
    }
}

struct Candidate {
    states: Vec<State>,
    controls: Vec<Control>,
}

fn best_candidate(ego: State, path: &Path, ctx: &Context, s0: f64) -> Option<Vec<Control>> {
    let ticks = (PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize;
    let mut launch = Vec::new();
    let mut start = ego;
    while start.speed < CUBIC_LAUNCH_SPEED_MPS && launch.len() + 1 < ticks {
        let control = Control {
            acceleration: MAX_LON_ACCEL,
            curvature: 0.0,
        };
        start = world_step(start, control, ctx.road.dt);
        launch.push(control);
    }
    let launch_ticks = launch.len();
    let duration = PLANNING_HORIZON_S - launch_ticks as f64 * ctx.road.dt;
    let projected = path.project_near(start.position(), s0, 15.0);
    let heading = angle_delta(path.heading_at(projected.s), start.pose.yaw);
    let scale = 1.0 - path.curvature_at(projected.s) * projected.d;
    if scale <= 0.1 || ego.speed < 0.0 || heading.cos() < 0.0 {
        return None;
    }
    let lon = [projected.s, start.speed * heading.cos() / scale];
    let lat = [projected.d, start.speed * heading.sin()];
    let width = (ctx.road.half_width - EGO_FOOTPRINT.width / 2.0).max(0.0);
    let lateral_count = lateral_sample_count(ctx.compute_budget);
    let mut candidates = Vec::new();
    for (distance, speed) in longitudinal_targets(start.speed, ctx.road.dt, ctx.compute_budget, duration) {
        let station = lon[0] + distance;
        if station > path.length() {
            continue;
        }
        // A reflected lateral endpoint strengthens receding-horizon centerline recovery.
        let targets = std::iter::once((-lat[0], 0.0)).chain((0..lateral_count).flat_map(|i| {
            let lateral = lerp(-width, width, i as f64 / (lateral_count - 1) as f64);
            TERMINAL_LATERAL_SPEEDS_MPS.into_iter().map(move |v| (lateral, v))
        }));
        for (lateral, lateral_speed) in targets {
            let motion = Motion {
                longitudinal: CubicPolynomial::from_boundary(lon[0], lon[1], station, speed, duration),
                lateral: CubicPolynomial::from_boundary(lat[0], lat[1], lateral, lateral_speed, duration),
            };
            let candidate = ctx.time("kinodynamic", || {
                rollout(ego, ctx, ticks, |tick, actual| {
                    if tick < launch_ticks {
                        return Some(launch[tick]);
                    }
                    let t = ((tick - launch_ticks) as f64 + 0.5) * ctx.road.dt;
                    let (state, mut control) = motion.at(path, t.min(duration))?;
                    let analytic = Control {
                        acceleration: commanded_accel_for_net(control.acceleration, state.speed),
                        ..control
                    };
                    if Kinodynamic.is_violated(&Sample::default().with_control(analytic, state.speed)) {
                        return None;
                    }
                    control.acceleration = commanded_accel_for_net(control.acceleration, actual.speed);
                    Some(control)
                })
            });
            if let Some(candidate) = candidate {
                candidates.push((station, candidate));
            }
        }
    }
    let candidates = candidates
        .into_iter()
        .filter_map(|(station, candidate)| {
            let cost = ctx.time("cost", || -compute_score(&candidate, path, ctx, station));
            cost.is_finite().then_some((cost, candidate))
        })
        .collect();
    select_candidate(candidates, path, ctx)
}

fn select_candidate(mut candidates: Vec<(f64, Candidate)>, path: &Path, ctx: &Context) -> Option<Vec<Control>> {
    // Stable order preserves sampling order for equal costs.
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    if let Some(diagnostics) = ctx.diagnostics {
        // Show the sampled options without collision-checking them just for display.
        for (_, candidate) in &candidates {
            diagnostics.record_trajectory(candidate.states.iter().map(|state| state.position()).collect());
        }
    }
    for (_, candidate) in candidates {
        if ctx.time("collision", || road_and_collision_feasible(&candidate, path, ctx)) {
            return Some(candidate.controls);
        }
    }
    None
}

/// Cheap first pass: check unclipped commands and cache their Cartesian rollout once.
fn rollout(
    ego: State,
    ctx: &Context,
    ticks: usize,
    mut control_at: impl FnMut(usize, State) -> Option<Control>,
) -> Option<Candidate> {
    let mut candidate = Candidate {
        states: Vec::with_capacity(ticks + 1),
        controls: Vec::with_capacity(ticks),
    };
    candidate.states.push(ego);
    let mut state = ego;
    for tick in 0..ticks {
        ctx.work(1);
        let control = control_at(tick, state)?;
        if Kinodynamic.is_violated(&Sample::default().with_control(control, state.speed)) {
            return None;
        }
        state = world_step_unclamped(state, control, ctx.road.dt);
        if state.speed < -1e-9
            || !state.speed.is_finite()
            || !state.pose.yaw.is_finite()
            || !state.position().is_finite()
        {
            return None;
        }
        candidate.controls.push(control);
        candidate.states.push(state);
    }
    Some(candidate)
}

fn sampling_scale(budget: ComputeBudget) -> f64 {
    let nominal = 4 * STATION_SAMPLES_PER_INTERVAL * SPEED_SAMPLES_PER_INTERVAL * LATERAL_SAMPLES;
    (budget.scale(nominal, 1) as f64 / nominal as f64).cbrt()
}

fn lateral_sample_count(budget: ComputeBudget) -> usize {
    // Odd counts always include the centerline, including at the lowest budget.
    2 * (((LATERAL_SAMPLES / 2) as f64 * sampling_scale(budget)).round() as usize).max(1) + 1
}

fn sample_counts(budget: ComputeBudget) -> (usize, usize) {
    // Scale the three position/speed axes together; lateral velocity keeps three samples.
    let scale = sampling_scale(budget);
    let station_count = ((STATION_SAMPLES_PER_INTERVAL as f64 * scale).round() as usize).max(2);
    let speed_count = ((SPEED_SAMPLES_PER_INTERVAL as f64 * scale).round() as usize).max(2);
    (station_count, speed_count)
}

/// Cartesian product of station and terminal-speed samples around the zero-thrust rollout.
fn longitudinal_targets(initial_speed: f64, dt: f64, budget: ComputeBudget, duration: f64) -> Vec<(f64, f64)> {
    let rollout = |acceleration| {
        let mut state = State {
            speed: initial_speed,
            ..Default::default()
        };
        let ticks = (duration / dt).ceil() as usize;
        for tick in 0..ticks {
            let step_dt = dt.min(duration - tick as f64 * dt);
            state = world_step(
                state,
                Control {
                    acceleration,
                    curvature: 0.0,
                },
                step_dt,
            );
            // Maximum braking ends at rest, rather than continuing in reverse.
            state.speed = state.speed.max(0.0);
        }
        (state.position().x, state.speed)
    };
    let nominal = rollout(0.0);
    let min = rollout(MIN_LON_ACCEL);
    let max = rollout(MAX_LON_ACCEL);

    let samples = |min, nominal, max, count| {
        // Include both extrema and nominal once: [min, nominal), then [nominal, max].
        // Densify the upper interval near nominal so slow rolling cubics are not skipped.
        (0..count)
            .map(move |i| lerp(min, nominal, i as f64 / count as f64))
            .chain((0..count).map(move |i| lerp(nominal, max, (i as f64 / (count - 1) as f64).powi(2))))
    };
    let (station_count, speed_count) = sample_counts(budget);
    let stations = samples(min.0, nominal.0, max.0, station_count);
    let speeds = samples(min.1, nominal.1, max.1, speed_count);
    let mut targets = Vec::with_capacity(stations.clone().count() * speeds.clone().count());
    for station in stations {
        for speed in speeds.clone() {
            targets.push((station, speed));
        }
    }
    targets
}

fn feasible_candidate_trajectory(ego: State, controls: &[Control], path: &Path, ctx: &Context) -> Option<Candidate> {
    let candidate = rollout(ego, ctx, controls.len(), |tick, _| Some(controls[tick]))?;
    if !road_and_collision_feasible(&candidate, path, ctx) {
        return None;
    }
    if let Some(diag) = ctx.diagnostics {
        diag.record_trajectory(candidate.states.iter().map(|state| state.position()).collect());
    }
    Some(candidate)
}

/// Expensive last pass, using cached Cartesian states in increasing cost order.
fn road_and_collision_feasible(candidate: &Candidate, path: &Path, ctx: &Context) -> bool {
    let ego = candidate.states[0];
    let mut station = ctx.project_ego(ego).s;
    let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, ego.speed, station);
    let (end, end_yaw) = path.pose_at(path.length());
    let forward = Position::from_angle(end_yaw);
    for (tick, pair) in candidate.states.windows(2).enumerate() {
        ctx.work(1);
        let [prev, state] = [pair[0], pair[1]];
        let (s, mut sample) = state_sample(
            path,
            &state,
            (tick + 1) as f64 * ctx.road.dt,
            Some(station + prev.speed * ctx.road.dt),
        );
        station = s;
        sample.road_bounds = Some(ctx.road.lateral_bounds_at(station));
        let beyond_end = dot((state.position() - end).xy(), forward.xy()) > 0.0 && station >= path.length() - 1e-6;
        if beyond_end
            || collides_with_road_barrier(state, ctx.road)
            || constraints.is_transition_violated(prev, state, ROAD_FOOTPRINT, ctx.road, &sample)
            || actor_collision(state.pose(), sample.t, ctx)
        {
            return false;
        }
    }
    true
}

fn compute_score(candidate: &Candidate, path: &Path, ctx: &Context, end_station_hint: f64) -> f64 {
    let ego = candidate.states[0];
    let end = candidate.states.last().unwrap();
    let progress = path.project_near(end.position(), end_station_hint, 15.0).s - ctx.project_ego(ego).s;
    // The shared objective only needs endpoint progress, not all trajectory projections.
    metrics::progress_score(progress, ego.speed, candidate.controls.len() as f64 * ctx.road.dt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::geometry::wrap_angle;
    use crate::planning::{Diagnostics, test_ctx, test_road, test_run, test_run_on};
    use crate::simulation::Position;
    use crate::simulation::world_step;
    use crate::track::Track;

    #[test]
    #[ignore = "manual fixed-workload timing; no hardware-dependent assertion"]
    fn benchmark_fixed_workloads() {
        let straight = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let track = Track::from_catalog(1);
        let bend = track.prepared().window(250.0, 8.0, 0.1);
        let (point, yaw) = track.pose(250.0);
        for (name, road, ego) in [
            (
                "straight",
                &straight,
                State {
                    speed: 20.0,
                    ..Default::default()
                },
            ),
            ("bend", &bend, State::from((point, yaw, 8.0))),
        ] {
            let mut timings = Vec::new();
            for i in 0..8 {
                let ctx = test_ctx(road, &[]);
                let start = std::time::Instant::now();
                let controls = FrenetSamplingPlanner::default().plan(ego, &ctx);
                std::hint::black_box(controls);
                if i > 0 {
                    timings.push(start.elapsed().as_secs_f64() * 1e3);
                }
            }
            timings.sort_by(f64::total_cmp);
            eprintln!("{name}: median {:.3} ms", timings[timings.len() / 2]);
        }
    }

    #[test]
    fn frenet_cubics_match_boundaries_and_follow_a_curved_reference() {
        for end_velocity in TERMINAL_LATERAL_SPEEDS_MPS {
            let polynomial = CubicPolynomial::from_boundary(2.0, 0.3, -1.0, end_velocity, 3.0);
            assert_eq!(polynomial.at(0.0)[..2], [2.0, 0.3]);
            let end = polynomial.at(3.0);
            assert!((end[0] + 1.0).abs() < 1e-10);
            assert!((end[1] - end_velocity).abs() < 1e-10);
        }
        let radius = 40.0;
        let points: Vec<_> = (0..1800)
            .map(|i| {
                let angle = i as f64 * 0.002;
                Position::new(radius * angle.cos(), radius * angle.sin())
            })
            .collect();
        let path = Path::new(&points);
        let motion = Motion {
            longitudinal: CubicPolynomial([10.0, 8.0, 0.0, 0.0]),
            lateral: CubicPolynomial([2.0, 0.0, 0.0, 0.0]),
        };
        for t in [0.0, 2.0, 5.0, 10.0] {
            let (state, control) = motion.at(&path, t).unwrap();
            // A constant Frenet offset traces the concentric circle, not a Cartesian chord.
            assert!((state.position().x.hypot(state.position().y) - 38.0).abs() < 1e-4);
            assert!((state.speed - 7.6).abs() < 1e-8);
            assert!((control.curvature - 1.0 / 38.0).abs() < 1e-8);
            assert!(control.acceleration.abs() < 1e-8);
            let tangent = state.position().y.atan2(state.position().x) + std::f64::consts::FRAC_PI_2;
            assert!(wrap_angle(state.pose.yaw - tangent).abs() < 1e-8);
        }
        // Frenet chart singularities and backwards station motion are not valid trajectories.
        let singular = Motion {
            lateral: CubicPolynomial([41.0, 0.0, 0.0, 0.0]),
            ..motion
        };
        assert!(singular.at(&path, 1.0).is_none());
        let reverse = Motion {
            longitudinal: CubicPolynomial([10.0, -1.0, 0.0, 0.0]),
            ..singular
        };
        assert!(reverse.at(&path, 1.0).is_none());
    }

    #[test]
    fn frenet_transform_accounts_for_changing_reference_curvature() {
        let points: Vec<_> = (0..3000)
            .map(|i| {
                let x = i as f64 * 0.05;
                Position::new(x, 0.01 * x * x)
            })
            .collect();
        let path = Path::new(&points);
        let motion = Motion {
            longitudinal: CubicPolynomial([10.0, 8.0, 0.15, 0.0]),
            lateral: CubicPolynomial([2.0, 0.3, 0.1, 0.0]),
        };
        let t = 1.234;
        let epsilon = 1e-5;
        let (before, _) = motion.at(&path, t - epsilon).unwrap();
        let (state, control) = motion.at(&path, t).unwrap();
        let (after, _) = motion.at(&path, t + epsilon).unwrap();
        assert!(path.sharpness_at(motion.longitudinal.at(t)[0]).abs() > 1e-5);
        let acceleration = (after.speed - before.speed) / (2.0 * epsilon);
        let curvature = wrap_angle(after.pose.yaw - before.pose.yaw) / (2.0 * epsilon * state.speed);
        assert!((control.acceleration - acceleration).abs() < 1e-6);
        assert!((control.curvature - curvature).abs() < 1e-5);
    }

    #[test]
    fn kinodynamic_pass_rejects_raw_controls_before_cost_or_collision_work() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let ego = State {
            speed: 20.0,
            ..Default::default()
        };
        let ctx = test_ctx(&road, &[]);
        for control in [
            Control {
                acceleration: MAX_LON_ACCEL + 0.01,
                curvature: 0.0,
            },
            Control {
                acceleration: MIN_LON_ACCEL - 0.01,
                curvature: 0.0,
            },
            Control {
                acceleration: 0.0,
                curvature: crate::simulation::curvature_limit(ego.speed) + 0.01,
            },
            Control {
                acceleration: f64::NAN,
                curvature: 0.0,
            },
        ] {
            let mut calls = 0;
            assert!(
                rollout(ego, &ctx, 100, |_, _| {
                    calls += 1;
                    Some(control)
                })
                .is_none()
            );
            assert_eq!(calls, 1);
        }
        let latency = crate::planning::Latency::default();
        let ctx = Context {
            latency: Some(&latency),
            ..test_ctx(&road, &[])
        };
        assert!(best_candidate(ego, ctx.path(), &ctx, 20.0).is_some());
        let spans = latency.take();
        let count = |name: &str| spans.iter().filter(|span| span.name == name).count();
        assert!(count("kinodynamic") > count("cost"));
        assert!(count("cost") > 1);
        assert!(count("collision") > 0 && count("collision") < count("cost"));
        let first_cost = spans.iter().position(|span| span.name == "cost").unwrap();
        assert!(spans[..first_cost].iter().all(|span| span.name == "kinodynamic"));
        assert!(spans[first_cost..].iter().all(|span| span.name != "kinodynamic"));
        let first_collision = spans.iter().position(|span| span.name == "collision").unwrap();
        assert!(spans[first_collision..].iter().all(|span| span.name == "collision"));
        eprintln!(
            "stages: {} generated, {} scored, {} collision checked",
            count("kinodynamic"),
            count("cost"),
            count("collision")
        );
    }

    #[test]
    fn cost_order_skips_a_collision_beyond_the_output_horizon_and_stops_at_first_clear_candidate() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let actors = [State::from((Position::new(40.0, 0.0), 0.0, 0.0))];
        let latency = crate::planning::Latency::default();
        let ctx = Context {
            horizon: 1,
            latency: Some(&latency),
            ..test_ctx(&road, &actors)
        };
        let ticks = (PLANNING_HORIZON_S / road.dt).ceil() as usize;
        let accelerating = rollout(ego, &ctx, ticks, |_, _| {
            Some(Control {
                acceleration: MAX_LON_ACCEL,
                curvature: 0.0,
            })
        })
        .unwrap();
        let braking = fallback_controls(ego, ctx.path(), &ctx);
        let clear = rollout(ego, &ctx, ticks, |tick, _| Some(braking[tick])).unwrap();
        let unvisited = rollout(ego, &ctx, ticks, |tick, _| Some(braking[tick])).unwrap();
        let candidates = vec![(3.0, unvisited), (1.0, accelerating), (2.0, clear)];
        let chosen = select_candidate(candidates, ctx.path(), &ctx).unwrap();
        assert_eq!(chosen, braking);
        assert_eq!(latency.take().iter().filter(|span| span.name == "collision").count(), 2);
    }

    #[test]
    fn completes_small_track_with_five_opponents() {
        use crate::planning::{Latency, PlannerKind};
        use crate::world::LiveWorld;

        let mut world = LiveWorld::with_track(1, 1, PlannerKind::FrenetSampling, 5, 0.1);
        let lap = world.track.lap_length().unwrap();
        let latency = Latency::default();
        for _ in 0..800 {
            let previous = world.ego();
            world.tick_recording_latency_blocking(&latency);
            latency.take();
            assert_eq!(
                world.ego_collision_count,
                0,
                "progress {}, previous {:?}, ego {:?}, control {:?}",
                world.track_progress,
                previous,
                world.ego(),
                world.actuation()
            );
            if world.track_progress >= lap {
                break;
            }
        }
        assert!(
            world.track_progress >= lap,
            "progress {} / {lap}, speed {}, contacts {}",
            world.track_progress,
            world.ego().speed,
            world.ego_collision_count
        );
        assert_eq!(world.ego_collision_count, 0);
    }

    #[test]
    fn restarts_from_the_stopped_small_track_snapshot() {
        use crate::planning::{ComputeBudget, PlannerKind};
        use crate::world::{EgoStart, LiveWorld};

        let world = LiveWorld::with_track_at(
            1,
            1,
            PlannerKind::FrenetSampling,
            0,
            0.1,
            EgoStart {
                progress: 294.902,
                ..Default::default()
            },
        );
        let ego = State::from((
            Position::new(153.13624873412616, -21.39139512061829),
            -1.938120058194263,
            0.0,
        ));
        let mut ctx = test_ctx(&world.road, &[]);
        for percent in [5.0, 100.0] {
            ctx.compute_budget = ComputeBudget::from_percent(percent);
            let controls = FrenetSamplingPlanner::default().plan(ego, &ctx);
            assert!(controls[0].acceleration > 0.0, "budget {percent}: {:?}", controls[0]);
            let mut state = ego;
            for _ in 0..100 {
                let mut ctx = test_ctx(&world.road, &[]);
                ctx.compute_budget = ComputeBudget::from_percent(percent);
                state = world_step(
                    state,
                    FrenetSamplingPlanner::default().plan(state, &ctx)[0],
                    world.road.dt,
                );
            }
            assert!(
                state.position().distance(ego.position()) > 10.0,
                "budget {percent}: {state:?}"
            );
        }
    }

    #[test]
    fn stops_for_a_blocking_opponent_and_restarts_when_clear() {
        use crate::common::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};

        // Leave no room to pass: lateral sampling should still stop when blocked.
        let road = crate::track::Road::new(vec![[-20.0, 0.0], [1_000.0, 0.0]], 2.0, 0.1);
        let ego = State::from((Position::new(0.0, 0.0), 0.0, 8.0));
        let actor = State::from((Position::new(30.0, 0.0), 0.0, 0.0));
        let trace = test_run_on(&mut FrenetSamplingPlanner::default(), &road, ego, &[actor], 150);
        let stopped = *trace.last().unwrap();
        assert!(stopped.position().x < actor.position().x);
        assert!(stopped.speed.abs() < 0.1, "speed {}", stopped.speed);
        let controls = FrenetSamplingPlanner::default().plan(stopped, &test_ctx(&road, &[]));
        assert!(controls[0].acceleration > 0.0);
        assert_eq!(controls[0].curvature, 0.0);
        assert!(trace.iter().all(|state| !footprints_overlap(
            state.pose(),
            EGO_FOOTPRINT,
            actor.pose(),
            CAR_FOOTPRINT
        )));
    }

    #[test]
    fn converges_to_centerline() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 3.0), 0.0),
            8.0,
        );
        let trace = test_run(&mut FrenetSamplingPlanner::default(), ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 0.4, "offset {}", end.position().y);
    }

    #[test]
    fn rolling_road_window_recovers_from_a_lateral_offset() {
        let track = Track::from_catalog(1);
        let road = test_road(&track.centerline(-50.0, 250.0, 15.0));
        let path = Path::new(road.centerline());
        let (p, yaw) = path.pose_at(50.0);
        let left = Position::from_angle(yaw + std::f64::consts::FRAC_PI_2);
        let ego = State::from((Position::new(p.x + 3.0 * left.x, p.y + 3.0 * left.y), yaw, 8.0));

        // Free lateral sampling may favor an offset on a bend, but must recover safely.
        let trace = test_run_on(&mut FrenetSamplingPlanner::default(), &road, ego, &[], 200);
        let d = path.project(trace.last().unwrap().position()).d;
        assert!(d.abs() < 3.0, "offset {d}");
        assert!(
            trace
                .iter()
                .all(|state| !crate::common::geometry::barrier::collides_with_road_barrier(*state, &road))
        );
    }

    #[test]
    fn accelerates_on_a_clear_straight() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let controls = FrenetSamplingPlanner::default().plan(State::default(), &test_ctx(&road, &[]));
        assert!(controls[0].acceleration > 0.0);
        assert!(controls.iter().all(|u| u.acceleration <= MAX_LON_ACCEL));
        assert!(controls.iter().all(|u| u.curvature.is_finite()));
    }

    #[test]
    fn targets_use_relative_station_and_resistance_over_the_planning_horizon() {
        let speed = 8.0;
        let targets = longitudinal_targets(speed, 0.1, ComputeBudget::NOMINAL, PLANNING_HORIZON_S);
        let nominal = (0..100).fold(
            State {
                speed,
                ..Default::default()
            },
            |state, _| world_step(state, Control::default(), 0.1),
        );
        assert!(targets.contains(&(nominal.position().x, nominal.speed)));
        assert!(nominal.position().x < speed * PLANNING_HORIZON_S);
        assert!(nominal.speed < speed);
        assert_eq!(targets.first().unwrap().1, 0.0);
        assert!(targets.first().unwrap().0 > 0.0);
        let station_count = 2 * STATION_SAMPLES_PER_INTERVAL;
        let speed_count = 2 * SPEED_SAMPLES_PER_INTERVAL;
        assert_eq!(targets.len(), station_count * speed_count);
        let mut stations: Vec<_> = targets.iter().map(|target| target.0).collect();
        stations.dedup();
        let mut speeds: Vec<_> = targets[..speed_count].iter().map(|target| target.1).collect();
        speeds.dedup();
        assert_eq!(stations.len(), station_count);
        assert_eq!(speeds.len(), speed_count);
        assert_eq!(
            stations.iter().filter(|&&s| s < nominal.position().x).count(),
            STATION_SAMPLES_PER_INTERVAL
        );
        assert_eq!(
            speeds.iter().filter(|&&v| v < nominal.speed).count(),
            SPEED_SAMPLES_PER_INTERVAL
        );
        for station in stations {
            for &speed in &speeds {
                assert!(targets.contains(&(station, speed)));
            }
        }
        let end = (0..100).fold(
            State {
                speed,
                ..Default::default()
            },
            |state, _| {
                world_step(
                    state,
                    Control {
                        acceleration: MAX_LON_ACCEL,
                        curvature: 0.0,
                    },
                    0.1,
                )
            },
        );
        assert_eq!(*targets.last().unwrap(), (end.position().x, end.speed));
        assert!(end.position().x < speed * PLANNING_HORIZON_S + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2));
    }

    #[test]
    fn candidate_count_scales_with_compute_budget() {
        let nominal = longitudinal_targets(8.0, 0.1, ComputeBudget::NOMINAL, PLANNING_HORIZON_S);
        let nominal_target =
            nominal[STATION_SAMPLES_PER_INTERVAL * 2 * SPEED_SAMPLES_PER_INTERVAL + SPEED_SAMPLES_PER_INTERVAL];
        let mut previous = 0;
        for percent in crate::planning::COMPUTE_BUDGET_BREAKPOINTS {
            let targets = longitudinal_targets(8.0, 0.1, ComputeBudget::from_percent(percent), PLANNING_HORIZON_S);
            let count = targets.len() * (1 + lateral_sample_count(ComputeBudget::from_percent(percent)) * 3);
            assert!(count > previous, "{percent}%: {count}");
            previous = count;
            assert_eq!(targets.first(), nominal.first());
            assert_eq!(targets.last(), nominal.last());
            assert!(targets.contains(&nominal_target));
            assert!(
                targets
                    .iter()
                    .all(|(station, speed)| station.is_finite() && speed.is_finite())
            );
        }
    }

    #[test]
    fn rejects_targets_beyond_the_road_window() {
        let road = test_road(&[[-5.0, 0.0], [1.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let ego = State {
            speed: 10.0,
            ..Default::default()
        };
        assert!(best_candidate(ego, ctx.path(), &ctx, 5.0).is_none());
    }

    #[test]
    fn stops_inside_a_finite_road_window() {
        let road = test_road(&[[-20.0, 0.0], [80.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let trace = test_run_on(&mut FrenetSamplingPlanner::default(), &road, ego, &[], 200);
        assert!(trace.iter().all(|state| state.position().x <= 80.0));
        assert!(trace.last().unwrap().speed.abs() < 0.1, "end {:?}", trace.last());
    }

    #[test]
    fn returns_a_full_horizon() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let plan = FrenetSamplingPlanner::default().plan(
            State::new(
                crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
                6.0,
            ),
            &ctx,
        );
        assert_eq!(plan.len(), ctx.horizon);
    }

    #[test]
    fn fixed_horizon_and_relative_origin_do_not_depend_on_output_length_or_road_origin() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let mut ctx = test_ctx(&road, &[]);
        let short = FrenetSamplingPlanner::default().plan(ego, &ctx);
        ctx.horizon = (PLANNING_HORIZON_S / road.dt).ceil() as usize;
        let full = FrenetSamplingPlanner::default().plan(ego, &ctx);
        assert_eq!(short, full[..short.len()]);
        ctx.horizon = 0;
        assert!(FrenetSamplingPlanner::default().plan(ego, &ctx).is_empty());

        // Rotating and shifting the road changes absolute station and yaw, not relative targets.
        let shifted = test_road(&[[30.0, -100.0], [30.0, 400.0]]);
        let shifted_ego = State::from((Position::new(30.0, 0.0), std::f64::consts::FRAC_PI_2, ego.speed));
        let shifted_plan = FrenetSamplingPlanner::default().plan(shifted_ego, &test_ctx(&shifted, &[]));
        for (a, b) in short.iter().zip(&shifted_plan) {
            assert!((a.acceleration - b.acceleration).abs() < 1e-9);
            assert!((a.curvature - b.curvature).abs() < 1e-9);
        }
    }

    #[test]
    fn diagnostics_show_candidates_without_changing_selection_or_collision_work() {
        use crate::planning::Diagnostics;

        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let diagnostics = Diagnostics::default();
        let latency = crate::planning::Latency::default();
        let mut ctx = Context {
            latency: Some(&latency),
            ..test_ctx(&road, &[])
        };
        let plain = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
        let collision_checks =
            |spans: Vec<crate::planning::Span>| spans.iter().filter(|span| span.name == "collision").count();
        let plain_checks = collision_checks(latency.take());
        ctx.diagnostics = Some(&diagnostics);
        let displayed = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
        let spans = latency.take();
        let scored = spans.iter().filter(|span| span.name == "cost").count();
        assert_eq!(displayed, plain);
        assert_eq!(collision_checks(spans), plain_checks);
        let data = diagnostics.take();

        assert!(data.trajectories.len() > 1);
        assert_eq!(data.trajectories.len(), scored);
        // Infeasible cubic commands are rejected rather than clipped into feasible rollouts.
        assert!(
            data.trajectories.len()
                < longitudinal_targets(0.0, road.dt, ctx.compute_budget, PLANNING_HORIZON_S).len()
                    * (1 + lateral_sample_count(ctx.compute_budget) * 3)
        );
        assert!(
            data.trajectories
                .iter()
                .all(|trajectory| trajectory.len() == (PLANNING_HORIZON_S / road.dt).ceil() as usize + 1)
        );
    }

    #[test]
    fn feasibility_rejects_collisions_independently_of_progress_score() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let controls = [Control::default(); 3];
        let ctx = test_ctx(&road, &[]);
        let trajectory = feasible_candidate_trajectory(ego, &controls, ctx.path(), &ctx).unwrap();
        assert_eq!(trajectory.states.len(), controls.len() + 1);
        assert_eq!(trajectory.states[0], ego);
        assert_eq!(trajectory.controls.len() + 1, trajectory.states.len());
        let score = compute_score(&trajectory, ctx.path(), &ctx, 23.0);
        let aligned = trajectory
            .controls
            .iter()
            .copied()
            .chain([*trajectory.controls.last().unwrap()])
            .collect();
        let kinematics = crate::common::kinematics::TrajectoryKinematics::new(
            trajectory.states.clone(),
            aligned,
            road.dt,
            ctx.path(),
        );
        assert_eq!(score, metrics::evaluate(&kinematics));

        let accelerated = [Control {
            acceleration: MAX_LON_ACCEL,
            ..Default::default()
        }; 3];
        let trajectory = feasible_candidate_trajectory(ego, &accelerated, ctx.path(), &ctx).unwrap();
        let accelerated_score = compute_score(&trajectory, ctx.path(), &ctx, 23.0);
        // The analytic baseline ignores the drag present in the rollout.
        assert!(accelerated_score < 1.0);
        assert!(accelerated_score > score);

        let actors = [ego];
        let diagnostics = crate::planning::Diagnostics::default();
        let blocked = Context {
            diagnostics: Some(&diagnostics),
            ..test_ctx(&road, &actors)
        };
        assert!(feasible_candidate_trajectory(ego, &controls, blocked.path(), &blocked).is_none());
        assert!(diagnostics.take().trajectories.is_empty());

        // The footprint hits the barrier even while the center is inside the road.
        let near_barrier = State::from((Position::new(0.0, road.half_width - 0.1), 0.0, ego.speed));
        let clear = Context {
            diagnostics: Some(&diagnostics),
            ..test_ctx(&road, &[])
        };
        assert!(feasible_candidate_trajectory(near_barrier, &controls, clear.path(), &clear).is_none());
        assert!(diagnostics.take().trajectories.is_empty());
        assert!(feasible_candidate_trajectory(ego, &controls, clear.path(), &clear).is_some());
        assert_eq!(diagnostics.take().trajectories[0].len(), controls.len() + 1);

        let infeasible = [Control {
            acceleration: MAX_LON_ACCEL + 1.0,
            curvature: 0.0,
        }];
        assert!(feasible_candidate_trajectory(ego, &infeasible, clear.path(), &clear).is_none());
        assert!(diagnostics.take().trajectories.is_empty());
    }

    #[test]
    fn baked_track_predictions_stay_inside_road_for_full_horizon() {
        use crate::common::geometry::barrier::collides_with_road_barrier;
        use crate::track::{Road, TRACK_PRESETS};

        for track_index in 0..TRACK_PRESETS.len() {
            let track = Track::from_catalog(track_index);
            let lap = track.lap_length().unwrap();
            for n in 0..20 {
                let progress = lap * n as f64 / 20.0;
                let centerline = track.centerline(progress - 50.0, progress + 250.0, 15.0);
                let road = Road::new(centerline, track.half_width(progress), 0.1);
                let (p, yaw) = track.pose(progress);
                let ego = State::from((p, yaw, 20.0));
                let ctx = Context::new(&road, &[], 100, crate::planning::ComputeBudget::NOMINAL, None, None);
                let mut state = ego;
                for (tick, control) in FrenetSamplingPlanner::default().plan(ego, &ctx).into_iter().enumerate() {
                    state = world_step(state, control, road.dt);
                    let d = Path::new(road.centerline()).project(state.position()).d;
                    assert!(
                        !collides_with_road_barrier(state, &road),
                        "track {track_index} progress {progress} width {} tick {tick} d {d} state {state:?}",
                        road.half_width
                    );
                }
            }
        }
    }
    #[test]
    fn accelerates_on_an_empty_straight_at_cruising_speeds() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let controls: Vec<_> = [0.0, 10.0, 20.0, 40.0]
            .into_iter()
            .map(|speed| {
                let ego = State {
                    speed,
                    ..Default::default()
                };
                (
                    speed,
                    FrenetSamplingPlanner::default().plan(ego, &test_ctx(&road, &[]))[0],
                )
            })
            .collect();
        assert!(
            controls.iter().all(|(_, u)| u.acceleration > 0.0),
            "empty-road first controls: {controls:?}"
        );
    }

    #[test]
    fn evaluates_shared_horizon_independently_of_requested_controls() {
        for dt in [0.1, 0.2, 0.3] {
            let mut road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
            road.dt = dt;
            let diagnostics = Diagnostics::default();
            let mut ctx = Context::new(&road, &[], 1, ComputeBudget::NOMINAL, None, Some(&diagnostics));
            let short = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
            assert_eq!(short.len(), 1);
            assert!(short[0].acceleration > 0.0);
            let trajectories = diagnostics.take().trajectories;
            assert!(!trajectories.is_empty());
            let ticks = (PLANNING_HORIZON_S / dt).ceil() as usize;
            assert!(trajectories.iter().all(|points| points.len() == ticks + 1));
            ctx.horizon = ticks;
            let long = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
            assert_eq!(long.len(), ticks);
            assert_eq!(short[0], long[0]);
            assert!(
                long.iter()
                    .all(|u| u.acceleration.is_finite() && u.curvature.is_finite())
            );
            ctx.horizon = 0;
            assert!(FrenetSamplingPlanner::default().plan(State::default(), &ctx).is_empty());
        }
    }
}
