//! Dense search over centerline targets at a fixed planning horizon.

use crate::common::geometry::wrap_angle;
use crate::common::interp::lerp;
use crate::common::kinematics::{TrajectoryKinematics, commanded_accel_to_stop};
use crate::constraints::{Constraints, Sample};
use crate::metrics;
use crate::planning::policy::centerline_curvature;
use crate::planning::steering::{CubicSteer, steer_controls};
use crate::planning::{ComputeBudget, Context, PLANNING_HORIZON_S, Planner};
use crate::simulation::{Control, Position, State, world_step};
use crate::track::{Path, Road};
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

const STATION_SAMPLES_PER_INTERVAL: usize = 9;
const SPEED_SAMPLES_PER_INTERVAL: usize = 5;

pub(crate) struct BasicPlanner;

impl Planner for BasicPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let (path, s0) = ctx.time("route", || route(ego, ctx));
        ctx.time("fit", || {
            best_candidate(ego, path, ctx, s0, ego.speed).unwrap_or_else(|| fallback_controls(ego, path, ctx))
        })
    }
}

fn route<'a>(ego: State, ctx: &'a Context<'_>) -> (&'a Path, f64) {
    let path = ctx.path();
    let (s0, _) = ctx.project_ego(ego);
    (path, s0)
}

/// Brake along the road to a stop.
fn fallback_controls(mut state: State, path: &Path, ctx: &Context) -> Vec<Control> {
    (0..ctx.horizon)
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

fn best_candidate(ego: State, path: &Path, ctx: &Context, s0: f64, initial_speed: f64) -> Option<Vec<Control>> {
    let ticks = (PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize;
    let correction = lateral_target_correction(ego, path, s0);
    let mut best: Option<(f64, Vec<Control>)> = None;
    for (distance, speed) in longitudinal_targets(initial_speed, ctx.road.dt, ctx.compute_budget) {
        // Relative station zero is the ego projection; the window edge is not a stop target.
        if s0 + distance > path.length() {
            continue;
        }
        let target = corrected_target(path, s0 + distance, speed, correction);
        let steer = CubicSteer::from_states(&ego, &target, PLANNING_HORIZON_S);
        let (controls, _) = steer_controls(ego, &steer, ctx.road.dt, ticks, 1.0, false);
        let Some(trajectory) = feasible_candidate_trajectory(ego, &controls, path, ctx) else {
            continue;
        };
        let score = ctx.time("cost", || compute_score(&trajectory, ctx.road));
        if score.is_finite() && best.as_ref().is_none_or(|(best_score, _)| score > *best_score) {
            best = Some((score, controls));
        }
    }
    best.map(|(_, mut controls)| {
        controls.truncate(ctx.horizon);
        controls
    })
}

/// World-space displacement opposite the ego's current lateral offset.
///
/// Project onto the left normal of the road heading at `s0`. Applying this
/// displacement to each terminal point strengthens the initial correction of
/// the repeatedly replanned cubic. Use the ego's road frame even
/// when the road heading changes before the terminal station.
fn lateral_target_correction(ego: State, path: &Path, s0: f64) -> Position {
    let (origin, heading) = path.pose_at(s0);
    let offset = ego.position() - origin;
    let lateral = offset.y * heading.cos() - offset.x * heading.sin();
    let left = Position::from_angle(heading + std::f64::consts::FRAC_PI_2);
    Position::new(-lateral * left.x, -lateral * left.y)
}

/// Shift the terminal centerline position while retaining its road heading and sampled speed.
fn corrected_target(path: &Path, station: f64, speed: f64, correction: Position) -> State {
    let (position, yaw) = path.pose_at(station);
    State::from((position + correction, yaw, speed))
}

fn sample_counts(budget: ComputeBudget) -> (usize, usize) {
    // Scale both axes by sqrt(budget) so total candidates scale approximately linearly.
    let nominal_count = 4 * STATION_SAMPLES_PER_INTERVAL * SPEED_SAMPLES_PER_INTERVAL;
    let scale = (budget.scale(nominal_count, 1) as f64 / nominal_count as f64).sqrt();
    let station_count = ((STATION_SAMPLES_PER_INTERVAL as f64 * scale).round() as usize).max(2);
    let speed_count = ((SPEED_SAMPLES_PER_INTERVAL as f64 * scale).round() as usize).max(2);
    (station_count, speed_count)
}

/// Cartesian product of station and terminal-speed samples around the zero-thrust rollout.
fn longitudinal_targets(initial_speed: f64, dt: f64, budget: ComputeBudget) -> Vec<(f64, f64)> {
    let rollout = |acceleration| {
        let mut state = State {
            speed: initial_speed,
            ..Default::default()
        };
        let ticks = (PLANNING_HORIZON_S / dt).ceil() as usize;
        for tick in 0..ticks {
            let step_dt = dt.min(PLANNING_HORIZON_S - tick as f64 * dt);
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
        (0..count)
            .map(move |i| lerp(min, nominal, i as f64 / count as f64))
            .chain((0..count).map(move |i| lerp(nominal, max, i as f64 / (count - 1) as f64)))
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

fn feasible_candidate_trajectory(
    ego: State,
    controls: &[Control],
    path: &Path,
    ctx: &Context,
) -> Option<TrajectoryKinematics> {
    let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, ego.speed, ctx.road.dt);
    let mut x = ego;
    let mut states = Vec::with_capacity(controls.len() + 1);
    states.push(ego);
    for (tick, &u) in controls.iter().enumerate() {
        let prev = x;
        x = world_step(x, u, ctx.road.dt);
        let sample = constraint_sample(x, path, (tick + 1) as f64 * ctx.road.dt).with_control(u, prev.speed);
        if constraints.is_transition_violated(prev, x, ctx.road, &sample) {
            return None;
        }
        states.push(x);
    }
    if let Some(diag) = ctx.diagnostics {
        diag.record_trajectory(states.iter().map(|state| state.position()).collect());
    }
    // Align controls with states, including the final state, as required by metrics.
    let controls = controls
        .iter()
        .copied()
        .chain([controls.last().copied().unwrap_or_default()])
        .collect();
    Some(TrajectoryKinematics::new(states, controls, ctx.road.dt))
}

fn compute_score(trajectory: &TrajectoryKinematics, road: &Road) -> f64 {
    metrics::evaluate(trajectory, road).score
}

fn constraint_sample(state: State, path: &Path, time: f64) -> Sample {
    let (s, lateral) = path.project(state.position());
    let (_, lane_yaw) = path.pose_at(s);
    Sample {
        position: state.position(),
        lateral,
        road_bounds: None,
        heading_err: wrap_angle(state.pose.yaw - lane_yaw),
        speed: state.speed,
        control: None,
        station_speed: None,
        t: time,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_ctx, test_road, test_run, test_run_on};
    use crate::simulation::Position;
    use crate::simulation::world_step;
    use crate::track::Track;

    #[test]
    fn converges_to_centerline() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 3.0), 0.0),
            8.0,
        );
        let trace = test_run(&mut BasicPlanner, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 0.4, "offset {}", end.position().y);
    }

    #[test]
    fn rolling_road_window_still_returns_to_centerline() {
        let track = Track::from_catalog(1);
        let road = test_road(&track.centerline(-50.0, 250.0, 15.0));
        let path = Path::new(road.centerline());
        let (p, yaw) = path.pose_at(50.0);
        let left = Position::from_angle(yaw + std::f64::consts::FRAC_PI_2);
        let ego = State::from((Position::new(p.x + 3.0 * left.x, p.y + 3.0 * left.y), yaw, 8.0));

        // Receding ten-second connectors converge more slowly than the old short first segment.
        let trace = test_run_on(&mut BasicPlanner, &road, ego, &[], 200);
        let (_, d) = path.project(trace.last().unwrap().position());
        assert!(d.abs() < 1.0, "offset {d}");
    }

    #[test]
    fn accelerates_on_a_clear_straight() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let controls = BasicPlanner.plan(State::default(), &test_ctx(&road, &[]));
        assert!(controls[0].acceleration > 0.0);
        assert!(controls.iter().all(|u| u.acceleration <= MAX_LON_ACCEL));
        assert!(controls.iter().all(|u| u.curvature == 0.0));
    }

    #[test]
    fn targets_use_relative_station_and_resistance_over_the_planning_horizon() {
        let speed = 8.0;
        let targets = longitudinal_targets(speed, 0.1, ComputeBudget::NOMINAL);
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
        let nominal = longitudinal_targets(8.0, 0.1, ComputeBudget::NOMINAL);
        let nominal_target =
            nominal[STATION_SAMPLES_PER_INTERVAL * 2 * SPEED_SAMPLES_PER_INTERVAL + SPEED_SAMPLES_PER_INTERVAL];
        for (percent, expected) in crate::planning::COMPUTE_BUDGET_BREAKPOINTS
            .into_iter()
            .zip([16, 24, 32, 96, 180, 364, 880])
        {
            let targets = longitudinal_targets(8.0, 0.1, ComputeBudget::from_percent(percent));
            assert_eq!(targets.len(), expected, "{percent}%");
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
        assert!(best_candidate(ego, ctx.path(), &ctx, 5.0, ego.speed).is_none());
    }

    #[test]
    fn returns_a_full_horizon() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let plan = BasicPlanner.plan(
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
        let short = BasicPlanner.plan(ego, &ctx);
        ctx.horizon = (PLANNING_HORIZON_S / road.dt).ceil() as usize;
        let full = BasicPlanner.plan(ego, &ctx);
        assert_eq!(short, full[..short.len()]);
        ctx.horizon = 0;
        assert!(BasicPlanner.plan(ego, &ctx).is_empty());

        // Rotating and shifting the road changes absolute station and yaw, not relative targets.
        let shifted = test_road(&[[30.0, -100.0], [30.0, 400.0]]);
        let shifted_ego = State::from((Position::new(30.0, 0.0), std::f64::consts::FRAC_PI_2, ego.speed));
        let shifted_plan = BasicPlanner.plan(shifted_ego, &test_ctx(&shifted, &[]));
        for (a, b) in short.iter().zip(&shifted_plan) {
            assert!((a.acceleration - b.acceleration).abs() < 1e-9);
            assert!((a.curvature - b.curvature).abs() < 1e-9);
        }
    }

    #[test]
    fn records_feasible_candidate_trajectories_when_requested() {
        use crate::planning::Diagnostics;

        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let diagnostics = Diagnostics::default();
        let ctx = Context {
            diagnostics: Some(&diagnostics),
            ..test_ctx(&road, &[])
        };
        BasicPlanner.plan(State::default(), &ctx);
        let data = diagnostics.take();

        assert!(!data.trajectories.is_empty());
        // Infeasible cubic commands are rejected rather than clipped into feasible rollouts.
        assert!(data.trajectories.len() < longitudinal_targets(0.0, road.dt, ctx.compute_budget).len());
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
        assert_eq!(trajectory.controls.len(), trajectory.states.len());
        let score = compute_score(&trajectory, &road);
        assert_eq!(score, metrics::evaluate(&trajectory, &road).score);

        let accelerated = [Control {
            acceleration: MAX_LON_ACCEL,
            ..Default::default()
        }; 3];
        let trajectory = feasible_candidate_trajectory(ego, &accelerated, ctx.path(), &ctx).unwrap();
        let accelerated_score = compute_score(&trajectory, &road);
        assert!((accelerated_score - 1.0).abs() < 1e-9);
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
                for (tick, control) in BasicPlanner.plan(ego, &ctx).into_iter().enumerate() {
                    state = world_step(state, control, road.dt);
                    let (_, d) = Path::new(road.centerline()).project(state.position());
                    assert!(
                        !collides_with_road_barrier(state, &road),
                        "track {track_index} progress {progress} width {} tick {tick} d {d} state {state:?}",
                        road.half_width
                    );
                }
            }
        }
    }
}
