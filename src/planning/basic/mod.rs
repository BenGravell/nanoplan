//! Small exhaustive search over centerline-following poly-cubic trajectories.

use crate::common::geometry::barrier::collide_with_road_barriers;
use crate::common::geometry::wrap_angle;
use crate::common::kinematics::{TrajectoryKinematics, speed_after_distance};
use crate::metrics;
use crate::planning::constraints::{HardConstraints, Sample};
use crate::planning::search_tree::{brake_controls, stop_controls};
use crate::planning::steering::{CubicSteer, steer_controls};
use crate::planning::{Context, Planner};
use crate::simulation::{Control, State, world_step};
use crate::track::{Path, Road};
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

const FIRST_TARGETS_M: [f64; 3] = [10.0, 25.0, 40.0];
const DURATION_FACTORS: [f64; 3] = [1.0, 1.5, 2.0];
const CENTERLINE_SEGMENT_M: f64 = 15.0;

pub(crate) struct BasicPlanner;

impl Planner for BasicPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0, lane_speed) = ctx.time("route", || {
            let path = ctx.path();
            let (s0, _) = path.project(ego.position());
            let (_, lane_yaw) = path.pose_at(s0);
            let heading_err = wrap_angle(ego.pose.yaw - lane_yaw);
            let lane_speed = (ego.speed * heading_err.cos()).max(0.0);
            (path, s0, lane_speed)
        });
        ctx.time("fit", || {
            best_candidate(ego, path, ctx, s0, lane_speed).unwrap_or_else(|| brake_controls(ego, ctx, MIN_LON_ACCEL))
        })
    }
}

fn best_candidate(ego: State, path: &Path, ctx: &Context, s0: f64, lane_speed: f64) -> Option<Vec<Control>> {
    let mut best: Option<(f64, Vec<Control>)> = None;
    for distance in FIRST_TARGETS_M {
        for factor in DURATION_FACTORS {
            let Some(controls) = candidate(ego, path, ctx, s0, lane_speed, distance, factor) else {
                continue;
            };
            let Some(trajectory) = feasible_candidate_trajectory(ego, &controls, path, ctx) else {
                continue;
            };
            let score = ctx.time("cost", || candidate_score(&trajectory, ctx.road));
            if !score.is_finite() {
                continue;
            }
            if best.as_ref().is_none_or(|(best_score, _)| score > *best_score) {
                best = Some((score, controls));
            }
        }
    }
    best.map(|(_, controls)| controls)
}

fn candidate(
    ego: State,
    path: &Path,
    ctx: &Context,
    s0: f64,
    lane_speed: f64,
    first_distance: f64,
    duration_factor: f64,
) -> Option<Vec<Control>> {
    let (mut duration, cruise_speed) = first_segment_motion(lane_speed, first_distance, duration_factor);
    let mut target_s = s0 + first_distance;
    let mut x = ego;
    let mut controls = Vec::with_capacity(ctx.horizon);

    loop {
        // The window edge is missing geometry, not a stopping destination.
        if target_s > path.length() {
            return None;
        }
        let (position, yaw) = path.pose_at(target_s);
        let target = State::from((position, yaw, cruise_speed));
        append_segment(&mut controls, &mut x, target, duration, ctx);
        if controls.len() >= ctx.horizon {
            break;
        }
        if target.speed <= 0.01 {
            controls.extend(stop_controls(x, ctx, ctx.horizon - controls.len()));
            break;
        }
        duration = CENTERLINE_SEGMENT_M / cruise_speed;
        target_s += CENTERLINE_SEGMENT_M;
    }
    Some(controls)
}

/// Constant-acceleration timing seeds the first cubic's terminal speed.
fn first_segment_motion(lane_speed: f64, distance: f64, duration_factor: f64) -> (f64, f64) {
    let fastest_speed = speed_after_distance(lane_speed, MAX_LON_ACCEL, distance);
    let fastest_average_speed = (lane_speed + fastest_speed) / 2.0;
    let fastest_duration = distance / fastest_average_speed;
    let duration = fastest_duration * duration_factor;

    let average_speed = distance / duration;
    let terminal_speed = 2.0 * average_speed - lane_speed;
    let cruise_speed = terminal_speed.max(0.0);
    (duration, cruise_speed)
}

fn append_segment(controls: &mut Vec<Control>, x: &mut State, target: State, duration: f64, ctx: &Context) {
    let remaining = ctx.horizon - controls.len();
    let ticks = remaining.min((duration / ctx.road.dt).round().max(1.0) as usize);
    let duration = ticks as f64 * ctx.road.dt;
    let steer = CubicSteer::from_states(x, &target, duration);
    let (segment, end) = steer_controls(*x, &steer, ctx.road.dt, ticks, 1.0, true);
    controls.extend(segment);
    *x = end;
}

fn feasible_candidate_trajectory(
    ego: State,
    controls: &[Control],
    path: &Path,
    ctx: &Context,
) -> Option<TrajectoryKinematics> {
    let constraints = HardConstraints::new(ctx.road.half_width, ctx.actors, path, ego.speed, ctx.road.dt);
    let mut x = ego;
    let mut states = Vec::with_capacity(controls.len() + 1);
    states.push(ego);
    let mut feasible = true;
    for (tick, &u) in controls.iter().enumerate() {
        let prev = x;
        x = world_step(x, u, ctx.road.dt);
        if collide_with_road_barriers(prev, x, crate::common::geometry::EGO_FOOTPRINT, ctx.road) != x {
            feasible = false;
        }
        let sample = constraint_sample(x, path, (tick + 1) as f64 * ctx.road.dt);
        if constraints.is_violated(&sample) {
            feasible = false;
        }
        states.push(x);
    }
    if let Some(diag) = ctx.diagnostics {
        diag.record_trajectory(states.iter().map(|state| state.position()).collect());
    }
    if !feasible {
        return None;
    }
    // Align controls with states, including the final state, as required by metrics.
    let controls = controls
        .iter()
        .copied()
        .chain([controls.last().copied().unwrap_or_default()])
        .collect();
    Some(TrajectoryKinematics::new(states, controls, ctx.road.dt))
}

fn candidate_score(trajectory: &TrajectoryKinematics, road: &Road) -> f64 {
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

        let trace = test_run_on(&mut BasicPlanner, &road, ego, &[], 20);
        let (_, d) = path.project(trace.last().unwrap().position());
        assert!(d.abs() < 1.0, "offset {d}");
    }

    #[test]
    fn accelerates_on_a_clear_straight() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let controls = BasicPlanner.plan(State::default(), &test_ctx(&road, &[]));
        assert_eq!(controls[0].acceleration, MAX_LON_ACCEL);
        assert!(controls.iter().all(|u| u.curvature == 0.0));
    }

    #[test]
    fn window_endpoint_does_not_request_a_stop() {
        let road = test_road(&[[-5.0, 0.0], [20.0, 0.0]]);
        let ctx = Context::new(&road, &[], 20, crate::planning::ComputeBudget::NOMINAL, None, None);
        let ego = State {
            speed: 10.0,
            ..Default::default()
        };
        let (fastest_duration, _) = first_segment_motion(ego.speed, 20.0, 1.0);
        let duration_factor = 2.0 / fastest_duration;
        let controls = candidate(ego, ctx.path(), &ctx, 5.0, ego.speed, 20.0, duration_factor).unwrap();
        let end = controls
            .iter()
            .fold(ego, |state, &control| world_step(state, control, road.dt));

        assert_eq!(controls.len(), ctx.horizon);
        assert!(end.position().x > 18.0, "position {}", end.position().x);
        assert!(end.speed > 8.0, "speed {}", end.speed);
    }

    #[test]
    fn rejects_candidates_that_need_geometry_beyond_the_window() {
        let road = test_road(&[[-5.0, 0.0], [20.0, 0.0]]);
        let ctx = Context::new(&road, &[], 100, crate::planning::ComputeBudget::NOMINAL, None, None);
        let ego = State {
            speed: 10.0,
            ..Default::default()
        };

        assert!(candidate(ego, ctx.path(), &ctx, 5.0, ego.speed, 20.0, 1.0).is_none());
        assert!(candidate(ego, ctx.path(), &ctx, 5.0, ego.speed, 40.0, 1.0).is_none());
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
    fn records_every_candidate_trajectory_when_requested() {
        use crate::planning::Diagnostics;

        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let diagnostics = Diagnostics::default();
        let ctx = Context {
            diagnostics: Some(&diagnostics),
            ..test_ctx(&road, &[])
        };
        BasicPlanner.plan(State::default(), &ctx);
        let data = diagnostics.take();

        assert_eq!(data.trajectories.len(), FIRST_TARGETS_M.len() * DURATION_FACTORS.len());
        assert!(
            data.trajectories
                .iter()
                .all(|trajectory| trajectory.len() == ctx.horizon + 1)
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
        let score = candidate_score(&trajectory, &road);
        assert_eq!(score, metrics::evaluate(&trajectory, &road).score);

        let accelerated = [Control {
            acceleration: MAX_LON_ACCEL,
            ..Default::default()
        }; 3];
        let trajectory = feasible_candidate_trajectory(ego, &accelerated, ctx.path(), &ctx).unwrap();
        let accelerated_score = candidate_score(&trajectory, &road);
        assert!((accelerated_score - 1.0).abs() < 1e-9);
        assert!(accelerated_score > score);

        let actors = [ego];
        let diagnostics = crate::planning::Diagnostics::default();
        let blocked = Context {
            diagnostics: Some(&diagnostics),
            ..test_ctx(&road, &actors)
        };
        assert!(feasible_candidate_trajectory(ego, &controls, blocked.path(), &blocked).is_none());
        assert_eq!(diagnostics.take().trajectories[0].len(), controls.len() + 1);

        // The footprint hits the barrier even while the center is inside the road.
        let near_barrier = State::from((Position::new(0.0, road.half_width - 0.1), 0.0, ego.speed));
        assert!(feasible_candidate_trajectory(near_barrier, &controls, ctx.path(), &ctx).is_none());
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
