//! Small exhaustive search over centerline-following poly-cubic trajectories.

use crate::common::math::wrap_angle;
use crate::geometry::barrier::collide_with_road_barriers;
use crate::planning::cost::{HardConstraints, Sample};
use crate::planning::search_tree::{brake_controls, stop_controls};
use crate::planning::steering::{CubicSteer, steer_controls};
use crate::planning::{Context, Planner};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

pub(crate) struct BasicPlanner;

impl Planner for BasicPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0, lane_speed) = ctx.time("route", || {
            let path = Path::new(&ctx.road.centerline);
            let (s0, _) = path.project(ego.position());
            let (_, lane_yaw) = path.pose_at(s0);
            let heading_err = wrap_angle(ego.yaw - lane_yaw);
            let lane_speed = (ego.speed * heading_err.cos()).max(0.0);
            (path, s0, lane_speed)
        });
        ctx.time("fit", || {
            let mut best: Option<(f64, Vec<Control>)> = None;
            for distance in FIRST_TARGETS_M {
                for factor in DURATION_FACTORS {
                    let Some(controls) =
                        candidate(ego, &path, ctx, s0, lane_speed, distance, factor)
                    else {
                        continue;
                    };
                    let Some(cost) = candidate_cost(ego, &controls, &path, ctx) else {
                        continue;
                    };
                    if best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost) {
                        best = Some((cost, controls));
                    }
                }
            }
            best.map(|(_, controls)| controls)
                .unwrap_or_else(|| brake_controls(ego, ctx, MIN_LON_ACCEL))
        })
    }
}

const FIRST_TARGETS_M: [f64; 3] = [10.0, 25.0, 40.0];
const DURATION_FACTORS: [f64; 3] = [1.0, 1.5, 2.0];
const CENTERLINE_SEGMENT_M: f64 = 15.0;
const GOAL_BUFFER_M: f64 = 1.0;

fn candidate(
    ego: State,
    path: &Path,
    ctx: &Context,
    s0: f64,
    lane_speed: f64,
    first_distance: f64,
    duration_factor: f64,
) -> Option<Vec<Control>> {
    let first_distance = first_distance.min(path.length() - s0);
    if first_distance <= 0.0 {
        return None;
    }
    let fastest_speed = (lane_speed * lane_speed + 2.0 * MAX_LON_ACCEL * first_distance)
        .sqrt()
        .min(ctx.road.target_speed);
    let fastest_duration = 2.0 * first_distance / (lane_speed + fastest_speed).max(1.0);
    let mut duration = fastest_duration * duration_factor;
    let mut cruise_speed =
        (2.0 * first_distance / duration - lane_speed).clamp(0.0, ctx.road.target_speed);
    let mut target_s = s0 + first_distance;
    let mut x = ego;
    let mut controls = Vec::with_capacity(ctx.horizon);

    loop {
        let at_end = target_s >= path.length() - 1e-6;
        if at_end {
            cruise_speed = 0.0;
            duration = duration.max(x.speed.max(0.0) / -MIN_LON_ACCEL);
        }
        let goal_s = if at_end {
            (target_s - GOAL_BUFFER_M).max(s0)
        } else {
            target_s
        };
        append_segment(
            &mut controls,
            &mut x,
            path_state(path, goal_s, cruise_speed),
            duration,
            ctx,
        );
        if controls.len() >= ctx.horizon {
            break;
        }
        if at_end || cruise_speed <= 0.01 {
            controls.extend(stop_controls(x, ctx, ctx.horizon - controls.len()));
            break;
        }
        let next_s = (target_s + CENTERLINE_SEGMENT_M).min(path.length());
        duration = (next_s - target_s) / cruise_speed;
        target_s = next_s;
    }
    Some(controls)
}

fn append_segment(
    controls: &mut Vec<Control>,
    x: &mut State,
    target: State,
    duration: f64,
    ctx: &Context,
) {
    let remaining = ctx.horizon - controls.len();
    let ticks = remaining.min((duration / ctx.road.dt).round().max(1.0) as usize);
    let duration = ticks as f64 * ctx.road.dt;
    let steer = CubicSteer::from_states(x, &target, duration);
    let (segment, end) = steer_controls(*x, &steer, ctx.road.dt, ticks, 1.0);
    controls.extend(segment);
    *x = end;
}

fn candidate_cost(ego: State, controls: &[Control], path: &Path, ctx: &Context) -> Option<f64> {
    let constraints = HardConstraints::new(ctx.road.half_width, ctx.actors, path);
    let mut x = ego;
    let mut total = 0.0;
    let mut feasible = true;
    let mut trajectory = ctx.diagnostics.map(|_| vec![ego.position().into()]);
    for (tick, &u) in controls.iter().enumerate() {
        let prev = x;
        x = world_step(x, u, ctx.road.dt);
        if collide_with_road_barriers(prev, x, ctx.road) != x {
            feasible = false;
        }
        if let Some(points) = &mut trajectory {
            points.push(x.position().into());
        }
        let (s, lateral) = path.project(x.position());
        let (_, lane_yaw) = path.pose_at(s);
        let sample = Sample {
            xy: x.position().into(),
            lateral,
            heading_err: wrap_angle(x.yaw - lane_yaw),
            speed: x.speed,
            curvature: u.curvature,
            accel: u.acceleration,
            t: (tick + 1) as f64 * ctx.road.dt,
        };
        let cost = ctx.time("cost", || constraints.point_cost(&sample));
        if !cost.is_finite() {
            feasible = false;
        } else {
            total += cost;
        }
    }
    if let (Some(diag), Some(trajectory)) = (ctx.diagnostics, trajectory) {
        diag.record_trajectory(trajectory);
    }
    feasible.then_some(total)
}

fn path_state(path: &Path, s: f64, speed: f64) -> State {
    let ([x, y], yaw) = path.pose_at(s);
    State { x, y, yaw, speed }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_ctx, test_road, test_run, test_run_on};
    use crate::simulation::MAX_TERMINAL_SPEED_MPS;
    use crate::simulation::world_step;
    use crate::track::Track;

    #[test]
    fn converges_to_centerline() {
        let ego = State {
            y: 3.0,
            speed: 8.0,
            ..Default::default()
        };
        let trace = test_run(&mut BasicPlanner, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.4, "offset {}", end.y);
    }

    #[test]
    fn rolling_road_window_still_returns_to_centerline() {
        let track = Track::new(1);
        let mut road = test_road(&track.centerline(-50.0, 250.0, 15.0));
        road.target_speed = *MAX_TERMINAL_SPEED_MPS;
        let path = Path::new(&road.centerline);
        let (p, yaw) = path.pose_at(50.0);
        let ego = State {
            x: p[0] - 3.0 * yaw.sin(),
            y: p[1] + 3.0 * yaw.cos(),
            yaw,
            speed: 8.0,
        };

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
    fn stops_at_short_route_goal() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let road = test_road(&[[-5.0, 0.0], [25.0, 0.0]]);
        let trace = test_run_on(&mut BasicPlanner, &road, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.x > 20.0 && end.x < 30.0, "x {}", end.x);
        assert!(end.speed < 0.8, "speed {}", end.speed);
    }

    #[test]
    fn preview_plan_stops_at_goal_instead_of_overshooting() {
        let road = test_road(&[[-5.0, 0.0], [50.0, 0.0]]);
        let ctx = Context {
            road: &road,
            actors: &[],
            horizon: 100,
            latency: None,
            diagnostics: None,
        };
        let mut state = State {
            speed: 8.0,
            ..Default::default()
        };
        let mut max_x = state.x;
        for u in BasicPlanner.plan(state, &ctx) {
            state = world_step(state, u, road.dt);
            max_x = max_x.max(state.x);
        }
        assert!(max_x <= 52.0, "preview reached x {max_x}");
        assert!(state.speed < 1.0, "speed {}", state.speed);
    }

    #[test]
    fn route_goal_state_has_full_stop_boundary() {
        let path = Path::new(&[[-5.0, 0.0], [20.0, 5.0]]);
        let goal = path_state(&path, path.length(), 0.0);
        let ([x, y], yaw) = path.pose_at(path.length());
        assert_eq!((goal.x, goal.y, goal.yaw), (x, y, yaw));
        assert_eq!(goal.speed, 0.0);
    }

    #[test]
    fn returns_a_full_horizon() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let plan = BasicPlanner.plan(
            State {
                y: 2.0,
                speed: 6.0,
                ..Default::default()
            },
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

        assert_eq!(
            data.trajectories.len(),
            FIRST_TARGETS_M.len() * DURATION_FACTORS.len()
        );
        assert!(
            data.trajectories
                .iter()
                .all(|trajectory| trajectory.len() == ctx.horizon + 1)
        );
    }

    #[test]
    fn generated_track_predictions_stay_inside_road_for_full_horizon() {
        use crate::geometry::barrier::collides_with_road_barrier;
        use crate::track::Road;

        for seed in 0..10 {
            let track = Track::new(seed);
            let lap = track.lap_length().unwrap();
            for n in 0..20 {
                let progress = lap * n as f64 / 20.0;
                let centerline = track.centerline(progress - 50.0, progress + 250.0, 15.0);
                let road = Road::new(
                    centerline,
                    *MAX_TERMINAL_SPEED_MPS,
                    track.half_width(progress),
                    0.1,
                );
                let (p, yaw) = track.pose(progress);
                let ego = State {
                    x: p[0],
                    y: p[1],
                    yaw,
                    speed: 20.0,
                };
                let ctx = Context {
                    road: &road,
                    actors: &[],
                    horizon: 100,
                    latency: None,
                    diagnostics: None,
                };
                let mut state = ego;
                for (tick, control) in BasicPlanner.plan(ego, &ctx).into_iter().enumerate() {
                    state = world_step(state, control, road.dt);
                    let (_, d) = Path::new(&road.centerline).project(state.position());
                    assert!(
                        !collides_with_road_barrier(state, &road),
                        "seed {seed} progress {progress} width {} tick {tick} d {d} state {state:?}",
                        road.half_width
                    );
                }
            }
        }
    }
}
