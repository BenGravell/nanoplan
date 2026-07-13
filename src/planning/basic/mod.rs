//! Basic centerline-return planner using the shared cubic steering curve.

use crate::math::smoothstep;
use crate::math::wrap_angle;
use crate::planning::search_tree::{centerline_follow_controls, stop_controls};
use crate::planning::steering::{CubicSteer, steer_controls};
use crate::planning::{Context, Planner};
use crate::simulation::{Control, State};
use crate::track::Path;
use crate::vehicle::MIN_LON_ACCEL;

pub struct BasicPlanner;

impl Planner for BasicPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0, d0, lane_speed) = ctx.time("route", || {
            let path = Path::new(&ctx.road.centerline);
            let (s0, d0) = path.project(ego.position());
            let (_, lane_yaw) = path.pose_at(s0);
            let heading_err = wrap_angle(ego.yaw - lane_yaw);
            let lane_speed = (ego.speed * heading_err.cos()).max(0.0);
            (path, s0, d0, lane_speed)
        });
        let (steer, duration, stop_at_goal) = ctx.time("fit", || {
            let duration = settle_time(ego, ctx, d0);
            let preview_m = 0.5 * (lane_speed + ctx.road.target_speed) * duration;
            let remaining_m = (path.length() - s0).max(0.0);
            let stopping_m = lane_speed * lane_speed / (-2.0 * MIN_LON_ACCEL);
            let plan_reach_m =
                lane_speed.max(ctx.road.target_speed) * ctx.horizon as f64 * ctx.road.dt;
            let stop_at_goal = remaining_m <= plan_reach_m + preview_m + stopping_m
                || s0 + preview_m >= path.length();
            let duration = if stop_at_goal {
                goal_duration(ego, ctx, d0, lane_speed, remaining_m)
            } else {
                duration
            };
            let target_s = if stop_at_goal {
                path.length()
            } else {
                s0 + preview_m
            };
            let target = if stop_at_goal {
                route_goal_state(&path)
            } else {
                cruise_goal_state(&path, target_s.min(path.length()), ctx.road.target_speed)
            };
            (
                CubicSteer::from_states(&ego, &target, duration),
                duration,
                stop_at_goal,
            )
        });
        ctx.time("extract", || {
            let ticks = ctx
                .horizon
                .min((duration / ctx.road.dt + 0.5).floor() as usize);
            let (mut controls, x) = steer_controls(ego, &steer, ctx.road.dt, ticks, 1.0);
            let rest = ctx.horizon - controls.len();
            if rest > 0 {
                if stop_at_goal {
                    controls.extend(stop_controls(x, ctx, rest));
                } else {
                    controls.extend(centerline_follow_controls(x, &path, ctx, rest));
                }
            }
            controls
        })
    }
}

const STOPPED_LOOKAHEAD_M: f64 = 4.0;
const MAX_LOOKAHEAD_M: f64 = 60.0;
const LOOKAHEAD_ROLLOFF_SPEED: f64 = 5.0;

fn settle_time(ego: State, ctx: &Context, d0: f64) -> f64 {
    let limit = ctx.road.target_speed.max(1.0);
    let speed = ego.speed.max(0.0);
    let lookahead_m = lookahead_m(speed, limit, d0);
    let avg_speed = ((speed + limit) * 0.5).max(2.0);
    (lookahead_m / avg_speed).clamp(1.0, 6.0)
}

fn lookahead_m(speed: f64, limit: f64, d0: f64) -> f64 {
    let far = (1.2 * limit + 0.8 * speed + 2.0 * d0.abs())
        .min(MAX_LOOKAHEAD_M)
        .max(STOPPED_LOOKAHEAD_M);
    let ratio = smoothstep(speed / LOOKAHEAD_ROLLOFF_SPEED);
    STOPPED_LOOKAHEAD_M + (far - STOPPED_LOOKAHEAD_M) * ratio
}

fn goal_duration(ego: State, ctx: &Context, d0: f64, lane_speed: f64, remaining_m: f64) -> f64 {
    let cruise = ctx.road.target_speed.max(1.0);
    let brake_t = lane_speed.max(ctx.road.target_speed) / -MIN_LON_ACCEL;
    settle_time(ego, ctx, d0)
        .max(remaining_m / cruise + brake_t)
        .max(2.0 * remaining_m / lane_speed.max(cruise))
}

fn route_goal_state(path: &Path) -> State {
    let ([x, y], yaw) = path.pose_at(path.length());
    State {
        x,
        y,
        yaw,
        speed: 0.0,
    }
}

fn cruise_goal_state(path: &Path, s: f64, target_speed: f64) -> State {
    let ([x, y], yaw) = path.pose_at(s);
    State {
        x,
        y,
        yaw,
        speed: target_speed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::model::step;
    use crate::planning::{test_ctx, test_road, test_run, test_run_on};

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
    fn tracks_road_target_speed() {
        let ego = State {
            speed: 10.0,
            ..Default::default()
        };
        let mut road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        road.target_speed = 6.0;
        let trace = test_run_on(&mut BasicPlanner, &road, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(
            (end.speed - road.target_speed).abs() < 0.5,
            "speed {}",
            end.speed
        );
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
            state = step(state, u, road.dt);
            max_x = max_x.max(state.x);
        }
        assert!(max_x <= 52.0, "preview reached x {max_x}");
        assert!(state.speed < 1.0, "speed {}", state.speed);
    }

    #[test]
    fn lookahead_rolls_from_short_stop_to_long_moving_target() {
        let limit = 10.0;
        assert_eq!(lookahead_m(0.0, limit, 0.0), STOPPED_LOOKAHEAD_M);
        let fast = lookahead_m(LOOKAHEAD_ROLLOFF_SPEED, limit, 0.0);
        assert!((fast - (1.2 * limit + 0.8 * LOOKAHEAD_ROLLOFF_SPEED)).abs() < 1e-9);
        let mid = lookahead_m(LOOKAHEAD_ROLLOFF_SPEED * 0.5, limit, 0.0);
        assert!(mid > STOPPED_LOOKAHEAD_M && mid < fast);
    }

    #[test]
    fn stopped_far_route_gets_short_positive_preview_target() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let ego = State::default();
        let preview_m = 0.5 * ctx.road.target_speed * settle_time(ego, &ctx, 0.0);
        assert!(preview_m >= STOPPED_LOOKAHEAD_M, "preview {preview_m}");
        assert!(preview_m < 10.0, "preview {preview_m}");
    }

    #[test]
    fn route_goal_state_has_full_stop_boundary() {
        let path = Path::new(&[[-5.0, 0.0], [20.0, 5.0]]);
        let goal = route_goal_state(&path);
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
}
