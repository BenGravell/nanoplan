//! Shared control generation and vehicle rollouts for planners.

use crate::common::differencing::forward_difference;
use crate::common::types::FrenetPosition;
use crate::planning::Context;
use crate::planning::policy::centerline_curvature;
use crate::simulation::{Control, Position, State, world_step};
use crate::track::Path;
use crate::vehicle::MIN_LON_ACCEL;

pub(crate) fn repeat_last_controls(controls: &[Control], horizon: usize) -> Vec<Control> {
    (0..horizon).map(|t| controls[t.min(controls.len() - 1)]).collect()
}

pub(crate) fn brake_controls(ego: State, ctx: &Context, accel: f64) -> Vec<Control> {
    let mut x = ego;
    (0..ctx.horizon)
        .map(|_| {
            let u = Control {
                acceleration: accel,
                curvature: 0.0,
            };
            x = world_step(x, u, ctx.road.dt);
            u
        })
        .collect()
}

pub(crate) fn stop_controls(ego: State, ctx: &Context, horizon: usize) -> Vec<Control> {
    let mut x = ego;
    (0..horizon)
        .map(|_| {
            let accel = MIN_LON_ACCEL.max(forward_difference(x.speed, 0.0, ctx.road.dt));
            let u = Control {
                acceleration: accel,
                curvature: 0.0,
            };
            x = world_step(x, u, ctx.road.dt);
            u
        })
        .collect()
}

/// Roll `actions` out open-loop from `x0` through the kinematic model,
/// letting [`world_step`] enforce the state and action limits.
pub(crate) fn rollout_constrained(x0: State, actions: &[Control], dt: f64) -> (Vec<State>, Vec<Control>) {
    let mut xs = Vec::with_capacity(actions.len() + 1);
    let mut us = Vec::with_capacity(actions.len());
    xs.push(x0);
    let mut x = x0;
    for &u in actions {
        x = world_step(x, u, dt);
        xs.push(x);
        us.push(u);
    }
    (xs, us)
}

/// Road-centerline PD base policy for local trajectory optimizers.
pub(crate) fn centerline_follow_controls(ego: State, path: &Path, ctx: &Context, horizon: usize) -> Vec<Control> {
    let mut x = ego;
    let mut controls = Vec::with_capacity(horizon);
    for _ in 0..horizon {
        let s = path.project(x.position()).s;
        let traffic_brake = ctx
            .actors
            .iter()
            .filter_map(|actor| {
                let FrenetPosition { s: actor_s, d } = path.project(actor.position());
                (d.abs() < 2.0 && actor_s > s)
                    .then(|| (actor.speed * actor.speed - x.speed * x.speed) / (2.0 * (actor_s - s - 5.0).max(1.0)))
            })
            .min_by(f64::total_cmp)
            .unwrap_or(0.0);
        let accel = traffic_brake.clamp(MIN_LON_ACCEL, 0.0);
        let u = Control {
            acceleration: accel,
            curvature: centerline_curvature(path, &x),
        };
        x = world_step(x, u, ctx.road.dt);
        controls.push(u);
    }
    controls
}

// Pure-pursuit extraction for sampled tree geometry. The curvature gain is
// deliberately assertive because `step` still clamps impossible curvature
// requests to the plant's steering and lateral-grip limits.
const PATH_TRACK_LOOKAHEAD_TICKS: f64 = 8.0;
const PATH_TRACK_LOOKAHEAD_MIN_M: f64 = 3.0;
const PATH_TRACK_LOOKAHEAD_MAX_M: f64 = 10.0;
const PATH_TRACK_CURVATURE_GAIN: f64 = 1.0;

pub(crate) fn path_to_controls(ego: State, path: &Path, speed: f64, ctx: &Context) -> Vec<Control> {
    let total_len = path.length();
    let dt = ctx.road.dt;
    let lookahead =
        (speed * dt * PATH_TRACK_LOOKAHEAD_TICKS).clamp(PATH_TRACK_LOOKAHEAD_MIN_M, PATH_TRACK_LOOKAHEAD_MAX_M);
    let mut x = ego;
    (0..ctx.horizon)
        .map(|i| {
            let s = (speed * dt * (i + 1) as f64 + lookahead).min(total_len);
            let (target, _) = path.pose_at(s);
            let dx = target.x - x.position().x;
            let dy = target.y - x.position().y;
            let left = Position::from_angle(x.pose.yaw + std::f64::consts::FRAC_PI_2);
            let local_y = dx * left.x + dy * left.y;
            let ld2 = (dx * dx + dy * dy).max(1e-6);
            let curvature = 2.0 * PATH_TRACK_CURVATURE_GAIN * local_y / ld2;
            let accel = (0.5 * (speed - x.speed)).clamp(-4.0, 2.0);
            let u = Control {
                acceleration: accel,
                curvature,
            };
            x = world_step(x, u, dt);
            u
        })
        .collect()
}
