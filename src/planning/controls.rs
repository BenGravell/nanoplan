//! Shared control generation and vehicle rollouts for planners.

use crate::common::differencing::forward_difference;
use crate::common::types::FrenetPosition;
use crate::planning::Context;
use crate::planning::policy::centerline_curvature;
use crate::simulation::{Control, State, world_step};
use crate::track::Path;
use crate::vehicle::MIN_LON_ACCEL;

pub(crate) fn repeat_last_controls(controls: &[Control], horizon: usize) -> Vec<Control> {
    (0..horizon).map(|t| controls[t.min(controls.len() - 1)]).collect()
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
