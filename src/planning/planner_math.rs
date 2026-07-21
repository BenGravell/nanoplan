//! Planner-specific math helpers.

mod trajectory_cost;

use crate::common::math::wrap_angle;
use crate::common::vector::{V2, V4};
use crate::planning::cost;
use crate::simulation::{Control, State};
use crate::track::Path;
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_LON_ACCEL, MIN_LON_ACCEL};

pub(crate) use trajectory_cost::TrajectoryCost;

pub(crate) fn clamp_u(u: V2) -> V2 {
    [
        u[0].clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
        u[1].clamp(-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE),
    ]
}

pub(crate) fn control(u: V2) -> Control {
    Control {
        acceleration: u[0],
        curvature: u[1],
    }
}

pub(crate) fn state(s: &State) -> V4 {
    [s.x, s.y, s.yaw, s.speed]
}

pub(crate) fn state_from_v4(v: V4) -> State {
    State {
        x: v[0],
        y: v[1],
        yaw: v[2],
        speed: v[3],
    }
}

pub(crate) fn state_sample(
    path: &Path,
    x: &State,
    t_s: f64,
    s_hint: Option<f64>,
) -> (f64, cost::Sample) {
    let p = x.position();
    let (s, d) = match s_hint {
        Some(h) => path.project_near(p, h, 15.0),
        None => path.project(p),
    };
    let (_, lane_yaw) = path.pose_at(s);
    (
        s,
        cost::Sample {
            xy: p.xy(),
            lateral: d,
            heading_err: wrap_angle(x.yaw - lane_yaw),
            speed: x.speed,
            curvature: 0.0,
            accel: 0.0,
            t: t_s,
        },
    )
}
