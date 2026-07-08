//! Tiny shared vector/matrix types and planner math helpers.

mod linalg;
mod matrix;
mod vector;

use crate::planning::cost;
use crate::scenarios::Path;
use crate::simulation::{Control, MAX_ABS_CURVATURE_RATE, MAX_ABS_LON_JERK, State};
use crate::wrap_angle;

pub(crate) use linalg::{dot, mat_add, mat_mul, mat_vec, transpose, vec_add};
pub(crate) use matrix::{M2, M4, M6, M22, M24, M26, M62, M66};
pub(crate) use vector::{V2, V4, V6};

pub(crate) fn clamp_u(u: V2) -> V2 {
    [
        u[0].clamp(-MAX_ABS_LON_JERK, MAX_ABS_LON_JERK),
        u[1].clamp(-MAX_ABS_CURVATURE_RATE, MAX_ABS_CURVATURE_RATE),
    ]
}

pub(crate) fn control(u: V2) -> Control {
    Control {
        jerk: u[0],
        curvature_rate: u[1],
    }
}

pub(crate) fn state4(s: &State) -> V4 {
    [s.x, s.y, s.yaw, s.speed]
}

pub(crate) fn state6(s: &State) -> V6 {
    [s.x, s.y, s.yaw, s.speed, s.accel, s.curvature]
}

pub(crate) fn state_from_v6(v: V6) -> State {
    State {
        x: v[0],
        y: v[1],
        yaw: v[2],
        speed: v[3],
        accel: v[4],
        curvature: v[5],
    }
}

pub(crate) fn state_sample(
    path: &Path,
    x: &State,
    t_s: f64,
    s_hint: Option<f64>,
) -> (f64, cost::Sample) {
    let p = [x.x, x.y];
    let (s, d) = match s_hint {
        Some(h) => path.project_near(p, h, 15.0),
        None => path.project(p),
    };
    let (_, lane_yaw) = path.pose_at(s);
    (
        s,
        cost::Sample {
            xy: p,
            lateral: d,
            heading_err: wrap_angle(x.yaw - lane_yaw),
            speed: x.speed,
            curvature: x.curvature,
            accel: x.accel,
            t: t_s,
        },
    )
}
