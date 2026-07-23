//! Planner-specific math helpers.

use crate::common::math::wrap_angle;
use crate::planning::constraints::Sample;
use crate::simulation::State;
use crate::track::Path;

pub(crate) fn state_sample(path: &Path, x: &State, t_s: f64, s_hint: Option<f64>) -> (f64, Sample) {
    let p = x.position();
    let (s, d) = match s_hint {
        Some(h) => path.project_near(p, h, 15.0),
        None => path.project(p),
    };
    let (_, lane_yaw) = path.pose_at(s);
    (
        s,
        Sample {
            xy: p.xy(),
            lateral: d,
            road_bounds: None,
            heading_err: wrap_angle(x.yaw - lane_yaw),
            speed: x.speed,
            station_speed: None,
            lon_jerk: 0.0,
            lat_jerk: 0.0,
            t: t_s,
        },
    )
}
