//! Planner-specific math helpers.

use crate::common::geometry::wrap_angle;
use crate::constraints::Sample;
use crate::simulation::State;
use crate::track::Path;

/// Station radius for trajectory-state projection around a supplied hint.
const STATE_SAMPLE_PROJECTION_RADIUS_M: f64 = 15.0;

pub(crate) fn state_sample(path: &Path, x: &State, t_s: f64, s_hint: Option<f64>) -> (f64, Sample) {
    let p = x.position();
    let (s, d) = match s_hint {
        Some(h) => path.project_near(p, h, STATE_SAMPLE_PROJECTION_RADIUS_M),
        None => path.project(p),
    };
    let (_, lane_yaw) = path.pose_at(s);
    (
        s,
        Sample {
            position: p,
            lateral: d,
            road_bounds: None,
            heading_err: wrap_angle(x.pose.yaw - lane_yaw),
            speed: x.speed,
            control: None,
            station_speed: None,
            t: t_s,
        },
    )
}
