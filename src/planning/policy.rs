use crate::common::geometry::wrap_angle;
use crate::simulation::State;
use crate::track::Path;

pub(crate) const CENTERLINE_LATERAL_GAIN: f64 = 0.02;
pub(crate) const CENTERLINE_HEADING_GAIN: f64 = 0.3;

pub(crate) fn centerline_curvature(path: &Path, x: &State) -> f64 {
    let (s, d) = path.project(x.position());
    let (_, lane_yaw) = path.pose_at(s);
    let heading_err = wrap_angle(x.pose.yaw - lane_yaw);
    -(CENTERLINE_LATERAL_GAIN * d + CENTERLINE_HEADING_GAIN * heading_err)
}
