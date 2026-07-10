use crate::math::wrap_angle;
use crate::scenarios::Path;
use crate::simulation::{Control, State};

pub(crate) const CENTERLINE_LATERAL_GAIN: f64 = 0.02;
pub(crate) const CENTERLINE_HEADING_GAIN: f64 = 0.3;
pub(crate) const SPEED_HOLD_GAIN: f64 = 0.5;
pub(crate) const SPEED_HOLD_MIN_ACCEL: f64 = -2.0;
pub(crate) const SPEED_HOLD_MAX_ACCEL: f64 = 1.5;

pub(crate) fn centerline_feedback(path: &Path, x: &State, target_speed: f64) -> Control {
    let (s, d) = path.project(x.position());
    let (_, lane_yaw) = path.pose_at(s);
    let heading_err = wrap_angle(x.yaw - lane_yaw);
    Control {
        acceleration: speed_hold_accel(x.speed, target_speed),
        curvature: -(CENTERLINE_LATERAL_GAIN * d + CENTERLINE_HEADING_GAIN * heading_err),
    }
}

pub(crate) fn speed_hold_accel(speed: f64, target_speed: f64) -> f64 {
    (SPEED_HOLD_GAIN * (target_speed - speed)).clamp(SPEED_HOLD_MIN_ACCEL, SPEED_HOLD_MAX_ACCEL)
}
