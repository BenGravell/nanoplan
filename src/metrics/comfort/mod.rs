//! Race-car comfort: a continuous score of longitudinal and lateral jerk.

use crate::common::differencing::forward_difference;
use crate::common::interp::interp1d;
use crate::common::kinematics::TrajectoryKinematics;
use crate::metrics::TickCtx;

const LON_JERK: &[f64] = &[0.0, 10.0, 20.0, 40.0];
const LON_SCORE: &[f64] = &[1.0, 0.99, 0.9, 0.0];
const LAT_JERK: &[f64] = &[0.0, 10.0, 15.0, 30.0];
const LAT_SCORE: &[f64] = &[1.0, 0.99, 0.9, 0.0];

pub(crate) fn score(ctx: &TickCtx, i: usize) -> f64 {
    let (lon, lat) = jerk_at(ctx.trajectory_kinematics, i);
    jerk_score(lon, lat)
}

pub(crate) fn jerk_score(lon: f64, lat: f64) -> f64 {
    let lon_score = interp1d(lon.abs(), LON_JERK, LON_SCORE);
    let lat_score = interp1d(lat.abs(), LAT_JERK, LAT_SCORE);
    lon_score * lat_score
}

/// Forward-difference jerk at tick `i`, padded by repeating the final jerk.
fn jerk_at(trajectory: &TrajectoryKinematics, i: usize) -> (f64, f64) {
    if trajectory.len() < 2 {
        return (0.0, 0.0);
    }

    let a = i.min(trajectory.len() - 2);
    let b = a + 1;
    let lon = forward_difference(
        trajectory.controls[a].acceleration,
        trajectory.controls[b].acceleration,
        trajectory.dt,
    );
    let lat = forward_difference(
        trajectory.lateral_acceleration[a],
        trajectory.lateral_acceleration[b],
        trajectory.dt,
    );

    (lon, lat)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::{Control, State};

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
    }

    #[test]
    fn longitudinal_jerk_uses_linear_anchor_interpolation() {
        for (&jerk, &expected) in LON_JERK.iter().zip(LON_SCORE) {
            close(jerk_score(jerk, 0.0), expected);
            close(jerk_score(-jerk, 0.0), expected);
        }
        close(jerk_score(5.0, 0.0), 0.995);
        close(jerk_score(15.0, 0.0), 0.945);
        close(jerk_score(30.0, 0.0), 0.45);
        close(jerk_score(50.0, 0.0), 0.0);
    }

    #[test]
    fn lateral_jerk_uses_linear_anchor_interpolation() {
        for (&jerk, &expected) in LAT_JERK.iter().zip(LAT_SCORE) {
            close(jerk_score(0.0, jerk), expected);
            close(jerk_score(0.0, -jerk), expected);
        }
        close(jerk_score(0.0, 5.0), 0.995);
        close(jerk_score(0.0, 12.5), 0.945);
        close(jerk_score(0.0, 22.5), 0.45);
        close(jerk_score(0.0, 40.0), 0.0);
    }

    #[test]
    fn longitudinal_and_lateral_penalties_compound() {
        close(jerk_score(20.0, 15.0), 0.81);
    }

    #[test]
    fn acceleration_comes_from_controls_and_jerk_is_differenced() {
        let ego = vec![
            State {
                speed: 10.0,
                ..Default::default()
            };
            3
        ];
        let controls = [
            Control::default(),
            Control {
                acceleration: 2.0,
                curvature: 0.01,
            },
            Control {
                acceleration: 4.0,
                curvature: 0.02,
            },
        ];
        let trajectory = TrajectoryKinematics::new(ego, controls.to_vec(), 0.1);
        assert_eq!(jerk_at(&trajectory, 0), (20.0, 10.0));
        assert_eq!(jerk_at(&trajectory, 1), (20.0, 10.0));
        assert_eq!(jerk_at(&trajectory, 2), (20.0, 10.0));
        close(jerk_score(20.0, 10.0), 0.891);
    }
}
