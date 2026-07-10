//! Actor state prediction shared by metrics, planners, and tuning.

use crate::math::wrap_angle;
use crate::scenarios::Path;
use crate::simulation::State;

pub use crate::geometry::project;

/// Heading alignment within which an actor counts as travelling *along* the
/// lane, so [`predict`] rolls it forward following the lane's curve.
const LANE_ASSOC_HEADING_RAD: f64 = std::f64::consts::FRAC_PI_4; // 45 deg

/// Largest lateral offset at which an actor is still associated with the lane.
const LANE_ASSOC_LATERAL_M: f64 = 4.0;

/// Time constant of the exponential return to lane center.
const LANE_RETURN_TAU_S: f64 = 2.0;

/// Kinematic actor prediction with lane association.
///
/// When `lane` is `Some` and the actor is moving along it, the actor is
/// rolled forward along the lane curve at constant speed while its lateral
/// offset decays toward the centerline. Otherwise this falls back to
/// [`project`].
pub fn predict(s: &State, lane: Option<&Path>, t: f64) -> State {
    let Some(path) = lane else {
        return project(s, t);
    };
    let (s0, d0) = path.project(s.position());
    let (_, lane_yaw) = path.pose_at(s0);
    if wrap_angle(s.yaw - lane_yaw).abs() > LANE_ASSOC_HEADING_RAD
        || d0.abs() > LANE_ASSOC_LATERAL_M
    {
        return project(s, t);
    }
    let s_t = s0 + s.speed * t;
    let d_t = d0 * (-t / LANE_RETURN_TAU_S).exp();
    let (_, yaw_t) = path.pose_at(s_t);
    let xy = path.frenet_to_xy(s_t, d_t);
    State {
        x: xy[0],
        y: xy[1],
        yaw: yaw_t,
        ..*s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn predict_without_a_lane_is_the_straight_line_projection() {
        let a = State {
            x: 10.0,
            y: 3.0,
            yaw: 0.5,
            speed: 8.0,
            ..Default::default()
        };
        assert_eq!(predict(&a, None, 2.0), project(&a, 2.0));
    }

    #[test]
    fn predict_ignores_the_lane_for_crossing_and_oncoming_traffic() {
        let lane = Path::new(&[[0.0, 0.0], [100.0, 0.0]]);
        let crossing = State {
            x: 40.0,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 6.0,
            ..Default::default()
        };
        let oncoming = State {
            x: 40.0,
            y: 2.0,
            yaw: std::f64::consts::PI,
            speed: 6.0,
            ..Default::default()
        };
        for a in [crossing, oncoming] {
            assert_eq!(predict(&a, Some(&lane), 1.5), project(&a, 1.5));
        }
    }

    #[test]
    fn predict_returns_an_aligned_actor_toward_the_lane_center() {
        let lane = Path::new(&[[0.0, 0.0], [100.0, 0.0]]);
        let a = State {
            x: 20.0,
            y: 2.0,
            yaw: 0.0,
            speed: 10.0,
            ..Default::default()
        };
        let p = predict(&a, Some(&lane), 2.0);
        assert!((p.x - 40.0).abs() < 1e-9);
        assert!(p.y > 0.0 && p.y < 2.0, "y {}", p.y);
        assert_eq!(p.yaw, 0.0);
    }

    #[test]
    fn predict_follows_the_lane_around_a_bend() {
        let lane = Path::new(&[[0.0, 0.0], [50.0, 0.0], [50.0, 50.0]]);
        let a = State {
            x: 40.0,
            y: 0.0,
            yaw: 0.0,
            speed: 10.0,
            ..Default::default()
        };
        let curved = predict(&a, Some(&lane), 2.0);
        let straight = project(&a, 2.0);
        assert!(lane.project(curved.position()).1.abs() < 0.5);
        assert!(lane.project(straight.position()).1.abs() > 5.0);
    }
}
