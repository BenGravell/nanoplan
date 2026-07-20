//! Actor state prediction.

use crate::simulation::State;
use crate::track::Path;

/// Time constant of the exponential return to the track centerline.
const CENTERLINE_RETURN_TAU_S: f64 = 2.0;

/// Constant-speed actor prediction in the track's Frenet frame.
///
/// Longitudinal position advances at the actor's current speed while its
/// current lateral offset decays smoothly toward the centerline.
pub(crate) fn predict(actor: &State, track: &Path, t: f64) -> State {
    let (s0, d0, _) = track.actor_projection(*actor);
    let s = s0 + actor.speed * t;
    let d = d0 * (-t / CENTERLINE_RETURN_TAU_S).exp();
    let (xy, yaw) = (track.frenet_to_xy(s, d), track.pose_at(s).1);
    State {
        x: xy[0],
        y: xy[1],
        yaw,
        ..*actor
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maintains_speed_and_smoothly_returns_to_centerline() {
        let track = Path::new(&[[0.0, 0.0], [100.0, 0.0]]);
        let actor = State {
            x: 10.0,
            y: 3.0,
            speed: 8.0,
            ..Default::default()
        };
        let predicted = predict(&actor, &track, 2.0);
        assert!((predicted.x - 26.0).abs() < 1e-9);
        assert!((predicted.y - 3.0 / std::f64::consts::E).abs() < 1e-9);
        assert_eq!(predicted.speed, actor.speed);
        let _ = predict(&actor, &track, 3.0);
        assert_eq!(track.cached_actor_count(), 1);
    }

    #[test]
    fn follows_the_track_around_a_bend() {
        let track = Path::new(&[[0.0, 0.0], [50.0, 0.0], [50.0, 50.0]]);
        let actor = State {
            x: 40.0,
            y: 2.0,
            speed: 10.0,
            ..Default::default()
        };
        let predicted = predict(&actor, &track, 2.0);
        let (s, d) = track.project(predicted.position());
        assert!((s - 60.0).abs() < 1e-9);
        assert!((d - 2.0 / std::f64::consts::E).abs() < 1e-9);
        assert_eq!(predicted.yaw, std::f64::consts::FRAC_PI_2);
    }
}
