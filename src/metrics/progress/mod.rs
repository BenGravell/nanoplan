//! Forward progress along the race track, normalized by the progress possible
//! under full acceleration from the rollout's initial speed.

use crate::metrics::TickCtx;
use crate::simulation::speed_after_max_accel;

pub(crate) fn score(ctx: &TickCtx, i: usize) -> f64 {
    let station = ctx.station;
    let n = station.len();
    let ds = if i + 1 < n {
        station[i + 1] - station[i]
    } else if i > 0 {
        station[i] - station[i - 1]
    } else {
        0.0
    };
    // The final sample reuses the preceding interval, so reuse that interval's baseline tick as well.
    let ticks = i.min(n.saturating_sub(2));
    speed_score(
        ds / ctx.trajectory_kinematics.dt,
        ctx.trajectory_kinematics.states[0].speed,
        ticks,
        ctx.trajectory_kinematics.dt,
    )
}

/// Normalized speed score.
pub(crate) fn speed_score(speed: f64, initial_speed: f64, ticks: usize, dt: f64) -> f64 {
    let baseline = speed_after_max_accel(initial_speed, ticks, dt);
    if baseline <= 0.0 {
        f64::from(speed >= baseline)
    } else {
        (speed / baseline).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_acceleration_is_the_fair_baseline() {
        let dt = 0.1;
        let initial = 12.0;
        let baseline = speed_after_max_accel(initial, 20, dt);
        assert_eq!(speed_score(baseline, initial, 20, dt), 1.0);
        assert!(speed_score(initial, initial, 20, dt) < 1.0);
    }

    #[test]
    fn faster_forward_progress_scores_higher() {
        let score = |speed| speed_score(speed, 10.0, 10, 0.1);
        assert!(score(20.0) > score(10.0));
    }
}
