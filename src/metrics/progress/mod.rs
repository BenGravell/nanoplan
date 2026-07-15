//! Forward progress along the race track, normalized by the vehicle's
//! drag-limited terminal speed and clamped to [0, 1].

use crate::metrics::TickCtx;
use crate::simulation::physics::MAX_TERMINAL_SPEED_MPS;

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
    speed_score(ds / ctx.dt)
}

/// Forward speed as the same normalized score used by rollout evaluation.
pub(crate) fn speed_score(speed: f64) -> f64 {
    (speed / *MAX_TERMINAL_SPEED_MPS).clamp(0.0, 1.0)
}
