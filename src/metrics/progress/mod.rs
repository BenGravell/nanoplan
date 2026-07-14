//! Progress ratio: the ego's station rate at this tick relative to driving
//! at the vehicle's physical maximum speed, clamped to [0, 1]. Smooth —
//! aggregates by average.
//! ponytail: no expert trajectory; the physical envelope stands in

use crate::metrics::TickCtx;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let station = ctx.station;
    let n = station.len();
    let ds = if i + 1 < n {
        station[i + 1] - station[i]
    } else if i > 0 {
        station[i] - station[i - 1]
    } else {
        0.0
    };
    (ds / ctx.dt / ctx.max_speed.max(0.1)).clamp(0.0, 1.0)
}
