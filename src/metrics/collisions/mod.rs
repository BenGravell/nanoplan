//! No at-fault collisions: 1 while the ego is collision-free at this tick.
//! Event-driven — aggregates by worst case (min).

use crate::metrics::{CAR_RADIUS_M, TickCtx, gap};

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let ego = &ctx.ego[i];
    if ctx.actors_at[i]
        .iter()
        .any(|a| gap(ego, a) < 2.0 * CAR_RADIUS_M)
    {
        0.0
    } else {
        1.0
    }
}
