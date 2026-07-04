//! Speed limit compliance: overspeed at this tick normalized by nuPlan's
//! max_overspeed_value_threshold. Smooth — aggregates by average.

use crate::metrics::TickCtx;

pub const MAX_OVERSPEED_MS: f64 = 2.23;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let overspeed = (ctx.ego[i].speed - ctx.speed_limit).max(0.0);
    (1.0 - overspeed / MAX_OVERSPEED_MS).clamp(0.0, 1.0)
}
