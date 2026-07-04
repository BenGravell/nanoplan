//! Driving direction compliance: backward movement along the route over the
//! trailing window ending at this tick (nuPlan thresholds 2 m / 6 m).
//! Event-driven — aggregates by worst case (min).

use crate::metrics::TickCtx;

const WINDOW_S: f64 = 1.0;
const COMPLIANCE_M: f64 = 2.0;
const VIOLATION_M: f64 = 6.0;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let window = ((WINDOW_S / ctx.dt) as usize).max(1);
    let backward = ctx.station[i.saturating_sub(window)] - ctx.station[i];
    if backward <= COMPLIANCE_M {
        1.0
    } else if backward <= VIOLATION_M {
        0.5
    } else {
        0.0
    }
}
