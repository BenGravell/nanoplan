//! Drivable area compliance: 1 while the ego is within the road half width
//! of the centerline. Event-driven — aggregates by worst case (min).

use crate::metrics::TickCtx;

// shared with the planners' cost function (`planning::cost`) so a planner's
// notion of "off the road" agrees with what this metric scores as 0
pub(crate) const ROAD_HALF_WIDTH_M: f64 = 5.5;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    if ctx.lateral[i].abs() > ROAD_HALF_WIDTH_M {
        0.0
    } else {
        1.0
    }
}
