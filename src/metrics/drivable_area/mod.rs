//! Drivable area compliance: 1 while the ego is within the road half width
//! of the centerline. Event-driven — aggregates by worst case (min).

use crate::metrics::TickCtx;

/// Default road half-width, in meters: the value used when a scenario does
/// not specify its own ([`crate::scenarios::MapData::road_half_width`]). The
/// actual bound this metric scores against is the road's own half-width
/// ([`TickCtx::road_half_width`], from [`crate::scenarios::Road::half_width`]),
/// so scoring and the planners' cost function
/// ([`crate::planning`]) agree on "off the road" for whatever road is
/// actually being driven — not just this default.
pub(crate) const ROAD_HALF_WIDTH_M: f64 = 5.5;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    if ctx.lateral[i].abs() > ctx.road_half_width {
        0.0
    } else {
        1.0
    }
}
