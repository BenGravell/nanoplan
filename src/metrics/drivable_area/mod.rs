//! Drivable area compliance: 1 while the ego is within the road half width
//! of the centerline. Event-driven — aggregates by worst case (min).

// shared with the planners' cost function (`planning::cost`) so a planner's
// notion of "off the road" agrees with what this metric scores as 0
pub(crate) const ROAD_HALF_WIDTH_M: f64 = 5.5;

/// `lateral_offset` is the ego's signed Frenet offset from the centerline.
pub fn score(lateral_offset: f64) -> f64 {
    if lateral_offset.abs() > ROAD_HALF_WIDTH_M {
        0.0
    } else {
        1.0
    }
}
