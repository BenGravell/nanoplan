//! Driving direction compliance: backward movement along the route over the
//! trailing window ending at this tick (nuPlan thresholds 2 m / 6 m).
//! Event-driven — aggregates by worst case (min).

const WINDOW_S: f64 = 1.0;
const COMPLIANCE_M: f64 = 2.0;
const VIOLATION_M: f64 = 6.0;

/// `station` is the ego's arc length along the route at every tick.
pub fn score(station: &[f64], i: usize, dt: f64) -> f64 {
    let window = ((WINDOW_S / dt) as usize).max(1);
    let backward = station[i.saturating_sub(window)] - station[i];
    if backward <= COMPLIANCE_M {
        1.0
    } else if backward <= VIOLATION_M {
        0.5
    } else {
        0.0
    }
}
