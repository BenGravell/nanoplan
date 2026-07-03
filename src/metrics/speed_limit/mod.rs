//! Speed limit compliance: overspeed at this tick normalized by nuPlan's
//! max_overspeed_value_threshold. Smooth — aggregates by average.

pub const MAX_OVERSPEED_MS: f64 = 2.23;

pub fn score(speed: f64, limit: f64) -> f64 {
    let overspeed = (speed - limit).max(0.0);
    (1.0 - overspeed / MAX_OVERSPEED_MS).clamp(0.0, 1.0)
}
