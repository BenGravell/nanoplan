//! Progress ratio: the ego's station rate at this tick relative to driving
//! at the speed limit, clamped to [0, 1]. Smooth — aggregates by average.
//! ponytail: no expert trajectory; the speed limit stands in

/// `station` is the ego's arc length along the route at every tick.
pub fn score(station: &[f64], i: usize, dt: f64, speed_limit: f64) -> f64 {
    let n = station.len();
    let ds = if i + 1 < n {
        station[i + 1] - station[i]
    } else if i > 0 {
        station[i] - station[i - 1]
    } else {
        0.0
    };
    (ds / dt / speed_limit.max(0.1)).clamp(0.0, 1.0)
}
