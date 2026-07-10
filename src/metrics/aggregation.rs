//! Aggregation helpers for per-tick metric scores.

use super::{CompositeRole, MetricSpec, N_METRICS, TickCtx};

/// Worst case for event-driven metrics.
pub(crate) fn min(_: &TickCtx, per_tick: &[f64]) -> f64 {
    per_tick.iter().copied().fold(1.0, f64::min)
}

/// Average for smooth metrics.
pub(crate) fn avg(_: &TickCtx, per_tick: &[f64]) -> f64 {
    per_tick.iter().sum::<f64>() / per_tick.len().max(1) as f64
}

/// The nuPlan composite: multiplier metrics gate a weighted average.
pub(crate) fn composite(specs: &[MetricSpec; N_METRICS], scores: &[f64; N_METRICS]) -> f64 {
    let (mut product, mut weighted, mut total_weight) = (1.0, 0.0, 0.0);
    for (spec, s) in specs.iter().zip(scores) {
        match &spec.role {
            CompositeRole::Multiplier => product *= *s,
            CompositeRole::Weighted(w) => {
                weighted += *w * *s;
                total_weight += *w;
            }
        }
    }
    product * weighted / total_weight
}
