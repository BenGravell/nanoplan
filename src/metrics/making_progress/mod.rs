//! Making progress: thresholds the progress ratio (per tick for the tick
//! display, and the aggregated progress ratio for the scenario, as in
//! nuPlan's min_progress_threshold).

use crate::metrics::{TickCtx, agg, progress};

const MIN_PROGRESS_RATIO: f64 = 0.2;

fn of_ratio(progress_ratio: f64) -> f64 {
    if progress_ratio > MIN_PROGRESS_RATIO {
        1.0
    } else {
        0.0
    }
}

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    of_ratio(progress::score(ctx, i))
}

/// Metric-specific aggregation, as in nuPlan: threshold the *aggregated*
/// (average) progress ratio, not the average of the per-tick thresholds.
/// Recomputes the progress column from `ctx` — identical to the progress
/// metric's own per-tick scores — so the aggregation interface stays
/// uniform across metrics.
pub fn aggregate(ctx: &TickCtx, _per_tick: &[f64]) -> f64 {
    let ratios: Vec<f64> = (0..ctx.ego.len()).map(|i| progress::score(ctx, i)).collect();
    of_ratio(agg::avg(ctx, &ratios))
}
