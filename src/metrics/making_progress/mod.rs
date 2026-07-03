//! Making progress: thresholds the progress ratio (per tick for the tick
//! display, and the aggregated progress ratio for the scenario, as in
//! nuPlan's min_progress_threshold).

const MIN_PROGRESS_RATIO: f64 = 0.2;

pub fn score(progress_ratio: f64) -> f64 {
    if progress_ratio > MIN_PROGRESS_RATIO {
        1.0
    } else {
        0.0
    }
}
