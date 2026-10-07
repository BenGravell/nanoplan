use super::preview_metrics;
use crate::viewer::live::Live;

#[test]
fn preview_metrics_are_valid_scores() {
    let metrics = preview_metrics(&Live::default());
    assert!(metrics.score.is_finite());
    assert!(metrics.score_per_tick.iter().all(|score| score.is_finite()));
}
