use super::preview_metrics;
use crate::viewer::live::Live;

#[test]
fn preview_metrics_are_valid_scores() {
    let metrics = preview_metrics(&Live::default());
    assert!(
        metrics
            .aggregate
            .into_iter()
            .chain([metrics.score])
            .all(|score| (0.0..=1.0).contains(&score))
    );
}
