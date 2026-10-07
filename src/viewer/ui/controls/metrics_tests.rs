use super::preview_score;
use crate::viewer::live::Live;

#[test]
fn preview_score_is_finite() {
    let score = preview_score(&Live::default());
    assert!(score.is_finite());
}
