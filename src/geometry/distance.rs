use crate::common::types::position::Position;

pub(crate) fn dist(a: impl Into<Position>, b: impl Into<Position>) -> f64 {
    a.into().distance(b.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn measures_point_and_pose_distance() {
        assert_eq!(dist([0.0, 0.0], [3.0, 4.0]), 5.0);
        assert_eq!(
            dist(
                crate::simulation::Pose::default(),
                crate::simulation::Pose::new(3.0, 4.0, 1.0)
            ),
            5.0
        );
    }
}
