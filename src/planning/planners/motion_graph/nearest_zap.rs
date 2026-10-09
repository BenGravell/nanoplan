//! Single-parent tree connections using the nearest zero-action endpoint.

use super::{Edge, ZapIndex};
use crate::simulation::State;

fn nearest_parent(index: &ZapIndex, target: State) -> usize {
    // Break equal-distance ties by insertion order, independent of index traversal.
    index
        .0
        .nearest_neighbors(&ZapIndex::point(target))
        .iter()
        .map(|point| point.data)
        .min()
        .expect("layers are never empty")
}

pub(super) fn parents(index: &ZapIndex, target: State) -> Vec<usize> {
    vec![nearest_parent(index, target)]
}

pub(super) fn best_incoming(incoming: &[Edge]) -> usize {
    // A tree has no alternative arrival to relax.
    assert_eq!(incoming.len(), 1);
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::geometry::wrap_angle;
    use crate::common::kinematics::zero_action_point;
    use crate::planning::sampling::{Halton, QuasiMonteCarlo};
    use crate::simulation::{Pose, Position};

    fn zap_dist2(zap: State, target: State) -> f64 {
        (zap.position().x - target.position().x).powi(2)
            + (zap.position().y - target.position().y).powi(2)
            + wrap_angle(zap.pose.yaw - target.pose.yaw).powi(2)
            + (zap.speed - target.speed).powi(2)
    }

    #[test]
    fn zap_index_matches_linear_scan() {
        use std::f64::consts::TAU;

        let state = |i| {
            State::new(
                Pose::new(
                    Position::new(80.0 * Halton::coordinate(i, 0), 10.0 * Halton::coordinate(i, 1)),
                    (Halton::coordinate(i, 2) - 0.5) * 4.0 * TAU,
                ),
                20.0 * Halton::coordinate(i, 3),
            )
        };
        // Sparse arena ids ensure the index returns node ids, not offsets
        // into its point cloud. Vary tick count and step size to catch hard-coded coasting.
        for count in [1, 16, 256] {
            let parents: Vec<_> = (1..=count).map(|i| (3 * i, state(i))).collect();
            for (ticks, dt) in [(0, 0.1), (4, 0.1), (10, 0.1), (10, 0.03), (10, 0.2)] {
                let index = ZapIndex::new(parents.iter().copied(), ticks, dt);
                for i in 257..513 {
                    let target = state(i);
                    let expected = parents
                        .iter()
                        .min_by(|(_, a), (_, b)| {
                            zap_dist2(zero_action_point(*a, ticks, dt), target)
                                .total_cmp(&zap_dist2(zero_action_point(*b, ticks, dt), target))
                        })
                        .unwrap()
                        .0;
                    assert_eq!(nearest_parent(&index, target), expected);
                }
            }
        }
    }

    #[test]
    fn zap_index_wraps_yaw_and_breaks_ties_by_insertion_order() {
        use std::f64::consts::PI;

        let target = State::new(Pose::new(Position::new(0.0, 0.0), PI - 0.01), 0.0);
        let across_seam = State::new(Pose::new(target.position(), -PI + 0.01), 0.0);
        let farther = State::new(Pose::new(target.position(), PI - 0.1), 0.0);
        let faster = State { speed: 1.0, ..target };
        let parents = [(2, farther), (7, across_seam), (11, across_seam), (19, faster)];
        let index = ZapIndex::new(parents.into_iter(), 10, 0.1);
        assert_eq!(nearest_parent(&index, target), 7);

        // The nearest current position need not have the nearest coasting
        // endpoint. Match speed as well as position, even at zero duration.
        let target = State {
            speed: 8.0,
            ..State::default()
        };
        let moving = State::new(Pose::new(Position::new(-8.0, 0.0), 0.0), 8.0);
        let parents = [(4, State::default()), (9, moving)];
        let index = ZapIndex::new(parents.into_iter(), 10, 0.1);
        assert_eq!(nearest_parent(&index, zero_action_point(moving, 10, 0.1)), 9);
        let index = ZapIndex::new([(4, State::default()), (9, target)].into_iter(), 0, 0.1);
        assert_eq!(nearest_parent(&index, target), 9);
    }
}
