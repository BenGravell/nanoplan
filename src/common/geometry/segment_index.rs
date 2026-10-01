//! Exact nearest-segment lookup shared by road collision and Frenet projection.

use crate::planning::latency::geometry_work;
use crate::simulation::Position;
use rstar::{AABB, PointDistance, RTree, RTreeObject};

#[derive(Debug, Clone)]
struct Segment {
    a: Position,
    b: Position,
    index: usize,
    min_length_squared: f64,
}

impl Segment {
    fn projection(&self, p: Position) -> (f64, f64) {
        geometry_work(1);
        let ab = self.b - self.a;
        let len2 = ab.norm_squared().max(self.min_length_squared);
        let offset = p - self.a;
        let u = (offset.x * ab.x + offset.y * ab.y) / len2;
        let q = self.a + ab * u.clamp(0.0, 1.0);
        ((p - q).norm_squared(), u)
    }
}

impl RTreeObject for Segment {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.a.xy(), self.b.xy())
    }
}

impl PointDistance for Segment {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        self.projection((*point).into()).0
    }
}

#[derive(Debug)]
pub(crate) struct SegmentIndex(RTree<Segment>);

impl SegmentIndex {
    pub(crate) fn new(points: &[Position], closed: bool, min_length_squared: f64) -> Self {
        let count = points.len().saturating_sub(usize::from(!closed));
        crate::planning::latency::geometry_build_work(count as u64);
        geometry_work(count as u64);
        Self(RTree::bulk_load(
            (0..count)
                .map(|index| Segment {
                    a: points[index],
                    b: points[(index + 1) % points.len()],
                    index,
                    min_length_squared,
                })
                .collect(),
        ))
    }

    pub(crate) fn nearest_in_range(&self, p: Position, range: std::ops::Range<usize>) -> Option<(usize, f64)> {
        if range == (0..self.0.size()) {
            return self.nearest(p);
        }
        let mut neighbors = self
            .0
            .nearest_neighbor_iter_with_distance_2(&p.xy())
            .filter(|(segment, _)| range.contains(&segment.index));
        let (mut best, distance) = neighbors.next()?;
        for (segment, d) in neighbors {
            if d > distance {
                break;
            }
            if segment.index < best.index {
                best = segment;
            }
        }
        Some((best.index, best.projection(p).1))
    }

    pub(crate) fn nearest(&self, p: Position) -> Option<(usize, f64)> {
        // Match the old forward scan's first-segment tie break, including
        // duplicated laps, shared vertices and the closed-track seam.
        self.0
            .nearest_neighbors(&p.xy())
            .into_iter()
            .min_by_key(|segment| segment.index)
            .map(|segment| (segment.index, segment.projection(p).1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn restricted_index_matches_exhaustive_projection_including_outside_windows() {
        let points: Vec<_> = (0..100)
            .map(|i| Position::new(i as f64, (i as f64 * 0.2).sin()))
            .collect();
        let index = SegmentIndex::new(&points, false, 0.0);
        for range in [0..1, 0..50, 25..75, 98..99] {
            for p in [[-10.0, 2.0], [30.5, 0.1], [50.0, 100.0], [120.0, 0.0]] {
                let p = Position::from(p);
                let expected = index
                    .0
                    .iter()
                    .filter(|segment| range.contains(&segment.index))
                    .min_by(|a, b| {
                        a.distance_2(&p.xy())
                            .total_cmp(&b.distance_2(&p.xy()))
                            .then_with(|| a.index.cmp(&b.index))
                    })
                    .map(|segment| (segment.index, segment.projection(p).1));
                assert_eq!(index.nearest_in_range(p, range.clone()), expected);
            }
        }
    }

    #[test]
    fn nearest_preserves_endpoints_degenerate_segments_and_closed_seam() {
        let points: Vec<Position> = vec![[0.0, 0.0].into(), [10.0, 0.0].into(), [10.0, 10.0].into()];
        let open = SegmentIndex::new(&points, false, 1e-9);
        assert_eq!(open.nearest([-2.0, 0.0].into()), Some((0, -0.2)));
        assert_eq!(open.nearest([10.0, 12.0].into()), Some((1, 1.2)));
        assert_eq!(open.nearest([10.0, 0.0].into()), Some((0, 1.0)));
        let closed = SegmentIndex::new(&points, true, 1e-9);
        assert_eq!(closed.nearest([5.0, 5.0].into()), Some((2, 0.5)));
        assert_eq!(closed.nearest([0.0, 0.0].into()), Some((0, 0.0)));
        let degenerate = SegmentIndex::new(&[points[0], points[0], points[1]], false, 1e-9);
        assert_eq!(degenerate.nearest([0.0, 1.0].into()), Some((0, 0.0)));
        assert_eq!(SegmentIndex::new(&[], false, 1e-9).nearest([0.0, 0.0].into()), None);
    }
}
