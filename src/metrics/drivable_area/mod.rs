//! Drivable area compliance: 1 while the ego footprint fits between the
//! road's actual side barriers near the ego center's station. Event-driven
//! — aggregates by worst case (min).

use crate::barrier::Barrier;
use crate::geometry::EGO_FOOTPRINT;
use crate::metrics::TickCtx;
use crate::simulation::{Position, State};
use crate::track::Road;

const EPS: f64 = 1e-9;

pub(crate) fn compliance(ego: &[State], road: &Road) -> Vec<bool> {
    ego.iter()
        .copied()
        .map(|s| {
            barrier_geometry_contains(road, s)
                .unwrap_or_else(|| centerline_geometry_contains(road, s))
        })
        .collect()
}

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    if ctx.drivable_area[i] { 1.0 } else { 0.0 }
}

fn barrier_geometry_contains(road: &Road, state: State) -> Option<bool> {
    if road.barriers.chunks_exact(2).len() == 0 {
        return None;
    }
    Some(
        road.barriers
            .chunks_exact(2)
            .any(|b| barrier_pair_contains(b, state)),
    )
}

fn barrier_pair_contains(pair: &[Barrier], state: State) -> bool {
    let [left, right] = [pair[0], pair[1]];
    let p = state.position();
    signed_distance(left, p) + EGO_FOOTPRINT.support_radius(state.yaw, left.normal) <= EPS
        && signed_distance(right, p) + EGO_FOOTPRINT.support_radius(state.yaw, right.normal) <= EPS
        && within_segment(mid(left.a, right.a), mid(left.b, right.b), p)
}

fn signed_distance(b: Barrier, p: Position) -> f64 {
    (p.x - b.a.x) * b.normal[0] + (p.y - b.a.y) * b.normal[1]
}

fn centerline_geometry_contains(road: &Road, state: State) -> bool {
    road.centerline
        .windows(2)
        .any(|w| centerline_segment_contains(w[0].into(), w[1].into(), road.half_width, state))
}

fn centerline_segment_contains(a: Position, b: Position, half_width: f64, state: State) -> bool {
    let p = state.position();
    let ab = [b.x - a.x, b.y - a.y];
    let len = ab[0].hypot(ab[1]).max(1e-9);
    let dir = [ab[0] / len, ab[1] / len];
    let left = [-dir[1], dir[0]];
    let ap = [p.x - a.x, p.y - a.y];
    let along = ap[0] * dir[0] + ap[1] * dir[1];
    let lateral = ap[0] * left[0] + ap[1] * left[1];
    (-EGO_FOOTPRINT.length - EPS..=len + EGO_FOOTPRINT.length + EPS).contains(&along)
        && lateral.abs() + EGO_FOOTPRINT.support_radius(state.yaw, left) <= half_width + EPS
}

fn within_segment(a: Position, b: Position, p: Position) -> bool {
    let ab = [b.x - a.x, b.y - a.y];
    let len2 = (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-9);
    let len = len2.sqrt();
    let along = ((p.x - a.x) * ab[0] + (p.y - a.y) * ab[1]) / len;
    (-EGO_FOOTPRINT.length - EPS..=len + EGO_FOOTPRINT.length + EPS).contains(&along)
}

fn mid(a: Position, b: Position) -> Position {
    Position::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn road() -> Road {
        Road::new(vec![[0.0, 0.0], [20.0, 0.0]], 10.0, 3.0, 0.1)
    }

    #[test]
    fn uses_footprint_not_just_center_offset() {
        assert!(
            !compliance(
                &[State {
                    x: 10.0,
                    y: 2.5,
                    ..Default::default()
                }],
                &road()
            )[0]
        );
    }

    #[test]
    fn rejects_centers_beyond_the_road_segment_extent() {
        assert!(
            !compliance(
                &[State {
                    x: 30.0,
                    ..Default::default()
                }],
                &road()
            )[0]
        );
    }
}
