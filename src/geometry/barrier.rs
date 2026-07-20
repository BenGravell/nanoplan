//! Physical road-side barrier segments and dynamic-body collision response.

use super::{Footprint, RoadPolygon};
use crate::simulation::{Position, State};
use crate::track::Road;

/// Road-side barrier restitution: 0 = stick to the wall normal, 1 = elastic.
pub(crate) const BARRIER_RESTITUTION: f64 = 0.2;

/// A two-sided physical barrier segment. `normal` is only the reference side;
/// crossing either way clamps the vehicle to the segment and reflects the
/// velocity component through the crossed side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Barrier {
    pub(crate) a: Position,
    pub(crate) b: Position,
    pub(crate) normal: [f64; 2],
    pub(crate) restitution: f64,
}

impl Barrier {
    pub(crate) fn new(a: impl Into<Position>, b: impl Into<Position>, normal: [f64; 2]) -> Self {
        let n = normal[0].hypot(normal[1]).max(1e-9);
        Barrier {
            a: a.into(),
            b: b.into(),
            normal: [normal[0] / n, normal[1] / n],
            restitution: BARRIER_RESTITUTION,
        }
    }

    fn crossing(
        &self,
        prev: State,
        state: State,
        footprint: Footprint,
    ) -> Option<(f64, Position, [f64; 2])> {
        let p0 = prev.position();
        let p1 = state.position();
        let d0 = self.signed_distance(p0);
        let d1 = self.signed_distance(p1);
        let dd = d1 - d0;
        if dd.abs() < 1e-9 {
            return None;
        }

        let normal = if dd > 0.0 {
            self.normal
        } else {
            [-self.normal[0], -self.normal[1]]
        };
        let support = footprint.support_radius(state.yaw, normal);
        let boundary = if dd > 0.0 { -support } else { support };
        if (dd > 0.0 && (d0 > boundary || d1 <= boundary))
            || (dd < 0.0 && (d0 < boundary || d1 >= boundary))
        {
            return None;
        }

        let t = (boundary - d0) / dd;
        if !(0.0..=1.0).contains(&t) {
            return None;
        }
        let center = Position::new(p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t);
        self.footprint_overlaps_segment(center, state.yaw, footprint)
            .then_some((t, center, normal))
    }

    fn penetration(
        &self,
        prev: State,
        state: State,
        footprint: Footprint,
    ) -> Option<(f64, Position, [f64; 2])> {
        let p = state.position();
        let prev_d = self.signed_distance(prev.position());
        let d = self.signed_distance(p);
        let side = if prev_d > 0.0 && d >= prev_d {
            [-self.normal[0], -self.normal[1]]
        } else if prev_d >= 0.0 {
            self.normal
        } else {
            [-self.normal[0], -self.normal[1]]
        };
        let support = footprint.support_radius(state.yaw, side);
        let side_distance = (p.x - self.a.x) * side[0] + (p.y - self.a.y) * side[1];
        let depth = support - side_distance;
        if depth <= 0.0 {
            return None;
        }
        let corrected = Position::new(p.x + side[0] * depth, p.y + side[1] * depth);
        if !self.footprint_overlaps_segment(corrected, state.yaw, footprint) {
            return None;
        }
        Some((depth, corrected, [-side[0], -side[1]]))
    }

    fn signed_distance(&self, p: Position) -> f64 {
        (p.x - self.a.x) * self.normal[0] + (p.y - self.a.y) * self.normal[1]
    }

    fn footprint_overlaps_segment(&self, center: Position, yaw: f64, footprint: Footprint) -> bool {
        let ab = [self.b.x - self.a.x, self.b.y - self.a.y];
        let len = ab[0].hypot(ab[1]).max(1e-9);
        let tangent = [ab[0] / len, ab[1] / len];
        let along = (center.x - self.a.x) * tangent[0] + (center.y - self.a.y) * tangent[1];
        let support = footprint.support_radius(yaw, tangent);
        along + support >= -1e-9 && along - support <= len + 1e-9
    }

    fn collide_at(
        &self,
        state: State,
        mut p: Position,
        normal: [f64; 2],
        footprint: Footprint,
    ) -> State {
        let mut v = [state.speed * state.yaw.cos(), state.speed * state.yaw.sin()];
        let vn = v[0] * normal[0] + v[1] * normal[1];
        if vn > 0.0 {
            v[0] -= (1.0 + self.restitution) * vn * normal[0];
            v[1] -= (1.0 + self.restitution) * vn * normal[1];
        }
        let speed = v[0].hypot(v[1]);
        let yaw = if speed > 1e-6 {
            v[1].atan2(v[0])
        } else {
            state.yaw
        };
        let support = footprint.support_radius(yaw, normal);
        let depth = (p.x - self.a.x) * normal[0] + (p.y - self.a.y) * normal[1] + support;
        if depth > 1e-12 {
            p.x -= normal[0] * depth;
            p.y -= normal[1] * depth;
        }
        State {
            x: p.x,
            y: p.y,
            yaw,
            speed,
        }
    }

    fn collide_penetration(
        &self,
        state: State,
        p: Position,
        normal: [f64; 2],
        footprint: Footprint,
    ) -> State {
        self.collide_at(state, p, normal, footprint)
    }
}

pub(crate) fn road_side_barriers(road: &RoadPolygon) -> Vec<Barrier> {
    (0..road.segment_count())
        .flat_map(|i| {
            let next = (i + 1) % road.centerline().len();
            let (left_a, left_b) = (road.left_boundary()[i], road.left_boundary()[next]);
            let (right_a, right_b) = (road.right_boundary()[i], road.right_boundary()[next]);
            let left_tangent = [left_b[0] - left_a[0], left_b[1] - left_a[1]];
            let right_tangent = [right_b[0] - right_a[0], right_b[1] - right_a[1]];
            [
                Barrier::new(left_a, left_b, [-left_tangent[1], left_tangent[0]]),
                Barrier::new(right_a, right_b, [right_tangent[1], -right_tangent[0]]),
            ]
        })
        .collect()
}

pub(crate) fn collide_with_barriers(
    prev: State,
    state: State,
    footprint: Footprint,
    barriers: impl IntoIterator<Item = Barrier>,
) -> State {
    let prev_center = center_state(prev, footprint);
    let state_center = center_state(state, footprint);
    let result = collide_centers_with_barriers(prev_center, state_center, footprint, barriers);
    if result == state_center {
        state
    } else {
        rear_state(result, footprint)
    }
}

fn collide_centers_with_barriers(
    prev: State,
    state: State,
    footprint: Footprint,
    barriers: impl IntoIterator<Item = Barrier>,
) -> State {
    let barriers: Vec<_> = barriers.into_iter().collect();
    if let Some((b, t, p, n)) = barriers
        .iter()
        .filter_map(|&b| {
            b.crossing(prev, state, footprint)
                .map(|(t, p, n)| (b, t, p, n))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
    {
        let remaining = [
            (state.x - prev.x) * (1.0 - t),
            (state.y - prev.y) * (1.0 - t),
        ];
        let into_wall = (remaining[0] * n[0] + remaining[1] * n[1]).max(0.0);
        let p = Position::new(
            p.x + remaining[0] - n[0] * into_wall,
            p.y + remaining[1] - n[1] * into_wall,
        );
        return b.collide_at(state, p, n, footprint);
    }

    barriers
        .into_iter()
        .filter_map(|b| {
            b.penetration(prev, state, footprint)
                .map(|(d, p, n)| (b, d, p, n))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map_or(state, |(b, _, p, n)| {
            b.collide_penetration(state, p, n, footprint)
        })
}

/// Clamp a dynamic body to the road sides and reflect its outward velocity.
/// The barrier receives no reciprocal impulse: it has infinite inertia.
pub(crate) fn collide_with_road_barriers(
    prev: State,
    state: State,
    footprint: Footprint,
    road: &Road,
) -> State {
    let polygon = road.polygon();
    let centerline = polygon.centerline();
    let Some((i, segment_u)) = closest_centerline_segment(
        centerline,
        polygon.is_closed(),
        center_state(state, footprint).position(),
    ) else {
        return state;
    };
    // A rolling road window is open at both ends. Its side segments must not
    // become accidental end caps for bodies that continue along the track
    // beyond the materialized window.
    if !polygon.is_closed()
        && ((i == 0 && segment_u < 0.0) || (i + 2 == centerline.len() && segment_u > 1.0))
    {
        return state;
    }
    if polygon.is_closed() {
        let segment_count = polygon.segment_count();
        let barriers = [
            (i + segment_count - 1) % segment_count,
            i,
            (i + 1) % segment_count,
        ]
        .into_iter()
        .flat_map(|segment| {
            [
                road.barriers()[2 * segment],
                road.barriers()[2 * segment + 1],
            ]
        });
        return collide_with_barriers(prev, state, footprint, barriers);
    }
    let start = 2 * i.saturating_sub(1);
    let end = 2 * (i + 2).min(centerline.len() - 1);
    let Some(barriers) = road.barriers().get(start..end) else {
        return state;
    };
    collide_with_barriers(prev, state, footprint, barriers.iter().copied())
}

fn center_state(mut state: State, footprint: Footprint) -> State {
    let center = footprint.center(state.pose());
    state.x = center.x;
    state.y = center.y;
    state
}

fn rear_state(mut state: State, footprint: Footprint) -> State {
    state.x -= 0.5 * footprint.length * state.yaw.cos();
    state.y -= 0.5 * footprint.length * state.yaw.sin();
    state
}

pub(crate) fn collides_with_road_barrier(state: State, road: &Road) -> bool {
    collide_with_road_barriers(state, state, super::EGO_FOOTPRINT, road) != state
}

fn closest_centerline_segment(
    centerline: &[[f64; 2]],
    closed: bool,
    p: Position,
) -> Option<(usize, f64)> {
    let segment_count = centerline.len().saturating_sub(usize::from(!closed));
    (0..segment_count)
        .min_by(|&a, &b| {
            let a_next = (a + 1) % centerline.len();
            let b_next = (b + 1) % centerline.len();
            segment_projection(centerline[a].into(), centerline[a_next].into(), p)
                .0
                .total_cmp(
                    &segment_projection(centerline[b].into(), centerline[b_next].into(), p).0,
                )
        })
        .map(|i| {
            let next = (i + 1) % centerline.len();
            let (_, u) = segment_projection(centerline[i].into(), centerline[next].into(), p);
            (i, u)
        })
}

/// Squared distance to the finite segment and the unclamped projection along
/// its supporting line. Values outside 0..=1 lie beyond an endpoint.
fn segment_projection(a: Position, b: Position, p: Position) -> (f64, f64) {
    let ab = [b.x - a.x, b.y - a.y];
    let len2 = (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-9);
    let u = ((p.x - a.x) * ab[0] + (p.y - a.y) * ab[1]) / len2;
    let clamped = u.clamp(0.0, 1.0);
    let q = Position::new(a.x + ab[0] * clamped, a.y + ab[1] * clamped);
    ((p.x - q.x).powi(2) + (p.y - q.y).powi(2), u)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::EGO_FOOTPRINT;

    #[test]
    fn road_barriers_clamp_and_reflect_outward_motion() {
        let road = Road::new(vec![[0.0, 0.0], [100.0, 0.0]], 10.0, 3.5, 0.1);
        let prev = State {
            x: 12.0,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 10.0,
        };
        let hit = collide_with_road_barriers(
            prev,
            State {
                x: 12.0,
                y: 4.5,
                yaw: std::f64::consts::FRAC_PI_2,
                speed: 10.0,
            },
            EGO_FOOTPRINT,
            &road,
        );

        assert!(
            (hit.y + EGO_FOOTPRINT.support(hit.yaw, [0.0, 1.0]) - road.half_width).abs() < 1e-12
        );
        assert!((hit.speed - BARRIER_RESTITUTION * 10.0).abs() < 1e-9);
        assert!((hit.yaw + std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn road_barriers_ignore_nonlocal_route_segments() {
        let road = Road::new(
            vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
            10.0,
            3.5,
            0.1,
        );
        let on_first_segment = State {
            x: 50.0,
            y: 0.0,
            yaw: 0.0,
            speed: 8.0,
        };

        assert_eq!(
            collide_with_road_barriers(on_first_segment, on_first_segment, EGO_FOOTPRINT, &road,),
            on_first_segment
        );
    }

    #[test]
    fn road_side_barriers_do_not_cap_an_open_window() {
        let road = Road::new(vec![[0.0, 0.0], [10.0, 0.0]], 10.0, 3.5, 0.1);
        let previous = State {
            x: 10.5,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 10.0,
        };
        let beyond_window = State { y: 4.5, ..previous };

        assert_eq!(
            collide_with_road_barriers(previous, beyond_window, EGO_FOOTPRINT, &road),
            beyond_window
        );

        let before_window = State {
            x: -0.5,
            ..previous
        };
        let beyond_start = State {
            y: 4.5,
            ..before_window
        };
        assert_eq!(
            collide_with_road_barriers(before_window, beyond_start, EGO_FOOTPRINT, &road),
            beyond_start
        );
    }

    #[test]
    fn closed_road_collision_ignores_distant_barrier_segments() {
        let polygon = RoadPolygon::new(
            vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
            vec![10.0; 4],
            vec![10.0; 4],
            true,
        )
        .unwrap();
        let road = Road::from_polygon(polygon, 10.0, 0.1);
        let state = State {
            x: 50.0,
            y: 0.0,
            speed: 10.0,
            ..Default::default()
        };

        assert_eq!(
            collide_with_road_barriers(state, state, EGO_FOOTPRINT, &road),
            state
        );
    }

    #[test]
    fn barrier_entity_is_two_sided_physics() {
        let wall = Barrier::new([0.0, 0.0], [100.0, 0.0], [0.0, 1.0]);
        let inside = State {
            x: 12.0,
            y: -6.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 10.0,
        };
        assert_eq!(
            collide_with_barriers(inside, inside, EGO_FOOTPRINT, [wall]),
            inside
        );

        let hit = collide_with_barriers(inside, State { y: 1.0, ..inside }, EGO_FOOTPRINT, [wall]);
        assert_eq!((hit.x, hit.y), (12.0, 0.0));
        assert!((hit.speed - BARRIER_RESTITUTION * 10.0).abs() < 1e-9);

        let reverse = collide_with_barriers(
            State {
                y: 6.0,
                yaw: -std::f64::consts::FRAC_PI_2,
                ..inside
            },
            State {
                y: -1.0,
                yaw: -std::f64::consts::FRAC_PI_2,
                ..inside
            },
            EGO_FOOTPRINT,
            [wall],
        );
        assert_eq!((reverse.x, reverse.y), (12.0, 0.0));
        assert!((reverse.speed - BARRIER_RESTITUTION * 10.0).abs() < 1e-9);
        assert!((reverse.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn road_barriers_are_continuous_through_polyline_joints() {
        let road = RoadPolygon::uniform(vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]], 3.5).unwrap();
        let barriers = road_side_barriers(&road);

        assert_eq!(barriers[0].b, barriers[2].a);
        assert_eq!(barriers[1].b, barriers[3].a);
    }

    #[test]
    fn barriers_are_the_polygon_boundary_segments() {
        let road = RoadPolygon::new(
            vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]],
            vec![1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
            false,
        )
        .unwrap();
        let barriers = road_side_barriers(&road);

        for i in 0..road.segment_count() {
            assert_eq!(barriers[2 * i].a, road.left_boundary()[i].into());
            assert_eq!(barriers[2 * i].b, road.left_boundary()[i + 1].into());
            assert_eq!(barriers[2 * i + 1].a, road.right_boundary()[i].into());
            assert_eq!(barriers[2 * i + 1].b, road.right_boundary()[i + 1].into());
        }
    }

    #[test]
    fn rectangle_cannot_clip_past_a_barrier_endpoint() {
        let wall = Barrier::new([0.0, 0.0], [10.0, 0.0], [0.0, 1.0]);
        let overlapping = State {
            x: 10.0,
            y: -0.5,
            yaw: std::f64::consts::FRAC_PI_4,
            speed: 5.0,
        };

        let hit = collide_with_barriers(overlapping, overlapping, EGO_FOOTPRINT, [wall]);

        assert_ne!(hit, overlapping);
        assert_eq!(collide_with_barriers(hit, hit, EGO_FOOTPRINT, [wall]), hit);
    }
}
