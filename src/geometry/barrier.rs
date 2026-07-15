//! Physical road-side barrier segments and ego collision response.

use super::EGO_FOOTPRINT;
use crate::simulation::{Position, State};
use crate::track::Road;

/// Road-side barrier restitution: 0 = stick to the wall normal, 1 = elastic.
pub const BARRIER_RESTITUTION: f64 = 0.2;

/// A two-sided physical barrier segment. `normal` is only the reference side;
/// crossing either way clamps the vehicle to the segment and reflects the
/// velocity component through the crossed side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Barrier {
    pub a: Position,
    pub b: Position,
    pub normal: [f64; 2],
    pub restitution: f64,
}

impl Barrier {
    pub fn new(a: impl Into<Position>, b: impl Into<Position>, normal: [f64; 2]) -> Self {
        let n = normal[0].hypot(normal[1]).max(1e-9);
        Barrier {
            a: a.into(),
            b: b.into(),
            normal: [normal[0] / n, normal[1] / n],
            restitution: BARRIER_RESTITUTION,
        }
    }

    fn crossing(&self, prev: State, state: State) -> Option<(f64, Position, [f64; 2])> {
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
        let support = EGO_FOOTPRINT.support_radius(state.yaw, normal);
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
        self.footprint_overlaps_segment(center, state.yaw)
            .then_some((t, center, normal))
    }

    fn penetration(&self, prev: State, state: State) -> Option<(f64, Position, [f64; 2])> {
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
        let support = EGO_FOOTPRINT.support_radius(state.yaw, side);
        let side_distance = (p.x - self.a.x) * side[0] + (p.y - self.a.y) * side[1];
        let depth = support - side_distance;
        if depth <= 0.0 {
            return None;
        }
        let corrected = Position::new(p.x + side[0] * depth, p.y + side[1] * depth);
        if !self.footprint_overlaps_segment(corrected, state.yaw) {
            return None;
        }
        Some((depth, corrected, [-side[0], -side[1]]))
    }

    fn signed_distance(&self, p: Position) -> f64 {
        (p.x - self.a.x) * self.normal[0] + (p.y - self.a.y) * self.normal[1]
    }

    fn footprint_overlaps_segment(&self, center: Position, yaw: f64) -> bool {
        let ab = [self.b.x - self.a.x, self.b.y - self.a.y];
        let len = ab[0].hypot(ab[1]).max(1e-9);
        let tangent = [ab[0] / len, ab[1] / len];
        let along = (center.x - self.a.x) * tangent[0] + (center.y - self.a.y) * tangent[1];
        let support = EGO_FOOTPRINT.support_radius(yaw, tangent);
        along + support >= -1e-9 && along - support <= len + 1e-9
    }

    fn collide_at(&self, state: State, mut p: Position, normal: [f64; 2]) -> State {
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
        let support = EGO_FOOTPRINT.support_radius(yaw, normal);
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

    fn collide_penetration(&self, state: State, p: Position, normal: [f64; 2]) -> State {
        self.collide_at(state, p, normal)
    }
}

pub fn road_side_barriers(centerline: &[[f64; 2]], half_width: f64) -> Vec<Barrier> {
    if centerline.len() < 2 {
        return vec![];
    }
    let normals: Vec<_> = centerline
        .windows(2)
        .map(|w| {
            let len = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]).max(1e-9);
            [-(w[1][1] - w[0][1]) / len, (w[1][0] - w[0][0]) / len]
        })
        .collect();
    let offset: Vec<_> = centerline
        .iter()
        .enumerate()
        .map(|(i, _)| {
            let (miter, denominator) = if i == 0 {
                (normals[0], 1.0)
            } else if i == centerline.len() - 1 {
                (normals[i - 1], 1.0)
            } else {
                let prev = normals[i - 1];
                let next = normals[i];
                (
                    [prev[0] + next[0], prev[1] + next[1]],
                    1.0 + prev[0] * next[0] + prev[1] * next[1],
                )
            };
            [
                half_width * miter[0] / denominator.max(1e-9),
                half_width * miter[1] / denominator.max(1e-9),
            ]
        })
        .collect();

    centerline
        .windows(2)
        .enumerate()
        .flat_map(|(i, w)| {
            let (a, b, left) = (w[0], w[1], normals[i]);
            [
                Barrier::new(
                    [a[0] + offset[i][0], a[1] + offset[i][1]],
                    [b[0] + offset[i + 1][0], b[1] + offset[i + 1][1]],
                    left,
                ),
                Barrier::new(
                    [a[0] - offset[i][0], a[1] - offset[i][1]],
                    [b[0] - offset[i + 1][0], b[1] - offset[i + 1][1]],
                    [-left[0], -left[1]],
                ),
            ]
        })
        .collect()
}

pub fn collide_with_barriers(
    prev: State,
    state: State,
    barriers: impl IntoIterator<Item = Barrier>,
) -> State {
    let barriers: Vec<_> = barriers.into_iter().collect();
    if let Some((b, t, p, n)) = barriers
        .iter()
        .filter_map(|&b| b.crossing(prev, state).map(|(t, p, n)| (b, t, p, n)))
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
        return b.collide_at(state, p, n);
    }

    barriers
        .into_iter()
        .filter_map(|b| b.penetration(prev, state).map(|(d, p, n)| (b, d, p, n)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map_or(state, |(b, _, p, n)| b.collide_penetration(state, p, n))
}

/// Clamp the ego to the road sides and reflect its outward velocity component.
pub(crate) fn collide_with_road_barriers(prev: State, state: State, road: &Road) -> State {
    let Some(i) = closest_centerline_segment(&road.centerline, state.position()) else {
        return state;
    };
    let start = 2 * i.saturating_sub(1);
    let end = 2 * (i + 2).min(road.centerline.len() - 1);
    let Some(barriers) = road.barriers.get(start..end) else {
        return state;
    };
    collide_with_barriers(prev, state, barriers.iter().copied())
}

pub(crate) fn collides_with_road_barrier(state: State, road: &Road) -> bool {
    collide_with_road_barriers(state, state, road) != state
}

fn closest_centerline_segment(centerline: &[[f64; 2]], p: Position) -> Option<usize> {
    centerline
        .windows(2)
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            segment_distance2(a[0].into(), a[1].into(), p).total_cmp(&segment_distance2(
                b[0].into(),
                b[1].into(),
                p,
            ))
        })
        .map(|(i, _)| i)
}

fn segment_distance2(a: Position, b: Position, p: Position) -> f64 {
    let ab = [b.x - a.x, b.y - a.y];
    let len2 = (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-9);
    let u = (((p.x - a.x) * ab[0] + (p.y - a.y) * ab[1]) / len2).clamp(0.0, 1.0);
    let q = Position::new(a.x + ab[0] * u, a.y + ab[1] * u);
    (p.x - q.x).powi(2) + (p.y - q.y).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;

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
            &road,
        );

        let support = EGO_FOOTPRINT.support_radius(prev.yaw, [0.0, 1.0]);
        assert_eq!((hit.x, hit.y), (12.0, road.half_width - support));
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
            collide_with_road_barriers(on_first_segment, on_first_segment, &road),
            on_first_segment
        );
    }

    #[test]
    fn barrier_entity_is_two_sided_physics() {
        let wall = Barrier::new([0.0, 0.0], [100.0, 0.0], [0.0, 1.0]);
        let inside = State {
            x: 12.0,
            y: -3.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 10.0,
        };
        assert_eq!(collide_with_barriers(inside, inside, [wall]), inside);

        let hit = collide_with_barriers(inside, State { y: 1.0, ..inside }, [wall]);
        let support = EGO_FOOTPRINT.support_radius(inside.yaw, [0.0, 1.0]);
        assert_eq!((hit.x, hit.y), (12.0, -support));
        assert!((hit.speed - BARRIER_RESTITUTION * 10.0).abs() < 1e-9);

        let reverse = collide_with_barriers(
            State {
                y: 3.0,
                yaw: -std::f64::consts::FRAC_PI_2,
                ..inside
            },
            State {
                y: -1.0,
                yaw: -std::f64::consts::FRAC_PI_2,
                ..inside
            },
            [wall],
        );
        assert_eq!((reverse.x, reverse.y), (12.0, support));
        assert!((reverse.speed - BARRIER_RESTITUTION * 10.0).abs() < 1e-9);
        assert!((reverse.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn road_barriers_are_continuous_through_polyline_joints() {
        let barriers = road_side_barriers(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]], 3.5);

        assert_eq!(barriers[0].b, barriers[2].a);
        assert_eq!(barriers[1].b, barriers[3].a);
    }

    #[test]
    fn rectangle_cannot_clip_past_a_barrier_endpoint() {
        let wall = Barrier::new([0.0, 0.0], [10.0, 0.0], [0.0, 1.0]);
        let overlapping = State {
            x: 11.0,
            y: -0.5,
            yaw: std::f64::consts::FRAC_PI_4,
            speed: 5.0,
        };

        let hit = collide_with_barriers(overlapping, overlapping, [wall]);

        assert_ne!(hit, overlapping);
        assert_eq!(
            hit.y,
            -EGO_FOOTPRINT.support_radius(overlapping.yaw, [0.0, 1.0])
        );
    }
}
