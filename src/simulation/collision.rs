use super::State;
use crate::geometry::{Footprint, overlap_mtv};

/// Restitution shared by every dynamic vehicle collision.
const VEHICLE_RESTITUTION: f64 = 0.1;
const SEPARATION_EPSILON_M: f64 = 1e-9;

/// A finite-mass body participating in vehicle-to-vehicle collisions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DynamicBody {
    pub(crate) state: State,
    pub(crate) footprint: Footprint,
}

impl DynamicBody {
    pub(crate) const fn new(state: State, footprint: Footprint) -> Self {
        Self { state, footprint }
    }
}

/// Resolve all dynamic bodies.
///
/// Each contact shares positional correction equally and applies one impulse
/// to both bodies, conserving momentum along the contact normal. Repeated
/// passes let the correction propagate through small piles of vehicles.
pub(crate) fn collide_dynamic_bodies(bodies: &mut [DynamicBody]) {
    for _ in 0..32 * bodies.len().max(1) {
        let mut had_contact = false;
        for a in 0..bodies.len() {
            for b in a + 1..bodies.len() {
                let (left, right) = bodies.split_at_mut(b);
                had_contact |= collide_pair(&mut left[a], &mut right[0]);
            }
        }
        if !had_contact {
            break;
        }
    }
}

fn collide_pair(a: &mut DynamicBody, b: &mut DynamicBody) -> bool {
    let Some(hit) = overlap_mtv(a.state.pose(), a.footprint, b.state.pose(), b.footprint) else {
        return false;
    };

    // Both bodies have the same finite inertia, so neither gets privileged as
    // an immovable obstacle.
    let correction = 0.5 * (hit.depth + SEPARATION_EPSILON_M);
    a.state.x += hit.normal[0] * correction;
    a.state.y += hit.normal[1] * correction;
    b.state.x -= hit.normal[0] * correction;
    b.state.y -= hit.normal[1] * correction;

    let mut va = velocity(a.state);
    let mut vb = velocity(b.state);
    let relative_normal_speed = (va[0] - vb[0]) * hit.normal[0] + (va[1] - vb[1]) * hit.normal[1];
    if relative_normal_speed < 0.0 {
        // Equal inverse masses (1 + 1) split the collision impulse equally.
        let impulse = -(1.0 + VEHICLE_RESTITUTION) * relative_normal_speed / 2.0;
        va[0] += impulse * hit.normal[0];
        va[1] += impulse * hit.normal[1];
        vb[0] -= impulse * hit.normal[0];
        vb[1] -= impulse * hit.normal[1];
        a.state = with_velocity(a.state, va, a.footprint);
        b.state = with_velocity(b.state, vb, b.footprint);
    }
    true
}

fn velocity(state: State) -> [f64; 2] {
    [state.speed * state.yaw.cos(), state.speed * state.yaw.sin()]
}

fn with_velocity(mut state: State, velocity: [f64; 2], footprint: Footprint) -> State {
    let speed = velocity[0].hypot(velocity[1]);
    if speed > 1e-9 {
        // State stores a rear reference point and has no independent velocity
        // direction, so keep the physical center fixed while aligning the
        // body with its post-impact velocity.
        let center = footprint.center(state.pose());
        state.yaw = velocity[1].atan2(velocity[0]);
        state.x = center.x - 0.5 * footprint.length * state.yaw.cos();
        state.y = center.y - 0.5 * footprint.length * state.yaw.sin();
    }
    state.speed = speed;
    state
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{CAR_FOOTPRINT, footprints_overlap};

    fn body(state: State) -> DynamicBody {
        DynamicBody::new(state, CAR_FOOTPRINT)
    }

    fn vx(state: State) -> f64 {
        state.speed * state.yaw.cos()
    }

    #[test]
    fn vehicle_collision_moves_and_bounces_both_bodies() {
        let mut bodies = [
            body(State {
                x: 0.0,
                speed: 10.0,
                ..Default::default()
            }),
            body(State {
                x: 4.0,
                ..Default::default()
            }),
        ];

        collide_dynamic_bodies(&mut bodies);

        assert!(bodies[0].state.x < 0.0);
        assert!(bodies[1].state.x > 4.0);
        assert!(vx(bodies[0].state) < 10.0);
        assert!(vx(bodies[1].state) > 0.0);
        assert!(!footprints_overlap(
            bodies[0].state.pose(),
            bodies[0].footprint,
            bodies[1].state.pose(),
            bodies[1].footprint,
        ));
    }

    #[test]
    fn equal_vehicle_collision_conserves_linear_momentum() {
        let mut bodies = [
            body(State {
                x: 0.0,
                speed: 8.0,
                ..Default::default()
            }),
            body(State {
                x: 7.6,
                yaw: std::f64::consts::PI,
                speed: 2.0,
                ..Default::default()
            }),
        ];
        let momentum_before = vx(bodies[0].state) + vx(bodies[1].state);

        collide_dynamic_bodies(&mut bodies);

        let momentum_after = vx(bodies[0].state) + vx(bodies[1].state);
        assert!((momentum_after - momentum_before).abs() < 1e-9);
        assert!(
            vx(bodies[0].state) < vx(bodies[1].state),
            "states after collision: {:?}",
            bodies
        );
    }

    #[test]
    fn three_vehicle_contact_propagates_to_every_actor() {
        let mut bodies = [
            body(State {
                x: 0.0,
                speed: 12.0,
                ..Default::default()
            }),
            body(State {
                x: 4.0,
                ..Default::default()
            }),
            body(State {
                x: 8.0,
                ..Default::default()
            }),
        ];

        collide_dynamic_bodies(&mut bodies);

        assert!(vx(bodies[1].state) > 0.0);
        assert!(vx(bodies[2].state) > 0.0);
        assert!(bodies.windows(2).all(|pair| !footprints_overlap(
            pair[0].state.pose(),
            pair[0].footprint,
            pair[1].state.pose(),
            pair[1].footprint,
        )));
    }
}
