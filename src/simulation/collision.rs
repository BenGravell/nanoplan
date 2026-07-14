use super::State;
use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, Footprint, overlap_mtv};

const ACTOR_RESTITUTION: f64 = 0.1;

pub(crate) fn collide_with_actors(
    state: State,
    actors: impl IntoIterator<Item = (State, Footprint)>,
) -> State {
    actors.into_iter().fold(state, |s, (actor, footprint)| {
        collide_with_actor(s, actor, footprint)
    })
}

pub(crate) fn collide_with_car_actors(
    state: State,
    actors: impl IntoIterator<Item = State>,
) -> State {
    collide_with_actors(state, actors.into_iter().map(|s| (s, CAR_FOOTPRINT)))
}

fn collide_with_actor(state: State, actor: State, actor_footprint: Footprint) -> State {
    let Some(hit) = overlap_mtv(state.pose(), EGO_FOOTPRINT, actor.pose(), actor_footprint) else {
        return state;
    };
    let corrected = State {
        x: state.x + hit.normal[0] * hit.depth,
        y: state.y + hit.normal[1] * hit.depth,
        ..state
    };
    let mut v = [
        corrected.speed * corrected.yaw.cos(),
        corrected.speed * corrected.yaw.sin(),
    ];
    let vn = v[0] * hit.normal[0] + v[1] * hit.normal[1];
    if vn < 0.0 {
        v[0] -= (1.0 + ACTOR_RESTITUTION) * vn * hit.normal[0];
        v[1] -= (1.0 + ACTOR_RESTITUTION) * vn * hit.normal[1];
    }
    let speed = v[0].hypot(v[1]);
    State { speed, ..corrected }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actor_collision_separates_rendered_footprints() {
        let actor = State::default();
        let ego = State {
            x: 4.0,
            speed: 5.0,
            yaw: std::f64::consts::PI,
            ..Default::default()
        };

        let hit = collide_with_car_actors(ego, [actor]);

        assert!(!crate::geometry::footprints_overlap(
            hit.pose(),
            EGO_FOOTPRINT,
            actor.pose(),
            CAR_FOOTPRINT
        ));
        assert!(hit.speed < ego.speed);
    }

    #[test]
    fn actor_collision_does_not_instantly_rotate_the_ego() {
        let actor = State::default();
        let ego = State {
            x: 4.0,
            y: 1.0,
            yaw: std::f64::consts::PI,
            speed: 20.0,
        };

        let hit = collide_with_car_actors(ego, [actor]);

        assert_eq!(hit.yaw, ego.yaw);
        assert!(!crate::geometry::footprints_overlap(
            hit.pose(),
            EGO_FOOTPRINT,
            actor.pose(),
            CAR_FOOTPRINT
        ));
    }
}
