//! Time to collision within bound: 1 while the recorded future ego and actor
//! trajectories stay on-road and apart for at least `LEAST_MIN_TTC_S`.
//! Event-driven — aggregates by worst case (min).

use crate::geometry::barrier::collides_with_road_barrier;
use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};
use crate::metrics::TickCtx;
use crate::simulation::State;

const TTC_HORIZON_S: f64 = 3.0;
const LEAST_MIN_TTC_S: f64 = 0.95;

pub(crate) fn score(ctx: &TickCtx, i: usize) -> f64 {
    let min_ttc = time_to_unsafe_state(ctx, i).unwrap_or(f64::INFINITY);
    if min_ttc >= LEAST_MIN_TTC_S { 1.0 } else { 0.0 }
}

fn time_to_unsafe_state(ctx: &TickCtx, i: usize) -> Option<f64> {
    ctx.ego[i..]
        .iter()
        .zip(&ctx.actors_at[i..])
        .enumerate()
        .take_while(|(offset, _)| *offset as f64 * ctx.dt <= TTC_HORIZON_S)
        .find_map(|(offset, (ego, actors))| {
            let unsafe_state = collides_with_road_barrier(*ego, ctx.road)
                || actors.iter().any(|actor| collides(ego, actor));
            unsafe_state.then_some(offset as f64 * ctx.dt)
        })
}

fn collides(ego: &State, actor: &State) -> bool {
    footprints_overlap(ego.pose(), EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metrics::evaluate;
    use crate::track::Road;

    fn road() -> Road {
        Road::new(vec![[-20.0, 0.0], [100.0, 0.0]], 10.0, 20.0, 0.1)
    }

    #[test]
    fn ttc_uses_the_actual_braking_ego_trajectory() {
        let mut ego = vec![
            State {
                speed: 10.0,
                ..Default::default()
            };
            20
        ];
        for state in &mut ego[1..] {
            state.x = 1.0;
            state.speed = 0.0;
        }
        let parked = vec![
            State {
                x: 10.0,
                ..Default::default()
            };
            ego.len()
        ];

        assert_eq!(evaluate(&ego, &[parked], &road()).per_tick[0][0], 1.0);
    }

    #[test]
    fn ttc_uses_the_actual_accelerating_actor_trajectory() {
        let ego: Vec<State> = (0..20)
            .map(|i| State {
                x: i as f64,
                speed: 10.0,
                ..Default::default()
            })
            .collect();
        let actor: Vec<State> = (0..20)
            .map(|i| State {
                x: 10.0 + 2.0 * i as f64,
                speed: if i == 0 { 0.0 } else { 20.0 },
                ..Default::default()
            })
            .collect();

        assert_eq!(evaluate(&ego, &[actor], &road()).per_tick[0][0], 1.0);
    }

    #[test]
    fn ttc_detects_a_future_collision_from_the_actual_trajectories() {
        let ego = vec![State::default(); 20];
        let mut actor = vec![
            State {
                x: 10.0,
                ..Default::default()
            };
            20
        ];
        actor[5].x = 0.0;

        assert_eq!(evaluate(&ego, &[actor], &road()).per_tick[0][0], 0.0);
    }
}
