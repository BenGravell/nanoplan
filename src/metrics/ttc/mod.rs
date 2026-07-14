//! Time to collision within bound: 1 while the constant-velocity projections
//! of ego stay on-road and apart from every actor for at least
//! `LEAST_MIN_TTC_S`.
//! Event-driven — aggregates by worst case (min).

use crate::barrier::collides_with_road_barrier;
use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap, project};
use crate::metrics::TickCtx;
use crate::simulation::State;
use crate::track::Road;

const TTC_HORIZON_S: f64 = 3.0;
const TTC_STEP_S: f64 = 0.1;
const LEAST_MIN_TTC_S: f64 = 0.95;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let ego = &ctx.ego[i];
    let min_ttc = ctx.actors_at[i]
        .iter()
        .filter_map(|a| time_to_collision(ego, a))
        .chain(time_to_road_exit(ego, ctx.road))
        .fold(f64::INFINITY, f64::min);
    if min_ttc >= LEAST_MIN_TTC_S { 1.0 } else { 0.0 }
}

fn time_to_collision(ego: &State, actor: &State) -> Option<f64> {
    let mut t = 0.0;
    while t <= TTC_HORIZON_S {
        if collides(&project(ego, t), &project(actor, t)) {
            return Some(t);
        }
        t += TTC_STEP_S;
    }
    None
}

fn collides(ego: &State, actor: &State) -> bool {
    footprints_overlap(ego.pose(), EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT)
}

fn time_to_road_exit(ego: &State, road: &Road) -> Option<f64> {
    let mut t = 0.0;
    while t <= TTC_HORIZON_S {
        if collides_with_road_barrier(project(ego, t), road) {
            return Some(t);
        }
        t += TTC_STEP_S;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ttc_is_zero_if_and_only_if_footprints_currently_overlap() {
        let ego = State {
            speed: 10.0,
            ..Default::default()
        };
        let actors = [
            State::default(),
            State {
                x: EGO_FOOTPRINT.length - 0.01,
                speed: 100.0,
                ..Default::default()
            },
            State {
                x: 8.0,
                speed: 0.0,
                ..Default::default()
            },
            State {
                x: 20.0,
                ..Default::default()
            },
        ];

        for actor in actors {
            assert_eq!(
                time_to_collision(&ego, &actor) == Some(0.0),
                collides(&ego, &actor)
            );
        }
    }
}
