//! No at-fault collisions: 1 while the ego is collision-free at this tick.
//! Event-driven — aggregates by worst case (min).

use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};
use crate::metrics::TickCtx;
use crate::simulation::State;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let ego = &ctx.ego[i];
    if ctx.actors_at[i].iter().any(|a| collides(ego, a)) {
        0.0
    } else {
        1.0
    }
}

fn collides(ego: &State, actor: &State) -> bool {
    footprints_overlap(ego.pose(), EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT)
}
