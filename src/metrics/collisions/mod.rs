//! No at-fault collisions: 1 while the ego is collision-free at this tick.
//! Event-driven — aggregates by worst case (min).

use crate::metrics::{CAR_RADIUS_M, gap};
use crate::simulation::State;

pub fn score(ego: &State, actors: &[State]) -> f64 {
    if actors.iter().any(|a| gap(ego, a) < 2.0 * CAR_RADIUS_M) {
        0.0
    } else {
        1.0
    }
}
