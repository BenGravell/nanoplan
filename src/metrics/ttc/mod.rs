//! Time to collision within bound: 1 while the constant-velocity projections
//! of ego and every actor stay apart for at least `LEAST_MIN_TTC_S`.
//! Event-driven — aggregates by worst case (min).

use crate::metrics::{CAR_RADIUS_M, TickCtx, gap, project};

const TTC_HORIZON_S: f64 = 3.0;
const TTC_STEP_S: f64 = 0.1;
const LEAST_MIN_TTC_S: f64 = 0.95;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    let ego = &ctx.ego[i];
    let min_ttc = ctx.actors_at[i]
        .iter()
        .filter_map(|a| {
            let mut t = TTC_STEP_S;
            while t <= TTC_HORIZON_S {
                if gap(&project(ego, t), &project(a, t)) < 2.0 * CAR_RADIUS_M {
                    return Some(t);
                }
                t += TTC_STEP_S;
            }
            None
        })
        .fold(f64::INFINITY, f64::min);
    if min_ttc >= LEAST_MIN_TTC_S { 1.0 } else { 0.0 }
}
