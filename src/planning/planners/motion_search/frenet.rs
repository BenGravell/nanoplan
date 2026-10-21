//! Broad endpoint grid, fitted as one cubic per coordinate with no local refinement.

use super::{Candidate, CubicPolynomial, Optimizer, OptimizerConfig, Start, rollout_pair};
use crate::planning::Context;
use crate::planning::frenet::{lateral_targets, longitudinal_targets};

#[derive(Default)]
pub(crate) struct Frenet;

impl Optimizer for Frenet {
    const NAME: &'static str = "Frenet sampling";

    fn config(&self) -> OptimizerConfig {
        OptimizerConfig {
            num_nodes: 1,
            iterations: 0,
            ..Default::default()
        }
    }

    fn seeds(&self, start: &Start, ctx: &Context) -> Vec<Candidate> {
        let mut candidates = Vec::new();
        for (offset, lateral_speed) in lateral_targets(start.initial[2], ctx.road.half_width, ctx.compute_budget) {
            let lateral = CubicPolynomial::from_boundary(
                start.initial[2],
                start.initial[3],
                offset,
                lateral_speed,
                start.duration,
            );
            for (distance, speed) in longitudinal_targets(
                start.state.speed,
                ctx.road.dt,
                ctx.compute_budget,
                start.duration,
                (ctx.path(), start.initial[0], &lateral),
            ) {
                if start.initial[0] + distance <= ctx.path().length() {
                    candidates.extend(rollout_pair(&[[distance, speed, offset, lateral_speed]], start, ctx));
                }
            }
        }
        candidates
    }
}
