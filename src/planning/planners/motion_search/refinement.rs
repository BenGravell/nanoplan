//! Whole-trajectory grid/Gaussian search, with strategy-specific seeds and elite updates.
use super::*;
use crate::planning::frenet::longitudinal_targets;
use crate::planning::sampling::{self, Halton};

#[derive(Clone, Copy)]
pub(crate) struct OptimizerConfig {
    pub(super) num_rollouts: usize,
    pub(super) num_nodes: usize,
    /// Zero means select directly from the generated seeds, without Gaussian refinement.
    pub(super) iterations: usize,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            num_rollouts: 32,
            num_nodes: 4,
            iterations: 4,
        }
    }
}

pub(crate) trait Optimizer: Default + Send {
    const NAME: &'static str;
    fn config(&self) -> OptimizerConfig {
        OptimizerConfig::default()
    }
    fn seeds(&self, start: &Start, ctx: &Context) -> Vec<Candidate> {
        mpc_seeds(start, ctx, self.config().num_nodes)
    }
    fn num_elites(&self) -> usize {
        1
    }
    fn sigma(&self, _node: usize) -> Knot {
        [1.0; NU]
    }
    fn sample_knots(&mut self, nominal: &[Knot], sample_base: usize, num_rollouts: usize) -> Vec<Vec<Knot>> {
        let mut out = vec![nominal.to_vec()];
        for z in sampling::qmc_normals::<Halton>(sample_base, num_rollouts - 1, nominal.len() * NU) {
            let mut knots = nominal.to_vec();
            for (n, knot) in knots.iter_mut().enumerate() {
                for c in 0..NU {
                    knot[c] += self.sigma(n)[c] * SIGMA_SCALE[c] * z[n * NU + c];
                }
            }
            out.push(knots);
        }
        out
    }
    /// Inputs are feasible, sorted by descending reward, and limited to `num_elites`.
    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], _rewards: &[f64]) -> Vec<Knot> {
        sampled[0].clone()
    }
}

pub(super) fn initial_knots(start: &Start, ctx: &Context, count: usize) -> Vec<Knot> {
    let initial = start.initial;
    let lateral = CubicPolynomial::from_boundary(initial[2], initial[3], 0.0, 0.0, start.duration);
    let targets = longitudinal_targets(
        start.state.speed,
        ctx.road.dt,
        ctx.compute_budget,
        start.duration,
        (ctx.path(), initial[0], &lateral),
    );
    let fit = |&(distance, speed): &(f64, f64)| {
        let longitudinal = CubicPolynomial::from_boundary(0.0, initial[1], distance, speed, start.duration);
        (1..=count)
            .map(|n| {
                let t = n as f64 * start.duration / count as f64;
                let [s, sv, _] = longitudinal.at(t);
                let [d, dv, _] = lateral.at(t);
                [s, sv, d, dv]
            })
            .collect::<Vec<_>>()
    };
    // Reachable endpoints alone do not guarantee a dynamically valid cubic.
    targets
        .iter()
        .rev()
        .filter(|&&(distance, _)| initial[0] + distance <= ctx.path().length())
        .map(fit)
        .find(|knots| rollout(knots, start, ctx).is_some())
        .unwrap_or_else(|| fit(&targets[0]))
}

pub(super) fn endpoint_states(
    candidate: &Candidate,
    ctx: &Context,
    count: usize,
    launch: usize,
    duration: f64,
    shift: usize,
) -> Vec<State> {
    (1..=count)
        .map(|n| {
            let tick = launch + (n as f64 * duration / (count as f64 * ctx.road.dt)).round() as usize + shift;
            candidate.states.get(tick).copied().unwrap_or_else(|| {
                let state = *candidate.states.last().unwrap();
                world_step(state, road_control(state, ctx, MIN_LON_ACCEL), ctx.road.dt)
            })
        })
        .collect()
}

pub(super) fn project_knots(states: &[State], initial: Knot, ctx: &Context) -> Option<Vec<Knot>> {
    states
        .iter()
        .map(|&state| {
            let mut knot = initial_boundary(ctx.path(), state, ctx.path().project(state.position()).s)?;
            knot[0] -= initial[0];
            Some(knot)
        })
        .collect()
}

fn mpc_seeds(start: &Start, ctx: &Context, count: usize) -> Vec<Candidate> {
    let mut knots = vec![initial_knots(start, ctx, count)];
    let mut candidates = Vec::new();
    for acceleration in [1.5, 0.0, -2.0, MIN_LON_ACCEL] {
        let Some(mut candidate) = rollout_controls(start.ego, ctx, |tick, state| {
            Some(
                start
                    .launch
                    .get(tick)
                    .copied()
                    .unwrap_or_else(|| road_control(state, ctx, acceleration)),
            )
        }) else {
            continue;
        };
        let states = endpoint_states(&candidate, ctx, count, start.launch.len(), start.duration, 0);
        if let Some(projected) = project_knots(&states, start.initial, ctx) {
            candidate.station = start.initial[0] + projected.last().unwrap()[0];
            candidate.knots = projected.clone();
            knots.push(projected);
            candidates.push(candidate);
        }
    }
    if !ctx.actors.is_empty() {
        let offset = 0.75 * (ctx.road.half_width - crate::common::geometry::EGO_FOOTPRINT.width / 2.0).max(0.0);
        let detours: Vec<_> = knots
            .iter()
            .flat_map(|knots| {
                [-offset, offset].map(|offset| knots.iter().map(|knot| [knot[0], knot[1], offset, 0.0]).collect())
            })
            .collect();
        knots.extend(detours);
    }
    candidates.extend(knots.iter().filter_map(|knots| rollout(knots, start, ctx)));
    candidates
}

impl<O: Optimizer> SearchStrategy for O {
    const NAME: &'static str = O::NAME;
    fn search(&mut self, ego: State, warm: Option<&Candidate>, ctx: &Context) -> Option<Candidate> {
        let start = Start::new(ego, ctx)?;
        let start = &start;
        let cfg = self.config();
        let mut best = None;
        let mut candidates = self.seeds(start, ctx);
        if cfg.iterations > 0
            && let Some(previous) = warm
        {
            let states = endpoint_states(previous, ctx, cfg.num_nodes, start.launch.len(), start.duration, 1);
            if let Some(knots) = project_knots(&states, start.initial, ctx)
                && let Some(candidate) = rollout(&knots, start, ctx)
            {
                candidates.push(candidate);
            }
        }
        let candidates = score_candidates(candidates, ctx);
        if cfg.iterations == 0 {
            return select_candidates(candidates, 1, ctx).into_iter().next();
        }
        let mut nominal = candidates
            .first()
            .map(|candidate| candidate.knots.clone())
            .unwrap_or_else(|| initial_knots(start, ctx, cfg.num_nodes));
        if let Some(winner) = select_candidates(candidates, 1, ctx).into_iter().next() {
            nominal = winner.knots.clone();
            best = Some(winner);
        }
        let total = ctx.compute_budget.scale(cfg.iterations * cfg.num_rollouts, 6);
        let iterations = cfg.iterations.min(total / 6);
        let num_rollouts = (total / iterations.max(1)).max(6);
        ctx.time("optimize", || {
            for it in 0..iterations {
                let sampled = self.sample_knots(&nominal, 1 + it * num_rollouts, num_rollouts);
                let candidates = sampled.iter().filter_map(|knots| rollout(knots, start, ctx)).collect();
                let candidates = score_candidates(candidates, ctx);
                let elites = select_candidates(candidates, self.num_elites(), ctx);
                let Some(winner) = elites.first() else {
                    continue;
                };
                retain_best(&mut best, winner);
                let sampled: Vec<_> = elites.iter().map(|candidate| candidate.knots.clone()).collect();
                let rewards: Vec<_> = elites.iter().map(|candidate| candidate.reward).collect();
                let updated = self.update_nominal_knots(&sampled, &rewards);
                nominal = winner.knots.clone();
                // A mean of safe endpoints may collide or violate dynamics.
                if updated != nominal {
                    let candidates = rollout(&updated, start, ctx).into_iter().collect();
                    let candidates = score_candidates(candidates, ctx);
                    if let Some(candidate) = select_candidates(candidates, 1, ctx).first() {
                        retain_best(&mut best, candidate);
                        if candidate.reward > winner.reward {
                            nominal = updated;
                        }
                    }
                }
            }
        });
        best
    }
}
