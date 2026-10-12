//! Shared cubic sampling, strict rollout, terminal scoring, and cost-ordered validation.

mod cem;
mod frenet;
mod mppi;
mod ps;

pub(crate) use cem::Cem;
pub(crate) use mppi::Mppi;
pub(crate) use ps::PredictiveSampling;
pub(crate) type FrenetSamplingPlanner = SamplingPlanner<frenet::Frenet>;

use crate::common::kinematics::{clamp_control, commanded_accel_for_net, commanded_accel_to_stop};
use crate::common::polynomial::CubicPolynomial;
use crate::constraints::{Constraint, Kinodynamic, Sample};
use crate::metrics;
use crate::planning::feasibility::trajectory_is_feasible;
use crate::planning::frenet::{Motion, frenet_boundary, longitudinal_targets};
use crate::planning::policy::centerline_curvature;
use crate::planning::sampling::{self, Halton};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner, take_warm};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

const CUBIC_LAUNCH_SPEED_MPS: f64 = 2.0;
pub(crate) const NU: usize = 4;
/// Noise scales in metres, m/s, metres, m/s.
pub(crate) const SIGMA_SCALE: Knot = [4.0, 2.0, 2.0, 0.5];
/// [station relative to the post-launch boundary, station speed, lateral offset, lateral speed].
pub(crate) type Knot = [f64; NU];

#[derive(Clone, Copy)]
pub(crate) struct OptimizerConfig {
    num_rollouts: usize,
    num_nodes: usize,
    /// Zero means select directly from the generated seeds, without Gaussian refinement.
    iterations: usize,
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

/// All candidates share the same straight launch and remaining cubic horizon.
pub(crate) struct Start {
    ego: State,
    state: State,
    initial: Knot,
    launch: Vec<Control>,
    duration: f64,
}

impl Start {
    fn new(ego: State, ctx: &Context) -> Option<Self> {
        if ego.speed < 0.0 {
            return None;
        }
        let mut state = ego;
        let mut launch = Vec::new();
        let ticks = (PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize;
        while state.speed < CUBIC_LAUNCH_SPEED_MPS && launch.len() + 1 < ticks {
            let control = Control {
                acceleration: MAX_LON_ACCEL,
                curvature: 0.0,
            };
            state = world_step(state, control, ctx.road.dt);
            launch.push(control);
        }
        Some(Self {
            ego,
            state,
            initial: initial_boundary(ctx.path(), state, ctx.project_ego(ego).s)?,
            duration: PLANNING_HORIZON_S - launch.len() as f64 * ctx.road.dt,
            launch,
        })
    }
}

fn initial_boundary(path: &Path, state: State, station: f64) -> Option<Knot> {
    let (lon, lat) = frenet_boundary(path, state, station)?;
    Some([lon[0], lon[1], lat[0], lat[1]])
}

fn segments(initial: Knot, knots: &[Knot], duration: f64) -> Vec<Motion> {
    let duration = duration / knots.len() as f64;
    let mut start = initial;
    knots
        .iter()
        .map(|&knot| {
            let end = [initial[0] + knot[0], knot[1], knot[2], knot[3]];
            let motion = Motion {
                longitudinal: CubicPolynomial::from_boundary(start[0], start[1], end[0], end[1], duration),
                lateral: CubicPolynomial::from_boundary(start[2], start[3], end[2], end[3], duration),
            };
            start = end;
            motion
        })
        .collect()
}

fn motion_at(motions: &[Motion], path: &Path, t: f64, duration: f64) -> Option<(State, Control)> {
    let duration = duration / motions.len() as f64;
    let index = ((t / duration).floor() as usize).min(motions.len() - 1);
    motions[index].at(path, t - index as f64 * duration)
}

#[derive(Clone)]
pub(crate) struct Candidate {
    knots: Vec<Knot>,
    states: Vec<State>,
    controls: Vec<Control>,
    station: f64,
    reward: f64,
}

fn planning_ticks(ctx: &Context) -> usize {
    ((PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize).max(ctx.horizon)
}

/// Reject raw controls before integration, scoring, or collision checking.
fn rollout_controls(
    ego: State,
    ctx: &Context,
    mut control_at: impl FnMut(usize, State) -> Option<Control>,
) -> Option<Candidate> {
    let trajectory = ctx.time("kinodynamic", || {
        crate::simulation::rollout(ego, ctx.road.dt, planning_ticks(ctx), |tick, state| {
            ctx.work(1);
            control_at(tick, state)
        })
    })?;
    Some(Candidate {
        knots: Vec::new(),
        station: ctx.project_ego(ego).s,
        states: trajectory.states,
        controls: trajectory.controls,
        reward: 0.0,
    })
}

fn rollout(knots: &[Knot], start: &Start, ctx: &Context) -> Option<Candidate> {
    let motions = segments(start.initial, knots, start.duration);
    let mut candidate = rollout_controls(start.ego, ctx, |tick, actual| {
        if tick < start.launch.len() {
            return Some(start.launch[tick]);
        }
        let t = (((tick - start.launch.len()) as f64 + 0.5) * ctx.road.dt).min(start.duration);
        let (analytic, mut control) = motion_at(&motions, ctx.path(), t, start.duration)?;
        let analytic_control = Control {
            acceleration: commanded_accel_for_net(control.acceleration, analytic.speed),
            ..control
        };
        if Kinodynamic.is_violated(&Sample::default().with_control(analytic_control, analytic.speed)) {
            return None;
        }
        control.acceleration = commanded_accel_for_net(control.acceleration, actual.speed);
        Some(control)
    })?;
    candidate.knots = knots.to_vec();
    candidate.station = start.initial[0] + knots.last()?[0];
    Some(candidate)
}

fn score_candidates(mut candidates: Vec<Candidate>, ctx: &Context) -> Vec<Candidate> {
    for candidate in &mut candidates {
        candidate.reward = ctx.time("cost", || {
            metrics::evaluate(
                &candidate.states,
                ctx.road.dt,
                ctx.path(),
                Some([ctx.project_ego(candidate.states[0]).s, candidate.station]),
            )
        });
    }
    candidates.retain(|candidate| candidate.reward.is_finite());
    // Stable order preserves candidate generation order for ties.
    candidates.sort_by(|a, b| b.reward.total_cmp(&a.reward));
    if let Some(diag) = ctx.diagnostics {
        for candidate in &candidates {
            let points: Vec<_> = candidate.states.iter().map(|state| state.position()).collect();
            for &point in &points {
                diag.record_point(point);
            }
            diag.record_trajectory(points);
        }
    }
    candidates
}

/// Collision-check in reward order, stopping once the strategy has enough feasible elites.
fn select_candidates(candidates: Vec<Candidate>, count: usize, ctx: &Context) -> Vec<Candidate> {
    candidates
        .into_iter()
        .filter(|candidate| {
            ctx.time("collision", || {
                trajectory_is_feasible(&candidate.states, &candidate.controls, ctx)
            })
        })
        .take(count)
        .collect()
}

fn retain_best(best: &mut Option<Candidate>, candidate: &Candidate) {
    if best.as_ref().is_none_or(|old| candidate.reward > old.reward) {
        *best = Some(candidate.clone());
    }
}

fn road_control(state: State, ctx: &Context, acceleration: f64) -> Control {
    clamp_control(
        Control {
            acceleration: acceleration.max(commanded_accel_to_stop(state.speed, ctx.road.dt)),
            curvature: centerline_curvature(ctx.path(), &state),
        },
        state.speed,
    )
}

fn initial_knots(start: &Start, ctx: &Context, count: usize) -> Vec<Knot> {
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

fn endpoint_states(
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

fn project_knots(states: &[State], initial: Knot, ctx: &Context) -> Option<Vec<Knot>> {
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

pub(crate) struct SamplingPlanner<O: Optimizer> {
    opt: O,
    nominal: Option<Vec<State>>,
    expected_next: State,
    previous: Option<Candidate>,
}

impl<O: Optimizer> Default for SamplingPlanner<O> {
    fn default() -> Self {
        Self {
            opt: O::default(),
            nominal: None,
            expected_next: State::default(),
            previous: None,
        }
    }
}

impl<O: Optimizer> SamplingPlanner<O> {
    pub(crate) const NAME: &'static str = O::NAME;

    fn search(&mut self, start: &Start, ctx: &Context) -> Option<Candidate> {
        let cfg = self.opt.config();
        let mut best = None;
        let mut candidates = self.opt.seeds(start, ctx);
        if cfg.iterations > 0
            && let Some(states) = take_warm(&mut self.nominal, self.expected_next, start.ego)
            && states.len() == cfg.num_nodes
            && let Some(knots) = project_knots(&states, start.initial, ctx)
            && let Some(candidate) = rollout(&knots, start, ctx)
        {
            candidates.push(candidate);
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
                let sampled = self.opt.sample_knots(&nominal, 1 + it * num_rollouts, num_rollouts);
                let candidates = sampled.iter().filter_map(|knots| rollout(knots, start, ctx)).collect();
                let candidates = score_candidates(candidates, ctx);
                let elites = select_candidates(candidates, self.opt.num_elites(), ctx);
                let Some(winner) = elites.first() else {
                    continue;
                };
                retain_best(&mut best, winner);
                let sampled: Vec<_> = elites.iter().map(|candidate| candidate.knots.clone()).collect();
                let rewards: Vec<_> = elites.iter().map(|candidate| candidate.reward).collect();
                let updated = self.opt.update_nominal_knots(&sampled, &rewards);
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

impl<O: Optimizer> Planner for SamplingPlanner<O> {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let cfg = self.opt.config();
        let previous = take_warm(&mut self.previous, self.expected_next, ego);
        let start = ctx.time("route", || Start::new(ego, ctx));
        let mut best = start.as_ref().and_then(|start| self.search(start, ctx));

        if best.is_none() {
            let braking = rollout_controls(ego, ctx, |_, state| Some(road_control(state, ctx, MIN_LON_ACCEL))).unwrap();
            let candidates = score_candidates(vec![braking.clone()], ctx);
            best = select_candidates(candidates, 1, ctx).into_iter().next();
            if best.is_none()
                && let Some(previous) = previous
            {
                let index = previous
                    .states
                    .iter()
                    .take(previous.controls.len())
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        let error = |state: &State| {
                            state.position().distance(ego.position()) + (state.speed - ego.speed).abs() * ctx.road.dt
                        };
                        error(a).total_cmp(&error(b))
                    })
                    .map(|(index, _)| index)
                    .unwrap_or(0);
                let candidate = rollout_controls(ego, ctx, |tick, state| {
                    Some(
                        previous
                            .controls
                            .get(index + tick)
                            .copied()
                            .unwrap_or_else(|| road_control(state, ctx, MIN_LON_ACCEL)),
                    )
                });
                let candidates = score_candidates(candidate.into_iter().collect(), ctx);
                best = select_candidates(candidates, 1, ctx).into_iter().next();
            }
            if best.is_none() {
                self.nominal = None;
                return braking.controls.into_iter().take(ctx.horizon).collect();
            }
        }
        let winner = best.unwrap();
        self.nominal =
            start.map(|start| endpoint_states(&winner, ctx, cfg.num_nodes, start.launch.len(), start.duration, 1));
        self.expected_next = winner.states[1];
        let controls = winner.controls[..ctx.horizon].to_vec();
        self.previous = Some(winner);
        controls
    }
}

#[cfg(test)]
mod frenet_tests;
#[cfg(test)]
mod tests;
