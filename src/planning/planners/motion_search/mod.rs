//! Motion search using the Strategy pattern: one execution lifecycle, interchangeable searches.

mod cem;
mod frenet;
mod graph;
mod lattice;
mod mppi;
mod nearest_zap;
mod ps;
mod refinement;

pub(crate) use cem::Cem;
pub(crate) use graph::Connections;
pub(crate) use mppi::Mppi;
pub(crate) use ps::PredictiveSampling;
use refinement::{Optimizer, OptimizerConfig};

pub(crate) type SamplingPlanner<O> = MotionPlanner<O>;
pub(crate) type FrenetSamplingPlanner = MotionPlanner<frenet::Frenet>;
pub(crate) type GraphPlanner = MotionPlanner<Connections>;

use crate::common::kinematics::{clamp_control, commanded_accel_for_net, commanded_accel_to_stop};
use crate::common::polynomial::CubicPolynomial;
use crate::constraints::{Constraint, Kinodynamic, Sample};
use crate::metrics;
use crate::planning::feasibility::trajectory_is_feasible;
use crate::planning::frenet::{Motion, frenet_boundary};
use crate::planning::policy::centerline_curvature;
use crate::planning::{Context, PLANNING_HORIZON_S, Planner, take_warm};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

const CUBIC_LAUNCH_SPEED_MPS: f64 = 2.0;
pub(crate) const NU: usize = 4;
pub(crate) const SIGMA_SCALE: Knot = [4.0, 2.0, 2.0, 0.5];
/// [relative station, station speed, lateral offset, lateral speed].
pub(crate) type Knot = [f64; NU];

/// Search returns a validated full-horizon trajectory, or no solution.
/// Only graph construction, sampling, and candidate ranking belong to strategies.
pub(crate) trait SearchStrategy: Send {
    const NAME: &'static str;
    fn search(&mut self, ego: State, warm: Option<&Candidate>, ctx: &Context) -> Option<Candidate>;
}

/// Whole-trajectory samples share a straight launch and remaining cubic horizon.
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

fn fit_segment(start: Knot, end: Knot, duration: f64) -> Motion {
    Motion {
        longitudinal: CubicPolynomial::from_boundary(start[0], start[1], end[0], end[1], duration),
        lateral: CubicPolynomial::from_boundary(start[2], start[3], end[2], end[3], duration),
    }
}

fn segments(initial: Knot, knots: &[Knot], duration: f64) -> Vec<Motion> {
    let duration = duration / knots.len() as f64;
    let mut start = initial;
    knots
        .iter()
        .map(|&knot| {
            let end = [initial[0] + knot[0], knot[1], knot[2], knot[3]];
            let motion = fit_segment(start, end, duration);
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
        let motion = motion_at(&motions, ctx.path(), t, start.duration)?;
        cubic_control(motion, actual, ctx.road.dt, ControlLimits::Reject)
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

/// Graph steering projects to vehicle limits; sampled cubics reject violations.
#[derive(Clone, Copy)]
enum ControlLimits {
    Reject,
    Clamp,
}

fn cubic_control(
    (analytic, mut control): (State, Control),
    actual: State,
    dt: f64,
    limits: ControlLimits,
) -> Option<Control> {
    if matches!(limits, ControlLimits::Reject) {
        let analytic_control = Control {
            acceleration: commanded_accel_for_net(control.acceleration, analytic.speed),
            ..control
        };
        if Kinodynamic.is_violated(&Sample::default().with_control(analytic_control, analytic.speed)) {
            return None;
        }
    }
    control.acceleration = commanded_accel_for_net(control.acceleration, actual.speed);
    if matches!(limits, ControlLimits::Clamp) {
        control.acceleration = control.acceleration.max(commanded_accel_to_stop(actual.speed, dt));
        control = clamp_control(control, actual.speed);
    }
    Some(control)
}

pub(crate) struct MotionPlanner<S: SearchStrategy> {
    strategy: S,
    previous: Option<Candidate>,
    expected_next: State,
}

impl<S: SearchStrategy + Default> Default for MotionPlanner<S> {
    fn default() -> Self {
        Self::new(S::default())
    }
}

impl<S: SearchStrategy> MotionPlanner<S> {
    pub(crate) const NAME: &'static str = S::NAME;
    pub(crate) fn new(strategy: S) -> Self {
        Self {
            strategy,
            previous: None,
            expected_next: State::default(),
        }
    }
}

impl<S: SearchStrategy> Planner for MotionPlanner<S> {
    fn accept_optimized(&mut self, trajectory: crate::common::types::Trajectory, ctx: &Context) {
        self.expected_next = trajectory.states[1];
        self.previous = Some(Candidate {
            station: ctx.path().project(trajectory.states.last().unwrap().position()).s,
            states: trajectory.states,
            controls: trajectory.controls,
            knots: Vec::new(),
            reward: 0.0,
        });
    }

    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let previous = take_warm(&mut self.previous, self.expected_next, ego);
        let mut best = self.strategy.search(ego, previous.as_ref(), ctx);
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
                return braking.controls.into_iter().take(ctx.horizon).collect();
            }
        }
        let winner = best.unwrap();
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
