//! Predictive sampling, CEM, and MPPI over piecewise cubic Frenet motion.
//! Knots specify position and velocity at each segment endpoint; the ego
//! fixes the initial boundary. All optimizers share the same cubic rollout.

mod cem;
mod mppi;
mod ps;

pub(crate) use cem::Cem;
pub(crate) use mppi::Mppi;
pub(crate) use ps::PredictiveSampling;

use crate::common::kinematics::commanded_accel_for_net;
use crate::common::polynomial::CubicPolynomial;
use crate::constraints::Constraints;
use crate::planning::controls::{repeat_last_controls, stop_controls};
use crate::planning::frenet::{Motion, frenet_boundary, longitudinal_targets};
use crate::planning::planner_math::state_sample;
use crate::planning::sampling::{self, Halton};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner, take_warm};
use crate::simulation::{Control, State, world_step};
use crate::track::Path;

pub(crate) const NU: usize = 4;
/// Noise scales in metres, m/s, metres, m/s, respectively.
pub(crate) const SIGMA_SCALE: Knot = [4.0, 2.0, 2.0, 0.5];
/// Cubic endpoint: [station relative to ego, station speed, lateral offset, lateral speed].
pub(crate) type Knot = [f64; NU];

#[derive(Debug, Clone, Copy)]
pub(crate) struct OptimizerConfig {
    pub(crate) num_rollouts: usize,
    /// Number of cubic segments, each ending at a sampled knot.
    pub(crate) num_nodes: usize,
    pub(crate) use_noise_ramp: bool,
    pub(crate) noise_ramp: f64,
    pub(crate) iterations: usize,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            num_rollouts: 32,
            num_nodes: 4,
            use_noise_ramp: false,
            noise_ramp: 2.5,
            iterations: 4,
        }
    }
}

/// Judo-style sampling and reward aggregation, independent of the motion model.
pub(crate) trait Optimizer: Default + Send {
    const NAME: &'static str;
    fn config(&self) -> OptimizerConfig;
    /// Include the unperturbed nominal as the first candidate.
    fn sample_knots(&mut self, nominal: &[Knot], sample_base: usize, num_rollouts: usize) -> Vec<Vec<Knot>>;
    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot>;
}

pub(crate) fn noised_knots(
    nominal: &[Knot],
    num_rollouts: usize,
    sample_base: usize,
    sigma: impl Fn(usize) -> Knot,
) -> Vec<Vec<Knot>> {
    let z = sampling::qmc_normals::<Halton>(sample_base, num_rollouts - 1, nominal.len() * NU);
    let mut out = Vec::with_capacity(num_rollouts);
    out.push(nominal.to_vec());
    for zk in z {
        let mut knots = nominal.to_vec();
        for (n, knot) in knots.iter_mut().enumerate() {
            let s = sigma(n);
            for c in 0..NU {
                knot[c] += s[c] * SIGMA_SCALE[c] * zk[n * NU + c];
            }
        }
        out.push(knots);
    }
    out
}

pub(crate) fn ramp(cfg: &OptimizerConfig, n: usize) -> f64 {
    if cfg.use_noise_ramp {
        cfg.noise_ramp * (n + 1) as f64 / cfg.num_nodes as f64
    } else {
        1.0
    }
}

/// Fit adjacent endpoints with shared position and velocity (C1 continuity).
/// The initial boundary has absolute station; sampled knots have relative station.
fn segments(initial: Knot, knots: &[Knot]) -> Vec<Motion> {
    let duration = PLANNING_HORIZON_S / knots.len() as f64;
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

fn motion_at(motions: &[Motion], path: &Path, t: f64) -> Option<(State, Control)> {
    let duration = PLANNING_HORIZON_S / motions.len() as f64;
    let index = ((t / duration).floor() as usize).min(motions.len() - 1);
    motions[index].at(path, t - index as f64 * duration)
}

fn initial_boundary(path: &Path, ego: State, station: f64) -> Option<Knot> {
    let (lon, lat) = frenet_boundary(path, ego, station)?;
    Some([lon[0], lon[1], lat[0], lat[1]])
}

/// Seed the endpoints from one road-following cubic with a reachable acceleration.
fn initial_knots(initial: Knot, ego: State, path: &Path, ctx: &Context, count: usize) -> Vec<Knot> {
    let lateral = CubicPolynomial::from_boundary(initial[2], initial[3], 0.0, 0.0, PLANNING_HORIZON_S);
    let targets = longitudinal_targets(
        ego.speed,
        ctx.road.dt,
        ctx.compute_budget,
        PLANNING_HORIZON_S,
        (path, initial[0], &lateral),
    );
    let &(distance, speed) = targets
        .iter()
        .rev()
        .find(|&&(distance, _)| initial[0] + distance <= path.length())
        .unwrap_or(&targets[0]);
    let longitudinal = CubicPolynomial::from_boundary(0.0, initial[1], distance, speed, PLANNING_HORIZON_S);
    (1..=count)
        .map(|n| {
            let t = n as f64 * PLANNING_HORIZON_S / count as f64;
            let [s, sv, _] = longitudinal.at(t);
            let [d, dv, _] = lateral.at(t);
            [s, sv, d, dv]
        })
        .collect()
}

struct Rollout {
    states: Vec<State>,
    controls: Vec<Control>,
    reward: f64,
}

/// Integrate cubic-derived commands through the same plant used for execution.
/// Physical violations retain the shared soft costs; invalid Frenet charts are rejected.
fn rollout(knots: &[Knot], initial: Knot, ego: State, path: &Path, ctx: &Context) -> Option<Rollout> {
    let motions = segments(initial, knots);
    let duration = PLANNING_HORIZON_S / knots.len() as f64;
    for motion in &motions {
        motion.at(path, 0.0)?;
        motion.at(path, duration)?;
    }
    let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, ego.speed, initial[0]);
    let ticks = (PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize;
    let mut result = Rollout {
        states: vec![ego],
        controls: Vec::with_capacity(ticks),
        reward: 0.0,
    };
    let mut state = ego;
    let mut station = initial[0];
    for tick in 0..ticks {
        ctx.work(1);
        let t = ((tick as f64 + 0.5) * ctx.road.dt).min(PLANNING_HORIZON_S);
        let (_, mut control) = motion_at(&motions, path, t)?;
        control.acceleration = commanded_accel_for_net(control.acceleration, state.speed);
        let speed = state.speed;
        state = world_step(state, control, ctx.road.dt);
        let (s, sample) = state_sample(
            path,
            &state,
            (tick + 1) as f64 * ctx.road.dt,
            Some(station + speed * ctx.road.dt),
        );
        station = s;
        result.reward -= ctx.time("cost", || {
            constraints.soft_point_cost(&sample.with_control(control, speed))
        });
        result.states.push(state);
        result.controls.push(control);
    }
    result.reward.is_finite().then_some(result)
}

pub(crate) struct SamplingPlanner<O: Optimizer> {
    opt: O,
    /// Cubic endpoints shifted by one tick, stored in world coordinates so a
    /// moving road window cannot change their meaning on the next plan call.
    nominal: Option<Vec<State>>,
    expected_next: State,
}

impl<O: Optimizer> Default for SamplingPlanner<O> {
    fn default() -> Self {
        Self {
            opt: O::default(),
            nominal: None,
            expected_next: State::default(),
        }
    }
}

impl<O: Optimizer> SamplingPlanner<O> {
    pub(crate) const NAME: &'static str = O::NAME;
}

impl<O: Optimizer> Planner for SamplingPlanner<O> {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let cfg = self.opt.config();
        let total_rollouts = ctx.compute_budget.scale(cfg.iterations * cfg.num_rollouts, 6);
        let iterations = cfg.iterations.min(total_rollouts / 6).max(1);
        let num_rollouts = (total_rollouts / iterations).max(6);
        let path = ctx.time("route", || ctx.path());
        let Some(initial) = initial_boundary(path, ego, ctx.project_ego(ego).s) else {
            self.nominal = None;
            return stop_controls(ego, ctx, ctx.horizon);
        };
        let mut nominal = ctx.time("warm_start", || {
            take_warm(&mut self.nominal, self.expected_next, ego)
                .filter(|states| states.len() == cfg.num_nodes)
                .and_then(|states| {
                    states
                        .into_iter()
                        .map(|state| {
                            let mut knot = initial_boundary(path, state, path.project(state.position()).s)?;
                            knot[0] -= initial[0];
                            Some(knot)
                        })
                        .collect::<Option<Vec<_>>>()
                })
                .unwrap_or_else(|| initial_knots(initial, ego, path, ctx, cfg.num_nodes))
        });

        ctx.time("optimize", || {
            for it in 0..iterations {
                let sampled = self.opt.sample_knots(&nominal, 1 + it * num_rollouts, num_rollouts);
                let mut valid = Vec::new();
                let mut rewards = Vec::new();
                for knots in sampled {
                    let Some(candidate) = rollout(&knots, initial, ego, path, ctx) else {
                        continue;
                    };
                    if it == iterations - 1
                        && let Some(diag) = ctx.diagnostics
                    {
                        let points: Vec<_> = candidate.states.iter().map(|state| state.position()).collect();
                        for &point in &points {
                            diag.record_point(point);
                        }
                        diag.record_trajectory(points);
                    }
                    valid.push(knots);
                    rewards.push(candidate.reward);
                }
                // Never feed invalid motions/infinities into CEM or MPPI statistics.
                let Some(best) = (0..rewards.len()).max_by(|&a, &b| rewards[a].total_cmp(&rewards[b])) else {
                    break;
                };
                let updated = self.opt.update_nominal_knots(&valid, &rewards);
                nominal = if updated == valid[best]
                    || rollout(&updated, initial, ego, path, ctx).is_some_and(|r| r.reward >= rewards[best])
                {
                    updated
                } else {
                    valid[best].clone()
                };
            }
        });

        let Some(winner) = ctx.time("extract", || rollout(&nominal, initial, ego, path, ctx)) else {
            self.nominal = None;
            return stop_controls(ego, ctx, ctx.horizon);
        };
        let motions = segments(initial, &nominal);
        self.nominal = (1..=cfg.num_nodes)
            .map(|n| {
                let t = n as f64 * PLANNING_HORIZON_S / cfg.num_nodes as f64 + ctx.road.dt;
                motion_at(&motions, path, t).map(|(state, _)| state)
            })
            .collect();
        self.expected_next = winner.states[1];
        repeat_last_controls(&winner.controls, ctx.horizon)
    }
}

#[cfg(test)]
pub(crate) fn run_planner<O: Optimizer>(ego: State, actors: &[State], ticks: usize) -> Vec<State> {
    crate::planning::test_run(&mut SamplingPlanner::<O>::default(), ego, actors, ticks)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cubic_segments_match_endpoints_and_share_velocity() {
        let initial = [20.0, 8.0, 2.0, 0.3];
        let knots = [[40.0, 9.0, -1.0, -0.2], [90.0, 11.0, 1.0, 0.5]];
        let motions = segments(initial, &knots);
        let duration = PLANNING_HORIZON_S / knots.len() as f64;
        let mut start = initial;
        for (motion, end) in motions.iter().zip(knots) {
            let lon = motion.longitudinal.at(0.0);
            let lat = motion.lateral.at(0.0);
            assert_eq!([lon[0], lon[1], lat[0], lat[1]], start);
            let lon = motion.longitudinal.at(duration);
            let lat = motion.lateral.at(duration);
            start = [initial[0] + end[0], end[1], end[2], end[3]];
            for (actual, expected) in [lon[0], lon[1], lat[0], lat[1]].into_iter().zip(start) {
                assert!((actual - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn cubic_controls_follow_curved_frenet_reference() {
        let radius = 80.0;
        let points: Vec<_> = (0..2000)
            .map(|i| {
                let angle = i as f64 * 0.001;
                crate::simulation::Position::new(radius * angle.sin(), radius * (1.0 - angle.cos()))
            })
            .collect();
        let path = Path::new(&points);
        let knots = [[40.0, 8.0, 2.0, 0.0], [80.0, 8.0, 2.0, 0.0]];
        let motions = segments([10.0, 8.0, 2.0, 0.0], &knots);
        for t in [0.0, 1.25, 5.0, 7.5, 10.0] {
            let (state, control) = motion_at(&motions, &path, t).unwrap();
            assert!((state.position().x.hypot(state.position().y - radius) - 78.0).abs() < 1e-4);
            assert!((state.speed - 7.8).abs() < 1e-8);
            assert!((control.curvature - 1.0 / 78.0).abs() < 1e-8);
        }
    }

    #[test]
    fn invalid_frenet_segments_are_rejected() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let initial = [20.0, 8.0, 0.0, 0.0];
        for knot in [
            [40.0, -8.0, 0.0, 0.0],
            [500.0, 8.0, 0.0, 0.0],
            [f64::NAN, 8.0, 0.0, 0.0],
        ] {
            assert!(rollout(&[knot], initial, ego, ctx.path(), &ctx).is_none());
        }
    }

    #[test]
    fn output_length_does_not_change_optimization_horizon() {
        let mut road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        for dt in [0.1, 0.2, 0.3] {
            road.dt = dt;
            let mut ctx = crate::planning::test_ctx(&road, &[]);
            let short = SamplingPlanner::<PredictiveSampling>::default().plan(State::default(), &ctx);
            assert!(short[0].acceleration > 0.0);
            ctx.horizon = (PLANNING_HORIZON_S / dt).ceil() as usize;
            let long = SamplingPlanner::<PredictiveSampling>::default().plan(State::default(), &ctx);
            assert_eq!(short, long[..short.len()]);
            assert_eq!(long.len(), ctx.horizon);
            ctx.horizon = 0;
            assert!(
                SamplingPlanner::<PredictiveSampling>::default()
                    .plan(State::default(), &ctx)
                    .is_empty()
            );
        }
    }

    #[test]
    fn warm_start_survives_a_new_road_origin_and_rejects_divergence() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let mut planner = SamplingPlanner::<PredictiveSampling>::default();
        let controls = planner.plan(ego, &ctx);
        let next = world_step(ego, controls[0], road.dt);
        assert_eq!(next, planner.expected_next);
        let shifted_targets = planner.nominal.clone().unwrap();
        let mut other = SamplingPlanner::<PredictiveSampling> {
            nominal: Some(shifted_targets),
            expected_next: next,
            ..Default::default()
        };
        let shifted_road = crate::planning::test_road(&[[-10.0, 0.0], [2000.0, 0.0]]);
        let a = planner.plan(next, &ctx);
        let b = other.plan(next, &crate::planning::test_ctx(&shifted_road, &[]));
        for (a, b) in a.iter().zip(b) {
            assert!((a.acceleration - b.acceleration).abs() < 1e-8);
            assert!((a.curvature - b.curvature).abs() < 1e-8);
        }

        // Replanning from an unrelated position must discard the old endpoints.
        let teleported = State::from((crate::simulation::Position::new(100.0, 0.0), 0.0, 8.0));
        assert_eq!(
            planner.plan(teleported, &ctx),
            SamplingPlanner::<PredictiveSampling>::default().plan(teleported, &ctx)
        );
    }

    #[test]
    fn seed_fits_a_finite_road_window_at_cruising_speed() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [250.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let ego = State {
            speed: 20.0,
            ..Default::default()
        };
        let initial = initial_boundary(ctx.path(), ego, 20.0).unwrap();
        let knots = initial_knots(initial, ego, ctx.path(), &ctx, 4);
        assert!(rollout(&knots, initial, ego, ctx.path(), &ctx).is_some());
        let controls = SamplingPlanner::<PredictiveSampling>::default().plan(ego, &ctx);
        assert_eq!(controls.len(), ctx.horizon);
        assert!(controls[0].acceleration > crate::vehicle::MIN_LON_ACCEL);
    }

    // --- closed-loop tests, one battery per optimizer -----------------
    //
    // Same style as every other planner's tests (see the "Test harness"
    // section of the README): a single `plan()` call proves little, so each
    // optimizer is driven closed-loop and its realized trajectory checked.

    /// From an initial lateral offset, stay on-road and accelerate without
    /// exceeding the vehicle's physical terminal envelope.
    fn stays_on_road_and_accelerates<O: Optimizer>() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        );
        let trace = run_planner::<O>(ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 5.5, "{} offset {}", O::NAME, end.position().y);
        assert!(end.speed > ego.speed, "{} speed {}", O::NAME, end.speed);
        assert!(
            end.speed <= *crate::simulation::MAX_TERMINAL_SPEED_MPS + 1e-9,
            "{} speed {}",
            O::NAME,
            end.speed
        );
    }

    /// Swerve around a stationary obstacle straddling the centerline, keep
    /// real clearance, and still make it past — the point of the whole
    /// exercise.
    fn avoids_stopped_obstacle<O: Optimizer>() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let trace = run_planner::<O>(ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.position().x - 40.0).hypot(s.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "{} min gap {min_gap}", O::NAME);
        assert!(
            trace.last().unwrap().position().x > 50.0,
            "{} did not pass, x {}",
            O::NAME,
            trace.last().unwrap().position().x
        );
    }

    /// The knot noise is QMC (a pure function of the sample index), the
    /// nominal is a deterministic Frenet cubic, and there is no `Rng`: two
    /// fresh planners replanning from the identical state must produce the
    /// identical plan, like RRT* and unlike PI²-DDP.
    fn is_a_pure_function_of_state<O: Optimizer>() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let actors = [obstacle];
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &actors);
        let a = SamplingPlanner::<O>::default().plan(ego, &ctx);
        let b = SamplingPlanner::<O>::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }

    #[test]
    fn ps_stays_on_road_and_accelerates() {
        stays_on_road_and_accelerates::<PredictiveSampling>();
    }
    #[test]
    fn ps_avoids_stopped_obstacle() {
        avoids_stopped_obstacle::<PredictiveSampling>();
    }
    #[test]
    fn ps_is_a_pure_function_of_state() {
        is_a_pure_function_of_state::<PredictiveSampling>();
    }

    #[test]
    fn cem_stays_on_road_and_accelerates() {
        stays_on_road_and_accelerates::<Cem>();
    }
    #[test]
    fn cem_avoids_stopped_obstacle() {
        avoids_stopped_obstacle::<Cem>();
    }
    #[test]
    fn cem_is_a_pure_function_of_state() {
        is_a_pure_function_of_state::<Cem>();
    }

    #[test]
    fn mppi_stays_on_road_and_accelerates() {
        stays_on_road_and_accelerates::<Mppi>();
    }
    #[test]
    fn mppi_avoids_stopped_obstacle() {
        avoids_stopped_obstacle::<Mppi>();
    }
    #[test]
    fn mppi_is_a_pure_function_of_state() {
        is_a_pure_function_of_state::<Mppi>();
    }

    #[test]
    fn records_diagnostics_when_requested() {
        use crate::planning::Diagnostics;
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let diag = Diagnostics::default();
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let mut ctx = crate::planning::test_ctx(&road, &[]);
        ctx.diagnostics = Some(&diag);
        SamplingPlanner::<Mppi>::default().plan(ego, &ctx);
        let data = diag.take();
        // Valid final-iteration samples span the full optimization horizon.
        let cfg = OptimizerConfig::default();
        assert!(!data.trajectories.is_empty());
        assert!(data.trajectories.len() <= cfg.num_rollouts);
        let horizon = (PLANNING_HORIZON_S / road.dt).ceil() as usize;
        assert!(data.trajectories.iter().all(|t| t.len() == horizon + 1));
        assert_eq!(data.points.len(), data.trajectories.len() * (horizon + 1));
    }
}
