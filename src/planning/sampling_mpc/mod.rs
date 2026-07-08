//! Sampling-based predictive-control optimizers ported from **judo**
//! (<https://github.com/rai-opensource/judo>): predictive sampling, the
//! cross-entropy method (CEM), and MPPI. All three share judo's structure —
//! an [`Optimizer`] samples control-knot perturbations around a nominal
//! trajectory, the caller rolls each one out and scores it, and the
//! optimizer folds the scores back into a new nominal — and differ *only*
//! in how they sample ([`Optimizer::sample_control_knots`]) and how they
//! aggregate ([`Optimizer::update_nominal_knots`]), exactly as in judo's
//! `base.py` / `ps.py` / `cem.py` / `mppi.py`.
//!
//! ## Fitting judo into the nanoplan framework
//!
//! judo's optimizers are pure array math over `(num_rollouts, num_nodes,
//! nu)` knot tensors; a separate simulator turns knots into rollouts and
//! rewards. [`SamplingPlanner`] is that surrounding machinery, adapted to
//! nanoplan's [`Planner`] trait:
//!
//! - **Knots are deviations from a road-model base policy.** The single
//!   most important adaptation. judo's knots *are* the raw controls, applied
//!   open-loop over the horizon. That works for judo's short-horizon,
//!   feedback-stabilized tasks but not for tracking a lane over 10 s: a
//!   car's lateral dynamics integrate curvature twice, so raw open-loop
//!   knots diverge metres off-road over the horizon and every candidate
//!   scores as garbage. Instead each interpolated knot is a *deviation*
//!   added to a critically-damped PD lane-keeping + speed-hold **base
//!   policy** (`base_policy`) evaluated on the current
//!   rollout state — genuine feedback, so every rollout stays on the road
//!   and the QMC exploration prices real maneuvers (an obstacle swerve)
//!   rather than open-loop drift. This mirrors PI²-DDP, which likewise rolls
//!   out with feedback gains rather than raw nominal controls, and is the
//!   "hybrid road model" the sampling explores around. The nominal starts at
//!   *zero* deviation — the bare base policy — the judo-typical zero nominal.
//! - **Control knots → controls.** The `num_nodes` deviation knots
//!   (`[accel, curvature]`) are spread over the planning horizon and
//!   linearly interpolated to a per-tick sequence (`control_at`) — judo's
//!   spline interpolation, at its simplest order.
//! - **The shared forward model.** Every rollout advances through
//!   [`crate::simulation::drive`] — the exact model the plant steps the ego
//!   with — seeded from [`Context::ego_control`](crate::planning::Context), so
//!   each candidate is priced on the *actually applied* control after the
//!   actuation limits (including jerk and steering rate) clip it. The base
//!   policy plus knot deviation is the *commanded* control; `drive` turns it
//!   into what the car does, so the sampler plans steering and throttle ramps
//!   that fit the limits rather than steps the plant would clip.
//! - **The shared cost function.** Each rolled-out state is priced by
//!   [`crate::planning::cost::point_cost`], the same scalar the Frenet
//!   lattice, PI²-DDP, and RRT* agree on, with a hard violation made finite
//!   ([`cost::HARD_VIOLATION_PENALTY`]) since MPPI's and CEM's reward
//!   aggregation can't absorb an infinity — the same reason PI²-DDP makes
//!   that swap. Three planner-specific terms sit on top, each mirroring one
//!   PI²-DDP keeps: an undiscounted speed-tracking term (`SPEED_WEIGHT`), a
//!   control-effort penalty on the deviation (`CONTROL_WEIGHT`, the
//!   "linear-solvability" `½uᵀR⁻¹u`), and the centerline pull folded into
//!   `state_cost`.
//! - **The shared QMC sampler.** The knot noise comes from
//!   [`crate::planning::sampling::qmc_normals`], the *same* low-discrepancy
//!   sequence RRT* draws its targets from (see that module's parity note),
//!   not judo's pseudo-random `np.random.randn`. These optimizers are
//!   therefore deterministic pure functions of the ego state, like RRT* and
//!   unlike PI²-DDP — pinned by `*_is_a_pure_function_of_state`.
//! - **Warm start across ticks.** The winning deviations are carried to the
//!   next tick when the ego followed the plan, so each 0.1 s replan refines
//!   the last rather than restarting from the base policy.

mod cem;
mod mppi;
mod ps;

pub use cem::Cem;
pub use mppi::Mppi;
pub use ps::PredictiveSampling;

use crate::planning::cost::{self, Sample};
use crate::planning::sampling::{self, Halton};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State, drive, step};
use crate::wrap_angle;

/// Control dimension: `[accel, curvature]`. judo's `nu`.
pub(crate) const NU: usize = 2;

/// Planning horizon in ticks: 10 s at the simulator's 0.1 s tick rate, the
/// same look-ahead the lattice, PI²-DDP, and RRT* use
/// ([`PLANNING_HORIZON_S`]). The knots span this whole horizon; the
/// returned control trajectory is sampled from it.
const HORIZON: usize = (PLANNING_HORIZON_S / 0.1) as usize;

/// Physical std the dimensionless judo `sigma` multiplies, per control
/// dimension: `[accel, curvature]` — but of the knot *deviation* from the
/// base policy (see [`SamplingPlanner::base_policy`]), not an absolute
/// control. judo normalizes its controls to roughly `[-1, 1]`, so its
/// `sigma` values (0.05–0.1) are unitless; here a `sigma` of 1 means an
/// accel-deviation std of 0.5 m/s² and a curvature-deviation std of 0.08.
/// The base policy already holds speed and centers the lane, so the
/// acceleration deviation is explored only narrowly — the interesting
/// decisions (lane tracking, obstacle swerves) live in curvature, which
/// carries the bulk of the exploration, sized (like PI²-DDP's
/// lane-width-derived `sigma_kappa`) to span roughly the lane width over the
/// look-ahead. A wider acceleration std mainly gave MPPI's and CEM's
/// reward-weighted averages room to drift the speed off target for no gain.
pub(crate) const SIGMA_SCALE: [f64; NU] = [0.4, 0.08];

/// Weight on an explicit speed-tracking term, `Σ_t (speed_t − target)²`.
/// The shared cost prices overspeed only lightly (it is one term of many),
/// which leaves "slow but smoothly centering" scoring about as well as "at
/// speed and centered" — fine for the single-best pick PS and CEM's elite
/// mean make, but MPPI's reward-weighted *average* then leans toward the
/// slow cluster and the ego cruises below target. A firm, explicit speed
/// term breaks that tie toward the target speed for every optimizer, while
/// staying far below the hard-collision penalty so the planner still slows
/// or swerves for an obstacle.
const SPEED_WEIGHT: f64 = 0.5;

/// Weight on a control-effort penalty over the knot *deviation* from the
/// base policy, `Σ_t Σ_c (dev_c / SIGMA_SCALE_c)²` — the deviation scaled by
/// its own sampling std, i.e. PI²-DDP's "linear-solvability" control cost
/// (`½ uᵀR⁻¹u` with `R⁻¹` the inverse exploration covariance), kept for the
/// same reason. It pulls a deviation back toward zero — the bare,
/// lane-tracking, speed-holding base policy — unless the rollout cost pays
/// for departing: an obstacle swerve easily earns its curvature deviation
/// against the collision penalty, but on an open road there's nothing to buy,
/// so the deviation decays and the base policy tracks. Without it, MPPI's and
/// CEM's reward-weighted averages let the acceleration deviation drift until
/// it cancelled the base policy's speed hold, plateauing the ego below the
/// target speed (PS, taking the single best, didn't suffer this).
const CONTROL_WEIGHT: f64 = 0.3;

/// One control knot: `[accel, curvature]`.
pub type Knot = [f64; NU];

/// Base configuration shared by every optimizer — judo's `OptimizerConfig`.
/// Each optimizer wraps this with its own sampling/aggregation parameters
/// (sigma, temperature, elite count).
#[derive(Debug, Clone, Copy)]
pub struct OptimizerConfig {
    /// Number of sampled rollouts per iteration (judo's `num_rollouts`).
    /// The first is always the un-noised nominal.
    pub num_rollouts: usize,
    /// Number of control knots per trajectory (judo's `num_nodes`).
    pub num_nodes: usize,
    /// Whether to ramp the sampling std up along the horizon (judo's
    /// `use_noise_ramp`): near knots are perturbed less than far ones.
    pub use_noise_ramp: bool,
    /// Ramp magnitude when `use_noise_ramp` is set (judo's `noise_ramp`).
    pub noise_ramp: f64,
    /// How many sample→rollout→update iterations to run per `plan()` call.
    /// judo runs one optimizer step per control cycle and relies on the
    /// controller's replan rate; nanoplan replans every tick but affords a
    /// few refinement iterations, mirroring PI²-DDP's `GENERATIONS`. Not a
    /// judo field — a nanoplan adaptation of judo's controller loop.
    pub iterations: usize,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        OptimizerConfig {
            num_rollouts: 32,
            num_nodes: 4,
            use_noise_ramp: false,
            noise_ramp: 2.5,
            iterations: 4,
        }
    }
}

/// A judo optimizer: the two-method sampling/aggregation strategy, with
/// everything else (rollout, cost, warm start) supplied by
/// [`SamplingPlanner`]. Mirrors judo's abstract `Optimizer` base — the
/// three concrete optimizers are `impl`s of this and nothing more.
pub trait Optimizer: Default {
    /// Display name for the planner registry.
    const NAME: &'static str;

    /// This optimizer's base configuration.
    fn config(&self) -> OptimizerConfig;

    /// judo `sample_control_knots`: `num_rollouts` candidate knot-sets, the
    /// first the un-noised `nominal` itself and the rest `nominal` plus
    /// low-discrepancy Gaussian noise (scaled by the optimizer's own
    /// sigma). `sample_base` is the [`sampling::qmc_normals`] index this
    /// iteration draws from, kept distinct across iterations by the caller.
    fn sample_control_knots(&mut self, nominal: &[Knot], sample_base: usize) -> Vec<Vec<Knot>>;

    /// judo `update_nominal_knots`: fold the sampled knot-sets and their
    /// rewards (higher is better) into the next nominal. `&mut self` because
    /// CEM adapts its per-node sigma here; PS and MPPI don't touch `self`.
    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot>;
}

/// Shared body of judo's `sample_control_knots` (PS and MPPI use it
/// verbatim; CEM supplies its own per-node adaptive sigma through the same
/// `sigma` closure): prepend the un-noised nominal, then add
/// `sigma(node) * SIGMA_SCALE * z` to a copy for each of the remaining
/// `num_rollouts - 1` rollouts, `z` a low-discrepancy standard normal from
/// the shared QMC sequence.
pub(crate) fn noised_knots(
    nominal: &[Knot],
    num_rollouts: usize,
    sample_base: usize,
    sigma: impl Fn(usize) -> Knot,
) -> Vec<Vec<Knot>> {
    let num_nodes = nominal.len();
    let z = sampling::qmc_normals::<Halton>(sample_base, num_rollouts - 1, num_nodes * NU);
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

/// The per-node noise ramp judo optionally applies: knots near the ego are
/// perturbed less than distant ones. Returns a scalar multiplier for node
/// `n` of `num_nodes`; `1.0` when the ramp is off.
pub(crate) fn ramp(cfg: &OptimizerConfig, n: usize) -> f64 {
    if cfg.use_noise_ramp {
        cfg.noise_ramp * (n + 1) as f64 / cfg.num_nodes as f64
    } else {
        1.0
    }
}

/// Interpolated control at tick `t` from `knots` spread over `span` ticks:
/// knot `i` sits at tick `i·(span-1)/(num_nodes-1)`, and the control
/// between knots is a linear blend — judo's spline reconstruction at first
/// order. Beyond the last knot it holds the last value.
fn control_at(knots: &[Knot], t: usize, span: usize) -> Knot {
    let num_nodes = knots.len();
    if num_nodes == 1 {
        return knots[0];
    }
    let pos = t as f64 / (span - 1).max(1) as f64 * (num_nodes - 1) as f64;
    let i = pos.floor() as usize;
    if i >= num_nodes - 1 {
        return knots[num_nodes - 1];
    }
    let u = pos - i as f64;
    [
        knots[i][0] + (knots[i + 1][0] - knots[i][0]) * u,
        knots[i][1] + (knots[i + 1][1] - knots[i][1]) * u,
    ]
}

/// A receding-horizon sampling planner parameterized by its judo optimizer:
/// `SamplingPlanner<PredictiveSampling>`, `SamplingPlanner<Cem>`, and
/// `SamplingPlanner<Mppi>` are the three planners the registry exposes. The
/// generic holds all the machinery judo keeps outside the optimizer —
/// rollout, cost, the road-informed nominal, warm start — so each optimizer
/// stays a two-method strategy.
pub struct SamplingPlanner<O: Optimizer> {
    opt: O,
    /// Last tick's winning nominal knots, carried forward as this tick's
    /// starting nominal when the ego followed the plan (warm start).
    nominal: Option<Vec<Knot>>,
    /// Predicted next ego state, to check the warm start is still valid.
    expected_next: State,
}

impl<O: Optimizer> Default for SamplingPlanner<O> {
    fn default() -> Self {
        SamplingPlanner {
            opt: O::default(),
            nominal: None,
            expected_next: State::default(),
        }
    }
}

impl<O: Optimizer> SamplingPlanner<O> {
    pub const NAME: &'static str = O::NAME;

    /// The road-model base policy the knots deviate from — the "hybrid road
    /// model" half of the sampling, and what makes the open-loop knot
    /// rollout stable. A **pure-pursuit** steer toward the centerline plus a
    /// proportional speed hold, both computed from the *current* rollout
    /// state, i.e. genuine feedback (the same tracker PI²-DDP initializes
    /// from and Bezier+IDM follows).
    ///
    /// The knots don't replace this policy, they *add* to it
    /// ([`SamplingPlanner::command`]). This is the crucial adaptation of judo's
    /// otherwise open-loop knot sampling to a 10 s driving horizon: a car's
    /// lateral dynamics integrate curvature twice, so raw open-loop knots
    /// diverge metres off-lane over the horizon (the feedback-free rollout
    /// has nothing to correct a small early heading error), and every
    /// candidate then scores as garbage. Sampling *deviations from a
    /// stabilizing feedback base* keeps every rollout on the road — exactly
    /// why PI²-DDP rolls out with its feedback gains `K` rather than raw
    /// nominal controls — so the QMC exploration prices real maneuvers
    /// (an obstacle swerve) instead of open-loop drift.
    fn base_policy(path: &Path, x: &State, ctx: &Context) -> Knot {
        // critically-damped PD lane keeper on the Frenet offset `d` and the
        // heading error: `κ = −(K_d·d + K_h·heading_err)`. For the kinematic
        // model this closes a second-order lateral loop with damping ratio
        // `K_h / (2·√K_d)`; `K_d = 0.02`, `K_h = 0.3` puts it right around
        // critical, so the base centres the lane with essentially no
        // overshoot — important because the optimizers *add* to this policy,
        // and MPPI's reward-weighted average in particular biases toward
        // aggressive correction, which an under-damped base (e.g. short-
        // lookahead pure pursuit) turns into a lateral oscillation.
        let (s, d) = path.project([x.x, x.y]);
        let (_, lane_yaw) = path.pose_at(s);
        let heading_err = wrap_angle(x.yaw - lane_yaw);
        let curvature = -(0.02 * d + 0.3 * heading_err);
        let accel = (0.5 * (ctx.road.target_speed - x.speed)).clamp(-2.0, 1.5);
        [accel, curvature]
    }

    /// The control *commanded* at rollout state `x` for knot-deviation `dev`:
    /// the base policy plus the deviation. It is not clamped here — the rollout
    /// pushes it through the shared forward model ([`drive`]), which applies
    /// the actuation limits (including jerk and steering rate) against the
    /// previously applied control, exactly as the plant will.
    fn command(path: &Path, x: &State, dev: Knot, ctx: &Context) -> Control {
        let base = Self::base_policy(path, x, ctx);
        Control {
            accel: base[0] + dev[0],
            curvature: base[1] + dev[1],
        }
    }

    /// Cost of being at `x` at tick `t` having just applied `u` — the
    /// shared cost function plus a centerline-pull term, with hard
    /// violations made finite (see the module doc). Timed under the `cost`
    /// seam, like every other search planner.
    fn state_cost(path: &Path, x: &State, u: Knot, t: usize, ctx: &Context) -> f64 {
        let (s, d) = path.project([x.x, x.y]);
        let (_, lane_yaw) = path.pose_at(s);
        let sample = Sample {
            xy: [x.x, x.y],
            lateral: d,
            heading_err: wrap_angle(x.yaw - lane_yaw),
            speed: x.speed,
            curvature: u[1],
            accel: u[0],
            t: t as f64 * ctx.road.dt,
        };
        // The shared cost with hard violations made finite by a depth-scaled
        // escape slope (`soft_point_cost`): a flat penalty plateau leaves
        // CEM's and MPPI's reward-weighted averages no gradient back onto the
        // road once every sampled rollout is briefly off it, so they can
        // settle off-road; the depth slope pulls them back in.
        0.5 * d * d
            + ctx.time("cost", || {
                cost::soft_point_cost(
                    &sample,
                    ctx.road.target_speed,
                    ctx.road.half_width,
                    ctx.actors,
                    Some(path),
                )
            })
    }

    /// Roll a knot-set out from the ego over the full horizon, applying each
    /// interpolated knot as a *deviation* from the base policy ([`command`]),
    /// and return the visited states (for diagnostics) and the reward
    /// (negated total cost; judo maximizes reward). The terminal state is
    /// weighted like PI²-DDP's, pricing position and speed once more at the
    /// end.
    fn rollout(&self, knots: &[Knot], path: &Path, ego: State, ctx: &Context) -> (Vec<State>, f64) {
        let mut x = ego;
        // seed the rollout's actuation state with the control the plant applied
        // to the ego last tick, so the jerk/steering-rate limiting inside
        // `drive` starts where the car actually is (see `Context::ego_control`)
        let mut prev = ctx.ego_control;
        let mut xs = vec![ego];
        let mut total = 0.0;
        for t in 0..HORIZON {
            let dev = control_at(knots, t, HORIZON);
            let cmd = Self::command(path, &x, dev, ctx);
            let (nx, u) = drive(x, prev, cmd, ctx.road.dt);
            total += Self::state_cost(path, &x, [u.accel, u.curvature], t, ctx);
            // control-effort penalty on the deviation from the base policy
            // (see CONTROL_WEIGHT)
            total += CONTROL_WEIGHT
                * ((dev[0] / SIGMA_SCALE[0]).powi(2) + (dev[1] / SIGMA_SCALE[1]).powi(2));
            // firm speed tracking (see SPEED_WEIGHT)
            let sv = x.speed - ctx.road.target_speed;
            total += SPEED_WEIGHT * sv * sv;
            (x, prev) = (nx, u);
            xs.push(x);
        }
        total += 5.0 * Self::state_cost(path, &x, [0.0, 0.0], HORIZON, ctx);
        (xs, -total)
    }
}

impl<O: Optimizer> Planner for SamplingPlanner<O> {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let cfg = self.opt.config();
        let num_nodes = cfg.num_nodes;
        let path = ctx.time("route", || Path::new(&ctx.road.centerline));

        // Warm start: reuse last tick's nominal knot-deviations when the ego
        // followed the plan (they still describe a good maneuver to refine),
        // otherwise start from zero deviation — the bare base policy, which
        // already tracks the lane and holds speed. Custom seam like
        // PI²-DDP's, mirroring its warm-start-or-reinit split.
        let mut nominal = ctx.time("warm_start", || match self.nominal.take() {
            Some(n)
                if n.len() == num_nodes
                    && (self.expected_next.x - ego.x).hypot(self.expected_next.y - ego.y) < 1.0 =>
            {
                n
            }
            _ => vec![[0.0; NU]; num_nodes],
        });

        // judo's optimize loop: sample knot-sets, roll each out and score
        // it, fold the scores into a new nominal — repeated `iterations`
        // times, each iteration drawing a fresh, non-overlapping slice of
        // the shared QMC sequence.
        let mut last_rollouts: Vec<Vec<State>> = Vec::new();
        ctx.time("optimize", || {
            for it in 0..cfg.iterations {
                let sample_base = 1 + it * cfg.num_rollouts;
                let sampled = self.opt.sample_control_knots(&nominal, sample_base);
                let mut rewards = Vec::with_capacity(sampled.len());
                let mut states = Vec::with_capacity(sampled.len());
                for knots in &sampled {
                    let (xs, reward) = self.rollout(knots, &path, ego, ctx);
                    rewards.push(reward);
                    states.push(xs);
                }
                nominal = self.opt.update_nominal_knots(&sampled, &rewards);
                if it == cfg.iterations - 1 {
                    last_rollouts = states;
                }
            }
        });

        // Diagnostics: the final iteration's sampled rollouts, both as a
        // point cloud and as trajectories — mirroring PI²-DDP.
        if let Some(diag) = ctx.diagnostics {
            for xs in &last_rollouts {
                let pts: Vec<[f64; 2]> = xs.iter().map(|s| [s.x, s.y]).collect();
                for &p in &pts {
                    diag.record_point(p);
                }
                diag.record_trajectory(pts);
            }
        }

        // Extract: roll the winning deviations forward through the base
        // policy and the shared forward model from the true ego state (both
        // the base policy and the actuation limiting are feedback, so each
        // control depends on the state and applied control the previous one
        // reached) and emit the actually-applied control sequence.
        let controls = ctx.time("extract", || {
            let mut x = ego;
            let mut prev = ctx.ego_control;
            (0..ctx.horizon)
                .map(|t| {
                    let cmd = Self::command(&path, &x, control_at(&nominal, t, HORIZON), ctx);
                    let (nx, u) = drive(x, prev, cmd, ctx.road.dt);
                    (x, prev) = (nx, u);
                    u
                })
                .collect::<Vec<_>>()
        });

        self.expected_next = step(ego, controls[0], ctx.road.dt);
        self.nominal = Some(nominal);
        controls
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
    fn control_at_interpolates_between_knots() {
        let knots = [[0.0, 0.0], [2.0, 0.2]];
        // start and end land on the knots
        assert_eq!(control_at(&knots, 0, 11), [0.0, 0.0]);
        assert_eq!(control_at(&knots, 10, 11), [2.0, 0.2]);
        // midpoint is the average
        let mid = control_at(&knots, 5, 11);
        assert!((mid[0] - 1.0).abs() < 1e-9 && (mid[1] - 0.1).abs() < 1e-9);
    }

    #[test]
    fn base_policy_steers_toward_the_lane() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let path = Path::new(&road.centerline);
        // from y = +2 (left of the lane), the base policy steers right
        // (negative curvature) and accelerates toward the target speed
        let x = State {
            y: 2.0,
            speed: 8.0,
            ..Default::default()
        };
        let base = SamplingPlanner::<PredictiveSampling>::base_policy(&path, &x, &ctx);
        assert!(base[1] < 0.0, "curvature {}", base[1]);
        assert!(base[0] > 0.0, "accel {}", base[0]);
    }

    // --- closed-loop tests, one battery per optimizer -----------------
    //
    // Same style as every other planner's tests (see the "Test harness"
    // section of the README): a single `plan()` call proves little, so each
    // optimizer is driven closed-loop and its realized trajectory checked.

    /// From an initial lateral offset, converge to the lane and hold the
    /// target speed — the basic "can it track the road" bar.
    fn tracks_centerline_and_speed<O: Optimizer>() {
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let trace = run_planner::<O>(ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 1.2, "{} offset {}", O::NAME, end.y);
        assert!(
            (end.speed - 10.0).abs() < 2.5,
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
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = run_planner::<O>(ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "{} min gap {min_gap}", O::NAME);
        assert!(
            trace.last().unwrap().x > 50.0,
            "{} did not pass, x {}",
            O::NAME,
            trace.last().unwrap().x
        );
    }

    /// The knot noise is QMC (a pure function of the sample index), the
    /// nominal is a deterministic road tracker, and there is no `Rng`: two
    /// fresh planners replanning from the identical state must produce the
    /// identical plan, like RRT* and unlike PI²-DDP.
    fn is_a_pure_function_of_state<O: Optimizer>() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let actors = [obstacle];
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &actors);
        let a = SamplingPlanner::<O>::default().plan(ego, &ctx);
        let b = SamplingPlanner::<O>::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }

    #[test]
    fn ps_tracks_centerline_and_speed() {
        tracks_centerline_and_speed::<PredictiveSampling>();
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
    fn cem_tracks_centerline_and_speed() {
        tracks_centerline_and_speed::<Cem>();
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
    fn mppi_tracks_centerline_and_speed() {
        tracks_centerline_and_speed::<Mppi>();
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
        // the final iteration's num_rollouts sampled trajectories, each a
        // full HORIZON + 1 state polyline, and every state flattened into
        // the point cloud
        let cfg = OptimizerConfig::default();
        assert_eq!(data.trajectories.len(), cfg.num_rollouts);
        assert!(data.trajectories.iter().all(|t| t.len() == HORIZON + 1));
        assert_eq!(data.points.len(), cfg.num_rollouts * (HORIZON + 1));
    }
}
