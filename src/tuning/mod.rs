//! Autotuning of the shared cost function's soft weights from expert
//! demonstrations, via maximum-entropy inverse reinforcement learning —
//! the DriveIRL recipe (Phan-Minh et al., "Driving in Real Life with
//! Inverse Reinforcement Learning", arXiv:2206.03004) applied to
//! [`crate::planning::cost`].
//!
//! The premise: the expert trajectories in nuPlan logs were produced by a
//! human driver optimizing a well-tuned cost of the same linear form the
//! planners share — `WEIGHTS · features` per sampled point. Under the
//! maximum-entropy model, the expert picks trajectory `τ` from a candidate
//! set with probability `P(τ) ∝ exp(-w·φ(τ))`, where `φ(τ)` sums the point
//! features over the trajectory. Fitting `w` is then a convex
//! maximum-likelihood problem: minimize the negative log-likelihood of the
//! expert demonstration per scenario, whose gradient is the classic MaxEnt
//! IRL feature-matching residual `φ(expert) − E_P[φ]`.
//!
//! **The hard rules are not learned.** Collision and leaving the drivable
//! area yield infinite cost / zero probability by fiat, exactly as
//! [`cost::features`](crate::planning::cost::features) hardcodes them: a
//! candidate that hard-violates is dropped from the distribution's support
//! entirely (DriveIRL's safety filter plays the same role), and a scenario
//! whose *expert* hard-violates under the model is skipped — a demonstration
//! the model calls infinitely bad carries no usable preference signal.
//!
//! Per scenario, the candidate set mirrors DriveIRL's lattice generator at
//! nanoplan scale: a deterministic grid of Frenet maneuvers from the ego's
//! starting state — target lateral offsets × maneuver durations ×
//! exponential speed approaches to a grid of settled cruise speeds —
//! rolled out over the shared planning horizon, plus
//! the expert trajectory itself (resampled to the same tick grid), so the
//! demonstration is always in the distribution's support. Every trajectory,
//! expert included, is featurized by the same code path: project onto the
//! centerline, estimate curvature with the same Menger formula the lattice
//! planner uses, difference speeds for accel, and sum
//! [`cost::features`](crate::planning::cost::features) over the ticks.
//!
//! Optimization is plain projected gradient descent (weights clamped ≥ 0 —
//! these are penalties) with backtracking line search, on the mean NLL plus
//! a small L2 pull toward the *current* hand weights: a Gaussian prior
//! centered on them, so a feature the data expresses no preference about
//! keeps its hand-tuned value instead of drifting to zero. Features are
//! rescaled to unit mean magnitude internally (they span several orders of
//! magnitude) and the learned weights mapped back to raw units at the end.
//!
//! Everything is deterministic: same scenarios in, same weights out.

use crate::planning::PLANNING_HORIZON_S;
use crate::planning::cost::{self, FEATURE_NAMES, N_FEATURES, Sample, WEIGHTS};
use crate::scenarios::{Path, Scenario, replay};
use crate::simulation::State;
use crate::wrap_angle;

/// Simulator tick length the trajectories are featurized at.
const DT: f64 = 0.1;
/// Ticks per trajectory: the shared planning horizon at [`DT`] (rounded —
/// a plain cast of 10.0/0.1 truncates the inexact quotient to 99).
const TICKS: usize = (PLANNING_HORIZON_S / DT + 0.5) as usize;

/// The candidate maneuver grid, DriveIRL-lattice style: where to end up
/// laterally, how quickly to make the lateral move, and which cruise speed
/// (relative to the scenario target) to settle at, how quickly. Longitudinal
/// profiles are exponential approaches to a settled speed rather than
/// constant accelerations, because that is the shape human speed profiles
/// take — against constant-accel candidates that never settle, a real expert
/// is cheaper than *everything* by a wide margin, the softmax saturates, and
/// the likelihood gradient vanishes.
const LATERAL_TARGETS_M: [f64; 7] = [-3.0, -1.5, -0.5, 0.0, 0.5, 1.5, 3.0];
const MANEUVER_S: [f64; 2] = [3.0, 6.0];
const SPEED_OFFSETS_MS: [f64; 6] = [-4.0, -2.0, -1.0, 0.0, 1.0, 2.0];
const SPEED_TAU_S: [f64; 2] = [2.0, 4.0];

/// Tuner knobs. `Default` is the intended starting point.
pub struct Options {
    /// Gradient-descent iterations.
    pub iters: usize,
    /// L2 pull toward the current hand weights (prior strength), in
    /// normalized-feature space.
    pub l2: f64,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            iters: 500,
            l2: 1e-3,
        }
    }
}

/// What [`tune`] learned, plus enough bookkeeping to judge whether to trust
/// it. `expert_top1_*` counts scenarios whose expert is the minimum-cost
/// member of its candidate set — the interpretable "would the planner have
/// picked what the human did" number.
pub struct TuneResult {
    /// Learned weights in raw feature units, drop-in for `cost::WEIGHTS`.
    pub weights: [f64; N_FEATURES],
    /// Mean per-scenario negative log-likelihood under the current weights.
    pub nll_before: f64,
    /// Same, under the learned weights.
    pub nll_after: f64,
    pub expert_top1_before: usize,
    pub expert_top1_after: usize,
    pub scenarios_used: usize,
    /// Scenarios without a usable expert: none logged, shorter than the
    /// planning horizon, or hard-violating under the model.
    pub scenarios_skipped: usize,
}

impl TuneResult {
    /// Human-readable summary, ending in a `WEIGHTS` line to paste into
    /// src/planning/cost.rs.
    pub fn report(&self) -> String {
        use std::fmt::Write;
        let mut out = String::new();
        writeln!(out, "maxent-irl cost-weight autotune").unwrap();
        writeln!(
            out,
            "  scenarios: {} used, {} skipped (no expert, expert shorter than the {PLANNING_HORIZON_S} s horizon, or expert hard-violating)",
            self.scenarios_used, self.scenarios_skipped
        )
        .unwrap();
        if self.scenarios_used == 0 {
            writeln!(out, "  nothing to learn from; weights unchanged").unwrap();
            return out;
        }
        writeln!(
            out,
            "  mean NLL/scenario:      {:.4} -> {:.4}",
            self.nll_before, self.nll_after
        )
        .unwrap();
        writeln!(
            out,
            "  expert is min-cost in:  {}/{} -> {}/{} scenarios",
            self.expert_top1_before,
            self.scenarios_used,
            self.expert_top1_after,
            self.scenarios_used
        )
        .unwrap();
        writeln!(
            out,
            "\n  {:16} {:>10} {:>10}",
            "feature", "current", "learned"
        )
        .unwrap();
        for k in 0..N_FEATURES {
            writeln!(
                out,
                "  {:16} {:>10.4} {:>10.4}",
                FEATURE_NAMES[k], WEIGHTS[k], self.weights[k]
            )
            .unwrap();
        }
        let list: Vec<String> = self.weights.iter().map(|w| format!("{w:.4}")).collect();
        writeln!(
            out,
            "\npaste into src/planning/cost.rs:\npub(crate) const WEIGHTS: [f64; N_FEATURES] = [{}];",
            list.join(", ")
        )
        .unwrap();
        out
    }
}

/// Fit the soft cost weights to the expert trajectories in `scenarios`.
/// Scenarios without a usable expert are skipped (counted in the result);
/// with none usable at all, the current weights come back unchanged.
pub fn tune(scenarios: &[Scenario], opts: &Options) -> TuneResult {
    // per scenario: feasible candidates' feature vectors, expert at index 0
    let mut data: Vec<Vec<[f64; N_FEATURES]>> = vec![];
    let mut skipped = 0;
    for sc in scenarios {
        match scenario_candidates(sc) {
            Some(c) => data.push(c),
            None => skipped += 1,
        }
    }
    let unchanged = TuneResult {
        weights: WEIGHTS,
        nll_before: 0.0,
        nll_after: 0.0,
        expert_top1_before: 0,
        expert_top1_after: 0,
        scenarios_used: data.len(),
        scenarios_skipped: skipped,
    };
    if data.is_empty() {
        return unchanged;
    }

    // rescale each feature to unit mean magnitude across every candidate —
    // purely optimizer conditioning; raw magnitudes span several orders
    let mut scale = [0.0; N_FEATURES];
    let mut count = 0usize;
    for cands in &data {
        for f in cands {
            for k in 0..N_FEATURES {
                scale[k] += f[k].abs();
            }
        }
        count += cands.len();
    }
    for s in &mut scale {
        *s = (*s / count as f64).max(1e-12);
    }
    for cands in &mut data {
        for f in cands {
            for k in 0..N_FEATURES {
                f[k] /= scale[k];
            }
        }
    }
    // w·φ is invariant under the rescale when weights carry the inverse, so
    // the hand weights map to w0 and rankings/NLLs are comparable throughout
    let w0: [f64; N_FEATURES] = std::array::from_fn(|k| WEIGHTS[k] * scale[k]);

    let (nll_before, _) = nll_grad(&data, &w0);
    let top1_before = expert_top1(&data, &w0);

    // projected gradient descent with backtracking on
    // mean NLL + l2·|w − w0|² (prior centered on the hand weights)
    let objective = |w: &[f64; N_FEATURES]| {
        let (nll, _) = nll_grad(&data, w);
        nll + opts.l2
            * w.iter()
                .zip(w0)
                .map(|(a, b)| (a - b) * (a - b))
                .sum::<f64>()
    };
    let mut w = w0;
    let mut obj = objective(&w);
    let mut lr = 0.05;
    'descent: for _ in 0..opts.iters {
        let (_, mut g) = nll_grad(&data, &w);
        for k in 0..N_FEATURES {
            g[k] += 2.0 * opts.l2 * (w[k] - w0[k]);
        }
        loop {
            let w_new: [f64; N_FEATURES] = std::array::from_fn(|k| (w[k] - lr * g[k]).max(0.0));
            let obj_new = objective(&w_new);
            if obj_new <= obj {
                (w, obj) = (w_new, obj_new);
                lr *= 1.5; // grow freely; the backtracking above is the guard
                break;
            }
            lr *= 0.5;
            if lr < 1e-12 {
                break 'descent; // converged: no descent direction left
            }
        }
    }

    let (nll_after, _) = nll_grad(&data, &w);
    TuneResult {
        weights: std::array::from_fn(|k| w[k] / scale[k]),
        nll_before,
        nll_after,
        expert_top1_before: top1_before,
        expert_top1_after: expert_top1(&data, &w),
        ..unchanged
    }
}

/// Mean NLL of the expert (candidate 0) under `P(τ) ∝ exp(-w·φ(τ))`, and its
/// gradient `E_P[φ] − φ(expert)` averaged over scenarios. Stabilized by the
/// per-scenario minimum cost, so the largest exponent is exactly 0.
fn nll_grad(data: &[Vec<[f64; N_FEATURES]>], w: &[f64; N_FEATURES]) -> (f64, [f64; N_FEATURES]) {
    let n = data.len() as f64;
    let mut nll = 0.0;
    let mut grad = [0.0; N_FEATURES];
    for cands in data {
        let costs: Vec<f64> = cands.iter().map(|f| dot(w, f)).collect();
        let min = costs.iter().copied().fold(f64::INFINITY, f64::min);
        let e: Vec<f64> = costs.iter().map(|c| (-(c - min)).exp()).collect();
        let z: f64 = e.iter().sum();
        nll += (costs[0] - min + z.ln()) / n;
        for (j, f) in cands.iter().enumerate() {
            let p = e[j] / z;
            for k in 0..N_FEATURES {
                // φ_E − Σ_j p_j φ_j, negated: this is the NLL's gradient
                grad[k] += ((if j == 0 { 1.0 } else { 0.0 }) - p) * f[k] / n;
            }
        }
    }
    (nll, grad)
}

/// Scenarios whose expert (candidate 0) is min-cost under `w` (ties count).
fn expert_top1(data: &[Vec<[f64; N_FEATURES]>], w: &[f64; N_FEATURES]) -> usize {
    data.iter()
        .filter(|cands| {
            let e = dot(w, &cands[0]);
            cands[1..].iter().all(|f| e <= dot(w, f))
        })
        .count()
}

fn dot(w: &[f64; N_FEATURES], f: &[f64; N_FEATURES]) -> f64 {
    w.iter().zip(f).map(|(a, b)| a * b).sum()
}

/// Feature vectors of one scenario's candidate set — the resampled expert at
/// index 0, then every feasible maneuver from the grid. `None` when the
/// scenario has no usable expert: none logged, coverage short of the
/// planning horizon, or hard-violating (collision / off-road) under the
/// model — an "infinitely bad" demonstration teaches the soft weights
/// nothing.
fn scenario_candidates(sc: &Scenario) -> Option<Vec<[f64; N_FEATURES]>> {
    if sc.expert.last()?.t < PLANNING_HORIZON_S - 1e-9 {
        return None;
    }
    let path = Path::new(&sc.centerline);
    // actor states at t = 0, the same current-tick view a planner gets;
    // cost::features projects them forward itself, by the sample's t
    let actors: Vec<State> = sc
        .actors
        .iter()
        .map(|a| {
            if a.trajectory.is_empty() {
                a.init
            } else {
                replay(&a.trajectory, 0.0)
            }
        })
        .collect();

    let expert_states: Vec<State> = (0..=TICKS)
        .map(|i| replay(&sc.expert, i as f64 * DT))
        .collect();
    let expert = trajectory_features(&expert_states, &path, sc.target_speed, &actors)?;

    let (s0, d0) = path.project([sc.ego.x, sc.ego.y]);
    let mut cands = vec![expert];
    for m in maneuver_grid(sc.target_speed) {
        let states = maneuver_states(&path, s0, d0, sc.ego.speed, &m);
        // hard-violating candidates get zero probability: out of the
        // support entirely, like the planners reject them
        if let Some(f) = trajectory_features(&states, &path, sc.target_speed, &actors) {
            cands.push(f);
        }
    }
    (cands.len() >= 2).then_some(cands)
}

/// Summed [`cost::features`] over a tick-sampled trajectory, or `None` if
/// any tick hard-violates. Curvature is the Menger estimate off consecutive
/// points (the same one the lattice planner uses) and accel the speed
/// difference quotient, so logged experts and generated maneuvers are
/// featurized identically.
fn trajectory_features(
    states: &[State],
    path: &Path,
    target_speed: f64,
    actors: &[State],
) -> Option<[f64; N_FEATURES]> {
    let mut total = [0.0; N_FEATURES];
    for (i, st) in states.iter().enumerate() {
        let p = [st.x, st.y];
        let (s, lateral) = path.project(p);
        let curvature = if i == 0 || i + 1 == states.len() {
            0.0
        } else {
            let (a, b) = (states[i - 1], states[i + 1]);
            cost::curvature_of([a.x, a.y], p, [b.x, b.y])
        };
        let accel = if i + 1 < states.len() {
            (states[i + 1].speed - st.speed) / DT
        } else {
            (st.speed - states[i - 1].speed) / DT
        };
        let sample = Sample {
            xy: p,
            lateral,
            heading_err: wrap_angle(st.yaw - path.pose_at(s).1),
            speed: st.speed,
            curvature,
            accel,
            t: i as f64 * DT,
        };
        let f = cost::features(&sample, target_speed, actors)?;
        for (tot, x) in total.iter_mut().zip(f) {
            *tot += x;
        }
    }
    Some(total)
}

/// One cell of the candidate grid: end up `d_end` off the centerline in
/// `dur` seconds, while the speed settles exponentially to `v_settle` with
/// time constant `tau`.
struct Maneuver {
    d_end: f64,
    dur: f64,
    v_settle: f64,
    tau: f64,
}

/// Every maneuver in the grid, with settled speeds placed relative to the
/// scenario's target speed.
fn maneuver_grid(target_speed: f64) -> Vec<Maneuver> {
    let mut grid = vec![];
    for dur in MANEUVER_S {
        for d_end in LATERAL_TARGETS_M {
            for v_off in SPEED_OFFSETS_MS {
                for tau in SPEED_TAU_S {
                    grid.push(Maneuver {
                        d_end,
                        dur,
                        v_settle: (target_speed + v_off).max(0.0),
                        tau,
                    });
                }
            }
        }
    }
    grid
}

/// One candidate maneuver, tick-sampled over the planning horizon, starting
/// from the ego's Frenet state `(s0, d0, speed0)`. Yaw follows the direction
/// of travel; a stopped car keeps its last heading.
fn maneuver_states(path: &Path, s0: f64, d0: f64, speed0: f64, m: &Maneuver) -> Vec<State> {
    let mut xy = Vec::with_capacity(TICKS + 1);
    let mut speeds = Vec::with_capacity(TICKS + 1);
    let mut s = s0;
    for i in 0..=TICKS {
        let t = i as f64 * DT;
        let u = (t / m.dur).min(1.0);
        let d = d0 + (m.d_end - d0) * u * u * (3.0 - 2.0 * u);
        xy.push(path.frenet_to_xy(s, d));
        let speed = (m.v_settle + (speed0 - m.v_settle) * (-t / m.tau).exp()).max(0.0);
        speeds.push(speed);
        s += speed * DT;
    }
    let mut yaw = path.pose_at(s0).1;
    (0..xy.len())
        .map(|i| {
            let (a, b) = if i + 1 < xy.len() {
                (xy[i], xy[i + 1])
            } else {
                (xy[i - 1], xy[i])
            };
            if (b[0] - a[0]).hypot(b[1] - a[1]) > 1e-9 {
                yaw = (b[1] - a[1]).atan2(b[0] - a[0]);
            }
            State {
                x: xy[i][0],
                y: xy[i][1],
                yaw,
                speed: speeds[i],
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenarios::{Actor, MapData, Waypoint};
    use crate::simulation::Control;

    /// A scenario whose expert is the min-cost maneuver from the tuner's own
    /// grid under ground-truth weights `gt` — a synthetic demonstration with
    /// a known generating cost.
    fn scenario_with_expert(gt: &[f64; N_FEATURES], d0: f64, speed0: f64) -> Scenario {
        let centerline: Vec<[f64; 2]> = vec![[-20.0, 0.0], [500.0, 0.0]];
        let path = Path::new(&centerline);
        let ego = State {
            y: d0,
            speed: speed0,
            ..Default::default()
        };
        let mut best: Option<(f64, Vec<State>)> = None;
        for m in maneuver_grid(10.0) {
            let states = maneuver_states(&path, ego.x + 20.0, d0, speed0, &m);
            if let Some(f) = trajectory_features(&states, &path, 10.0, &[]) {
                let c = dot(gt, &f);
                if best.as_ref().is_none_or(|(b, _)| c < *b) {
                    best = Some((c, states));
                }
            }
        }
        let expert = best
            .unwrap()
            .1
            .into_iter()
            .enumerate()
            .map(|(i, state)| Waypoint {
                t: i as f64 * DT,
                state,
            })
            .collect();
        Scenario {
            name: "irl".into(),
            ego,
            actors: vec![],
            centerline,
            target_speed: 10.0,
            map: MapData::default(),
            expert,
        }
    }

    #[test]
    fn recovers_a_shifted_preference_from_demonstrations() {
        // ground truth cares 10x more about heading error than the hand
        // weights do; the expert therefore makes lane returns more gently
        // than the current cost would choose
        let mut gt = WEIGHTS;
        gt[3] *= 10.0;
        let scenarios: Vec<Scenario> = [(4.5, 8.0), (-4.5, 10.0), (4.5, 12.0), (-4.2, 6.0)]
            .into_iter()
            .map(|(d0, v0)| scenario_with_expert(&gt, d0, v0))
            .collect();

        // lightly regularized so the data, not the prior on the hand
        // weights, dominates what the test asserts
        let opts = Options {
            iters: 2000,
            l2: 1e-5,
        };
        let r = tune(&scenarios, &opts);
        assert_eq!(r.scenarios_used, scenarios.len());
        assert_eq!(r.scenarios_skipped, 0);
        // the fit explains the demonstrations far better: each expert is a
        // duplicate of one grid candidate, so ln 2 ≈ 0.69 is the NLL floor,
        // and the hand weights start above 4
        assert!(
            r.nll_after < r.nll_before,
            "{} !< {}",
            r.nll_after,
            r.nll_before
        );
        assert!(
            r.nll_after < 1.5,
            "nll {} far from the ln 2 floor",
            r.nll_after
        );
        assert!(
            r.expert_top1_after >= r.expert_top1_before,
            "{} < {}",
            r.expert_top1_after,
            r.expert_top1_before
        );
        // and the learned weights moved decisively in the ground truth's
        // direction (exact recovery isn't implied by maximum likelihood —
        // near-tie candidates trade top-1 rank against total likelihood)
        assert!(
            r.weights[3] > 2.0 * WEIGHTS[3],
            "heading weight {} did not increase toward the truth",
            r.weights[3]
        );
    }

    #[test]
    fn hard_violating_expert_is_skipped_and_weights_stay_put() {
        // the expert drives straight through a parked car: infinite cost by
        // fiat, so the scenario teaches nothing and must be skipped
        let expert: Vec<Waypoint> = (0..=TICKS)
            .map(|i| Waypoint {
                t: i as f64 * DT,
                state: State {
                    x: 8.0 * i as f64 * DT,
                    speed: 8.0,
                    ..Default::default()
                },
            })
            .collect();
        let sc = Scenario {
            name: "through-a-parked-car".into(),
            ego: State {
                speed: 8.0,
                ..Default::default()
            },
            actors: vec![Actor {
                init: State {
                    x: 40.0,
                    ..Default::default()
                },
                control: Control::default(),
                trajectory: vec![],
            }],
            centerline: vec![[-20.0, 0.0], [500.0, 0.0]],
            target_speed: 10.0,
            map: MapData::default(),
            expert,
        };
        let r = tune(&[sc], &Options::default());
        assert_eq!(r.scenarios_used, 0);
        assert_eq!(r.scenarios_skipped, 1);
        assert_eq!(r.weights, WEIGHTS);
        assert!(r.report().contains("weights unchanged"));
    }

    #[test]
    fn scenarios_without_or_with_short_experts_are_skipped() {
        let mut scenarios = crate::scenarios::synthetic_batch(2, 3);
        // a logged expert shorter than the planning horizon
        scenarios[1].expert = vec![Waypoint {
            t: 0.0,
            state: scenarios[1].ego,
        }];
        let r = tune(&scenarios, &Options::default());
        assert_eq!(r.scenarios_used, 0);
        assert_eq!(r.scenarios_skipped, 2);
        assert_eq!(r.weights, WEIGHTS);
    }
}
