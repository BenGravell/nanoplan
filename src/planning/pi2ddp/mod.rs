//! Sampling-based DDP planner following PI²-DDP (Lefebvre & Crevecoeur,
//! "Path Integral Policy Improvement with Differential Dynamic Programming").
//!
//! Per Algorithm 2 of the paper: each generation samples K control rollouts
//! around a nominal trajectory with feedback, weights them by exponentiated
//! cost-to-go (eq. 12), and extracts DDP-like updates from the reward-weighted
//! statistics — feedforward k = Σₖ pₖ(δu − Kδx), feedback K = Σᵤₓ Σₓₓ†, and
//! perturbation covariance Σᵤ = Σᵤᵤ − ΣᵤₓΣₓₓ†Σₓᵤ + λ_exp R⁻¹ (eq. 37) — with
//! the trust-region exploration heuristic of eq. 38 (adaptive v2: a worse
//! generation is discarded entirely).
//!
//! The sampling distribution is road-model informed: the initial nominal
//! control sequence tracks the lane centerline and brakes for traffic
//! and the initial curvature variance is sized so sampled trajectories span
//! the lane width at the preview distance.

use crate::common::kinematics::clamp_control;
use crate::common::rng::Rng;
use crate::common::types::matrix::{M2, M4, M6, M24};
use crate::common::types::state;
use crate::common::types::vector::V2;
use crate::planning::search_tree::centerline_follow_controls;
use crate::planning::{Context, PLANNING_TICKS, Planner, TrajectoryCost, take_warm};
use crate::simulation::{Control, Position, State, world_step};
use crate::track::Path;

const HORIZON: usize = PLANNING_TICKS;
const ROLLOUTS: usize = 32; // K in the paper; K > n + m with margin
const GENERATIONS: usize = 4;
const BETA: f64 = 10.0; // baseline sensitivity (eq. 12)
const ALPHA: f64 = 0.5; // covariance damping (eq. 36)
const LAMBDA_REG: f64 = 1e-3; // inverse regularization heuristic
const SIGMA_ACCEL: f64 = 4.0; // [m/s²] exploration std
const LANE_HALF_M: f64 = 1.75;
// physical action limits; also keep near-singular Σₓₓ inversions from
// blowing the policy up when near-stationary rollouts lack state diversity

// --- tiny fixed-size linear algebra ---

fn inv4(a: &M4, reg: f64) -> M4 {
    let mut m = *a;
    let scale = (0..4).map(|i| m[i][i].abs()).fold(1e-12, f64::max);
    for (i, row) in m.iter_mut().enumerate() {
        row[i] += reg * scale;
    }
    let mut inv = [[0.0; 4]; 4];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..4 {
        let piv = (col..4)
            .max_by(|&r, &s| m[r][col].abs().total_cmp(&m[s][col].abs()))
            .unwrap();
        m.swap(col, piv);
        inv.swap(col, piv);
        let d = m[col][col];
        if d.abs() < 1e-12 {
            continue;
        }
        for j in 0..4 {
            m[col][j] /= d;
            inv[col][j] /= d;
        }
        for r in 0..4 {
            if r != col {
                let f = m[r][col];
                for j in 0..4 {
                    m[r][j] -= f * m[col][j];
                    inv[r][j] -= f * inv[col][j];
                }
            }
        }
    }
    inv
}

/// Sample from N(0, sigma) via the analytic 2x2 Cholesky factor.
fn sample2(rng: &mut Rng, sigma: &M2) -> V2 {
    let l11 = sigma[0][0].max(1e-12).sqrt();
    let l21 = sigma[1][0] / l11;
    let l22 = (sigma[1][1] - l21 * l21).max(1e-12).sqrt();
    let (z1, z2) = (rng.normal(), rng.normal());
    [l11 * z1, l21 * z1 + l22 * z2]
}

/// Warm-started per-step policy carried between receding-horizon replans.
struct Policy {
    u: Vec<V2>,           // nominal controls
    gains: Vec<M24>,      // feedback K
    sigma_u: Vec<M2>,     // perturbation covariance
    sigma_tau: Vec<M6>,   // joint state-action covariance
    expected_next: State, // predicted next ego state, for warm-start reuse
    lambda_exp: f64,      // exploration magnitude (eq. 38)
    prev_cost: f64,       // noise-free rollout cost of the last generation
}

pub(crate) struct Pi2DdpPlanner {
    rng: Rng,
    policy: Option<Policy>,
}

impl Default for Pi2DdpPlanner {
    fn default() -> Self {
        Pi2DdpPlanner {
            rng: Rng(0x9E3779B97F4A7C15),
            policy: None,
        }
    }
}

impl Pi2DdpPlanner {
    /// Road-informed nominal: shared centerline tracking and traffic braking,
    /// rolled out over the horizon. The progress reward supplies throttle.
    fn init_policy(path: &Path, ego: State, ctx: &Context, sigma_init: M2) -> Policy {
        let u = centerline_follow_controls(ego, path, ctx, HORIZON)
            .into_iter()
            .map(|c| [c.acceleration, c.curvature])
            .collect();
        let mut sigma_tau = [[0.0; 6]; 6];
        for (i, row) in sigma_tau.iter_mut().enumerate().take(4) {
            row[i] = 1e-4;
        }
        for i in 0..2 {
            for j in 0..2 {
                sigma_tau[4 + i][4 + j] = sigma_init[i][j];
            }
        }
        Policy {
            u,
            gains: vec![[[0.0; 4]; 2]; HORIZON],
            sigma_u: vec![sigma_init; HORIZON],
            sigma_tau: vec![sigma_tau; HORIZON],
            expected_next: ego,
            lambda_exp: 1.0,
            prev_cost: f64::INFINITY,
        }
    }
}

impl Planner for Pi2DdpPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || ctx.path());
        // Offline calibration: 4 × 32 rollouts is about 100 ms.
        let total_rollouts = ctx.compute_budget.scale(GENERATIONS * ROLLOUTS, 8);
        // PI²-DDP needs more samples than its six-dimensional state/action
        // covariance; shed generations before dropping below that floor.
        let generations = GENERATIONS.min(total_rollouts / 8).max(1);
        let rollouts = (total_rollouts / generations).max(8);

        // road-informed sampling distribution: curvature exploration sized to
        // cover the lane width at the preview distance (d ≈ ½ κ L²)
        let preview = ego.speed.max(2.0) * ctx.road.dt * HORIZON as f64;
        let sigma_kappa = (8.0 * LANE_HALF_M / (preview * preview)).clamp(0.005, 0.05);
        let sigma_init: M2 = [[SIGMA_ACCEL * SIGMA_ACCEL, 0.0], [0.0, sigma_kappa * sigma_kappa]];
        // Composite-metric cost of being at `x` at tick `j`. A hard violation
        // (collision, or off the drivable area) becomes a large but
        // finite `constraints::HARD_VIOLATION_PENALTY · (1 + depth)` via
        // `HardConstraints::soft_point_cost` rather than `f64::INFINITY` — the
        // min/max-normalized rollout weighting below (eq. 12) can't divide by
        // an infinite range, and the depth-scaled escape slope gives the
        // rollout average a gradient back onto the road.
        let trajectory_cost = TrajectoryCost::new(path, ctx, ego.speed);
        let state_cost = |x: &State, j: usize| trajectory_cost.stage(x, Control::default(), j, None);
        let noise_free = |u: &[V2]| -> (Vec<State>, f64) {
            let mut x = ego;
            let mut xs = vec![ego];
            let mut cost = 0.0;
            for (j, &uj) in u.iter().enumerate() {
                x = world_step(
                    x,
                    Control {
                        acceleration: uj[0],
                        curvature: uj[1],
                    },
                    ctx.road.dt,
                );
                cost += state_cost(&x, j + 1);
                xs.push(x);
            }
            (xs, cost)
        };

        // warm start: shift the previous policy one step if the sim followed it
        // (custom seam: includes the road-informed re-init when the shift misses)
        let expected_next = self.policy.as_ref().map_or(ego, |p| p.expected_next);
        let mut pol = ctx.time("warm_start", || match take_warm(&mut self.policy, expected_next, ego) {
            Some(mut p) => {
                p.u.rotate_left(1);
                p.gains.rotate_left(1);
                p.sigma_u.rotate_left(1);
                p.sigma_tau.rotate_left(1);
                *p.u.last_mut().unwrap() = [0.0, 0.0];
                p.prev_cost = f64::INFINITY;
                p
            }
            _ => Self::init_policy(path, ego, ctx, sigma_init),
        });

        for generation in 0..generations {
            let (x_nom, _) = noise_free(&pol.u);

            // K perturbed rollouts with feedback (Algorithm 2, lines 3-10);
            // custom seam: the sampling workload
            let mut xs = vec![vec![ego; HORIZON + 1]; rollouts];
            let mut us = vec![vec![[0.0; 2]; HORIZON]; rollouts];
            let mut ctg = vec![vec![0.0; HORIZON + 1]; rollouts]; // cost-to-go
            ctx.time("rollouts", || {
                for k in 0..rollouts {
                    let mut x = ego;
                    for j in 0..HORIZON {
                        let dx: [f64; 4] = std::array::from_fn(|i| state(&x)[i] - state(&x_nom[j])[i]);
                        let eps = sample2(&mut self.rng, &pol.sigma_u[j]);
                        let kx: V2 = [
                            pol.gains[j][0].iter().zip(&dx).map(|(a, b)| a * b).sum(),
                            pol.gains[j][1].iter().zip(&dx).map(|(a, b)| a * b).sum(),
                        ];
                        let u = clamp_control(
                            Control::from([pol.u[j][0] + kx[0] + eps[0], pol.u[j][1] + kx[1] + eps[1]]),
                            x.speed,
                        );
                        us[k][j] = [u.acceleration, u.curvature];
                        x = world_step(x, u, ctx.road.dt);
                        ctg[k][j] = state_cost(&x, j + 1);
                        xs[k][j + 1] = x;
                    }
                    ctg[k][HORIZON] = 0.0;
                    for j in (0..HORIZON).rev() {
                        ctg[k][j] += ctg[k][j + 1]; // suffix sums (eq. 10)
                    }
                }
            });

            // diagnostic overlay: the final generation's sampled rollouts,
            // both as a point cloud and as trajectories
            if generation == generations - 1
                && let Some(diag) = ctx.diagnostics
            {
                for traj in &xs {
                    let pts: Vec<crate::simulation::Position> = traj.iter().map(Into::into).collect();
                    for &p in &pts {
                        diag.record_point(p);
                    }
                    diag.record_trajectory(pts);
                }
            }

            // reward-weighted updates per time step (Algorithm 2, lines 11-18);
            // custom seam: the DDP-style gradient extraction
            let snapshot = (pol.u.clone(), pol.gains.clone(), pol.sigma_u.clone());
            let mut new_x_nom = vec![ego; HORIZON];
            ctx.time("policy_update", || {
                for j in 0..HORIZON {
                    let (lo, hi) = ctg
                        .iter()
                        .map(|c| c[j])
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c), h.max(c)));
                    let p: Vec<f64> = ctg
                        .iter()
                        .map(|c| (-BETA * (c[j] - lo) / (hi - lo).max(1e-12)).exp())
                        .collect();
                    let psum: f64 = p.iter().sum();

                    // Σ_τ ← (1−α)Σ_τ + α Σₖ pₖ δτ δτᵀ, δτ relative to the nominal
                    let mut s_tau = [[0.0; 6]; 6];
                    for k in 0..rollouts {
                        let xn = &x_nom[j];
                        let xk = &xs[k][j];
                        let dtau = [
                            xk.position().x - xn.position().x,
                            xk.position().y - xn.position().y,
                            xk.pose.yaw - xn.pose.yaw,
                            xk.speed - xn.speed,
                            us[k][j][0] - pol.u[j][0],
                            us[k][j][1] - pol.u[j][1],
                        ];
                        let w = p[k] / psum;
                        for a in 0..6 {
                            for b in 0..6 {
                                s_tau[a][b] += w * dtau[a] * dtau[b];
                            }
                        }
                    }
                    for (row, s_row) in pol.sigma_tau[j].iter_mut().zip(&s_tau) {
                        for (v, s) in row.iter_mut().zip(s_row) {
                            *v = (1.0 - ALPHA) * *v + ALPHA * s;
                        }
                    }

                    // K = Σᵤₓ Σₓₓ†, k = Σₖ pₖ(δu − Kδx), Σᵤ = Σᵤᵤ − ΣᵤₓΣₓₓ†Σₓᵤ + λ_exp R⁻¹
                    let st = &pol.sigma_tau[j];
                    let mut s_xx: M4 = [[0.0; 4]; 4];
                    let mut s_ux: M24 = [[0.0; 4]; 2];
                    let mut s_uu: M2 = [[0.0; 2]; 2];
                    for a in 0..4 {
                        for b in 0..4 {
                            s_xx[a][b] = st[a][b];
                        }
                    }
                    for a in 0..2 {
                        for b in 0..4 {
                            s_ux[a][b] = st[4 + a][b];
                        }
                        for b in 0..2 {
                            s_uu[a][b] = st[4 + a][4 + b];
                        }
                    }
                    let xx_inv = inv4(&s_xx, LAMBDA_REG);
                    let mut gain: M24 = [[0.0; 4]; 2];
                    for a in 0..2 {
                        for b in 0..4 {
                            gain[a][b] = (0..4).map(|c| s_ux[a][c] * xx_inv[c][b]).sum();
                        }
                    }
                    let mut k_ff = [0.0; 2];
                    for k in 0..rollouts {
                        let dx: [f64; 4] = std::array::from_fn(|i| state(&xs[k][j])[i] - state(&x_nom[j])[i]);
                        let w = p[k] / psum;
                        for a in 0..2 {
                            let kdx: f64 = gain[a].iter().zip(&dx).map(|(g, d)| g * d).sum();
                            k_ff[a] += w * (us[k][j][a] - pol.u[j][a] - kdx);
                        }
                    }
                    for a in 0..2 {
                        for b in 0..2 {
                            let uxxxu: f64 = (0..4)
                                .map(|c| (0..4).map(|d| s_ux[a][c] * xx_inv[c][d] * s_ux[b][d]).sum::<f64>())
                                .sum();
                            pol.sigma_u[j][a][b] =
                                s_uu[a][b] - uxxxu + if a == b { pol.lambda_exp * sigma_init[a][a] } else { 0.0 };
                        }
                    }
                    // PSD guard: the Schur complement of a noisy Σ_τ estimate can
                    // lose definiteness; fall back to the road-informed prior
                    let su = &pol.sigma_u[j];
                    if su[0][0] <= 0.0 || su[1][1] <= 0.0 || su[0][0] * su[1][1] <= su[0][1] * su[1][0] {
                        pol.sigma_u[j] = [
                            [pol.lambda_exp.max(0.05) * sigma_init[0][0], 0.0],
                            [0.0, pol.lambda_exp.max(0.05) * sigma_init[1][1]],
                        ];
                    }
                    pol.gains[j] = gain;
                    // nominal for the next generation: rollout mean plus feedforward
                    for a in 0..2 {
                        pol.u[j][a] = us.iter().map(|u| u[j][a]).sum::<f64>() / rollouts as f64 + k_ff[a];
                    }
                    let mean = |f: fn(&State) -> f64| xs.iter().map(|x| f(&x[j])).sum::<f64>() / rollouts as f64;
                    new_x_nom[j] = State::from((
                        Position::new(mean(|s| s.position().x), mean(|s| s.position().y)),
                        mean(|s| s.pose.yaw),
                        mean(|s| s.speed),
                    ));
                }
            });

            // close the loop: execute the updated policy noise-free
            let mut x = ego;
            let mut u_exec = Vec::with_capacity(HORIZON);
            for (j, nom) in new_x_nom.iter().enumerate() {
                let dx: [f64; 4] = std::array::from_fn(|i| state(&x)[i] - state(nom)[i]);
                let u = clamp_control(
                    Control::from([
                        pol.u[j][0] + pol.gains[j][0].iter().zip(&dx).map(|(a, b)| a * b).sum::<f64>(),
                        pol.u[j][1] + pol.gains[j][1].iter().zip(&dx).map(|(a, b)| a * b).sum::<f64>(),
                    ]),
                    x.speed,
                );
                u_exec.push([u.acceleration, u.curvature]);
                x = world_step(x, u, ctx.road.dt);
            }
            let (_, cost) = noise_free(&u_exec);

            // trust region on exploration (eq. 38), adaptive v2: reject a
            // worse generation outright
            if cost > pol.prev_cost {
                (pol.u, pol.gains, pol.sigma_u) = snapshot;
                pol.lambda_exp = (0.9 * pol.lambda_exp).max(1e-3);
            } else {
                if cost < 0.9 * pol.prev_cost {
                    pol.lambda_exp = (1.1 * pol.lambda_exp).min(1.0);
                }
                pol.u = u_exec;
                pol.prev_cost = cost;
            }
        }

        // Keep the road-model base policy when optimization makes the
        // noise-free rollout worse.
        let base = Self::init_policy(path, ego, ctx, sigma_init);
        let (_, base_cost) = noise_free(&base.u);
        let (_, opt_cost) = noise_free(&pol.u);
        if !opt_cost.is_finite() || opt_cost > base_cost {
            pol.u = base.u;
        }

        let out: Vec<Control> = pol.u.iter().copied().map(Control::from).collect();
        pol.expected_next = world_step(ego, out[0], ctx.road.dt);
        self.policy = Some(pol);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::test_run;

    fn run(ego: State, actors: &[State], ticks: usize) -> Vec<State> {
        test_run(&mut Pi2DdpPlanner::default(), ego, actors, ticks)
    }

    #[test]
    fn stays_on_road_and_accelerates() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        );
        let trace = run(ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 5.5, "offset {}", end.position().y);
        assert!(end.speed > ego.speed, "speed {}", end.speed);
    }

    #[test]
    fn stays_safe_behind_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let trace = run(ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.position().x - 40.0).hypot(s.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(
            trace
                .iter()
                .all(|s| s.position().x.is_finite() && s.position().y.is_finite())
        );
        assert!(trace.last().unwrap().position().x > 20.0, "gave up too early");
    }

    /// Regression: near-stationary rollouts once produced a singular Σxx,
    /// exploding feedback gains, and NaN states after ~9 s.
    #[test]
    fn stays_finite_and_safe_over_long_rollout() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(60.0, 0.0), 0.0),
            0.0,
        );
        let trace = run(ego, &[obstacle], 200);
        let min_gap = trace
            .iter()
            .map(|s| (s.position().x - 60.0).hypot(s.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(
            trace
                .iter()
                .all(|s| s.position().x.is_finite() && s.position().y.is_finite())
        );
        assert!(min_gap > 2.0, "min gap {min_gap}");
        let max_offset = trace.iter().map(|s| s.position().y.abs()).fold(0.0, f64::max);
        assert!(max_offset < 5.5, "left the road, max offset {max_offset}");
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
        Pi2DdpPlanner::default().plan(ego, &ctx);
        let data = diag.take();
        // the final generation's ROLLOUTS sampled trajectories
        assert_eq!(data.trajectories.len(), ROLLOUTS);
        assert!(data.trajectories.iter().all(|t| t.len() == HORIZON + 1));
        // every state along every rollout, flattened
        assert_eq!(data.points.len(), ROLLOUTS * (HORIZON + 1));
    }
}
