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
//! control sequence tracks the lane centerline (pure pursuit + speed hold)
//! and the initial curvature variance is sized so sampled trajectories span
//! the lane width at the preview distance.

use crate::Rng;
use crate::planning::cost::{self, Sample};
use crate::planning::{Context, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State, step};
use crate::wrap_angle;

// 10 s at the simulator's 0.1 s tick rate (see planning::PLANNING_HORIZON_S).
const HORIZON: usize = 100;
const ROLLOUTS: usize = 32; // K in the paper; K > n + m with margin
const GENERATIONS: usize = 4;
const BETA: f64 = 10.0; // baseline sensitivity (eq. 12)
const ALPHA: f64 = 0.5; // covariance damping (eq. 36)
const LAMBDA_REG: f64 = 1e-3; // inverse regularization heuristic
const SIGMA_ACCEL: f64 = 1.0; // [m/s²] exploration std
const LANE_HALF_M: f64 = 1.75;
// physical actuation limits; also keep near-singular Σₓₓ inversions from
// blowing the policy up when near-stationary rollouts lack state diversity
const ACCEL_LIMIT: f64 = 5.0;
const KAPPA_LIMIT: f64 = 0.2;

type V2 = [f64; 2];
type V4 = [f64; 4];
type M2 = [[f64; 2]; 2];
type M24 = [[f64; 4]; 2];
type M4 = [[f64; 4]; 4];
type M6 = [[f64; 6]; 6];

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

fn xv(s: &State) -> V4 {
    [s.x, s.y, s.yaw, s.speed]
}

fn clamp_u(u: V2) -> V2 {
    [
        u[0].clamp(-ACCEL_LIMIT, ACCEL_LIMIT),
        u[1].clamp(-KAPPA_LIMIT, KAPPA_LIMIT),
    ]
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

pub struct Pi2DdpPlanner {
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
    /// Road-informed nominal: pure pursuit toward the centerline plus a
    /// proportional speed hold, rolled out over the horizon.
    fn init_policy(path: &Path, ego: State, ctx: &Context, sigma_init: M2) -> Policy {
        let mut x = ego;
        let mut u = Vec::with_capacity(HORIZON);
        for _ in 0..HORIZON {
            let lookahead = (2.0 * x.speed).max(8.0);
            let (s, _) = path.project([x.x, x.y]);
            let (target, _) = path.pose_at(s + lookahead);
            let heading_err = (target[1] - x.y).atan2(target[0] - x.x) - x.yaw;
            let curvature = 2.0 * heading_err.sin() / lookahead;
            let accel = (0.5 * (ctx.road.target_speed - x.speed)).clamp(-2.0, 1.5);
            let c = Control { accel, curvature };
            u.push([accel, curvature]);
            x = step(x, c, ctx.road.dt);
        }
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
        let path = ctx.time("route", || Path::new(&ctx.road.centerline));

        // road-informed sampling distribution: curvature exploration sized to
        // cover the lane width at the preview distance (d ≈ ½ κ L²)
        let preview = ego.speed.max(2.0) * ctx.road.dt * HORIZON as f64;
        let sigma_kappa = (8.0 * LANE_HALF_M / (preview * preview)).clamp(0.005, 0.05);
        let sigma_init: M2 = [
            [SIGMA_ACCEL * SIGMA_ACCEL, 0.0],
            [0.0, sigma_kappa * sigma_kappa],
        ];
        // linear solvability: control cost is the inverse of the exploration
        let r_diag = [1.0 / sigma_init[0][0], 1.0 / sigma_init[1][1]];

        // Shared cost of being at `x` at tick `j` having just applied
        // `(accel, curvature)` — the road/actor/comfort terms every search
        // planner now agrees on (`cost::point_cost`), plus a planner-specific
        // centerline-pull term (no metric measures "distance from
        // centerline" directly — it's a structural bias toward the lane the
        // lattice and RRT* also keep as their own terms, not part of the
        // shared cost). `curvature`/`accel` are exactly the control just
        // applied: this kinematic model defines them as the instantaneous
        // curvature and acceleration, so there's nothing to estimate. A hard
        // violation (collision, or off the drivable area) becomes
        // `cost::HARD_VIOLATION_PENALTY`, a large but finite number, rather
        // than `f64::INFINITY` — the min/max-normalized rollout weighting
        // below (eq. 12) can't divide by an infinite range. The terminal
        // call (no control has been applied yet) passes zero for both, so it
        // prices position/speed but not comfort.
        let state_cost = |x: &State, curvature: f64, accel: f64, j: usize| {
            let (s, d) = path.project([x.x, x.y]);
            let (_, lane_yaw) = path.pose_at(s);
            let sample = Sample {
                xy: [x.x, x.y],
                lateral: d,
                heading_err: wrap_angle(x.yaw - lane_yaw),
                speed: x.speed,
                curvature,
                accel,
                t: j as f64 * ctx.road.dt,
            };
            let c = 0.5 * d * d
                + ctx.time("cost", || {
                    cost::point_cost(&sample, ctx.road.target_speed, ctx.road.half_width, ctx.actors)
                });
            if c.is_infinite() {
                cost::HARD_VIOLATION_PENALTY
            } else {
                c
            }
        };
        let running = |x: &State, u: &V2, j: usize| {
            state_cost(x, u[1], u[0], j) + 0.5 * (r_diag[0] * u[0] * u[0] + r_diag[1] * u[1] * u[1])
        };
        let noise_free = |u: &[V2]| -> (Vec<State>, f64) {
            let mut x = ego;
            let mut xs = vec![ego];
            let mut cost = 0.0;
            for (j, &uj) in u.iter().enumerate() {
                cost += running(&x, &uj, j);
                x = step(
                    x,
                    Control {
                        accel: uj[0],
                        curvature: uj[1],
                    },
                    ctx.road.dt,
                );
                xs.push(x);
            }
            (xs, cost + 5.0 * state_cost(&x, 0.0, 0.0, HORIZON))
        };

        // warm start: shift the previous policy one step if the sim followed it
        // (custom seam: includes the road-informed re-init when the shift misses)
        let mut pol = ctx.time("warm_start", || match self.policy.take() {
            Some(mut p) if (p.expected_next.x - ego.x).hypot(p.expected_next.y - ego.y) < 1.0 => {
                p.u.rotate_left(1);
                p.gains.rotate_left(1);
                p.sigma_u.rotate_left(1);
                p.sigma_tau.rotate_left(1);
                *p.u.last_mut().unwrap() = [0.0, 0.0];
                p.prev_cost = f64::INFINITY;
                p
            }
            _ => Self::init_policy(&path, ego, ctx, sigma_init),
        });

        for generation in 0..GENERATIONS {
            let (x_nom, _) = noise_free(&pol.u);

            // K perturbed rollouts with feedback (Algorithm 2, lines 3-10);
            // custom seam: the sampling workload
            let mut xs = vec![vec![ego; HORIZON + 1]; ROLLOUTS];
            let mut us = vec![vec![[0.0; 2]; HORIZON]; ROLLOUTS];
            let mut ctg = vec![vec![0.0; HORIZON + 1]; ROLLOUTS]; // cost-to-go
            ctx.time("rollouts", || {
                for k in 0..ROLLOUTS {
                    let mut x = ego;
                    for j in 0..HORIZON {
                        let dx = [
                            x.x - x_nom[j].x,
                            x.y - x_nom[j].y,
                            x.yaw - x_nom[j].yaw,
                            x.speed - x_nom[j].speed,
                        ];
                        let eps = sample2(&mut self.rng, &pol.sigma_u[j]);
                        let kx: V2 = [
                            pol.gains[j][0].iter().zip(&dx).map(|(a, b)| a * b).sum(),
                            pol.gains[j][1].iter().zip(&dx).map(|(a, b)| a * b).sum(),
                        ];
                        let u =
                            clamp_u([pol.u[j][0] + kx[0] + eps[0], pol.u[j][1] + kx[1] + eps[1]]);
                        ctg[k][j] = running(&x, &u, j);
                        us[k][j] = u;
                        x = step(
                            x,
                            Control {
                                accel: u[0],
                                curvature: u[1],
                            },
                            ctx.road.dt,
                        );
                        xs[k][j + 1] = x;
                    }
                    ctg[k][HORIZON] = 5.0 * state_cost(&x, 0.0, 0.0, HORIZON);
                    for j in (0..HORIZON).rev() {
                        ctg[k][j] += ctg[k][j + 1]; // suffix sums (eq. 10)
                    }
                }
            });

            // diagnostic overlay: the final generation's sampled rollouts,
            // both as a point cloud and as trajectories
            if generation == GENERATIONS - 1
                && let Some(diag) = ctx.diagnostics
            {
                for traj in &xs {
                    let pts: Vec<[f64; 2]> = traj.iter().map(|s| [s.x, s.y]).collect();
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
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| {
                            (l.min(c), h.max(c))
                        });
                    let p: Vec<f64> = ctg
                        .iter()
                        .map(|c| (-BETA * (c[j] - lo) / (hi - lo).max(1e-12)).exp())
                        .collect();
                    let psum: f64 = p.iter().sum();

                    // Σ_τ ← (1−α)Σ_τ + α Σₖ pₖ δτ δτᵀ, δτ relative to the nominal
                    let mut s_tau = [[0.0; 6]; 6];
                    for k in 0..ROLLOUTS {
                        let xn = xv(&x_nom[j]);
                        let xk = xv(&xs[k][j]);
                        let dtau = [
                            xk[0] - xn[0],
                            xk[1] - xn[1],
                            xk[2] - xn[2],
                            xk[3] - xn[3],
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
                    for k in 0..ROLLOUTS {
                        let dx = [
                            xs[k][j].x - x_nom[j].x,
                            xs[k][j].y - x_nom[j].y,
                            xs[k][j].yaw - x_nom[j].yaw,
                            xs[k][j].speed - x_nom[j].speed,
                        ];
                        let w = p[k] / psum;
                        for a in 0..2 {
                            let kdx: f64 = gain[a].iter().zip(&dx).map(|(g, d)| g * d).sum();
                            k_ff[a] += w * (us[k][j][a] - pol.u[j][a] - kdx);
                        }
                    }
                    for a in 0..2 {
                        for b in 0..2 {
                            let uxxxu: f64 = (0..4)
                                .map(|c| {
                                    (0..4)
                                        .map(|d| s_ux[a][c] * xx_inv[c][d] * s_ux[b][d])
                                        .sum::<f64>()
                                })
                                .sum();
                            pol.sigma_u[j][a][b] = s_uu[a][b] - uxxxu
                                + if a == b {
                                    pol.lambda_exp * sigma_init[a][a]
                                } else {
                                    0.0
                                };
                        }
                    }
                    // PSD guard: the Schur complement of a noisy Σ_τ estimate can
                    // lose definiteness; fall back to the road-informed prior
                    let su = &pol.sigma_u[j];
                    if su[0][0] <= 0.0
                        || su[1][1] <= 0.0
                        || su[0][0] * su[1][1] <= su[0][1] * su[1][0]
                    {
                        pol.sigma_u[j] = [
                            [pol.lambda_exp.max(0.05) * sigma_init[0][0], 0.0],
                            [0.0, pol.lambda_exp.max(0.05) * sigma_init[1][1]],
                        ];
                    }
                    pol.gains[j] = gain;
                    // nominal for the next generation: rollout mean plus feedforward
                    for a in 0..2 {
                        pol.u[j][a] =
                            us.iter().map(|u| u[j][a]).sum::<f64>() / ROLLOUTS as f64 + k_ff[a];
                    }
                    let mean = |f: fn(&State) -> f64| {
                        xs.iter().map(|x| f(&x[j])).sum::<f64>() / ROLLOUTS as f64
                    };
                    new_x_nom[j] = State {
                        x: mean(|s| s.x),
                        y: mean(|s| s.y),
                        yaw: mean(|s| s.yaw),
                        speed: mean(|s| s.speed),
                    };
                }
            });

            // close the loop: execute the updated policy noise-free
            let mut x = ego;
            let mut u_exec = Vec::with_capacity(HORIZON);
            for (j, nom) in new_x_nom.iter().enumerate() {
                let dx = [
                    x.x - nom.x,
                    x.y - nom.y,
                    x.yaw - nom.yaw,
                    x.speed - nom.speed,
                ];
                let u = clamp_u([
                    pol.u[j][0]
                        + pol.gains[j][0]
                            .iter()
                            .zip(&dx)
                            .map(|(a, b)| a * b)
                            .sum::<f64>(),
                    pol.u[j][1]
                        + pol.gains[j][1]
                            .iter()
                            .zip(&dx)
                            .map(|(a, b)| a * b)
                            .sum::<f64>(),
                ]);
                u_exec.push(u);
                x = step(
                    x,
                    Control {
                        accel: u[0],
                        curvature: u[1],
                    },
                    ctx.road.dt,
                );
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

        let out: Vec<Control> = pol
            .u
            .iter()
            .map(|u| Control {
                accel: u[0],
                curvature: u[1],
            })
            .collect();
        pol.expected_next = step(ego, out[0], ctx.road.dt);
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
    fn tracks_centerline_and_speed() {
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let trace = run(ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 1.0, "offset {}", end.y);
        assert!((end.speed - 10.0).abs() < 2.0, "speed {}", end.speed);
    }

    #[test]
    fn avoids_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = run(ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(
            trace.last().unwrap().x > 50.0,
            "did not pass, x {}",
            trace.last().unwrap().x
        );
    }

    /// Regression: near-stationary rollouts once produced a singular Σxx,
    /// exploding feedback gains, and NaN states after ~9 s.
    #[test]
    fn stays_finite_and_safe_over_full_scenario() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 60.0,
            ..Default::default()
        };
        let trace = run(ego, &[obstacle], 200);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 60.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        assert!(trace.iter().all(|s| s.x.is_finite() && s.y.is_finite()));
        assert!(min_gap > 2.0, "min gap {min_gap}");
        let max_offset = trace.iter().map(|s| s.y.abs()).fold(0.0, f64::max);
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
