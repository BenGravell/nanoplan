//! The treetop iLQR solver (`ilqr/{solver,backward_pass,solver_settings}.h`),
//! exposed standalone as [`IlqrPlanner`] and reused by
//! [`TreetopPlanner`](super::TreetopPlanner) to optimize the tree's path
//! candidates.
//!
//! iLQR alternates a **backward pass** — dynamic programming over
//! linearized dynamics and a quadratic expansion of the cost, producing an
//! affine feedback policy `u = u_ref + scale·k + K·(x − x_ref)` — with a
//! **forward pass** rolling that policy out closed-loop, accepting the new
//! trajectory only if the realized cost drop is a reasonable fraction of
//! the expansion's prediction (treetop's feedforward-gain scaling search,
//! backtracking `scale` by 0.2 up to 8 times). A regularization term on
//! `Q_uu`, scaled by the gradient norm `|Q_u|` (treetop's gradient-norm
//! scaling), adapts by the usual schedule: decrease on an immediately
//! accepted step, increase on a backtracked one, surge on a rejected one.
//!
//! ## Finite differences, not analytic derivatives — by design
//!
//! treetop's `Loss` carries ~200 lines of hand-derived gradients and
//! Hessians (`loss.h`), and its `Dynamics::jacobian` is closed-form.
//! nanoplan deliberately provides neither: its shared metric objective
//! ([`crate::planning::constraints::HardConstraints`]) and dynamics ([`world_step`]) are black-box scalars
//! (see the "no analytic derivatives" discussion in
//! `src/planning/README.md`). So this port differentiates **numerically**:
//! central finite differences for the state-cost gradient/Hessian
//! ([`stage_derivs`]) and the dynamics Jacobians
//! ([`dynamics_jacobian`]) — the planners still consume exactly the same
//! scalar interfaces every sampling planner does, they just probe them a
//! few dozen times per timestep instead of once. FD Hessians of a
//! piecewise cost are noisy near hinge corners; the backward pass
//! symmetrizes what it gets, and the `Q_uu` positive-definiteness check
//! (treetop asserts PD "by construction"; a finite-differenced cost can't
//! promise that) rejects a pass and surges regularization instead of
//! factorizing garbage.
//!
//! ## The optimal control problem ([`Ocp`])
//!
//! The running cost is the production composite-metric objective used by
//! every search planner. One adaptation makes it usable
//! under an optimizer that *differentiates* it rather than compares it:
//!
//! - **Hard violations get an escape slope.** `point_cost` returns
//!   `f64::INFINITY` on collision/off-road; optimizers need a finite
//!   replacement because their statistics and finite differences can't
//!   absorb an infinity. A *flat* plateau is equally useless to a
//!   finite-difference gradient (zero slope everywhere inside the
//!   violation), so this planner prices a violation as
//!   `HARD_VIOLATION_PENALTY · (1 + depth)` where `depth` is how far
//!   inside the violation the sample sits — same cliff at the boundary,
//!   but with a gradient pointing back out of it.
//!
//! **Seams**: `route`, `warm_start`, `optimize` (the whole solve), with
//! `derivs` (all backward-pass FD work), `fd_cost`, `fd_dynamics`, and
//! `rollout` (the line-search forward passes) nested inside, and `extract`.
//! Unlike the other search planners there is no per-call `cost` seam for
//! iLQR's FD probes: timing each shared-cost probe individually would cost
//! more than the call, so `fd_cost` is the aggregated useful split.
//!
//! **Diagnostics**: the initial-guess rollout and the optimized trajectory
//! as polylines (what the optimizer bought), the optimized states as
//! points.

use super::{TICKS, take_warm};
use crate::common::linalg::{mat_add, mat_mul, mat_vec, transpose, vec_add};
use crate::common::matrix::{M4, M22, M24, M42};
use crate::common::measure::dot;
use crate::common::state_control::state;
use crate::common::vector::{V2, V4};
use crate::planning::search_tree::{
    centerline_follow_controls, repeat_last_controls, rollout_constrained,
};
use crate::planning::{Context, Planner, TrajectoryCost};
use crate::prediction::predict;
use crate::simulation::{Control, State, world_step};
use crate::track::Path;

// ---- Solver settings (treetop `solver_settings.h`) ------------------------

const COST_CHANGE_TOL: f64 = 1e-4;
const COST_CHANGE_RATIO_MIN: f64 = 0.01;
const REG_INIT: f64 = 1.0;
const REG_MIN: f64 = 1e-8;
const REG_MAX: f64 = 1e8;
const REG_SURGE: f64 = 10.0;
const REG_INCREASE: f64 = 2.0;
const REG_DECREASE: f64 = 0.5;
const MAX_FFGS_ATTEMPTS: usize = 8;
const FFGS_DECREASE: f64 = 0.2;

// ---- The optimal control problem ------------------------------------------

/// One trajectory-optimization problem: minimize composite-metric cost from
/// a fixed start.
pub(crate) struct Ocp<'a, 'b> {
    pub(crate) path: &'a Path,
    /// treetop `Problem::initial_state`: where every rollout starts.
    pub(crate) start: State,
    pub(crate) ctx: &'a Context<'b>,
}

impl Ocp<'_, '_> {
    /// Running cost of being at `x` at tick `t` having applied `u`.
    /// `s_hint` narrows the Frenet projection during FD probing, where the
    /// state moves by ±[`H_COST`] around a known station.
    fn stage_cost(&self, x: &State, u: &Control, t: usize, s_hint: Option<f64>) -> f64 {
        TrajectoryCost::new(self.path, self.ctx, self.start.speed).stage(x, *u, t, s_hint)
    }

    fn stage_cost_with_predicted_actors(
        &self,
        x: &State,
        u: &Control,
        t: usize,
        s_hint: Option<f64>,
        predicted_actors: &[State],
    ) -> f64 {
        TrajectoryCost::new(self.path, self.ctx, self.start.speed).stage_with_predicted_actors(
            x,
            *u,
            t,
            s_hint,
            predicted_actors,
        )
    }

    fn terminal_cost(&self, x: &State) -> f64 {
        TrajectoryCost::new(self.path, self.ctx, self.start.speed).stage(
            x,
            Control::default(),
            TICKS,
            None,
        )
    }

    /// Total cost of a rolled-out trajectory (treetop `Loss::totalValue`).
    pub(crate) fn traj_cost(&self, xs: &[State], us: &[Control]) -> f64 {
        us.iter()
            .enumerate()
            .map(|(t, u)| self.stage_cost(&xs[t], u, t, None))
            .sum::<f64>()
            + self.terminal_cost(xs.last().unwrap())
    }
}

// ---- Finite-difference derivatives ----------------------------------------

/// FD step for cost derivatives. State and control coordinates are
/// O(0.01-10) here, so one step suits them all;
/// ~eps^(1/4) is the classic sweet spot for second differences.
const H_COST: f64 = 1e-3;

/// FD step for dynamics Jacobians (first differences only, so a smaller
/// step is optimal).
const H_DYN: f64 = 1e-5;

/// Quadratic expansion of the running cost at one `(x, u)` — treetop's
/// `Loss::gradientAndHessian`, numerically differentiated over state and
/// control together.
struct StageDerivs {
    lx: V4,
    lu: V2,
    lxx: M4,
    lxu: M42,
    luu: M22,
}

fn stage_derivs(ocp: &Ocp, x: &State, u: &Control, t: usize) -> StageDerivs {
    // one projection of the unperturbed state anchors every probe's
    // Frenet lookup (the probes move by ±H_COST, far less than the window)
    let s_hint = ocp.path.project(x.position()).0;
    let t_s = t as f64 * ocp.ctx.road.dt;
    let predicted_actors: Vec<State> = ocp
        .ctx
        .actors
        .iter()
        .map(|a| predict(a, ocp.path, t_s))
        .collect();
    let eval = |z: [f64; 6]| {
        ocp.stage_cost_with_predicted_actors(
            &State::from([z[0], z[1], z[2], z[3]]),
            &Control {
                acceleration: z[4],
                curvature: z[5],
            },
            t,
            Some(s_hint),
            &predicted_actors,
        )
    };
    let z = [x.x, x.y, x.yaw, x.speed, u.acceleration, u.curvature];
    let (grad, hess) = fd_grad_hess(&eval, z);
    StageDerivs {
        lx: [grad[0], grad[1], grad[2], grad[3]],
        lu: [grad[4], grad[5]],
        lxx: std::array::from_fn(|i| std::array::from_fn(|j| hess[i][j])),
        lxu: std::array::from_fn(|i| [hess[i][4], hess[i][5]]),
        luu: [[hess[4][4], hess[4][5]], [hess[5][4], hess[5][5]]],
    }
}

fn terminal_derivs(ocp: &Ocp, x: &State) -> (V4, M4) {
    fd_grad_hess(&|z| ocp.terminal_cost(&State::from(z)), state(x))
}

/// Central-difference gradient and Hessian of a black-box scalar. The
/// Hessian is symmetric by construction (each cross term is computed once
/// from the four corner probes and mirrored).
fn fd_grad_hess<const N: usize>(
    f: &impl Fn([f64; N]) -> f64,
    z: [f64; N],
) -> ([f64; N], [[f64; N]; N]) {
    let f0 = f(z);
    let mut fp = [0.0; N];
    let mut fm = [0.0; N];
    for i in 0..N {
        let mut zp = z;
        zp[i] += H_COST;
        fp[i] = f(zp);
        zp[i] = z[i] - H_COST;
        fm[i] = f(zp);
    }
    let grad = std::array::from_fn(|i| (fp[i] - fm[i]) / (2.0 * H_COST));
    let mut hess = [[0.0; N]; N];
    for i in 0..N {
        hess[i][i] = (fp[i] - 2.0 * f0 + fm[i]) / (H_COST * H_COST);
        for j in (i + 1)..N {
            let mut zz = z;
            zz[i] += H_COST;
            zz[j] += H_COST;
            let fpp = f(zz);
            zz[j] = z[j] - H_COST;
            let fpm = f(zz);
            zz[i] = z[i] - H_COST;
            let fmm = f(zz);
            zz[j] = z[j] + H_COST;
            let fmp = f(zz);
            let h = (fpp - fpm - fmp + fmm) / (4.0 * H_COST * H_COST);
            hess[i][j] = h;
            hess[j][i] = h;
        }
    }
    (grad, hess)
}

/// Dynamics Jacobians `A = ∂step/∂x`, `B = ∂step/∂u` by central finite
/// differences on [`world_step`] — treetop's closed-form `Dynamics::jacobian`,
/// derived numerically instead.
fn dynamics_jacobian(x: &State, u: &Control, dt: f64) -> (M4, M42) {
    let mut a = [[0.0; 4]; 4];
    for j in 0..4 {
        let mut zp = state(x);
        zp[j] += H_DYN;
        let mut zm = state(x);
        zm[j] -= H_DYN;
        let sp = state(&world_step(State::from(zp), *u, dt));
        let sm = state(&world_step(State::from(zm), *u, dt));
        for i in 0..4 {
            a[i][j] = (sp[i] - sm[i]) / (2.0 * H_DYN);
        }
    }
    let col = |up: Control, um: Control| -> V4 {
        let sp = state(&world_step(*x, up, dt));
        let sm = state(&world_step(*x, um, dt));
        std::array::from_fn(|i| (sp[i] - sm[i]) / (2.0 * H_DYN))
    };
    let ca = col(
        Control {
            acceleration: u.acceleration + H_DYN,
            ..*u
        },
        Control {
            acceleration: u.acceleration - H_DYN,
            ..*u
        },
    );
    let ck = col(
        Control {
            curvature: u.curvature + H_DYN,
            ..*u
        },
        Control {
            curvature: u.curvature - H_DYN,
            ..*u
        },
    );
    let b: M42 = std::array::from_fn(|i| [ca[i], ck[i]]);
    (a, b)
}

// ---- The solver ------------------------------------------------------------

/// The affine policy of one timestep: feedforward `k`, feedback `K`.
#[derive(Clone, Copy)]
struct Gains {
    k: V2,
    kk: M24,
}

/// treetop's `ExpectedCostChange`: the expansion's prediction of the cost
/// drop at feedforward scale `s`, `-(s·term1 + ½·s²·term2)`, against which
/// the line search judges a candidate trajectory.
struct Ecc {
    term1: f64,
    term2: f64,
}

impl Ecc {
    fn evaluate(&self, scale: f64) -> f64 {
        -(scale * self.term1 + 0.5 * scale * scale * self.term2)
    }
}

/// A solved trajectory (treetop's `Solution`, minus the policy — nanoplan
/// executes one control per tick and replans, so only the action sequence
/// leaves the solver).
#[derive(Clone)]
pub(crate) struct Solution {
    pub(crate) states: Vec<State>,
    pub(crate) controls: Vec<Control>,
    pub(crate) cost: f64,
}

/// One backward pass (treetop `BackwardPassRunner::run`): from the
/// terminal expansion, dynamic-programming back through FD cost expansions
/// and FD dynamics Jacobians, solving the regularized 2×2 `Q_uu` for the
/// gains at each step. `None` if `Q_uu` isn't positive definite anywhere
/// (or a derivative went non-finite) — the caller surges regularization
/// and retries, where treetop asserts PD by construction of its analytic
/// loss.
fn backward(ocp: &Ocp, xs: &[State], us: &[Control], reg: f64) -> Option<(Vec<Gains>, Ecc)> {
    let n = us.len();
    let (mut vx, mut vxx) = ocp.ctx.time("fd_cost", || terminal_derivs(ocp, &xs[n]));
    let mut gains = vec![
        Gains {
            k: [0.0; 2],
            kk: [[0.0; 4]; 2]
        };
        n
    ];
    let mut ecc = Ecc {
        term1: 0.0,
        term2: 0.0,
    };

    for t in (0..n).rev() {
        let sd = ocp
            .ctx
            .time("fd_cost", || stage_derivs(ocp, &xs[t], &us[t], t));
        let (a, b) = ocp.ctx.time("fd_dynamics", || {
            dynamics_jacobian(&xs[t], &us[t], ocp.ctx.road.dt)
        });
        let at = transpose(&a);
        let bt = transpose(&b);
        let atv: M4 = mat_mul(&at, &vxx);
        let btv: M24 = mat_mul(&bt, &vxx);

        let qx: V4 = vec_add(sd.lx, mat_vec(&at, &vx));
        let qu: V2 = vec_add(sd.lu, mat_vec(&bt, &vx));
        let qxx: M4 = mat_add(sd.lxx, mat_mul(&atv, &a));
        let qxu: M42 = mat_add(sd.lxu, mat_mul(&atv, &b));
        let quu: M22 = mat_add(sd.luu, mat_mul(&btv, &b));

        // regularize with treetop's gradient-norm scaling, then check PD
        // (2×2: positive leading element and determinant)
        let r = reg * qu[0].hypot(qu[1]);
        let m = [[quu[0][0] + r, quu[0][1]], [quu[1][0], quu[1][1] + r]];
        let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        if !(m[0][0] > 0.0 && det > 0.0 && det.is_finite()) {
            return None;
        }
        let inv: M22 = [
            [m[1][1] / det, -m[0][1] / det],
            [-m[1][0] / det, m[0][0] / det],
        ];

        let k: V2 = mat_vec(&inv, &qu).map(|v| -v);
        let kk: M24 = mat_mul(&inv, &transpose(&qxu)).map(|row| row.map(|v| -v));

        // value-function update (treetop `updateV`), symmetrized to shed
        // FD asymmetry before it compounds across 100 steps
        let kt: M42 = transpose(&kk);
        let w: M42 = mat_add(mat_mul(&kt, &quu), qxu);
        vx = vec_add(qx, vec_add(mat_vec(&w, &k), mat_vec(&kt, &qu)));
        let vxx_new: M4 = mat_add(
            qxx,
            mat_add(mat_mul(&w, &kk), mat_mul(&kt, &transpose(&qxu))),
        );
        vxx =
            std::array::from_fn(|i| std::array::from_fn(|j| 0.5 * (vxx_new[i][j] + vxx_new[j][i])));

        ecc.term1 += dot(k, qu);
        ecc.term2 += dot(k, mat_vec(&quu, &k));
        gains[t] = Gains { k, kk };

        if !vx.iter().all(|v| v.is_finite()) {
            return None;
        }
    }
    Some((gains, ecc))
}

/// One forward pass (treetop `rolloutClosedLoop`): roll the affine policy
/// out from the reference trajectory's start, unclamped as in treetop (the
/// final trajectory is re-realized under constraints by [`solve`]), and
/// price it. `None` if the rollout diverges to non-finite anywhere — the
/// line search treats that as a rejected scale.
fn forward(
    ocp: &Ocp,
    xs: &[State],
    us: &[Control],
    gains: &[Gains],
    scale: f64,
) -> Option<(Vec<State>, Vec<Control>, f64)> {
    let n = us.len();
    let mut nxs = Vec::with_capacity(n + 1);
    let mut nus = Vec::with_capacity(n);
    let mut x = xs[0];
    nxs.push(x);
    let mut cost_total = 0.0;
    for t in 0..n {
        let dx: V4 = std::array::from_fn(|i| state(&x)[i] - state(&xs[t])[i]);
        let fb = mat_vec(&gains[t].kk, &dx);
        let u = Control {
            acceleration: us[t].acceleration + scale * gains[t].k[0] + fb[0],
            curvature: us[t].curvature + scale * gains[t].k[1] + fb[1],
        };
        cost_total += ocp.stage_cost(&x, &u, t, None);
        x = world_step(x, u, ocp.ctx.road.dt);
        if !(x.x.is_finite() && x.y.is_finite() && x.yaw.is_finite() && x.speed.is_finite()) {
            return None;
        }
        nxs.push(x);
        nus.push(u);
    }
    cost_total += ocp.terminal_cost(&x);
    cost_total.is_finite().then_some((nxs, nus, cost_total))
}

/// Solve the problem from an initial action sequence — treetop
/// `Solver::solve`, with its iteration budget as a parameter (the treetop
/// planner runs a few iterations per candidate at 10 Hz; the standalone
/// planner affords more). The returned trajectory is re-realized under the
/// actuation constraints, exactly as treetop re-rollouts its solution.
pub(crate) fn solve(ocp: &Ocp, init_actions: &[Control], max_iters: usize) -> Solution {
    let dt = ocp.ctx.road.dt;
    let (mut xs, mut us) = rollout_constrained(ocp.start, init_actions, dt);
    let mut cost_now = ocp.ctx.time("rollout", || ocp.traj_cost(&xs, &us));
    let mut reg = REG_INIT;

    for _ in 0..max_iters {
        // backward pass, surging regularization on PD failure
        let mut pass = None;
        while pass.is_none() {
            pass = ocp.ctx.time("derivs", || backward(ocp, &xs, &us, reg));
            if pass.is_none() {
                if reg >= REG_MAX {
                    break;
                }
                reg = (reg * REG_SURGE).min(REG_MAX);
            }
        }
        let Some((gains, ecc)) = pass else { break };

        // feedforward-gain scaling search (treetop `feedfrwdGainSearch`)
        let mut scale = 1.0;
        let mut accepted = None;
        let mut attempts = 0;
        for attempt in 1..=MAX_FFGS_ATTEMPTS {
            attempts = attempt;
            if let Some((nxs, nus, ncost)) = ocp
                .ctx
                .time("rollout", || forward(ocp, &xs, &us, &gains, scale))
            {
                let change = cost_now - ncost;
                let acceptable = COST_CHANGE_RATIO_MIN * ecc.evaluate(scale) - COST_CHANGE_TOL;
                if change > acceptable {
                    accepted = Some((nxs, nus, ncost));
                    break;
                }
            }
            scale *= FFGS_DECREASE;
        }

        match accepted {
            Some((nxs, nus, ncost)) => {
                let improved = cost_now - ncost;
                xs = nxs;
                us = nus;
                cost_now = ncost;
                let factor = if attempts <= 1 {
                    REG_DECREASE
                } else {
                    REG_INCREASE
                };
                reg = (reg * factor).clamp(REG_MIN, REG_MAX);
                if improved < COST_CHANGE_TOL {
                    break; // converged (treetop `checkConvergence`)
                }
            }
            None => {
                // search failed: keep the trajectory, surge regularization
                reg = (reg * REG_SURGE).clamp(REG_MIN, REG_MAX);
            }
        }
    }

    // Re-rollout under the action constraints so the returned trajectory
    // is consistent and honors the limits (treetop does exactly this).
    let (fxs, fus) = rollout_constrained(xs[0], &us, dt);
    let fcost = ocp.ctx.time("rollout", || ocp.traj_cost(&fxs, &fus));
    Solution {
        states: fxs,
        controls: fus,
        cost: fcost,
    }
}

/// The standalone iLQR planner: optimize from a lane-keeping initial guess
/// (or last tick's shifted solution), no tree in front. This is trajectory
/// optimization at its most exposed — a purely local method whose result
/// is only as good as its initial guess, which is precisely the weakness
/// the treetop coordination exists to fix; kept standalone so the registry
/// can show that difference side by side.
#[derive(Default)]
pub(crate) struct IlqrPlanner {
    prev: Option<Vec<Control>>,
    expected_next: State,
}

/// The standalone planner's iteration budget: more than the treetop
/// planner gives each candidate (its tree hands it a near-feasible guess;
/// the lane-keeping guess here may start inside an obstacle's penalty and
/// needs the extra steps to climb out).
const SOLO_ITERS: usize = 12;

impl Planner for IlqrPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || Path::new(ctx.road.centerline()));
        let init = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego)
                .unwrap_or_else(|| centerline_follow_controls(ego, &path, ctx, TICKS))
        });

        let ocp = Ocp {
            path: &path,
            start: ego,
            ctx,
        };
        let sol = ctx.time("optimize", || solve(&ocp, &init, SOLO_ITERS));

        if let Some(diag) = ctx.diagnostics {
            for s in &sol.states {
                diag.record_point([s.x, s.y]);
            }
            diag.record_trajectory(sol.states.iter().map(|s| [s.x, s.y]).collect());
        }

        let controls = ctx.time("extract", || {
            repeat_last_controls(&sol.controls, ctx.horizon)
        });
        self.expected_next = world_step(ego, controls[0], ctx.road.dt);
        self.prev = Some(sol.controls);
        controls
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fd_gradient_matches_a_known_quadratic() {
        // f(z) = z·diag(1,2)·z + [3,4]·z: grad = 2 diag z + [3,4], hess = 2 diag
        let f = |z: [f64; 2]| z[0] * z[0] + 2.0 * z[1] * z[1] + 3.0 * z[0] + 4.0 * z[1];
        let (g, h) = fd_grad_hess(&f, [1.0, -2.0]);
        assert!((g[0] - 5.0).abs() < 1e-6, "g0 {}", g[0]);
        assert!((g[1] + 4.0).abs() < 1e-6, "g1 {}", g[1]);
        assert!((h[0][0] - 2.0).abs() < 1e-3);
        assert!((h[1][1] - 4.0).abs() < 1e-3);
        assert!(h[0][1].abs() < 1e-3);
    }

    #[test]
    fn fd_dynamics_jacobian_matches_the_analytic_one() {
        // the vehicle model's Jacobian is known in closed form (treetop
        // `Dynamics::jacobian`); the FD version must reproduce it
        let x = State {
            x: 1.0,
            y: 2.0,
            yaw: 0.3,
            speed: 8.0,
        };
        let u = Control {
            acceleration: 0.5,
            curvature: 0.03,
        };
        let dt = 0.1;
        let (a, b) = dynamics_jacobian(&x, &u, dt);
        let drag_slope = crate::vehicle::AIR_DENSITY_KG_M3 * crate::vehicle::DRAG_AREA_M2
            / crate::vehicle::EGO_MASS_KG
            * x.speed;
        let expect_a = [
            [1.0, 0.0, -x.speed * dt * x.yaw.sin(), dt * x.yaw.cos()],
            [0.0, 1.0, x.speed * dt * x.yaw.cos(), dt * x.yaw.sin()],
            [0.0, 0.0, 1.0, dt * u.curvature],
            [0.0, 0.0, 0.0, 1.0 - drag_slope * dt],
        ];
        let expect_b = [[0.0, 0.0], [0.0, 0.0], [0.0, x.speed * dt], [dt, 0.0]];
        for i in 0..4 {
            for j in 0..4 {
                assert!((a[i][j] - expect_a[i][j]).abs() < 1e-6, "A[{i}][{j}]");
            }
            for j in 0..2 {
                assert!((b[i][j] - expect_b[i][j]).abs() < 1e-6, "B[{i}][{j}]");
            }
        }
    }

    #[test]
    fn solve_improves_on_a_poor_initial_guess() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let path = Path::new(road.centerline());
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let ocp = Ocp {
            path: &path,
            start: ego,
            ctx: &ctx,
        };
        // a lazy guess: coast straight, ignoring the lane offset and speed
        let init = vec![Control::default(); TICKS];
        let (xs0, us0) = rollout_constrained(ego, &init, ctx.road.dt);
        let cost0 = ocp.traj_cost(&xs0, &us0);
        let sol = solve(&ocp, &init, 20);
        assert!(sol.cost < cost0, "no improvement: {} vs {cost0}", sol.cost);
        assert!(sol.states.last().unwrap().speed > xs0.last().unwrap().speed);
    }

    #[test]
    fn stage_derivs_include_composite_comfort() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let path = Path::new(road.centerline());
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let ocp = Ocp {
            path: &path,
            start: ego,
            ctx: &ctx,
        };
        let u = Control {
            acceleration: 3.0,
            curvature: -0.2,
        };

        let d = stage_derivs(&ocp, &ego, &u, 0);
        assert!(d.lu.into_iter().all(f64::is_finite));
        assert!(d.luu.iter().flatten().all(|v| v.is_finite()));
        assert_eq!(d.luu[0][1], d.luu[1][0]);
    }

    #[test]
    fn predicted_actor_stage_cost_matches_regular_stage_cost() {
        let actor = State {
            x: 40.0,
            speed: 8.0,
            ..Default::default()
        };
        let actors = [actor];
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &actors);
        let path = Path::new(road.centerline());
        let tc = TrajectoryCost::new(&path, &ctx, 8.0);
        let x = State {
            x: 5.0,
            speed: 8.0,
            ..Default::default()
        };
        let u = Control {
            acceleration: 0.3,
            curvature: 0.02,
        };
        let t = 3;
        let predicted = [predict(&actor, &path, t as f64 * road.dt)];

        let regular = tc.stage(&x, u, t, None);
        let reused = tc.stage_with_predicted_actors(&x, u, t, None, &predicted);
        assert!((regular - reused).abs() < 1e-9, "{regular} vs {reused}");
    }

    #[test]
    fn stays_on_road_and_accelerates() {
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let trace = crate::planning::test_run(&mut IlqrPlanner::default(), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 5.5, "offset {}", end.y);
        assert!(end.speed > ego.speed, "speed {}", end.speed);
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
        let trace = crate::planning::test_run(&mut IlqrPlanner::default(), ego, &[obstacle], 150);
        let (min_gap, _) = trace
            .iter()
            .map(|s| ((s.x - 40.0).hypot(s.y), *s))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap();
        assert!(min_gap > 2.0, "min gap {min_gap}");
    }

    #[test]
    fn plan_is_a_pure_function_of_state() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let a = IlqrPlanner::default().plan(ego, &ctx);
        let b = IlqrPlanner::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }
}
