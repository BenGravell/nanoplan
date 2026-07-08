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
//! nanoplan deliberately provides neither: its shared cost
//! ([`cost::point_cost`]) and dynamics ([`step`]) are black-box scalars
//! (see the "no analytic derivatives" discussion in
//! `src/planning/README.md`). So this port differentiates **numerically**:
//! central finite differences for the cost gradient/Hessian
//! ([`stage_derivs`], [`terminal_derivs`]) and the dynamics Jacobians
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
//! The running cost is the shared [`cost::point_cost`] — the same scalar
//! every search planner prices candidates with — plus the usual
//! planner-specific structural terms (centerline pull, speed tracking, and
//! a small control-effort quadratic that also keeps `l_uu` strictly
//! positive), scaled by `1/TICKS` as treetop scales by
//! `inverse_traj_length`. Two adaptations make the shared cost usable
//! under an optimizer that *differentiates* it rather than compares it:
//!
//! - **Hard violations get an escape slope.** `point_cost` returns
//!   `f64::INFINITY` on collision/off-road; PI²-DDP substitutes the flat
//!   [`cost::HARD_VIOLATION_PENALTY`] because its statistics can't absorb
//!   an infinity. A *flat* plateau is equally useless to a
//!   finite-difference gradient (zero slope everywhere inside the
//!   violation), so this planner prices a violation as
//!   `HARD_VIOLATION_PENALTY · (1 + depth)` where `depth` is how far
//!   inside the violation the sample sits — same cliff at the boundary,
//!   but with a gradient pointing back out of it.
//! - **The terminal cost is a quadratic pull to the goal state** (position,
//!   heading, speed) — treetop's terminal loss with its parking-grade
//!   `smoothAbs(·, 0.01)` tolerances swapped for lane-driving quadratics;
//!   the goal is the rolling lane target from
//!   [`goal_state`](super::goal_state).
//!
//! **Seams**: `route`, `warm_start`, `optimize` (the whole solve), with
//! `derivs` (all backward-pass FD work) and `rollout` (the line-search
//! forward passes) nested inside, and `extract`. Unlike the other search
//! planners there is no per-call `cost` seam: the FD probes call the
//! shared cost ~10⁵ times per plan, and timing each call individually
//! would cost more than the call — `derivs`/`rollout` are where those
//! calls live.
//!
//! **Diagnostics**: the initial-guess rollout and the optimized trajectory
//! as polylines (what the optimizer bought), the optimized states as
//! points.

use super::{TICKS, goal_state, rollout_constrained, take_warm};
use crate::planning::{Context, Planner, cost};
use crate::scenarios::Path;
use crate::simulation::{Control, State, action_toward, step};
use crate::wrap_angle;

// ---- Tiny fixed-size linear algebra --------------------------------------
// State dimension 6, action dimension 2 — small enough that hand-rolled
// const-generic helpers beat pulling in a matrix crate (and stay wasm-lean).
// Convention: a matrix is `[[f64; COLS]; ROWS]`.

type V2 = [f64; 2];
type V6 = [f64; 6];
type M66 = [[f64; 6]; 6];
type M62 = [[f64; 2]; 6]; // 6 rows × 2 cols
type M26 = [[f64; 6]; 2]; // 2 rows × 6 cols
type M22 = [[f64; 2]; 2];

fn mat_mul<const M: usize, const K: usize, const N: usize>(
    a: &[[f64; K]; M],
    b: &[[f64; N]; K],
) -> [[f64; N]; M] {
    let mut out = [[0.0; N]; M];
    for i in 0..M {
        for k in 0..K {
            let aik = a[i][k];
            for j in 0..N {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    out
}

fn mat_vec<const M: usize, const N: usize>(a: &[[f64; N]; M], v: &[f64; N]) -> [f64; M] {
    std::array::from_fn(|i| (0..N).map(|j| a[i][j] * v[j]).sum())
}

fn transpose<const M: usize, const N: usize>(a: &[[f64; N]; M]) -> [[f64; M]; N] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

fn vec_add<const N: usize>(a: [f64; N], b: [f64; N]) -> [f64; N] {
    std::array::from_fn(|i| a[i] + b[i])
}

fn mat_add<const M: usize, const N: usize>(a: [[f64; N]; M], b: [[f64; N]; M]) -> [[f64; N]; M] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] + b[i][j]))
}

fn dot<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    (0..N).map(|i| a[i] * b[i]).sum()
}

fn to_v6(x: &State) -> V6 {
    [x.x, x.y, x.yaw, x.speed, x.accel, x.curvature]
}

fn of_v6(v: V6) -> State {
    State {
        x: v[0],
        y: v[1],
        yaw: v[2],
        speed: v[3],
        accel: v[4],
        curvature: v[5],
    }
}

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

/// Weights of the structural running-cost terms on top of the shared cost
/// (see the module doc): centerline pull, speed tracking, and the
/// control-effort quadratics that also keep `l_uu` strictly positive
/// (curvature's weight is larger because curvature is numerically ~50×
/// smaller than acceleration).
const CENTER_W: f64 = 0.5;
const SPEED_W: f64 = 0.3;
const EFFORT_ACCEL_W: f64 = 0.1;
const EFFORT_CURV_W: f64 = 10.0;

/// Terminal quadratic weights: position, heading, speed pulls toward the
/// goal state.
const TERM_XY_W: f64 = 0.05;
const TERM_YAW_W: f64 = 5.0;
const TERM_SPEED_W: f64 = 0.2;

/// One trajectory-optimization problem: minimize running + terminal cost
/// from a fixed start — treetop's `Problem`, with the loss built on
/// nanoplan's shared cost function.
pub(crate) struct Ocp<'a, 'b> {
    pub path: &'a Path,
    /// treetop `Problem::initial_state`: where every rollout starts.
    pub start: State,
    pub goal: State,
    pub ctx: &'a Context<'b>,
}

impl Ocp<'_, '_> {
    /// Running cost of being at `x` at tick `t` having applied `u`.
    /// `s_hint` narrows the Frenet projection during FD probing, where the
    /// state moves by ±[`H_COST`] around a known station.
    fn stage_cost(&self, x: &State, u: &Control, t: usize, s_hint: Option<f64>) -> f64 {
        let p = [x.x, x.y];
        let (s, d) = match s_hint {
            Some(h) => self.path.project_near(p, h, 15.0),
            None => self.path.project(p),
        };
        let (_, lane_yaw) = self.path.pose_at(s);
        let sample = cost::Sample {
            xy: p,
            lateral: d,
            heading_err: wrap_angle(x.yaw - lane_yaw),
            speed: x.speed,
            curvature: x.curvature,
            accel: x.accel,
            t: t as f64 * self.ctx.road.dt,
        };
        let target = self.ctx.road.target_speed;
        // shared cost with a hard violation's escape slope (see the module
        // doc); the depth-scaled penalty lives in `cost::soft_point_cost`,
        // shared with the judo samplers and PI²-DDP.
        let shared = cost::soft_point_cost(
            &sample,
            target,
            self.ctx.road.half_width,
            self.ctx.actors,
            Some(self.path),
        );
        let dv = x.speed - target;
        let structural = CENTER_W * d * d
            + SPEED_W * dv * dv
            + EFFORT_ACCEL_W * u.jerk * u.jerk
            + EFFORT_CURV_W * u.curvature_rate * u.curvature_rate;
        (shared + structural) / TICKS as f64
    }

    /// Quadratic pull of the trajectory endpoint toward the goal state.
    fn terminal_cost(&self, x: &State) -> f64 {
        let (dx, dy) = (x.x - self.goal.x, x.y - self.goal.y);
        let dyaw = wrap_angle(x.yaw - self.goal.yaw);
        let dv = x.speed - self.goal.speed;
        TERM_XY_W * (dx * dx + dy * dy) + TERM_YAW_W * dyaw * dyaw + TERM_SPEED_W * dv * dv
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

/// FD step for cost derivatives. All six problem coordinates (meters,
/// radians, m/s, m/s², 1/m) are O(0.1–10) here, so one step suits them
/// all; ~eps^(1/4) is the classic sweet spot for second differences.
const H_COST: f64 = 1e-3;

/// FD step for dynamics Jacobians (first differences only, so a smaller
/// step is optimal).
const H_DYN: f64 = 1e-5;

/// Quadratic expansion of the running cost at one `(x, u)` — treetop's
/// `Loss::gradientAndHessian`, by central finite differences over the
/// packed 8-vector `z = (x, y, yaw, v, accel, curvature, jerk,
/// curvature_rate)`.
struct StageDerivs {
    lx: V6,
    lu: V2,
    lxx: M66,
    lxu: M62,
    luu: M22,
}

fn stage_derivs(ocp: &Ocp, x: &State, u: &Control, t: usize) -> StageDerivs {
    // one projection of the unperturbed state anchors every probe's
    // Frenet lookup (the probes move by ±H_COST, far less than the window)
    let s_hint = ocp.path.project([x.x, x.y]).0;
    let z0 = [
        x.x,
        x.y,
        x.yaw,
        x.speed,
        x.accel,
        x.curvature,
        u.jerk,
        u.curvature_rate,
    ];
    let eval = |z: [f64; 8]| {
        let xs = State {
            x: z[0],
            y: z[1],
            yaw: z[2],
            speed: z[3],
            accel: z[4],
            curvature: z[5],
        };
        let us = Control {
            jerk: z[6],
            curvature_rate: z[7],
        };
        ocp.stage_cost(&xs, &us, t, Some(s_hint))
    };
    let (grad, hess) = fd_grad_hess(&eval, z0);
    StageDerivs {
        lx: [grad[0], grad[1], grad[2], grad[3], grad[4], grad[5]],
        lu: [grad[6], grad[7]],
        lxx: std::array::from_fn(|i| std::array::from_fn(|j| hess[i][j])),
        lxu: std::array::from_fn(|i| std::array::from_fn(|j| hess[i][6 + j])),
        luu: std::array::from_fn(|i| std::array::from_fn(|j| hess[6 + i][6 + j])),
    }
}

/// Terminal gradient and Hessian, likewise by finite differences.
fn terminal_derivs(ocp: &Ocp, x: &State) -> (V6, M66) {
    let (grad, hess) = fd_grad_hess(&|z: [f64; 6]| ocp.terminal_cost(&of_v6(z)), to_v6(x));
    (grad, hess)
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
/// differences on [`step`] — treetop's closed-form `Dynamics::jacobian`,
/// derived numerically instead.
fn dynamics_jacobian(x: &State, u: &Control, dt: f64) -> (M66, M62) {
    let mut a = [[0.0; 6]; 6];
    for j in 0..6 {
        let mut zp = to_v6(x);
        zp[j] += H_DYN;
        let mut zm = to_v6(x);
        zm[j] -= H_DYN;
        let sp = to_v6(&step(of_v6(zp), *u, dt));
        let sm = to_v6(&step(of_v6(zm), *u, dt));
        for i in 0..6 {
            a[i][j] = (sp[i] - sm[i]) / (2.0 * H_DYN);
        }
    }
    let col = |up: Control, um: Control| -> V6 {
        let sp = to_v6(&step(*x, up, dt));
        let sm = to_v6(&step(*x, um, dt));
        std::array::from_fn(|i| (sp[i] - sm[i]) / (2.0 * H_DYN))
    };
    let cj = col(
        Control {
            jerk: u.jerk + H_DYN,
            ..*u
        },
        Control {
            jerk: u.jerk - H_DYN,
            ..*u
        },
    );
    let cr = col(
        Control {
            curvature_rate: u.curvature_rate + H_DYN,
            ..*u
        },
        Control {
            curvature_rate: u.curvature_rate - H_DYN,
            ..*u
        },
    );
    let b: M62 = std::array::from_fn(|i| [cj[i], cr[i]]);
    (a, b)
}

// ---- The solver ------------------------------------------------------------

/// The affine policy of one timestep: feedforward `k`, feedback `K`.
#[derive(Clone, Copy)]
struct Gains {
    k: V2,
    kk: M26,
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
    pub states: Vec<State>,
    pub controls: Vec<Control>,
    pub cost: f64,
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
    let (mut vx, mut vxx) = terminal_derivs(ocp, &xs[n]);
    let mut gains = vec![
        Gains {
            k: [0.0; 2],
            kk: [[0.0; 6]; 2]
        };
        n
    ];
    let mut ecc = Ecc {
        term1: 0.0,
        term2: 0.0,
    };

    for t in (0..n).rev() {
        let sd = stage_derivs(ocp, &xs[t], &us[t], t);
        let (a, b) = dynamics_jacobian(&xs[t], &us[t], ocp.ctx.road.dt);
        let at = transpose(&a);
        let bt = transpose(&b);
        let atv: M66 = mat_mul(&at, &vxx);
        let btv: M26 = mat_mul(&bt, &vxx);

        let qx: V6 = vec_add(sd.lx, mat_vec(&at, &vx));
        let qu: V2 = vec_add(sd.lu, mat_vec(&bt, &vx));
        let qxx: M66 = mat_add(sd.lxx, mat_mul(&atv, &a));
        let qxu: M62 = mat_add(sd.lxu, mat_mul(&atv, &b));
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
        let kk: M26 = mat_mul(&inv, &transpose(&qxu)).map(|row| row.map(|v| -v));

        // value-function update (treetop `updateV`), symmetrized to shed
        // FD asymmetry before it compounds across 100 steps
        let kt: M62 = transpose(&kk);
        let w: M62 = mat_add(mat_mul(&kt, &quu), qxu);
        vx = vec_add(qx, vec_add(mat_vec(&w, &k), mat_vec(&kt, &qu)));
        let vxx_new: M66 = mat_add(
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
        let dx: V6 = std::array::from_fn(|i| to_v6(&x)[i] - to_v6(&xs[t])[i]);
        let fb = mat_vec(&gains[t].kk, &dx);
        let u = Control {
            jerk: us[t].jerk + scale * gains[t].k[0] + fb[0],
            curvature_rate: us[t].curvature_rate + scale * gains[t].k[1] + fb[1],
        };
        cost_total += ocp.stage_cost(&x, &u, t, None);
        x = step(x, u, ocp.ctx.road.dt);
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

/// The base lane-keeping guess the standalone planner starts from when it
/// has no warm start: the same critically-damped PD lane keeper + speed
/// hold the judo planners use as their base policy, rolled out under the
/// constraints and recorded as actions.
fn base_guess(path: &Path, ego: State, ctx: &Context) -> Vec<Control> {
    let mut x = ego;
    let mut actions = Vec::with_capacity(TICKS);
    for _ in 0..TICKS {
        let (s, d) = path.project([x.x, x.y]);
        let (_, lane_yaw) = path.pose_at(s);
        let heading_err = wrap_angle(x.yaw - lane_yaw);
        let u = action_toward(
            x,
            (0.5 * (ctx.road.target_speed - x.speed)).clamp(-2.0, 1.5),
            -(0.02 * d + 0.3 * heading_err),
            ctx.road.dt,
        );
        x = step(x, u, ctx.road.dt);
        actions.push(u);
    }
    actions
}

/// The standalone iLQR planner: optimize from a lane-keeping initial guess
/// (or last tick's shifted solution), no tree in front. This is trajectory
/// optimization at its most exposed — a purely local method whose result
/// is only as good as its initial guess, which is precisely the weakness
/// the treetop coordination exists to fix; kept standalone so the registry
/// can show that difference side by side.
#[derive(Default)]
pub struct IlqrPlanner {
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
        let path = ctx.time("route", || Path::new(&ctx.road.centerline));
        let goal = goal_state(&path, ego, ctx);
        let init = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego)
                .unwrap_or_else(|| base_guess(&path, ego, ctx))
        });

        let ocp = Ocp {
            path: &path,
            start: ego,
            goal,
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
            (0..ctx.horizon)
                .map(|t| sol.controls[t.min(sol.controls.len() - 1)])
                .collect::<Vec<_>>()
        });
        self.expected_next = step(ego, controls[0], ctx.road.dt);
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
        // the kinematic model's Jacobian is known in closed form (treetop
        // `Dynamics::jacobian`); the FD version must reproduce it
        let x = State {
            x: 1.0,
            y: 2.0,
            yaw: 0.3,
            speed: 8.0,
            accel: 0.2,
            curvature: 0.03,
        };
        let u = Control {
            jerk: 0.5,
            curvature_rate: 0.05,
        };
        let dt = 0.1;
        let (a, b) = dynamics_jacobian(&x, &u, dt);
        let next_curvature = x.curvature + u.curvature_rate * dt;
        let expect_a = [
            [
                1.0,
                0.0,
                -x.speed * dt * x.yaw.sin(),
                dt * x.yaw.cos(),
                0.0,
                0.0,
            ],
            [
                0.0,
                1.0,
                x.speed * dt * x.yaw.cos(),
                dt * x.yaw.sin(),
                0.0,
                0.0,
            ],
            [0.0, 0.0, 1.0, dt * next_curvature, 0.0, x.speed * dt],
            [0.0, 0.0, 0.0, 1.0, dt, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ];
        let expect_b = [
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, x.speed * dt * dt],
            [dt * dt, 0.0],
            [dt, 0.0],
            [0.0, dt],
        ];
        for i in 0..6 {
            for j in 0..6 {
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
        let path = Path::new(&road.centerline);
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let goal = goal_state(&path, ego, &ctx);
        let ocp = Ocp {
            path: &path,
            start: ego,
            goal,
            ctx: &ctx,
        };
        // a lazy guess: coast straight, ignoring the lane offset and speed
        let init = vec![Control::default(); TICKS];
        let (xs0, us0) = rollout_constrained(ego, &init, ctx.road.dt);
        let cost0 = ocp.traj_cost(&xs0, &us0);
        let sol = solve(&ocp, &init, 20);
        assert!(sol.cost < cost0, "no improvement: {} vs {cost0}", sol.cost);
        // the optimized endpoint is much closer to the goal
        let end = sol.states.last().unwrap();
        let d0 = super::super::state_distance(xs0.last().unwrap(), &goal);
        let d1 = super::super::state_distance(end, &goal);
        assert!(d1 < d0, "endpoint got no closer: {d1} vs {d0}");
    }

    #[test]
    fn tracks_centerline_and_speed() {
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let trace = crate::planning::test_run(&mut IlqrPlanner::default(), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 1.2, "offset {}", end.y);
        assert!((end.speed - 10.0).abs() < 2.5, "speed {}", end.speed);
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
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
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
