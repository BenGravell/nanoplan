//! Planners ported from **treetop**
//! (<https://github.com/BenGravell/treetop>), a tree-initialized
//! trajectory-optimizing planner: an ego motion sampling tree provides a
//! strong initial trajectory guess, and iLQR (iterative
//! Linear Quadratic Regulator) optimizes that guess into a smooth
//! trajectory, whose solution warm-starts the tree next cycle. Following
//! the request that motivated this port, the two halves are *also* exposed
//! as standalone planners, so the registry gains three entries from this
//! directory (the same one-port-many-planners shape as
//! [`super::sampling_mpc`]):
//!
//! - [`GraphPlanner`](super::motion_graph::GraphPlanner) (`../motion_graph/mod.rs`) — the motion sampling tree alone
//!   (treetop's `tree/`), taking the tree's best path candidate as the
//!   plan with no optimization pass.
//! - [`IlqrPlanner`] (`ilqr.rs`) — the iLQR solver alone (treetop's
//!   `ilqr/`), optimizing from a simple lane-keeping initial guess instead
//!   of a tree path.
//! - [`TreetopPlanner`] (this file) — the coordinator glue (treetop's
//!   `planner.h`): tree expansion → path candidates → iLQR on each →
//!   best-candidate selection → solution fed back to warm-start the next
//!   tree.
//!
//! The shared motion graph owns the steering-layer constants and coasting
//! rollout; plant rollouts live in [`crate::planning::controls`]. nanoplan's
//! kinematic model keeps only pose/speed in state and uses direct
//! acceleration/curvature commands, so the treetop port reads those commands
//! from its flat-output curves before every rollout.
//!
//! ## Fitting treetop into the nanoplan framework
//!
//! - **Every tree layer samples reachable states.** The terminal layer uses
//!   the same warm/cold sampling as intermediate layers. Candidate selection
//!   uses the shared progress cost without a fixed goal state.
//! - **Obstacles are moving actors priced by the shared metric objective.**
//!   treetop collision-checks against static circles. Here every rolled-out
//!   state is checked and priced through
//!   [`Constraints`](crate::constraints::Constraints) at
//!   the absolute time the state is reached, which folds in the same
//!   actor prediction, drivable-area bound, and progress objective every
//!   other search planner uses.
//! - **Determinism.** treetop samples its tree with `std::mt19937` and
//!   jitters actions with pseudo-random noise. The port draws every sample
//!   from the shared Halton sequence ([`crate::planning::sampling`])
//!   instead, and drops the action jitter (whose whole point is randomized
//!   restarts), so all three planners are pure functions of the ego state
//!   like the judo planners — pinned by the
//!   `*_is_a_pure_function_of_state` tests.
//!
//! See `src/planning/planners/treetop/README.md` for the design write-up.

pub(crate) mod ilqr;

use crate::planning::planners::motion_graph::{Connections, Graph};
pub(crate) use ilqr::IlqrPlanner;

use crate::planning::controls::repeat_last_controls;
use crate::planning::warm_start::shift_actions;
use crate::planning::{Context, PLANNING_TICKS, Planner, take_warm};
use crate::simulation::{Control, State, world_step};

/// Planning-horizon length in ticks — treetop's `TRAJ_LENGTH_OPT`, here
/// 10 s at the simulator's 0.1 s tick rate, the same look-ahead every other
/// receding-horizon planner uses.
pub(crate) const TICKS: usize = PLANNING_TICKS;

/// How many samples the tree spends per `plan()` call, spread across the
/// layers. treetop's interactive default is 5000 across 19 layers; a 10 Hz
/// replan tick affords less, and the warm start means each tick refines the
/// last rather than starting cold.
const TREE_SAMPLES: usize = 450;

/// How many of the tree's path candidates get an iLQR pass — treetop's
/// `num_path_candidates` (default 2): the best candidate plus an alternate,
/// so a locally-poor tree path can be beaten by a differently-shaped one
/// after optimization.
const CANDIDATES: usize = 2;

/// iLQR iterations per candidate. treetop lets its solver run to
/// convergence (up to 200 iterations) because it replans on demand; at a
/// 10 Hz tick with finite-difference derivatives the budget is tighter, and
/// the warm-started tree hands iLQR a near-feasible guess that converges in
/// a handful of iterations anyway.
const OPT_ITERS: usize = 6;

/// The treetop planner: the motion sampling tree ([`Graph`]) provides path
/// candidates, iLQR ([`ilqr`]) optimizes each, the best optimized
/// trajectory is the plan — and its action sequence warm-starts the tree
/// next tick (treetop's `Planner::plan` loop). See the module doc and
/// `src/planning/planners/treetop/README.md`.
///
/// **Seams**: `route`, `warm_start`, then treetop's own two-phase timing
/// split (`TimingInfo { tree_exp, traj_opt }`) as `tree` (grow + candidate
/// extraction) and `traj_opt` (the iLQR passes), both nested under
/// `optimize`; `extract` for control emission. `cost` nests inside `tree`
/// (the tree prices edges through the shared metric objective, once per
/// sampled point); the iLQR passes bury their shared-cost calls inside
/// `derivs`/`rollout` instead — see [`ilqr`]'s seam note.
///
/// **Diagnostics**: every tree edge as a trajectory and every node as a
/// point (the search the tree considered), plus the winning candidate's
/// pre-optimization polyline and its post-iLQR trajectory — the pair that
/// shows what the optimizer bought.
#[derive(Default)]
pub(crate) struct TreetopPlanner {
    /// Last tick's optimized action sequence, re-fed to the tree as its
    /// warm start (treetop's `warm`/`use_hot` loop).
    prev: Option<Vec<Control>>,
    /// Predicted next ego state, to check the warm start is still valid.
    expected_next: State,
}

impl Planner for TreetopPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || ctx.path());
        let warm = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego).map(shift_actions)
        });
        // Offline calibration: 450 tree samples plus the fixed refinement
        // pass is about 100 ms.
        let tree_samples = ctx.compute_budget.scale(TREE_SAMPLES, 45);

        let (tree, candidates, best) = ctx.time("optimize", || {
            // ---- Tree expansion + path extraction (treetop `tree_exp`).
            let (tree, candidates) = ctx.time("tree", || {
                let tree = Graph::grow(ego, warm.as_deref(), tree_samples, path, ctx, Connections::NearestZap);
                let candidates = tree.path_candidates(CANDIDATES);
                (tree, candidates)
            });

            // Optimize each candidate and select by the shared progress cost.
            let ocp = ilqr::Ocp::new(path, ego, ctx);
            let best = ctx.time("traj_opt", || {
                candidates
                    .iter()
                    .enumerate()
                    .map(|(i, cand)| {
                        let sol = ilqr::solve(&ocp, &tree.actions_of(cand), OPT_ITERS);
                        (sol.cost, i, sol)
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .expect("at least one candidate")
            });
            (tree, candidates, best)
        });
        let (_, cand_ix, sol) = best;

        if let Some(diag) = ctx.diagnostics {
            tree.record_diagnostics(diag);
            // the winning candidate before optimization…
            let pre = std::iter::once(ego.position())
                .chain(
                    candidates[cand_ix]
                        .iter()
                        .flat_map(|&n| tree.nodes[n].states().iter().skip(1).map(Into::into)),
                )
                .collect();
            diag.record_trajectory(pre);
            // …and after
            diag.record_trajectory(sol.states.iter().map(Into::into).collect());
        }

        let controls = ctx.time("extract", || repeat_last_controls(&sol.controls, ctx.horizon));
        self.expected_next = world_step(ego, controls[0], ctx.road.dt);
        self.prev = Some(sol.controls);
        controls
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rollout_constrained_uses_the_shared_step() {
        let x0 = State {
            speed: 5.0,
            ..Default::default()
        };
        let actions = [Control {
            acceleration: 2.0,
            curvature: 0.1,
        }];
        let (xs, us) = crate::planning::controls::rollout_constrained(x0, &actions, 0.1);
        assert_eq!(us, actions);
        assert_eq!(xs[1], world_step(x0, actions[0], 0.1));
    }

    #[test]
    fn stays_on_road_and_accelerates() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        );
        let trace = crate::planning::test_run(&mut TreetopPlanner::default(), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 5.5, "offset {}", end.position().y);
        assert!(end.speed > ego.speed, "speed {}", end.speed);
    }

    #[test]
    fn avoids_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let trace = crate::planning::test_run(&mut TreetopPlanner::default(), ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.position().x - 40.0).hypot(s.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(
            trace.last().unwrap().position().x > 50.0,
            "did not pass, x {}",
            trace.last().unwrap().position().x
        );
    }

    #[test]
    fn plan_is_a_pure_function_of_state() {
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
        let a = TreetopPlanner::default().plan(ego, &ctx);
        let b = TreetopPlanner::default().plan(ego, &ctx);
        assert_eq!(a, b);
    }

    #[test]
    fn records_diagnostics_when_requested() {
        use crate::planning::Diagnostics;
        let diag = Diagnostics::default();
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let mut ctx = crate::planning::test_ctx(&road, &[]);
        ctx.diagnostics = Some(&diag);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        TreetopPlanner::default().plan(ego, &ctx);
        let data = diag.take();
        // tree nodes as points; tree edges + pre-opt + post-opt polylines
        assert!(!data.points.is_empty());
        assert!(data.trajectories.len() >= 2);
    }
}
