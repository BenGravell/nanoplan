//! Planners ported from **treetop**
//! (<https://github.com/BenGravell/treetop>), a tree-initialized
//! trajectory-optimizing planner: an ego motion sampling tree provides a
//! strong initial guess at a good path to the goal, and iLQR (iterative
//! Linear Quadratic Regulator) optimizes that guess into a smooth
//! trajectory, whose solution warm-starts the tree next cycle. Following
//! the request that motivated this port, the two halves are *also* exposed
//! as standalone planners, so the registry gains three entries from this
//! directory (the same one-port-many-planners shape as
//! [`super::sampling_mpc`]):
//!
//! - [`RrtPlanner`] (`rrt.rs`) — the motion sampling tree alone
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
//! This file also owns what treetop keeps in `core/` — the pieces *both*
//! halves stand on: the trajectory-length constants and the goal state
//! (shared rollout lives in [`crate::planning::search_tree`]). nanoplan's
//! kinematic model keeps only pose/speed in state and uses direct
//! acceleration/curvature commands, so the treetop port reads those commands
//! from its flat-output curves before every rollout.
//!
//! ## Fitting treetop into the nanoplan framework
//!
//! - **The goal is a moving lane target, not a parking pose.** treetop
//!   solves point-to-point: drive from a start pose to a user-placed goal
//!   pose in a fixed obstacle field. nanoplan's problem is a rolling one —
//!   follow the centerline at the target speed. [`goal_state`] bridges the
//!   two: the goal is the centerline pose a planning horizon ahead
//!   (at the average of current and target speed), at the target speed.
//!   Every treetop notion of "distance to goal" and "target hit" then
//!   carries over, with the hit tolerance loosened from treetop's
//!   parking-precision 0.01 to a lane-driving [`GOAL_HIT_TOL`].
//! - **Obstacles are moving actors priced by the shared cost function.**
//!   treetop collision-checks against static circles. Here every rolled-out
//!   state is checked and priced through
//!   [`cost::HardConstraints`](crate::planning::cost::HardConstraints) at
//!   the absolute time the state is reached, which folds in the same
//!   constant-velocity actor prediction, drivable-area bound, and comfort
//!   terms every other search planner uses.
//! - **Determinism.** treetop samples its tree with `std::mt19937` and
//!   jitters actions with pseudo-random noise. The port draws every sample
//!   from the shared Halton sequence ([`crate::planning::sampling`])
//!   instead, and drops the action jitter (whose whole point is randomized
//!   restarts), so all three planners are pure functions of the ego state
//!   like RRT* and the judo planners — pinned by the
//!   `*_is_a_pure_function_of_state` tests.
//!
//! See `src/planning/README.md` for the design write-up of each planner.

pub mod ilqr;
pub mod rrt;

pub use ilqr::IlqrPlanner;
pub use rrt::RrtPlanner;

use crate::planning::model::step;
use crate::planning::search_tree::repeat_last_controls;
use crate::planning::{Context, PLANNING_DT_S, PLANNING_TICKS, Planner, warm_start_matches};
use crate::scenarios::Path;
use crate::simulation::{Control, State};

/// Planning-horizon length in ticks — treetop's `TRAJ_LENGTH_OPT`, here
/// 10 s at the simulator's 0.1 s tick rate, the same look-ahead every other
/// receding-horizon planner uses.
pub(crate) const TICKS: usize = PLANNING_TICKS;

/// Ticks per steering segment (one tree edge) — treetop's
/// `TRAJ_LENGTH_STEER`, scaled to the same 1 s of driving (treetop: 5 ticks
/// of 0.2 s). treetop's comment on the trade-off applies unchanged: longer
/// segments lean harder on the steering function and cover state space
/// faster; shorter ones handle cusps better.
pub(crate) const STEER_TICKS: usize = 10;

/// Steering segments per trajectory — treetop's `NUM_STEER_SEGMENTS`; also
/// the number of tree layers past the root.
pub(crate) const SEGMENTS: usize = TICKS / STEER_TICKS;

/// Where the vehicle ends up coasting (zero action) for `t` seconds. The
/// tree attaches each sample to the previous layer's node whose zero-action
/// point is nearest, a cheap proxy for "the parent that needs the least
/// steering effort to get there."
pub(crate) fn zero_action_point(x: State, t: f64) -> State {
    let steps = (t / PLANNING_DT_S).ceil().max(1.0) as usize;
    let dt = t / steps as f64;
    let mut x = x;
    for _ in 0..steps {
        x = step(x, Control::default(), dt);
    }
    x
}

/// The goal state the tree grows toward and the iLQR terminal cost pulls
/// on: the centerline pose one planning horizon ahead of the ego's current
/// station (at the average of current and target speed, so a slow start
/// aims at a reachable point), at the road's target speed. This is the
/// treetop→nanoplan bridge — treetop's fixed user-placed goal pose becomes
/// a rolling lane target recomputed every replan.
pub(crate) fn goal_state(path: &Path, ego: State, ctx: &Context) -> State {
    let (s0, _) = path.project(ego.position());
    let horizon_s = TICKS as f64 * ctx.road.dt;
    let preview = 0.5 * (ego.speed + ctx.road.target_speed) * horizon_s;
    let s_goal = (s0 + preview).min(path.length());
    let ([gx, gy], gyaw) = path.pose_at(s_goal);
    State {
        x: gx,
        y: gy,
        yaw: gyaw,
        speed: ctx.road.target_speed,
    }
}

/// treetop's heuristic `stateDistance`: planar distance plus absolute yaw
/// and speed deltas, mixing meters, radians, and m/s equally — treetop's
/// own comment concedes this is "a decent choice empirically (even if it
/// is not very principled)".
pub(crate) fn state_distance(a: &State, b: &State) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
        + crate::math::wrap_angle(a.yaw - b.yaw).abs()
        + (a.speed - b.speed).abs()
}

/// A trajectory endpoint within this [`state_distance`] of the goal counts
/// as hitting it. treetop uses 0.01 (per-axis) because its goal is a
/// parking pose to be reached exactly; a rolling lane target only needs to
/// be reached *roughly* — the point is sustained progress, not terminal
/// precision.
pub(crate) const GOAL_HIT_TOL: f64 = 2.0;

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

/// The treetop planner: the motion sampling tree ([`rrt`]) provides path
/// candidates, iLQR ([`ilqr`]) optimizes each, the best optimized
/// trajectory is the plan — and its action sequence warm-starts the tree
/// next tick (treetop's `Planner::plan` loop). See the module doc and
/// `src/planning/README.md`.
///
/// **Seams**: `route`, `warm_start`, then treetop's own two-phase timing
/// split (`TimingInfo { tree_exp, traj_opt }`) as `tree` (grow + candidate
/// extraction) and `traj_opt` (the iLQR passes), both nested under
/// `optimize`; `extract` for control emission. `cost` nests inside `tree`
/// (the tree prices edges through the shared cost function, once per
/// sampled point); the iLQR passes bury their shared-cost calls inside
/// `derivs`/`rollout` instead — see [`ilqr`]'s seam note.
///
/// **Diagnostics**: every tree edge as a trajectory and every node as a
/// point (the search the tree considered), plus the winning candidate's
/// pre-optimization polyline and its post-iLQR trajectory — the pair that
/// shows what the optimizer bought.
#[derive(Default)]
pub struct TreetopPlanner {
    /// Last tick's optimized action sequence, re-fed to the tree as its
    /// warm start (treetop's `warm`/`use_hot` loop).
    prev: Option<Vec<Control>>,
    /// Predicted next ego state, to check the warm start is still valid.
    expected_next: State,
}

/// Shift a warm-start action sequence one tick forward (the simulator
/// executed its first control), holding the last action — shared by every
/// planner in this directory.
pub(crate) fn shift_actions(mut actions: Vec<Control>) -> Vec<Control> {
    if !actions.is_empty() {
        actions.remove(0);
        actions.push(*actions.last().unwrap());
    }
    actions
}

/// Take a warm start only if the ego ended up where the previous plan
/// predicted (within 1 m, the same gate PI²-DDP and the judo planners
/// use); a diverged warm start describes a maneuver from somewhere else.
pub(crate) fn take_warm(
    prev: &mut Option<Vec<Control>>,
    expected_next: State,
    ego: State,
) -> Option<Vec<Control>> {
    match prev.take() {
        Some(a) if warm_start_matches(expected_next, ego) => Some(shift_actions(a)),
        _ => None,
    }
}

impl Planner for TreetopPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = ctx.time("route", || Path::new(&ctx.road.centerline));
        let goal = goal_state(&path, ego, ctx);
        let warm = ctx.time("warm_start", || {
            take_warm(&mut self.prev, self.expected_next, ego)
        });

        let (tree, candidates, best) = ctx.time("optimize", || {
            // ---- Tree expansion + path extraction (treetop `tree_exp`).
            let (tree, candidates) = ctx.time("tree", || {
                let tree = rrt::Tree::grow(ego, goal, warm.as_deref(), TREE_SAMPLES, &path, ctx);
                let candidates = tree.path_candidates(CANDIDATES);
                (tree, candidates)
            });

            // ---- Trajectory optimization (treetop `traj_opt`): run iLQR
            // on each candidate's action sequence, then pick by treetop's
            // two-tier rule — the cheapest solution that still hits the
            // goal, else the one ending nearest it (a candidate that
            // merely optimized to a low cost by giving up on progress
            // must not beat one that gets there).
            let ocp = ilqr::Ocp {
                path: &path,
                start: ego,
                goal,
                ctx,
            };
            let best = ctx.time("traj_opt", || {
                let mut best_hit: Option<(f64, usize, ilqr::Solution)> = None;
                let mut best_any: Option<(f64, usize, ilqr::Solution)> = None;
                for (i, cand) in candidates.iter().enumerate() {
                    let actions = tree.actions_of(cand);
                    let sol = ilqr::solve(&ocp, &actions, OPT_ITERS);
                    let dist = state_distance(sol.states.last().unwrap(), &goal);
                    if dist < GOAL_HIT_TOL && best_hit.as_ref().is_none_or(|(c, ..)| sol.cost < *c)
                    {
                        best_hit = Some((sol.cost, i, sol.clone()));
                    }
                    if best_any.as_ref().is_none_or(|(d, ..)| dist < *d) {
                        best_any = Some((dist, i, sol));
                    }
                }
                best_hit.or(best_any).expect("at least one candidate")
            });
            (tree, candidates, best)
        });
        let (_, cand_ix, sol) = best;

        if let Some(diag) = ctx.diagnostics {
            tree.record_diagnostics(diag);
            // the winning candidate before optimization…
            let pre: Vec<[f64; 2]> = candidates[cand_ix]
                .iter()
                .flat_map(|&n| tree.nodes[n].states.iter().map(|s| [s.x, s.y]))
                .collect();
            diag.record_trajectory(pre);
            // …and after
            diag.record_trajectory(sol.states.iter().map(|s| [s.x, s.y]).collect());
        }

        let controls = ctx.time("extract", || {
            repeat_last_controls(&sol.controls, ctx.horizon)
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
    fn goal_state_sits_on_the_centerline_at_target_speed() {
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &[]);
        let path = Path::new(&road.centerline);
        let ego = State {
            speed: 10.0,
            ..Default::default()
        };
        let g = goal_state(&path, ego, &ctx);
        // 10 m/s for 10 s ahead of x = 0, on the lane, facing along it
        assert!((g.x - 100.0).abs() < 1e-6, "goal x {}", g.x);
        assert_eq!(g.y, 0.0);
        assert_eq!(g.yaw, 0.0);
        assert_eq!(g.speed, 10.0);
    }

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
        let (xs, us) = crate::planning::search_tree::rollout_constrained(x0, &actions, 0.1);
        assert_eq!(us, actions);
        assert_eq!(xs[1], step(x0, actions[0], 0.1));
    }

    #[test]
    fn zero_action_point_coasts_straight() {
        let x = State {
            x: 1.0,
            y: 2.0,
            yaw: 0.0,
            speed: 5.0,
            ..Default::default()
        };
        let z = zero_action_point(x, 2.0);
        assert_eq!(z.x, 11.0);
        assert_eq!(z.speed, x.speed);
        assert_eq!((z.y, z.yaw), (x.y, x.yaw));
    }

    #[test]
    fn tracks_centerline_and_speed() {
        let ego = State {
            y: 2.0,
            speed: 6.0,
            ..Default::default()
        };
        let trace = crate::planning::test_run(&mut TreetopPlanner::default(), ego, &[], 150);
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
        let trace =
            crate::planning::test_run(&mut TreetopPlanner::default(), ego, &[obstacle], 150);
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

    #[test]
    fn plan_is_a_pure_function_of_state() {
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
