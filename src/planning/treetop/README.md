# Treetop (RRT / iLQR / RRT+iLQR)

`treetop/` — `RrtPlanner` (`rrt.rs`), `IlqrPlanner` (`ilqr.rs`),
`TreetopPlanner` (`mod.rs`)

A port of [**treetop**](https://github.com/BenGravell/treetop), a
tree-initialized trajectory-optimizing planner: an ego motion sampling tree
provides a strong, collision-aware initial guess at a good path to the
goal, and iLQR (iterative Linear Quadratic Regulator) optimizes that guess
into a smooth trajectory whose solution warm-starts the tree next cycle.
Like the judo port, one upstream codebase yields several registry entries
from one directory — here deliberately three, so the tree and the optimizer
are each measurable *alone* before the coordination glue combines them:

```
treetop/
├── mod.rs   shared OCP core (treetop core/: limits, constrained rollout, goal) + TreetopPlanner glue (treetop planner.h)
├── rrt.rs   the ego motion sampling tree (treetop tree/) — RrtPlanner
└── ilqr.rs  the iLQR solver (treetop ilqr/), finite-difference derivatives — IlqrPlanner
```

nanoplan's kinematic model uses treetop's same pose/speed kinematics, but
with a four-dimensional state `(x, y, yaw, speed)` and direct
acceleration/curvature commands. Three adaptations recur throughout
(see the module doc): treetop's fixed user-placed goal pose becomes a **rolling lane
target** (`goal_state`: the centerline pose a planning horizon ahead, at
the target speed); treetop's static circular obstacles become **moving
actors priced through the shared metric objective** at the absolute time each
state is reached; and treetop's `std::mt19937` sampling and action jitter
are replaced by the **shared Halton QMC sequence** (jitter dropped
entirely — its purpose is randomized restarts), so all three planners are
pure functions of the ego state, pinned by `*_is_a_pure_function_of_state`
tests.

Shared `mod.rs` core, used by both halves: the horizon is `TICKS = 100`
ticks (10 s, the common `PLANNING_HORIZON_S`), split into `SEGMENTS = 10`
steering segments of `STEER_TICKS = 10` ticks, plus the shared rollout that
advances every candidate through `simulation::world_step`.

## RRT (treetop tree)

`treetop/rrt.rs` — `RrtPlanner`

An RRT variant shaped by its downstream job — feeding a trajectory
optimizer — rather than by asymptotic optimality (contrast
[RRT*](../rrt_star/README.md), which rewires toward the shortest path):

- **Time-layered, fixed-depth growth.** The tree has exactly `SEGMENTS`
  layers past the root, each one steering segment later in time, so *any*
  leaf in the final layer closes a full-horizon action sequence of exactly
  `TICKS` controls — precisely the input the iLQR pass wants. Moving
  obstacles come free: a layer's states have a known absolute time, so
  collision checks price actors where they *will be*.
- **Steering in action space.** `steer_actions` fits the shared cubic
  flat-output connector between two states' position and velocity boundary
  conditions, reads acceleration and curvature off the polynomial derivatives
  — the same differential-flatness idea as RRT*'s `CubicSteer` — and
  realizes those direct commands through the shared rollout.
  A secant against the start heading infers forward/reverse. The steer
  executes only its first segment; goal-directed samples steer along a
  cubic spanning the whole remaining horizon and keep just the first
  second of it.
- **Zero-action-point parenting.** A sample attaches to the previous
  layer's node whose coasting endpoint is nearest in `(x, y, yaw, v)` —
  "who reaches me with the least effort" under simplifying kinematic
  assumptions. treetop builds a nanoflann kd-tree per layer for this; a
  layer here holds a few dozen nodes, so a linear scan is simpler *and*
  faster than building the index.
- **Layered sampling, three ways** (treetop's goal 0.1 / warm 0.2 / cold
  0.7 split, drawn against a Halton coordinate instead of an RNG): *goal*
  samples steer toward the goal, *warm* samples perturb around the
  previous solution's trajectory, *cold* samples cover a road-frame
  `(station, lateral, heading error, speed)` box — treetop's axis-aligned
  world-frame box bent into the road frame so it follows a curved road.
- **A zero-action fallback chain** guarantees every layer is non-empty
  (so a full-length path always exists), deliberately ignoring collisions
  — treetop's `growZap`. Such nodes carry a `collides` flag and price
  violating stages at `HARD_VIOLATION_PENALTY`, so they lose to any
  genuine alternative and surface only as a better-than-nothing brace.
- **Edge cost = the metric objective.** Every rolled-out stage is priced by
  `point_cost`; hard violations reject ordinary samples, while fallback
  chains use the finite escape penalty. Path candidates rank goal-hitters (within
  `GOAL_HIT_TOL` of the goal, treetop's `checkTargetHit` loosened from
  parking precision to lane driving) by cost-to-come, everyone else by
  distance to goal; alternates are the next-best by the same ordering
  where treetop shuffles randomly.

The standalone planner takes the best path candidate as the plan, with its
own warm start (previous plan shifted one tick, replayed as treetop's
"hot" chain and sampled around as "warm") — the plan is exactly what the
treetop planner would hand to iLQR, un-optimized, so the registry can show
what the optimization pass buys.

**Seams**: `route`, `warm_start`, `optimize` (the whole grow), `extract`;
`cost` nests inside wherever an edge is priced.

**Diagnostics**: every tree node as a point and every edge's rollout
polyline as a trajectory — the whole search considered, mirroring RRT*.

## iLQR (treetop, finite differences)

`treetop/ilqr.rs` — `IlqrPlanner`

Iterative LQR: alternate a **backward pass** — dynamic programming over
linearized dynamics and a quadratic cost expansion, producing an affine
policy `u = u_ref + scale·k + K·(x − x_ref)` — with a **forward pass**
rolling that policy out closed-loop and accepting it only if the realized
cost drop is a reasonable fraction of the expansion's prediction
(treetop's feedforward-gain scaling search: backtrack `scale` by 0.2 up to
8 times). Regularization on `Q_uu` is scaled by the gradient norm `|Q_u|`
(treetop's gradient-norm scaling) and adapts by the usual schedule —
decrease on an immediate accept, increase on a backtracked one, surge on a
rejection. The accepted trajectory is finally re-realized under the action
constraints, exactly as treetop re-rollouts its solution.

**Finite differences everywhere, per the port's design brief.** treetop
carries ~200 lines of hand-derived loss gradients/Hessians and a
closed-form dynamics Jacobian; nanoplan deliberately provides neither (see
[the shared metric objective](../README.md#the-shared-metric-objective)). So this solver
differentiates numerically: central differences over the packed
`(x, y, yaw, v, accel, curvature)` vector for the cost gradient and
(symmetrized) Hessian — 73 probes of the black-box scalar per timestep —
and central differences on `simulation::world_step` for the dynamics Jacobians
`A`, `B` (pinned against the known closed form by
`fd_dynamics_jacobian_matches_the_analytic_one`). FD Hessians of a
piecewise cost are noisy near hinge corners, so where treetop asserts
`Q_uu ≻ 0` "by construction", this port *checks* it and surges
regularization on failure rather than factorizing garbage.

One cost adaptation makes the shared scalar usable under an optimizer that
differentiates it rather than compares it:

- **Hard violations get an escape slope.** `point_cost`'s
  `f64::INFINITY` would poison every difference; PI²-DDP's flat
  `HARD_VIOLATION_PENALTY` substitution is finite but a *flat* plateau has
  zero gradient — a trajectory stuck inside a violation would see no way
  out. This planner prices a violation as
  `HARD_VIOLATION_PENALTY · (1 + depth)`, `depth` being how far inside the
  violation the sample sits (meters past the road edge, meters of overlap
  with an actor) — the same cliff at the boundary, but with a finite-
  difference-visible slope pointing back out.

The terminal state is simply one more sample of the same metric objective;
there is no separate terminal-goal score.

The standalone planner optimizes from a lane-keeping PD initial guess (the
same critically-damped tracker the judo planners use as their base
policy), or its own previous solution shifted one tick. This is trajectory
optimization at its most exposed — a purely local method only as good as
its initial guess, which is precisely the weakness the treetop
coordination exists to fix; it's kept standalone so the registry shows
that difference side by side.

**Seams**: `route`, `warm_start`, `optimize`, `extract`, with `derivs`
(all backward-pass FD work) and `rollout` (forward passes and trajectory
pricing) nested inside `optimize`. Unlike the other search planners there
is no per-call `cost` seam: the FD probes call the shared objective ~10⁵ times
per plan, and timing each call would cost more than the call — `derivs`
and `rollout` are where those calls live.

**Diagnostics**: the optimized trajectory as a polyline and its states as
points.

## treetop (RRT + iLQR)

`treetop/mod.rs` — `TreetopPlanner`

The coordinator glue, treetop's `planner.h` loop: **tree → candidates →
iLQR → best → feed back.**

1. Grow the tree (450 samples), warm-started from last tick's *optimized*
   solution — replayed as the hot chain and sampled around by the warm
   samples, so the tree keeps refining the maneuver the optimizer chose
   rather than rediscovering a different one each tick.
2. Extract the best `num_path_candidates = 2` full-horizon path
   candidates.
3. Run iLQR on each candidate's action sequence (a handful of iterations —
   the tree's near-feasible guess converges fast, where treetop's
   on-demand replans afford up to 200).
4. Select by treetop's two-tier rule: the cheapest solution that still
   *hits the goal*, else the one ending nearest it — a candidate that
   optimized to a low cost by giving up on progress must not beat one that
   gets there.
5. Store the winner's action sequence as next tick's warm start, and
   drive its first control.

The division of labor is the point, and it's the same lesson RRT*'s
warm-start section tells from the other side: the tree contributes global,
discontinuity-crossing search (which side of the obstacle, brake vs.
swerve) that a local optimizer can't do, and iLQR contributes the smooth,
limit-respecting polish (and consistent tick-to-tick refinement) that a
bang-bang sampled tree path lacks. Treetop's action jitter — a third
mechanism for hopping out of local minima — is omitted deliberately: it's
pseudo-random by nature, and determinism (plan as a pure function of
state) is worth more here than its occasional escape.

**Seams**: `route`, `warm_start`, `optimize` split into treetop's own
`TimingInfo` pair — `tree` (grow + candidate extraction) and `traj_opt`
(the iLQR passes) — then `extract`. `cost` nests inside `tree`; the iLQR
passes bury their shared-cost calls in `derivs`/`rollout` as above.

**Diagnostics**: the whole tree (nodes as points, edges as trajectories),
plus the winning candidate's pre-optimization polyline and its post-iLQR
trajectory — the before/after pair that shows what the optimizer bought.
