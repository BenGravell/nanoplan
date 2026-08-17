# Metrics

## The shared metric objective

The search-based planners — the Frenet lattice, PI²-DDP, RRT\*, the three judo-derived sampling-MPC planners (predictive
sampling, CEM, MPPI), and the three treetop-derived planners (RRT, iLQR, treetop) — all price candidates with the same
scalar objective; `bezier_toppra` and `straight` don't (see the
[planner documentation](../planning/README.md#planner-implementations) for why they're out of scope here).

`HardConstraints::point_cost(sample)` is the complement of the production metrics composite: `1 -
composite([safety, progress, comfort])`.
Every planner calls it under the same seam name, `"cost"` (see
[latency diagnostics](../planning/README.md#latency-diagnostics)).
Because safety is a multiplier and progress and comfort aggregate by average, summing this cost over a fixed-length
feasible rollout gives the planner form of the same composite objective.

- **Hard collision and off-road rejection** — [`planning/constraints.rs`](../planning/constraints.rs) returns
  `f64::INFINITY` if a sampled point is closer than the shared car-width point proxy to any actor's predicted position, or
  further than `road_half_width` from the centerline.
  That bound is the road's *actual* drivable half-width (`Road::half_width`, the same value used to generate the barrier
  geometry that `ttc` scores), passed in per plan rather than read from a fixed constant — so on a narrow street the
  reject fires at the true edge.
  A planner should reject these outright, not merely disfavor them.
- **Progress and comfort are the production metrics** — forward speed is normalized by the speed reachable under maximum
  thrust acceleration from the current speed (using the plant's rolling resistance and drag), and longitudinal/lateral
  jerk goes through `metrics::comfort::jerk_score`.
  Their weights and safety's multiplier role come directly from the `METRICS` registry.
- **Actor prediction** goes through `prediction::predict` — the lane-aware kinematic model — instead of each planner
  reimplementing prediction independently.
  An actor travelling along the route is rolled forward along the lane's curve and eased back toward its center, so on a
  bend it is priced where it will actually be rather than off on the straight tangent; oncoming and crossing traffic fall
  back to `prediction::project`.
  The rollout's `metrics::safety` metric evaluates the resulting actual future ego and actor traces, so it does not
  duplicate the planner's prediction model.

**No analytic derivatives, by construction.** `point_cost` takes already-known numbers — position, speed, curvature,
accel — and returns a plain `f64`; there's no gradient anywhere in its signature or its callers.
This is a deliberate design constraint, not an oversight: nanoplan never *provides* a derivative of its cost or dynamics
— both are black-box scalars, and nothing may demand an analytic gradient of either.
Most planners live entirely within that constraint by sampling and comparing candidates.
The one family that genuinely optimizes —
[treetop's iLQR](../planning/treetop/README.md#ilqr-treetop-finite-differences) — respects it at the interface: it
consumes exactly the same black-box scalars and differentiates them **numerically** (central finite differences),
probing `point_cost` and `step` a few dozen times per timestep instead of once.
The scalar interface stays the single source of truth for what "good" means; no second, analytically-differentiated
definition of the cost can drift away from it.
Where a planner needs curvature as an input, it gets it one of two ways, both compatible with that constraint:

- **A closed-form fact about an already-*fixed* candidate curve.** RRT\*'s `CubicSteer::curvature` evaluates the curvature
  of a specific flat-output polynomial it already committed to — a geometric property of one candidate, not a gradient
  used to choose the next one.
- **A value recovered from a sampled trajectory.** The space-time lattice derives curvature from heading change over
  distance along its timed connector before rolling the resulting control through the plant.

**What stays planner-specific.** Sampling layouts, warm starts, feasibility margins, and search topology remain
planner-specific, but they do not add another outcome score.
Numeric optimizers replace `point_cost`'s `f64::INFINITY` with the finite, depth-scaled
`constraints::HARD_VIOLATION_PENALTY`; the lattice and RRT\* propagate the actual infinity and reject the candidate
outright.
