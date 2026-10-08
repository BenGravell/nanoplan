# Metrics

## The shared metric objective

Planners maximize forward progress subject to collision, road, and vehicle-dynamics constraints.
`Constraints::point_cost(sample)` returns `-metrics::progress_score(...)` for feasible samples and `f64::INFINITY` for
collision or road-bound violations.
There are no comfort weights or safety scores.
The viewer displays the final normalized progress score.
`evaluate` takes a state slice, timestep, reference path, and optional endpoint station hints; it projects only the
first and last states.
Hints preserve the intended branch on overlapping road segments.
Scoring does not construct `TrajectoryKinematics` or any per-state projection arrays.
`TrajectoryKinematics` stores states, controls, time, and lateral acceleration without road projections.
The viewer computes the full station sequence only when Station carpet coloring is selected.

- **Progress** is the actual Frenet station delta above zero-acceleration coasting: `(delta_s - v_ego * t) / (0.5 *
  MAX_LON_ACCEL * t²)`.
  Subtracting the coasting distance from both reference distances cancels the initial-speed term in the denominator.
  `progress_score` requires `t > 0`, making the denominator positive; the numerator is signed.
  Constant acceleration without resistance scores `acceleration / MAX_LON_ACCEL`.
  Actual rollout progress retains road geometry and vehicle dynamics; the reference needs no friction simulation.
  Braking or falling behind coasting scores negative, and exceeding maximum-acceleration reference progress scores above
  1.
  The trajectory score is computed directly at the final sample; a single-sample trajectory scores 1 and an empty one
  scores 0.
- **Collision and road constraints** reject samples inside the shared car-width actor clearance or outside the local
  drivable bounds.
  Planners with rectangular footprints additionally check actor and barrier contact.
- **Kinodynamic constraints** remain enforced by the shared plant and planner feasibility checks, including acceleration,
  curvature, and lateral-acceleration limits.
- **Actor prediction** uses `prediction::predict`, the shared lane-aware kinematic model.

**No analytic derivatives, by construction.** `point_cost` takes already-known numbers — position, speed, curvature,
accel — and returns a plain `f64`; there's no gradient anywhere in its signature or its callers.
This is a deliberate design constraint, not an oversight: nanoplan never *provides* a derivative of its cost or dynamics
— both are black-box scalars, and nothing may demand an analytic gradient of either.
Most planners live entirely within that constraint by sampling and comparing candidates.
The one family that genuinely optimizes —
[treetop's iLQR](../planning/planners/treetop/README.md#ilqr-treetop-finite-differences) — respects it at the interface:
it consumes exactly the same black-box scalars and differentiates them **numerically** (central finite differences),
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
