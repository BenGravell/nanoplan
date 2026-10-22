# Metrics

## The shared metric objective

Planners maximize forward progress subject to collision, road, and vehicle-dynamics constraints.
`Constraints::point_cost(sample)` returns `-metrics::progress_score(initial_station, sample.station)` for feasible
samples and `f64::INFINITY` for collision or road-bound violations.
There are no comfort weights or safety scores.
The viewer displays the final progress in meters.
Both trajectory scoring and planner costs use `metrics::progress_score` for signed Frenet station progress in meters.
`evaluate` takes a state slice, reference path, and optional endpoint station hints; it projects only the first and last
states.
Hints preserve the intended branch on overlapping road segments.
Scoring does not construct `TrajectoryKinematics` or any per-state projection arrays.
`TrajectoryKinematics` stores states, controls, time, and lateral acceleration without road projections.
The viewer computes the full station sequence only when Station carpet coloring is selected.

- **Progress** is the signed Frenet station delta in meters: `delta_s = s_final - s_initial`.
  There is no coasting subtraction or maximum-acceleration normalization, so greater distance traveled earns a higher
  score.
  The trajectory score uses the final sample; single-sample and empty trajectories score 0.
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
The one family that genuinely optimizes — [iLQR](../planning/planners/trajectory_optimization/README.md#ilqr) — respects
it at the interface: it consumes exactly the same black-box scalars and differentiates them **numerically** (central
finite differences), probing `point_cost` and `step` a few dozen times per timestep instead of once.
The scalar interface stays the single source of truth for what "good" means; no second, analytically-differentiated
definition of the cost can drift away from it.
Where a planner needs curvature as an input, it gets it one of two ways, both compatible with that constraint:

- **A closed-form fact about an already-*fixed* candidate curve.** `planning::frenet::Motion` evaluates the curvature of a
  specific flat-output polynomial it already committed to — a geometric property of one candidate, not a gradient used to
  choose the next one.
- **A value recovered from a sampled trajectory.** The space-time lattice derives curvature from heading change over
  distance along its timed connector before rolling the resulting control through the plant.

**What stays planner-specific.** Sampling layouts, warm starts, feasibility margins, and search topology remain
planner-specific, but they do not add another outcome score.
Numeric optimizers replace `point_cost`'s `f64::INFINITY` with the finite, depth-scaled penalty.
Its base exceeds the full signed station-cost range over the scored horizon, with `constraints::HARD_VIOLATION_PENALTY`
as an additional margin; the lattice and Frenet sampling planner propagate the actual infinity and reject the candidate
outright.
