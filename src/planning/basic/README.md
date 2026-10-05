# Basic cubic

`basic/mod.rs` — `BasicPlanner`

A small exhaustive search over centerline-following cubic trajectories.
It anchors relative station zero at the ego projection and samples several stations and terminal speeds on each side of
the zero-thrust nominal rollout.
Their Cartesian product gives 180 candidate trajectories at 100% compute budget.
Both sample counts scale with the square root of the compute budget, so the total scales approximately linearly, from 16
candidates at 5% to 880 at 500%.
Each interval retains at least two samples.
Both extrema and the nominal value are included, with the nominal value belonging to the upper interval.
Maximum braking and acceleration rollouts through the simulation API bound the interval, including rolling resistance
and aerodynamic drag; braking stops at zero speed.
Each target has zero relative yaw; station and terminal speed are sampled independently.
`lateral_target_correction` measures the ego's signed lateral offset using the road heading at its projection.
It converts that offset into a world-space displacement toward the opposite side of the centerline, computed once per
plan.
`corrected_target` adds this displacement to each sampled centerline position, retaining the terminal road heading and
sampled speed.
The displacement uses the road frame at the ego, not the terminal road frame.
For example, on a straight road an ego 3 m left of the centerline gets targets 3 m right of it; an ego on the centerline
gets unshifted targets.
This strengthens the initial lateral correction when only the first control of each ten-second cubic is executed before
replanning.
It addresses the observed rolling-window regression; it is a heuristic, not a general guarantee of convergence on
curves.
Shifted candidates still pass the full feasibility checks below.
A single cubic connects ego to each target at the fixed planning horizon.
The highest-scoring feasible full-horizon rollout supplies the requested control prefix.
Targets outside the available road geometry are skipped.
If none is feasible, it follows the centerline while braking to rest.

Candidates use the shared hard constraints and metric objective.
Each cubic's controls are realized through the vehicle model before scoring, including road barrier collisions and
predicted actors.

**Seams**: `route` (build and project onto the path), `fit` (generate and select candidates), with `cost` nested inside
candidate evaluation.

**Diagnostics**: only feasible candidate rollouts as trajectories.
