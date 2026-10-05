# Basic cubic

`basic/mod.rs` — `BasicPlanner`

A small exhaustive search over centerline-following cubic trajectories.
It anchors relative station zero at the ego projection and samples 10 stations and 5 terminal speeds on each side of the
zero-thrust nominal rollout.
Their Cartesian product gives up to 200 candidate trajectories.
Both extrema and the nominal value are included, with the nominal value belonging to the upper interval.
Maximum braking and acceleration rollouts through the simulation API bound the interval, including rolling resistance
and aerodynamic drag; braking stops at zero speed.
Each target has zero relative yaw; station and terminal speed are sampled independently.
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
