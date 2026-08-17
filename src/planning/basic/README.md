# Basic cubic

`basic/mod.rs` — `BasicPlanner`

A small exhaustive search over centerline-following cubic trajectories. It
tries three lookahead distances at three traversal durations, extends each
candidate along the centerline to fill the requested horizon, and selects the
lowest-cost feasible rollout. If none is feasible, it brakes.

Candidates use the shared hard constraints and metric objective. Each cubic's
controls are realized through the vehicle model before scoring, including road
barrier collisions and predicted actors.

**Seams**: `route` (build and project onto the path), `fit` (generate and
select candidates), with `cost` nested inside candidate evaluation.

**Diagnostics**: every evaluated candidate rollout as a trajectory.
