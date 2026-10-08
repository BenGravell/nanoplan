# Sampling MPC (judo)

`SamplingPlanner<PredictiveSampling>`, `SamplingPlanner<Cem>`, and `SamplingPlanner<Mppi>` optimize piecewise cubic
motion in Frenet coordinates.
The sampling and aggregation strategies are adapted from judo; the shared planner supplies trajectory construction,
vehicle rollout, costs, and warm starts.

Each knot is a cubic endpoint with four values:
`[station relative to ego, station speed, lateral offset, lateral speed]`.
The ego's projected position and velocity fix the first boundary.
Four segments span the shared 10-second planning horizon, with knots at 2.5, 5, 7.5, and 10 seconds.
Adjacent segments share position and velocity, giving C1 continuity.
`CubicPolynomial::from_boundary` fits each coordinate, and `planning::frenet::Motion` converts its derivatives into
Cartesian state, acceleration, and curvature.

The initial nominal samples a single centerline-recovery cubic using a reachable sustained-acceleration target from
`planning::frenet::longitudinal_targets`.
Deterministic Gaussian QMC noise perturbs all four endpoint dimensions:

- **Predictive sampling** keeps the best sampled trajectory.
- **CEM** fits the mean and per-dimension standard deviation to its best candidates.
- **MPPI** averages endpoints with Boltzmann reward weights, scaled by the median cost spread.

Each optimization pass includes the unchanged nominal.
An averaged update is rolled out and retained only if it scores at least as well as the best sample.
Invalid Frenet motions (backwards station motion, singular charts, or points outside the road window) are excluded from
optimizer statistics.

Controls are evaluated at tick midpoints, compensated for drag, and integrated through `world_step`.
The shared soft constraint costs score those actual vehicle states and the requested commands.
The winning rollout supplies the returned controls, so extraction uses the same motion and plant as scoring.
The requested output length does not change the optimization horizon.
If no valid motion is found, the planner returns braking controls.

For warm starts, the winning cubic is sampled at each endpoint time plus one tick; the final segment is extrapolated by
that tick.
These targets are stored in world coordinates and projected into the next road window, relative to the new ego.
Warm starts are discarded if the ego diverges from its predicted next position or the shifted targets no longer have a
valid Frenet representation.

Compute budget scales the number of rollouts and refinement iterations.
Diagnostics record the valid final-iteration rollouts as trajectories and points.
Timing spans are `route`, `warm_start`, `optimize`, `cost`, and `extract`.
