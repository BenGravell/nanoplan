# Sampling MPC (judo)

`SamplingPlanner<PredictiveSampling>`, `SamplingPlanner<Cem>`, and `SamplingPlanner<Mppi>` optimize piecewise cubic
motion in Frenet coordinates.
The sampling and aggregation strategies are adapted from judo; the shared planner supplies trajectory construction,
vehicle rollout, and selection.

Each knot is a cubic endpoint: `[station relative to ego, station speed, lateral offset, lateral speed]`.
The ego fixes the initial boundary.
Four segments span the shared 10-second planning horizon, with knots at 2.5, 5, 7.5, and 10 seconds.
Adjacent segments share position and velocity (C1 continuity).
`CubicPolynomial::from_boundary` fits each coordinate, and `planning::frenet::Motion` converts its derivatives into
Cartesian motion.
Commands are sampled at tick midpoints and compensated for drag before integrating through the shared plant.

## Search and execution feasibility

Search rewards use the shared finite soft constraint costs.
This lets Gaussian exploration move out of infeasible regions.
A separate feasibility flag checks whether a rollout can actually be selected for execution:

- Requested commands satisfy vehicle limits before any plant clamping.
- States are finite and do not reverse or exceed the available road window.
- The swept vehicle footprint clears road barriers, with the same clearance padding used by the Frenet sampler.
- Local, asymmetric road bounds and predicted actor footprints are respected.

These checks live in `planning::feasibility`, shared with `frenet_sampling`.
They cover the whole planning horizon even when only one control is requested.
Requests longer than the planning horizon are evaluated for their entire length; controls are never extended by
unchecked repetition.

Feasible trajectories always outrank infeasible ones.
Among feasible trajectories, the shared progress objective determines the winner.
The best feasible executable rollout is retained across all seeds and optimization iterations.

- **Predictive sampling** keeps the best sample in the current feasible set.
- **CEM** fits its mean and standard deviation to feasible elites when available.
- **MPPI** uses only feasible samples for its reward-weighted mean when available.

If an iteration has no feasible samples, finite rewards still guide its search, but those samples cannot replace the
feasible executable incumbent.
Every averaged endpoint sequence is rolled out and validated again: safe inputs do not imply a safe mean.
Invalid Frenet charts are excluded from optimizer statistics entirely.

## Seeds, continuation, and fallback

The planner includes an accelerating Frenet seed, slower acceleration, coasting, moderate braking, and full
road-following braking.
Controller-generated seeds are converted to cubic endpoints for optimization; their original validated control sequences
also remain eligible for execution.
With actors present, left and right detour seeds let the search explore passing as well as stopping.

Warm endpoints come from the selected plant rollout, shifted forward one tick and stored in world coordinates.
The last validated control sequence is separately shifted and rechecked against the current road and actors.
Both are discarded when the ego diverges from its predicted position.

Road-following braking undergoes the same full-horizon validation as any other candidate.
If no validated trajectory exists, the planner issues emergency road-following braking without caching it as safe.
An already unavoidable collision is not made safe by assigning it a finite optimization cost.

Compute budget scales sampled rollouts and refinement iterations.
Conservative seeds and fallback validation are always performed.
Diagnostics display the finite final-iteration rollouts, including infeasible exploration candidates.
Tests check swept contacts before collision response, unsafe averages, stale-plan revalidation, full-horizon
predictions, and live curved-track contact counts.
