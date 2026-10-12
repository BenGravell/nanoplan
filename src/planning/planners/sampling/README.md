# Frenet sampling and sampling MPC

`SamplingPlanner<O>` supplies one pipeline for the Frenet grid, predictive sampling, CEM, and MPPI.
The existing viewer and profiler entries keep their names.
`FrenetSamplingPlanner` uses the distinct `Frenet` strategy, rather than aliasing predictive sampling.

## Candidate generation and refinement

- **Frenet** (`frenet.rs`) generates a broad grid of terminal station, speed, lateral offset, and lateral speed.
  Each lateral cubic gets its own reachable longitudinal bounds, including resistance and the curved Frenet metric.
  The grid includes a reflected lateral target and sustained-acceleration targets.
  A single cubic per coordinate spans the remaining horizon; there is no iterative refinement.
- **Predictive sampling** (`ps.rs`) perturbs a nominal sequence with fixed-scale Gaussian noise and retains the best
  sample.
- **CEM** (`cem.rs`) fits the mean and per-knot standard deviation to up to six feasible elites.
- **MPPI** (`mppi.rs`) computes a reward-weighted mean of all feasible samples in the batch.

The three MPC strategies share deterministic Halton Gaussian sampling and four cubic segments.
Each knot specifies `[relative station, station speed, lateral offset, lateral speed]`.
Adjacent segments share position and velocity.
Seeds include an accelerating cubic and road-following acceleration, coasting, and braking rollouts.
The seed cubic is checked against vehicle limits, since reachable endpoints can still produce acceleration overshoot.
When actors are present, left and right detour seeds broaden the search.

Compute budget scales the Frenet grid density or the MPC rollout count and refinement iterations.
The grid sampler's endpoint bounds and transforms reuse [`planning::frenet`](../../frenet.rs).

## Shared launch, score, and validation

Below 2 m/s, a straight maximum-acceleration prefix establishes heading before fitting the cubics.
All candidates include that prefix, and the cubic duration is reduced so the total planning horizon remains 10 seconds.
Stations are relative to the projected state after launch.

Every batch follows the original Frenet sampler's evaluation order:

1. Reject invalid Frenet transforms and commands that violate vehicle limits at either the analytic or simulated speed.
   The shared vehicle rollout integrates without clipping and aborts at the first invalid command or state.
1. Score survivors using [`metrics::evaluate`](../../../metrics/mod.rs): realized terminal progress over the full horizon.
   Intermediate progress rewards are not accumulated.
   Stable sorting preserves generation order when scores tie.
1. Check cached rollouts against road boundaries, swept footprint barriers, and predicted actors in score order.
   Frenet and PS stop at the first feasible candidate; CEM stops after collecting its elites; MPPI checks the full batch.

Only feasible candidates enter optimizer updates or become executable incumbents.
Every averaged endpoint sequence is rolled out, scored, and validated again.
An unsafe mean or a batch without feasible samples cannot replace the best validated trajectory.
Diagnostics display scored candidates without triggering additional collision checks.

Validation covers the full planning horizon even when only one control is requested.
Longer output requests are simulated and validated for their entire length.

## Continuation and fallback

MPC warm endpoints come from the selected plant rollout, shifted forward one tick and stored in world coordinates.
They are reprojected and revalidated on the next call and discarded if the ego diverges from its predicted position.
Frenet regenerates its grid independently each call.

If no candidate passes, every strategy tries road-following braking, then the previous control sequence aligned to the
current state and extended with braking, validating each across the complete horizon.
If neither is feasible, emergency braking is issued without retaining it as a validated trajectory.

The shared tests cover launch boundaries, terminal scoring, early dynamics rejection, ordered collision checks, unsafe
means, stale plans, finite road windows, and closed-loop driving with actors and curved tracks.
