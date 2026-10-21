# Motion search

Tree, lattice, Frenet, predictive sampling, CEM, and MPPI use one `MotionPlanner<S>` in `mod.rs`.
It uses the [Strategy pattern](https://refactoring.guru/design-patterns/strategy): `SearchStrategy` supplies candidate
construction and search, while the planner owns warm-start validity, fallback, caching, and output length.
The viewer and profiler entries retain their names.

Cubic boundary fitting, Cartesian command generation, plant rollouts, and final feasibility checks are shared.
Frenet and Cartesian cubic steering and command conversion live in `common::steering`; raw-control and analytic-motion
rollouts live in `common::rollout`.
The planner owns Frenet endpoint conversion and candidate scoring.
Strategies retain the differences that define their searches:

| Strategy | Construction                                       | Selection and refinement                          |
| -------- | -------------------------------------------------- | ------------------------------------------------- |
| Frenet   | Broad terminal grid, one cubic per coordinate      | Best feasible terminal-progress score             |
| PS       | Four cubic segments, Gaussian perturbations        | Best feasible sample                              |
| CEM      | Same as PS                                         | Elite mean and adaptive variance                  |
| MPPI     | Same as PS                                         | Reward-weighted feasible mean                     |
| Tree     | Layered Halton samples, nearest zero-action parent | Deepest feasible path, then accumulated edge cost |
| Lattice  | Same layers, up to six parents per target          | Layered minimum-cost arrivals                     |

`refinement.rs` supplies the common whole-trajectory search loop; the small PS/CEM/MPPI implementations only change
elite selection and distribution updates.
`frenet.rs` retains its distinct endpoint grid.
`graph.rs` owns layered construction, with connection policies in `nearest_zap.rs` and `lattice.rs`.
A sampling sequence is not materialized as graph nodes and edges when no parent search is needed.

The planner panel can additionally enable [iLQR post-processing](../../README.md#optional-trajectory-optimization) for
any of these generators.
Accepted refined states and controls replace the warm trajectory before the next search.

## Shared execution

Every strategy returns a full-horizon candidate that passes raw-control dynamics checks and trajectory feasibility.
The common planner stores its realized states and controls, checks the predicted next position before warm starting, and
returns the requested prefix.
Validation covers at least ten seconds, including when only one control is requested; requests beyond that are simulated
and validated for their entire length.

If search fails, the planner tries road-following braking, then the previous controls aligned to the current state and
extended with braking.
Both must pass the same full-horizon validation.
If neither is feasible, emergency braking is issued without caching it as safe.

MPC warm endpoints are sampled from the previous plant rollout, shifted one tick and reprojected onto the current road.
Graph search replays shifted controls as a hot chain and samples nearby states.
Frenet regenerates its grid each call.

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
The grid sampler's endpoint bounds reuse [`planning::frenet`](../../frenet.rs); coordinate transforms live in
[`common::steering::frenet`](../../../common/steering/frenet.rs).

## Whole-trajectory launch, score, and validation

Below 2 m/s, a straight maximum-acceleration prefix establishes heading before fitting the cubics.
All whole-trajectory candidates include that prefix, and the cubic duration is reduced so the total planning horizon
remains 10 seconds.
Stations are relative to the projected state after launch.

Every batch follows the original Frenet sampler's evaluation order:

1. Reject invalid Frenet transforms and commands that violate vehicle limits at either the analytic or simulated speed.
   The shared vehicle rollout integrates without clipping and aborts at the first invalid command or state.
2. Score survivors using [`metrics::evaluate`](../../../metrics/mod.rs): realized terminal progress over the full horizon.
   Intermediate progress rewards are not accumulated.
   Stable sorting preserves generation order when scores tie.
3. Check cached rollouts against road boundaries, swept footprint barriers, and predicted actors in score order.
   Frenet and PS stop at the first feasible candidate; CEM stops after collecting its elites; MPPI checks the full batch.

Only feasible candidates enter optimizer updates or become executable incumbents.
Every averaged endpoint sequence is rolled out, scored, and validated again.
An unsafe mean or a batch without feasible samples cannot replace the best validated trajectory.
Diagnostics display scored candidates without triggering additional collision checks.

Validation covers the full planning horizon even when only one control is requested.
Longer output requests are simulated and validated for their entire length.

## Layered construction

`Graph::grow` accepts the number of layers, from one through the trajectory's tick count.
Tick boundaries partition the whole horizon even when the layer count does not divide it evenly.
Tree and lattice use ten layers across 100 ticks (ten seconds at the standard 0.1 s timestep).
Either can use the shared iLQR post-process through the planner panel's checkbox.

Both connection modes draw 150 targets at the nominal compute budget, spread over all layers.
Tree evaluates one nearest zero-action parent per target; lattice evaluates up to six and retains feasible incoming
edges.
Layer-order dynamic programming selects the least-cost arrival.
Coasting and warm-start chains add at most two edges per layer.

Graph steering clamps commands to vehicle limits, so its realized endpoint can differ from the target.
The winning arrival supplies the node's actual state before the next layer grows.
Alternative arrivals remain available for diagnostics but do not receive independent continuations.
Selected edge controls form a continuous plant trajectory.
Whole-trajectory sampling instead rejects cubics that exceed vehicle limits and uses the shared straight launch below 2
m/s; graph steering starts directly from the ego state.

Edges retain their accumulated stage-progress costs and time-dependent feasibility checks.
Whole-trajectory strategies retain terminal-progress scoring.
These objectives are deliberately distinct: changing them would change the search.

The coasting chain keeps every layer populated, but unsafe nodes cannot parent ordinary graph samples.
Standalone tree and lattice try full paths first, then shorter feasible prefixes with a braking tail.
The complete candidate, including that tail, must pass the shared validation before execution and caching.
Diagnostics retain every incoming edge's realized trajectory, including alternative lattice arrivals.

Tests cover variable layer counts and continuous paths, the shared lifecycle across all six strategies, distinct Frenet
candidate generation, launch boundaries, terminal scoring, ordered validation, unsafe means, stale plans, finite road
windows, and closed-loop driving with actors and curved tracks.
