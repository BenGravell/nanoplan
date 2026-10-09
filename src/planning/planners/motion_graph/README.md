# Tree and lattice motion graph

`motion_graph/mod.rs` — `GraphPlanner`, also used by treetop to seed iLQR.

Both planner entries use the same ten one-second layers, deterministic Halton samples, Frenet cubic steering,
constrained plant rollouts, progress cost, warm starts, and diagnostics.
The catalog selects a connection policy.
The strategy-specific connection and search logic lives in the `lattice` and `nearest_zap` submodules; this module owns
the shared graph and planning pipeline:

- [`nearest_zap.rs`](nearest_zap.rs) — `NearestZap` (Tree): attach each sampled target to the previous layer's nearest
  zero-action endpoint in `(x, y, wrapped yaw, speed)`.
  Each node has one incoming edge.
- [`lattice.rs`](lattice.rs) — `Lattice`: evaluate up to six distinct nearest ZAP parents per target and retain all
  feasible incoming edges.
  Layer-order dynamic programming chooses the least-cost arrival, including the parent's cost to come.
  No heuristic or priority queue is needed in this layered DAG.

Steering respects actuation limits, but its realized endpoint can differ from the sampled target.
A target therefore groups candidate arrivals; its winning arrival supplies the node's actual state.
That state is settled before the next layer is expanded.
This is an approximate state merge: other arrival states are retained for diagnostics, but do not receive independent
continuations.
Selected edge controls concatenate into a continuous plant rollout without snapping between targets or regenerating
edges during extraction.

Both modes draw 150 targets at the nominal compute budget, spread over all ten layers.
Tree evaluates at most one edge per target; lattice evaluates at most six.
Coasting and warm-start chains add at most twenty edges.
The compute percentage scales target count; these counts bound work, rather than guaranteeing a particular wall-clock
latency.

Edges check the shared constraints, local road widths, and swept vehicle-footprint road contact at absolute trajectory
times.
Unsafe coasting chains remain available as full-length optimization seeds for treetop, but ordinary samples only use
feasible parents.
The standalone planners prefer a feasible full-horizon path, otherwise use the deepest feasible prefix followed by
braking; if no prefix is feasible, they brake immediately.
Braking beyond that prefix is a fallback, not a verified safe path.

Diagnostics record every retained incoming edge's actual trajectory and endpoint, including alternative lattice
arrivals.
Timing uses `route`, `warm_start`, `optimize`, `cost`, and `extract`.
