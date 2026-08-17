# Frenet lattice

`lattice/mod.rs` — `LatticePlanner`

An EM/Apollo-style sparse **space-time** lattice. Ten one-second layers sample
station, lateral position, and speed (`5 × 8 × 5` coordinate levels) over the
drag-aware reachable envelope. Time is the layer index rather than another
Cartesian-product dimension. Local station successors and neighboring
lateral/speed levels keep the search sparse, and at most 1,000 trajectory
segments are evaluated per planning call.

Every edge is a complete one-second trajectory sampled at the plant's ten
0.1-second ticks. Cubic Frenet connectors determine position and yaw; their
requested acceleration and curvature are rolled through `world_step`.
Throttle/braking, curvature, lateral grip, full-footprint road containment,
and endpoint consistency are rejected before the shared metric objective is
called. Reachable station and speed bounds use the same rolling resistance and
air drag as the plant. Lateral samples use interpolated local left/right road
widths rather than the road window's minimum width.

The layered graph is searched lazily with A* / best-first search. Edge costs
are the nonnegative complement of the production composite metric. The
lattice supplies its directly tracked Frenet station rate so progress reflects
the shorter/longer path induced by corner offsets. If the edge budget is
reached before a ten-second goal is settled, the cheapest feasible root
segment remains a safe receding-horizon fallback; full braking is reserved for
the case where no root segment is feasible.

There is deliberately **no post speed profile**. Reconstructing the winning
parent chain simply reruns the same timed edge primitive and concatenates its
already-priced controls. On an open straight the metric selects maximum
throttle; through corners the coupled station/lateral/speed search trades
track width against curvature and achievable speed.

**Seams**: `route`, `optimize` (the A\* search loop) with `cost` (the shared
cost function — nested *inside* `optimize`; it's the hot loop, called once per
sampled point of each edge A\* expands) as a nested seam, then `extract`
(sample the winning path into `xy_to_controls`).

**Diagnostics**: every feasible evaluated edge records its endpoint and its
ten-tick actual plant rollout. The number of recorded trajectories is bounded
by the segment-evaluation budget.
