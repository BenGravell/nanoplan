# RRT*

`rrt_star/mod.rs` — `RrtStarPlanner`

Rapidly-exploring Random Tree Star: grows a tree of poses from the ego's
current state toward (station, lateral) samples in the road frame, connects
each new node to the cheapest collision-free nearby parent, and rewires
existing nodes when a cheaper path through the new node appears (the "star"
— plain RRT would just keep the first parent found, which isn't
asymptotically optimal).

**Despite the name, the sampling isn't actually random.** `plan()` samples
`GRID_STATIONS × GRID_LATERALS` points from a fixed, road-geometry-informed
grid — the same idea as the [Frenet lattice's](../lattice/README.md#frenet-lattice) station-
layers-by-laterals grid — then an equal number more from a 2D Halton
low-discrepancy sequence (`van_der_corput`, paired in bases 2 and 3) over
the same domain, filling in what the grid's fixed points miss with
well-distributed rather than clustered coverage. Both are pure functions of
the ego state and road context (`plan_is_a_pure_function_of_state` pins this
down), so no `Rng` appears anywhere in this module — unlike
[PI²-DDP](../pi2ddp/README.md#pi2-ddp), which still samples pseudo-randomly for its rollouts.
The grid runs first, in ascending-station order, building a connected
backbone across the full planning horizon before the Halton pass's
arbitrarily-ordered targets are tried, so they almost always land near an
existing node instead of failing for lack of one.

**The steering function is differential flatness, not a straight line or an
arc.** A unicycle/bicycle's heading (`atan2(y', x')`) and curvature
(`(x'y'' - y'x'') / |·|^3`) are both fully determined by its flat outputs
`(x, y)` and their derivatives — so `CubicSteer` fits independent cubic
polynomials to `x(s)` and `y(s)`, matching position and heading direction.
Acceleration is read back as a control, not treated as a state boundary.

**Steering-angle limiting, not post-hoc curvature rejection, is what makes
the tree grow at all.** Early on, this module aimed each new edge straight
at its sample (or matched every node's heading to the lane); either way, two
independently-chosen directions connected by a short flat-output curve can
need far more curvature than any real car has, and nearly every candidate
steer failed the curvature check. `max_yaw_change(step_len)` caps how far a
new edge's direction may turn away from its parent's own heading before the
cubic is even built; the finished edge is still checked against
`MAX_ABS_CURVATURE`. A real swerve is therefore built from several small,
individually gentle turns rather than one edge trying to do it all.

**Every edge moves forward in Frenet station.** Nearest-neighbor search,
parent candidates, and rewire candidates are all restricted to the correct
side of the new node's station (behind for parents, ahead for rewiring).
Early versions picked "nearest" by raw Euclidean distance alone, which could
pick a node already *further along* than a sample that was merely close to
it laterally — steering "toward" the sample then walked backward in station,
and stitched into the winning path's arc-length parameterization, made the
ego's own extracted trajectory momentarily reverse in `x` (caught by
eyeballing this module's own closed-loop test trace, not just its
pass/fail).

**A spatial index and k-nearest bounding keep it real-time.** The three
neighbor queries above were originally linear scans over every node, so the
per-tick cost grew with the square of the tree size — the planner's dominant
latency (tens of ms at p95). Two changes fix it while leaving the tree it
builds essentially unchanged:

- **An [`rstar`](https://docs.rs/rstar) R\*-tree** (a robust, pure-Rust,
  wasm-compatible spatial index) holds every node's position, grown one node
  at a time alongside `nodes`. Nearest-behind is its lazy nearest-first
  iterator stopped at the first node behind the target; near-vertex queries
  are its `nearest_neighbor_iter_with_distance_2` cut at `NEIGHBOR_RADIUS_M`.
  Each is `O(log n)` instead of `O(n)`.
- **`K_NEIGHBORS` bounds** the candidate parents and rewire targets to the
  closest few — a *k*-nearest RRT* rather than an every-node-in-radius one
  (both asymptotically optimal). Without it, the count of vertices inside the
  radius still grows with the tree; the closest ones are also the only ones
  that tend to win (a near parent is a short, cheap edge), so this barely
  changes the result while bounding the steer + feasibility + edge-cost work
  per new node.

With the linear scans gone, the remaining hot spot was `Path::project` (an
`O(centerline-length)` scan, run for every sampled point of every candidate
edge). Since RRT* already knows each segment's rough station, it calls
`Path::project_near` — the
same projection restricted to a generous arc-length window around the hint,
`O(window)` and exact. Together these bring p95 well under 10 ms and p100
under 50 ms on the synthetic batch (from ~55 ms / ~140 ms).

**Warm start, with hysteresis, is what makes obstacle avoidance consistent
tick to tick.** `RrtStarPlanner` remembers `prev_path`, last tick's winning
polyline, and replays whatever part of it is still ahead of the ego and
still collision-free against this tick's actors as a ready-made chain of
nodes before the grid/Halton sampling below runs. Without this, a tree
rebuilt from independent samples every 0.1 s tick can find a
differently-shaped detour
each time; since the simulator only ever executes one control per plan, a
closed-loop trajectory stitched from many such plans doesn't inherit any
single one's safety margin — the exact failure the `swerves_around_stopped_obstacle`
test caught (realized clearance well under any individual plan's own
`COLLISION_MARGIN_M`). Goal selection then *continues* a warm-started node —
takes its deepest node directly — as long as the replay still reaches within
`WARM_VIABLE_BAND_M` of the furthest progress any leaf makes, so a good
detour, once found, isn't abandoned for a marginally-cheaper alternative next
tick, while a stale replay that has fallen behind gives way to the fresh
tree before an obstacle. (This band replaced an older
one-`PROGRESS_TOLERANCE_M`-bucket margin that the per-tick progress jitter
kept crossing.)

**Deterministic bypass seeding is what makes a good detour reliably
*findable* in the first place.** Before the grid/Halton loop runs, every
actor gets a fixed, unconditional ramp of candidate waypoints tried on both
sides (station offsets `[-20, -10, -3, 3, 10, 20]` m around it, lateral
offset ramping `0.25× → 0.6× → 1.0× → 1.0× → 0.6× → 0` of a safe bypass
distance) via the same `try_extend` the general sampling loop uses, seeded
in increasing-station order so each waypoint chains onto the previous one on
the same side. Randomized "informed sampling" (try a safe offset next to a
random actor with some probability) found a wide detour on some ticks and a
narrower one on others — the same consistency problem warm start addresses,
one level up. Trying identical candidates every tick means the tree finds
(and keeps refining, via warm start and rewiring) the *same* detour every
time.

**Feasibility and edge cost both go through the [shared metric
objective](../../metrics/README.md#the-shared-metric-objective).** `feasible` additionally enforces its
own tighter margins before ever calling it — `drivable_bound` (the road's
own `half_width` less `DRIVABLE_MARGIN_M` = 0.5 m, so it holds just inside
the shared function's road-edge reject on whatever road is being driven) and
`COLLISION_MARGIN_M` (3.0 m, ahead of the shared car-width point proxy)
— headroom for the fact that a curve is only checked at `STEER_SAMPLES`
discrete points, so the true closest approach between samples can dip a
little further than what gets tested. `edge_cost` sums the composite-metric
cost at its sampled points; curvature comes from `CubicSteer::curvature`, a
closed-form fact about the already-fixed candidate curve, not a search
gradient.

**Effective progress — not raw distance, and biased toward the side already
committed to — decides the goal.** Ranking on raw station bucketed to
`PROGRESS_TOLERANCE_M` (rather than compared exactly) is most of it: without
bucketing, a node a hair's-breadth further along but squeezing past an
obstacle would beat a node a few centimeters short but giving it a much wider
berth, every single time, since station is compared before cost ever gets a
say. But raw progress alone
still let a *fresh* corner-cutter on the opposite side of an obstacle steal
the goal from the smooth continuing detour whenever it reached a hair further
— a left detour and its mirror-image right reach near-identical progress at
near-identical cost, so which one won was effectively a coin flip that landed
differently each tick and the ego chattered between the two. So each node
also carries `peak_lateral`, the furthest-out *signed* offset along its path
(which side it swings to and how far), and the goal ranks on **effective
progress**: station minus `CONTINUITY_WEIGHT · (peak_lateral −
committed_bias)²`, where `committed_bias` is an EMA of the executing plan's
side. A path on the wrong side loses a double-digit-metre chunk of effective
progress — several buckets — so it can't win by reaching marginally further,
while on an open or gently curved lane every path has `peak_lateral ≈ 0` and
the term is inert. `CONTINUITY_WEIGHT` is only a tie-stability bias for the
receding-horizon search; candidate cost still comes exclusively from the
metric composite.

**Seams**: `route` (build the `Path`), `warm_start` (custom — replaying the
previous winning path), `optimize` (the grid-plus-Halton tree-growing
loop; the deterministic bypass seeding and the final extract step aren't
timed separately since they're comparatively cheap), `extract` (resample the
winning path — itself a `Path` built from the tree's polyline —
at `v * dt` intervals and convert to controls via the same technique as the
[Frenet lattice's](../lattice/README.md#frenet-lattice) `xy_to_controls`). `cost` (the shared
cost function) nests inside all three of `warm_start`, the (untimed)
deterministic bypass seeding, and `optimize` alike, since `feasible` and
`edge_cost` — where it's called, once per sampled point — are shared by
every caller of `try_extend`.

**Diagnostics**: every tree node (after the root) as a `point`, and the
sampled polyline of the edge that added it as a `trajectory` — the whole
search tree considered, not just the winning path, mirroring the lattice's
approach.
