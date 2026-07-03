# `planning`

The `Planner` trait, the `Context` planners read, the `PlannerKind` registry
used to select and compare planners, latency diagnostics, and one
subdirectory per planner implementation.

```
planning/
├── mod.rs        Planner trait, Context, PlannerKind, test harness
├── latency.rs    Latency/LatencyStats/SeamStats — see "Latency diagnostics" below
├── straight/     strawman: zero control, always
├── bezier_idm/   cubic Bezier back to the centerline + IDM speed
├── lattice/      Frenet lattice, sampled grid + dynamic programming
├── pi2ddp/       sampling-based DDP (PI²-DDP)
└── rrt_star/     RRT*, cubic-polynomial (differential-flatness) steering
```

## The `Planner` trait

```rust
pub trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}
```

A planner is given the current ego `State` and a `Context`, and returns a
control trajectory. The [`Simulator`](../simulation/README.md) applies only
the **first** control and re-invokes `plan()` next tick — this is a receding
horizon / MPC-style loop, not open-loop trajectory execution. `&mut self`
lets a planner keep state between calls (PI²-DDP warm-starts its policy this
way); planners with no state to keep, like `StraightPlanner`, are
zero-sized unit structs.

An empty return value is treated as "coast" (zero control) by the simulator,
not an error — no planner currently exercises this, but it's a legal escape
hatch for "couldn't find anything, don't do anything worse."

## `Context`

```rust
pub struct Context<'a> {
    pub centerline: &'a [[f64; 2]],   // the lane the ego should follow
    pub actors: &'a [State],          // other vehicles, current states only
    pub target_speed: f64,            // desired cruise speed
    pub dt: f64,                      // tick length of the returned controls
    pub horizon: usize,               // requested control-trajectory length
    pub latency: Option<&'a Latency>, // recorder; see below
    pub diagnostics: Option<&'a Diagnostics>, // recorder; see below
}
```

Everything a planner needs besides its own state and the ego pose. Notably:

- **`actors` is current-tick only.** Planners see no future information
  about other vehicles — if they want a prediction, they compute one
  themselves (every existing planner does simple constant-velocity
  extrapolation inline; see [`src/scenarios/README.md`](../scenarios/README.md#trajectory-replay)
  for why the *simulated* actors can be smarter than that even though the
  planner's view of them isn't).
- **`horizon` is a request, not a contract.** A planner may return more or
  fewer controls; the simulator only ever consumes the first one during
  closed-loop simulation. The viewer's future-preview feature asks for a
  larger horizon (up to 100 ticks, `PLANNING_HORIZON_S`) to draw a longer plan.
- **`centerline` is a raw polyline**, not a `Path`. Every planner that needs
  Frenet operations (arc length, projection, curvature-following) builds its
  own `scenarios::Path` from it — see [`src/scenarios/README.md`](../scenarios/README.md#path-the-frenet-helper).

## `PlannerKind`

```rust
pub enum PlannerKind { Straight, BezierIdm, Lattice, Pi2Ddp, RrtStar }
```

The selection/comparison seam. `PlannerKind::ALL` is the definitive list the
viewer's dropdown and the batch runner iterate over; `.name()` gives the
display string; `.build()` returns a fresh `Box<dyn Planner>`.

**To add another planner:**

1. Create `planning/my_planner/mod.rs` implementing `Planner`.
2. Add `pub mod my_planner;` and `pub use my_planner::MyPlanner;` to
   `planning/mod.rs`.
3. Add a `PlannerKind::MyPlanner` variant, extend `ALL`, `name()`, and
   `build()`.

Nothing outside `planning/` needs to change — the viewer, the batch runner,
and the metrics evaluator all iterate `PlannerKind::ALL` or take
`Box<dyn Planner>` generically.

## Latency diagnostics

`latency.rs` implements a minimal seam-based timing interface, described in
full in its module doc. The short version:

- A **seam** is a named timed span inside one `plan()` call:
  `ctx.time("name", || { ...work... })`. `Context::time` is a no-op wrapper
  when diagnostics aren't being collected (`ctx.latency` is `None`, as in
  every test and in the future-preview replan), so instrumentation is free
  outside of `simulate()`.
- **Standardized seam names**, used wherever the phase exists so planners
  stay comparable across the table in the viewer:

  | Seam | Meaning | Recorded by |
  |---|---|---|
  | `total` | The whole `plan()` call | the `Simulator`, not the planner — every planner gets this for free |
  | `route` | Turning `centerline` into the planner's road representation (usually building a `Path`) | the planner |
  | `optimize` | Computing the trajectory/decision | the planner |
  | `extract` | Converting the internal solution into `Vec<Control>` | the planner |

- **Custom seams** are just additional string names a planner chooses for
  phases only it has. Seams may nest (they're independent spans, not a
  partition of `total`), and a seam recorded more than once inside one
  `plan()` call is summed for that call before being folded into the
  rollout statistics.
- `simulate()` (in [`src/simulation/`](../simulation/README.md)) drains the
  recorder every tick and accumulates `calls` / `total_ms` / `max_ms` per
  seam into `Rollout::latency: LatencyStats`. The viewer's latency table
  reads straight from that.

See each planner's section below for which custom seams it adds and why.

## Introspection diagnostics

`diagnostics.rs` is the same optional-recorder shape as `latency.rs`, for a
different purpose: exposing the search geometry a planner considered, not
timing it. `ctx.diagnostics` is `Some` only when the viewer's diagnostic
overlay is switched on (see
[`src/viewer/README.md`](../viewer/README.md#introspection-diagnostics)) —
everywhere else, including `simulate()`'s closed-loop tick loop, it's `None`
and planners record nothing, so there's no cost outside that one on-demand
replan.

`DiagnosticsData` has two plain fields planners push into as they see fit:

- `points: Vec<[f64; 2]>` — standalone samples (the lattice's grid nodes,
  PI²-DDP's rollout states).
- `trajectories: Vec<Vec<[f64; 2]>>` — polylines (the lattice's DP edges,
  PI²-DDP's sampled rollouts).

Only the Frenet lattice, PI²-DDP, and RRT* record anything —
`PlannerKind::has_diagnostics()` reports which — since the strawman and
Bezier+IDM planners have no receding-horizon search to show. See each
planner's section below for exactly what it records.

## Test harness

`planning/mod.rs` exposes two `#[cfg(test)]` helpers shared by every
planner's tests:

- `test_ctx(centerline, actors) -> Context` — a `Context` with sane defaults
  (`target_speed: 10.0`, `dt: 0.1`, `horizon: 10`, `latency: None`).
- `test_run(planner, ego, actors, ticks) -> Vec<State>` — drives a planner
  closed-loop through a fixed straight centerline for `ticks` steps and
  returns the ego trace, for assertions like "ends up within 0.5 m of the
  centerline" or "keeps more than 2 m of clearance."

Every planner's own tests are closed-loop in this style rather than
single-call unit tests, because a single `plan()` call proves much less than
"the receding-horizon loop actually converges/avoids/stops."

---

## Strawman

`straight/mod.rs` — `StraightPlanner`

```rust
fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
    vec![Control::default(); ctx.horizon]
}
```

Always drives straight ahead at whatever speed the ego already has (zero
acceleration, zero curvature). No seams beyond `total` — there's no `route`,
`optimize`, or `extract` phase because there's no computation. It exists to
be the floor every other planner is measured against: on any scenario with
an obstacle in the lane, it collides, and the batch runner's mean score
reliably shows this (~0.27 across a mixed synthetic batch, vs. 0.74-0.90 for
the others).

## Bezier + IDM

`bezier_idm/mod.rs` — `BezierIdmPlanner`

Steers back to the lane by fitting a cubic Bezier curve from the ego's
current pose to a lookahead point on the centerline (tangent to the ego's
heading at the start, tangent to the lane heading at the end), then follows
that curve's analytic curvature. Speed comes from the
[Intelligent Driver Model](https://en.wikipedia.org/wiki/Intelligent_driver_model):
free-road acceleration toward `target_speed`, or car-following against the
nearest actor detected ahead in the same lane (`lead_vehicle`, ±2 m Frenet
offset).

**Seams**: `route` (build the `Path`, project the ego), `bezier_fit` (custom
— compute the four Bezier control points), `lead_search` (custom — scan
`ctx.actors` for the in-lane lead), `extract` (walk the Bezier + IDM forward
`ctx.horizon` steps to produce controls; this also *is* the optimize step
here, since there's no separate search).

**Limitations worth knowing**: lead detection is a simple "within ±2 m
laterally, ahead in station" filter — no lane-change or multi-lane
awareness. There is no explicit obstacle-avoidance term for actors *not* in
the ego's lane (e.g. crossing traffic); the planner's only defense there is
IDM slowing for whatever it decides counts as a lead. It converges to the
centerline and target speed within ~0.3 m / 0.5 m/s over ~20 s
(`converges_to_centerline_and_target_speed`), and correctly stops short of a
stationary lead (`stops_behind_stopped_lead`).

## Frenet lattice

`lattice/mod.rs` — `LatticePlanner`

An EM/Apollo-style planner. Samples a grid in the road's Frenet frame —
`STATION_LAYERS = 5` layers spaced evenly out to `PLANNING_HORIZON_S = 10` s
at the assumed cruise speed (`stations_m[i] = v * PLANNING_HORIZON_S * (i+1)
/ STATION_LAYERS`) crossed with nine lateral offsets (`LATERALS_M = [-3.5,
-2.625, -1.75, -0.875, 0, 0.875, 1.75, 2.625, 3.5]` m) — connects consecutive
layers with cubic-Hermite lateral segments, costs every edge
(smoothness + centerline proximity + constant-velocity-predicted-obstacle
proximity, with a hard `f64::INFINITY` if predicted clearance drops below
`COLLISION_RADIUS_M = 2.5` m), and picks the best path with exact dynamic
programming over the layered DAG (a proper A* would only add bookkeeping
over a graph this small).

The path's initial segment matches the ego's *current* lateral rate (via the
Hermite tangent `m0_first`) rather than starting flat — without this, every
replan would restart a swerve from zero slope and the vehicle actually
executed would lag behind the plan into the obstacle it was trying to avoid.
This was found and fixed via the `swerves_around_stopped_obstacle` test.

Speed is currently a constant profile clamped to
`[2, target_speed]` — not IDM-coupled (see the `ponytail:` comment in the
source for the deferred upgrade path). If every sampled path collides, the
planner gives up and brakes straight ahead (`accel: -4.0`) rather than
returning a bad path.

**Seams**: `route`, `optimize` (the layer-by-layer DP loop) with `edge_costs`
(custom — nested *inside* `optimize`; it's the hot loop, called
`5 stations × 9 laterals × up to 9 predecessors` times per `plan()` call) as
a nested seam, then `extract` (sample the winning path into `xy_to_controls`).

**Diagnostics**: every `(station, lateral)` grid node as a `point` (plus the
tree root at the ego's current position), and every DP edge sampled at
`SAMPLES_PER_SEGMENT` points as a `trajectory` — the whole search graph the
DP considered, not just the winning path (that's the separate future-preview
line, always drawn regardless of the diagnostic overlay).

## PI2-DDP

`pi2ddp/mod.rs` — `Pi2DdpPlanner`

Sampling-based Differential Dynamic Programming, implementing Algorithm 2 of
Lefebvre & Crevecoeur, *"Path Integral Policy Improvement with Differential
Dynamic Programming"* (PI²-DDP). `HORIZON = 100` ticks, i.e.
`PLANNING_HORIZON_S = 10` s at the simulator's 0.1 s tick rate. Each `plan()`
call runs `GENERATIONS = 4` generations; each generation samples
`ROLLOUTS = 32` perturbed control
sequences around a nominal trajectory (with feedback), weights them by
exponentiated normalized cost-to-go (paper eq. 12), and extracts a DDP-style
update from the reward-weighted rollout statistics:

- feedforward `k = Σₖ pₖ(δu − Kδx)`
- feedback `K = Σᵤₓ Σₓₓ⁺`
- perturbation covariance `Σᵤ = Σᵤᵤ − ΣᵤₓΣₓₓ⁺Σₓᵤ + λ_exp R⁻¹` (eq. 37)

with the eq. 38 trust-region rule on the exploration magnitude `λ_exp` (the
paper's "adaptive v2" variant: a generation that makes the noise-free cost
worse is discarded outright rather than blended in).

**Road-model-informed sampling** (the point of the exercise): the initial
nominal control sequence isn't zero, it's a pure-pursuit tracker toward the
centerline plus proportional speed hold (`init_policy`); the initial
curvature exploration variance `σ_κ` is sized so sampled trajectories span
roughly the lane half-width (`LANE_HALF_M = 1.75` m) by the preview
distance, rather than an arbitrary constant. The running cost is also
road-based: lateral offset, heading-to-lane error, a road-edge hinge penalty
to discourage leaving the drivable width, speed tracking, and
constant-velocity actor proximity.

The policy **warm-starts** across ticks: if the ego ended up close to where
the previous plan predicted (`expected_next`, within 1 m), the policy shifts
one step and continues refining; otherwise it re-initializes from scratch.

**Stability guards**, added after closed-loop testing surfaced real
failures (see the `stays_finite_and_safe_over_full_scenario` regression
test):

- `clamp_u` bounds acceleration and curvature to physical limits
  (`ACCEL_LIMIT = 5.0`, `KAPPA_LIMIT = 0.2`) — near-stationary rollouts have
  little state diversity, which makes the `Σₓₓ` inverse in the gain
  computation nearly singular and can otherwise blow the policy up.
- A PSD guard on the perturbation covariance: if `Σᵤ`'s Schur complement
  loses positive-definiteness (noisy statistics), it's replaced with the
  road-informed prior scaled by `λ_exp` rather than propagated.

**Seams**: `route` (build the `Path`), `warm_start` (custom — includes the
occasional full road-informed re-init when the shift check misses),
`rollouts` (custom — the `ROLLOUTS × HORIZON` sampling loop, by far the most
expensive part: typically ~85-90% of `total` time), `policy_update` (custom
— the per-timestep DDP gradient extraction).

**Diagnostics**: the final generation's `ROLLOUTS` sampled state sequences —
each one recorded both as a `trajectory` (the polyline through its `HORIZON`
states) and flattened into `points` (every state along every rollout), so
the overlay can show the sampling distribution's spread either as paths or
as a density of points. Only the last generation is recorded; earlier
generations are refinement steps toward it, not additional information.

## RRT*

`rrt_star/mod.rs` — `RrtStarPlanner`

Rapidly-exploring Random Tree Star: grows a tree of poses from the ego's
current state toward random (station, lateral) samples in the road frame,
connects each new node to the cheapest collision-free nearby parent, and
rewires existing nodes when a cheaper path through the new node appears
(the "star" — plain RRT would just keep the first parent found, which isn't
asymptotically optimal). `MAX_ITERS = 300` samples per `plan()` call.

**The steering function is differential flatness, not a straight line or an
arc.** A unicycle/bicycle's heading (`atan2(y', x')`) and curvature
(`(x'y'' - y'x'') / |·|^3`) are both fully determined by its flat outputs
`(x, y)` and their derivatives — so `CubicSteer` fits an independent cubic
polynomial to `x(s)` and `y(s)` (Hermite form, tangent magnitude
`chord / 3`, the same heuristic [Bezier + IDM](#bezier--idm) uses) matching
position and heading *direction* at both ends, and the connection is
guaranteed kinematically smooth without ever solving for heading or
curvature directly.

**Steering-angle limiting, not post-hoc curvature rejection, is what makes
the tree grow at all.** Early on, this module aimed each new edge straight
at its random sample (or matched every node's heading to the lane); either
way, two independently-drawn directions connected by a *short* Hermite
tangent needs far more curvature than any real car has, and nearly every
candidate steer failed the curvature check — measured by instrumenting
`feasible`'s own rejections, under 10 of 300 samples per tick were passing,
even on an empty road. `max_yaw_change(step_len)` inverts this: it caps how
far a new edge's direction may turn away from its parent's *own* heading,
scaled so the resulting curve's peak curvature (`≈ 48 * dyaw / step_len` for
this tangent magnitude, found empirically) stays within `MAX_CURVATURE`.
Both of a new edge's tangents then point the same way — a straight hop, zero
curvature by construction — so a real swerve is built from several small,
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

**Warm start, with hysteresis, is what makes obstacle avoidance consistent
tick to tick.** `RrtStarPlanner` doesn't just keep an `Rng`
([PI²-DDP](#pi2-ddp)'s pattern) — it remembers `prev_path`, last tick's
winning polyline, and replays whatever part of it is still ahead of the ego
and still collision-free against this tick's actors as a ready-made chain of
nodes before any random sampling happens. Without this, a tree rebuilt from
independent samples every 0.1 s tick can find a differently-shaped detour
each time; since the simulator only ever executes one control per plan, a
closed-loop trajectory stitched from many such plans doesn't inherit any
single one's safety margin — the exact failure the `swerves_around_stopped_obstacle`
test caught (realized clearance well under any individual plan's own
`COLLISION_RADIUS_M`). Goal selection then *prefers* a warm-started node
over a fresh one unless the fresh one makes meaningfully more progress (more
than one `PROGRESS_TOLERANCE_M` bucket), so a good detour, once found, isn't
abandoned for a marginally-cheaper alternative next tick.

**Deterministic bypass seeding is what makes a good detour reliably
*findable* in the first place.** Before the random-sampling loop runs, every
actor gets a fixed, unconditional ramp of candidate waypoints tried on both
sides (station offsets `[-20, -10, -3, 3, 10, 20]` m around it, lateral
offset ramping `0.25× → 0.6× → 1.0× → 1.0× → 0.6× → 0` of a safe bypass
distance) via the same `try_extend` the random loop uses, seeded in
increasing-station order so each waypoint chains onto the previous one on
the same side. Randomized "informed sampling" (try a safe offset next to a
random actor with some probability) found a wide detour on some ticks and a
narrower one on others — the same consistency problem warm start addresses,
one level up. Trying identical candidates every tick means the tree finds
(and keeps refining, via warm start and rewiring) the *same* detour every
time.

**Progress, not raw distance, decides the goal**, bucketed to
`PROGRESS_TOLERANCE_M` rather than compared exactly: without bucketing, a
node a hair's-breadth further along but squeezing past an obstacle would beat
a node a few centimeters short but giving it a much wider berth, every
single time, since station is compared before cost (which includes an
obstacle-proximity term) ever gets a say.

**Seams**: `route` (build the `Path`), `warm_start` (custom — replaying the
previous winning path), `optimize` (the `MAX_ITERS`-sample tree-growing
loop; the deterministic bypass seeding and the final extract step aren't
timed separately since they're comparatively cheap), `extract` (resample the
winning path — itself a `scenarios::Path` built from the tree's polyline —
at `v * dt` intervals and convert to controls via the same technique as the
[Frenet lattice's](#frenet-lattice) `xy_to_controls`).

**Diagnostics**: every tree node (after the root) as a `point`, and the
sampled polyline of the edge that added it as a `trajectory` — the whole
search tree considered, not just the winning path, mirroring the lattice's
approach.
