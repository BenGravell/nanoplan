# `planning`

The `Planner` trait, the `Context` planners read, the `PlannerKind` registry
used to select and compare planners, latency diagnostics, and one
subdirectory per planner implementation.

```
planning/
├── mod.rs         Planner trait, Context, PlannerKind + PlannerSpec registry, test harness
├── engine.rs      asynchronous planner execution for native threads and Web Workers
├── latency.rs     Latency/LatencyStats/SeamStats — see "Latency diagnostics" below
├── constraints.rs hard rules and the shared composite-metric objective
├── sampling.rs    shared QMC low-discrepancy + road-frame sampler — see "Shared QMC sampling" below
├── basic/         cubic path planner
├── straight/      strawman: zero control, always
├── bezier_toppra/ cubic Bezier back to the centerline + TOPP-RA speed
├── lattice/       Frenet lattice, high-res sampled grid + A* search
├── pi2ddp/        sampling-based DDP (PI²-DDP)
├── rrt_star/      RRT*, cubic differential-flatness steering
├── sampling_mpc/  judo-derived sampling MPC: predictive sampling, CEM, MPPI
└── treetop/       treetop-derived: RRT motion sampling tree, finite-difference iLQR, and the RRT+iLQR treetop planner
```

## The `Planner` trait

```rust
pub trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}
```

A planner is given the current ego `State` and a `Context`, and returns a
direct acceleration/curvature command trajectory. The
[`Simulator`](../simulation/README.md) clamps the first command to the vehicle's
static limits before applying it. The
simulator applies only the **first** control and re-invokes `plan()` next tick
— this is a receding horizon / MPC-style loop, not open-loop trajectory
execution. `&mut self` lets a planner keep state between calls (PI²-DDP
warm-starts its policy this way); planners with no state to keep, like
`StraightPlanner`, are zero-sized unit structs.

An empty return value is treated as "coast" (zero control) by the simulator,
not an error — no planner currently exercises this, but it's a legal escape
hatch for "couldn't find anything, don't do anything worse."

## Planner engine

`engine.rs` keeps planner latency off the live simulation loop. On native
targets, `PlannerEngine` owns the planner on a background thread; on WebAssembly,
it sends the same serializable request to a Web Worker. The simulation remains
fixed-step and interacts with either implementation through the same
non-blocking `submit`, `poll`, and `is_slow` operations.

At each tick, `LiveWorld` first polls for a completed `PlanResult`, then submits
a snapshot of the current ego state, road, actors, horizon, compute budget, and
diagnostic setting. Only one request may be in flight, so ticks never build up
a queue of stale planning work. When no new result is ready, the simulation
continues with the remaining controls from the last accepted plan; after that
plan's horizon is exhausted, its final control is retained. Before the first
plan arrives, the empty plan produces the normal zero-control fallback.

A planner is considered too slow when either its completed runtime or the age
of its outstanding request exceeds one simulation timestep. `LiveWorld` exposes
that state as `planner_slow`, and the viewer displays **PLANNER TOO SLOW ·
REUSING LAST PLAN** until a timely result is accepted. Native tests and batch
measurement can explicitly wait for a result, but the live viewer never blocks
the simulation on planning.

## `Context`

```rust
pub struct Context<'a> {
    pub road: &'a Road,               // centerline + target speed + tick length
    pub actors: &'a [State],          // other vehicles, current states only
    pub horizon: usize,               // requested control-trajectory length
    pub latency: Option<&'a Latency>, // recorder; see below
    pub diagnostics: Option<&'a Diagnostics>, // recorder; see below
}
```

Everything a planner needs besides its own state and the ego pose. Notably:

- **`road` is the current planning window** — the `track::Road`
  parameter object bundling the track centerline, the desired cruise speed,
  and the tick length of the returned controls. Planners read
  `ctx.road.centerline()`, `ctx.road.target_speed`, and `ctx.road.dt`.
- **`actors` is current-tick only.** Planners see no future information
  about other vehicles — if they want a prediction, they compute one
  themselves. They all go through the shared `prediction::predict`: an actor
  driving along the route is rolled forward following the lane's curve and
  eased back toward its center (constant-speed, lane-associated kinematics),
  while oncoming or crossing traffic falls back to constant-velocity
  extrapolation.
- **`horizon` is a request, not a contract.** A planner may return more or
  fewer controls; the simulator only ever consumes the first one during
  closed-loop simulation. The viewer's future-preview feature asks for a
  larger horizon (up to 100 ticks, `PLANNING_HORIZON_S`) to draw a longer plan.
- **`road.centerline()` is a raw polyline**, not a `Path`. Every planner that
  needs Frenet operations (arc length, projection, curvature-following)
  builds its own `track::Path` from it.

## `PlannerKind` and the `PlannerSpec` registry

```rust
pub enum PlannerKind { Straight, BezierToppra, Lattice, Pi2Ddp, RrtStar }

pub struct PlannerSpec {
    pub kind: PlannerKind,
    pub name: &'static str,             // display string
    pub build: fn() -> Box<dyn Planner>, // fresh instance (Factory Method slot)
    pub has_diagnostics: bool,          // records into Diagnostics?
}
```

The selection/comparison seam. `PlannerKind` is just the key (a `Copy` enum,
usable as a hash-map key); everything else about a planner lives in its row
of the `SPECS` table, reached via `kind.spec()` — `.name()`, `.build()`, and
`.has_diagnostics()` are thin accessors over it. `PlannerKind::ALL` is the
definitive list the viewer's dropdown and the batch runner iterate over. A
`specs_align_with_kinds` test pins the table's row order to the enum's
discriminants.

**To add another planner:**

1. Create `planning/my_planner/mod.rs` implementing `Planner`.
2. Add `pub mod my_planner;` and `pub use my_planner::MyPlanner;` to
   `planning/mod.rs`.
3. Add a `PlannerKind::MyPlanner` variant, extend `ALL`, and add one
   complete `PlannerSpec` row to `SPECS` (name, constructor, whether it
   records diagnostics).

Nothing outside `planning/` needs to change — the viewer, the batch runner,
and the metrics evaluator all iterate `PlannerKind::ALL` or take
`Box<dyn Planner>` generically.

## Latency diagnostics

`latency.rs` implements a minimal seam-based timing interface shared by the
planner, live simulation, and viewer, described in full in its module doc.
The short version:

- A **seam** is a named timed span inside one `plan()` call:
  `ctx.time("name", || { ...work... })`. `Context::time` is a no-op wrapper
  when diagnostics aren't being collected (`ctx.latency` is `None`, as in
  every test and in the future-preview replan), so instrumentation is free
  outside of `simulate()`.
- **Standardized seam names**, used wherever the phase exists so planners
  stay comparable across the table in the viewer:

  | Seam | Meaning | Recorded by |
  |---|---|---|
  | `planner.total` | The whole `plan()` call | `LiveWorld`, not the planner — every planner gets this for free |
  | `route` | Turning `centerline` into the planner's road representation (usually building a `Path`) | the planner |
  | `optimize` | Computing the trajectory/decision | the planner |
  | `extract` | Converting the internal solution into `Vec<Control>` | the planner |

- **Custom seams** are just additional string names a planner chooses for
  phases only it has. Seams may nest (they're independent spans, not a
  partition of `total`), and a seam recorded more than once inside one
  `plan()` call is summed for that call before being folded into the
  rollout statistics.
- The live viewer drains the recorder after drawing each frame and accumulates
  `calls` / `total_ms` / `max_ms` per seam. Multiple fixed simulation ticks in
  one rendered frame are summed. Simulation seams use the `simulation.*`
  namespace and drawing seams use `visualization.*`.
- Every span also accumulates hardware-independent logical `clocks`. A clock
  represents one domain work item (for example an actor, trajectory sample, or
  rendered plan state); nested seams include their children's work. These
  deterministic totals can be asserted in normal unit tests even though wall
  milliseconds cannot.

See each planner's README for which custom seams it adds and why.

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

Every search planner records something — `PlannerKind::has_diagnostics()`
reports which — while the strawman and Bezier+TOPP-RA planners have no
receding-horizon search to show and record nothing.
See each planner's README for exactly what it records.

## Test harness

`planning/mod.rs` exposes three `#[cfg(test)]` helpers shared by every
planner's tests:

- `test_road(centerline) -> Road` — a `Road` with sane defaults
  (`target_speed: 10.0`, `dt: 0.1`).
- `test_ctx(&road, actors) -> Context` — a `Context` over that road
  (`horizon: 10`, no recorders).
- `test_run(planner, ego, actors, ticks) -> Vec<State>` — drives a planner
  closed-loop through a fixed straight centerline for `ticks` steps and
  returns the ego trace, for assertions like "ends up within 0.5 m of the
  centerline" or "keeps more than 2 m of clearance."

Every planner's own tests are closed-loop in this style rather than
single-call unit tests, because a single `plan()` call proves much less than
"the receding-horizon loop actually converges/avoids/stops."

## The shared metric objective

The search-based planners — the Frenet lattice, PI²-DDP, RRT*, the three
judo-derived sampling-MPC planners (predictive sampling, CEM, MPPI), and
the three treetop-derived planners (RRT, iLQR, treetop) — all
price candidates with the same scalar objective;
`bezier_toppra` and `straight` don't (see their own READMEs for why
they're out of scope here). Before this module existed, each planner priced
a candidate with its own inline formula, actor-prediction code,
point-collision proxy, and idea of
"off the road" — several different, undocumented definitions of "good."

`HardConstraints::point_cost(sample)` is the complement of the production
metrics composite: `1 - composite([safety, progress, comfort])`. Every planner
calls it under the same seam name, `"cost"` (see "Latency diagnostics"
above). Because safety is a multiplier and progress and comfort aggregate by
average, summing this cost over a fixed-length feasible rollout gives the
planner form of the same composite objective.

- **Hard collision and off-road rejection** — `constraints.rs` returns
  `f64::INFINITY` if a sampled point is closer than the shared car-width
  point proxy to any actor's predicted position, or further than
  `road_half_width` from the centerline. That bound is the road's *actual*
  drivable half-width (`Road::half_width`, the same value used to generate
  the barrier geometry that `ttc` scores), passed in per plan
  rather than read from a fixed constant — so on a narrow street the reject
  fires at the true edge. A planner should reject these outright, not merely
  disfavor them.
- **Progress and comfort are the production metrics** — forward speed is
  normalized by the speed reachable under maximum thrust acceleration
  from the current speed (using the plant's rolling resistance and drag), and
  longitudinal/lateral jerk goes through `metrics::comfort::jerk_score`.
  Their weights and safety's multiplier role come directly from the `METRICS`
  registry.
- **Actor prediction** goes through `prediction::predict` — the lane-aware
  kinematic model — instead of each planner reimplementing prediction
  independently. An actor travelling along the route is rolled forward along
  the lane's curve and eased back toward its center, so on a bend it is
  priced where it will actually be rather than off on the straight tangent;
  oncoming and crossing traffic fall back to `prediction::project`. The
  rollout's `metrics::safety` metric evaluates the resulting actual future ego
  and actor traces, so it does not duplicate the planner's prediction model.

**No analytic derivatives, by construction.** `point_cost` takes
already-known numbers — position, speed, curvature, accel — and returns a
plain `f64`; there's no gradient anywhere in its signature or its callers.
This is a deliberate design constraint, not an oversight: nanoplan never
*provides* a derivative of its cost or dynamics — both are black-box
scalars, and nothing may demand an analytic gradient of either. Most
planners live entirely within that constraint by sampling and comparing
candidates. The one family that genuinely optimizes —
[treetop's iLQR](treetop/README.md#ilqr-treetop-finite-differences) — respects it at the
interface: it consumes exactly the same black-box scalars and
differentiates them **numerically** (central finite differences), probing
`point_cost` and `step` a few dozen times per timestep instead of once.
The scalar interface stays the single source of truth for what "good"
means; no second, analytically-differentiated definition of the cost can
drift away from it. Where a planner needs curvature as an input, it gets it
one of two ways, both compatible with that constraint:

- **A closed-form fact about an already-*fixed* candidate curve.** RRT*'s
  `CubicSteer::curvature` evaluates the curvature of a specific flat-output
  polynomial it already committed to — a geometric property of one
  candidate, not a gradient used to choose the next one.
- **A value recovered from a sampled trajectory.** The space-time lattice
  derives curvature from heading change over distance along its timed
  connector before rolling the resulting control through the plant.

**What stays planner-specific.** Sampling layouts, warm starts, feasibility
margins, and search topology remain planner-specific, but they do not add
another outcome score. Numeric optimizers replace `point_cost`'s
`f64::INFINITY` with the finite, depth-scaled
`constraints::HARD_VIOLATION_PENALTY`; the lattice and RRT* propagate the
actual infinity and reject the candidate outright.

## Shared QMC sampling

`sampling.rs` is the single owner of the quasi-Monte-Carlo low-discrepancy
sampling every sampling planner draws from — the deterministic alternative
to a pseudo-random `Rng` that RRT* already relied on, now shared with the
judo-derived planners. Two things live here:

- **The QMC sequence, behind one trait.** `van_der_corput` (radical inverse
  in a prime base) is the building block; the `QuasiMonteCarlo` trait, with
  its single implementor `Halton`, is the *interface* every planner names.
  There is exactly one implementor, so "the whole codebase samples from one
  QMC construction" is a fact the compiler checks — a planner wanting a
  different sequence would have to name a different type, a compile error at
  the call site, not a silent drift between two hand-maintained
  radical-inverse loops.
- **The hybrid road-frame sampler.** `road_frame_samples::<Q>` lays down a
  fixed road-geometry grid over the `(station, lateral)` Frenet box (in
  ascending-station order) and then a Halton QMC pass filling its gaps — the
  hybrid RRT* grows its tree from, now generic over the same
  `Q: QuasiMonteCarlo` so the road model and the QMC fill are shared, not
  copied.

**Parity is enforced at the interface, not by convention.** RRT* calls
`road_frame_samples::<Halton>` for its Frenet targets; the judo optimizers
call `qmc_normals::<Halton>` (Halton coordinates pushed through an
inverse-normal-CDF, `inv_normal_cdf`) for their Gaussian control-knot noise.
Both go through the same `QuasiMonteCarlo` trait, so the parity is
*structural* (a type-level share, checked at compile time). On top of that,
RRT*'s `rrt_targets_match_shared_sampler` test pins the *numeric* parity —
that lifting its old inline loop into the shared function changed no sample.
Because the sequence is a pure function of the sample index, every planner
that samples through this module is a pure function of the ego state and
road context (`plan_is_a_pure_function_of_state`), the property that lets a
closed-loop rollout inherit any single plan's safety margin — PI²-DDP, which
keeps a real `Rng` for its rollouts, is now the lone exception.

## Planner implementations

- [Basic cubic](basic/README.md)
- [Straight strawman](straight/README.md)
- [Bezier + TOPP-RA](bezier_toppra/README.md)
- [Frenet lattice](lattice/README.md)
- [PI²-DDP](pi2ddp/README.md)
- [RRT*](rrt_star/README.md)
- [Sampling MPC](sampling_mpc/README.md)
- [Treetop](treetop/README.md)
