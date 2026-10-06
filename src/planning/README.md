# `planning`

The `Planner` trait, the `Context` planners read, the `PlannerKind` registry used to select and compare planners,
latency diagnostics, and one subdirectory per planner implementation.

```
planning/
├── mod.rs         Planner trait, Context, PlannerKind + PlannerSpec registry, test harness
├── engine.rs      asynchronous planner execution for native threads and Web Workers
├── latency.rs     Latency/LatencyStats/SeamStats — see "Latency diagnostics" below
├── sampling.rs    shared QMC low-discrepancy + road-frame sampler — see "Shared QMC sampling" below
├── basic/         cubic path planner
├── leeroy_jenkins/ maximum acceleration, zero steering
├── bezier_toppra/ cubic Bezier back to the centerline + TOPP-RA speed
├── lattice/       Frenet lattice, high-res sampled grid + A* search
├── frenetix/      lateral cubics × longitudinal cubics, shared horizon and cost
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

A planner is given the current ego `State` and a `Context`, and returns a direct acceleration/curvature command
trajectory.
The [`Simulator`](../simulation/README.md) clamps the first command to the vehicle's static limits before applying it.
The simulator applies only the **first** control and re-invokes `plan()` next tick — this is a receding horizon /
MPC-style loop, not open-loop trajectory execution.
`&mut self` lets a planner keep state between calls (PI²-DDP warm-starts its policy this way); planners with no state to
keep, like `LeeroyJenkinsPlanner`, are zero-sized unit structs.

An empty return value is treated as "coast" (zero control) by the simulator, not an error — no planner currently
exercises this, but it's a legal escape hatch for "couldn't find anything, don't do anything worse."

## Planner engine

`engine.rs` keeps planner latency off the live simulation loop.
On native targets, `PlannerEngine` owns the planner on a background thread; on WebAssembly, it sends the same
serializable request to a Web Worker.
The simulation remains fixed-step and interacts with either implementation through the same non-blocking `submit`,
`poll`, and `is_slow` operations.

Track selection prepares the immutable road geometry and indexes before entering Driving.
Native workers share that storage; browser workers receive a one-time track ID and timestep, prepare their own resident
geometry, and acknowledge readiness.
The selection screen waits for that acknowledgment.

Between simulation ticks, `LiveWorld` submits the current ego state, road-window range and projection hint, next traffic
states, horizon, compute budget, and diagnostic setting.
No track points, boundaries, barriers, or indexes cross the worker boundary per tick.
The worker gets the full 100 ms interval before the next step is due.
Only one request may be in flight.
The tick commits only after its matching `PlanResult` arrives: ego and traffic advance together by 100 ms. While
waiting, the viewer keeps rendering and polling without advancing the world or consuming the accumulated tick.
Traffic updates are staged so neither positions nor random state advance repeatedly during worker delays.
Changing the planner or traffic count discards pending snapshots by generation, while retaining prepared geometry.

This is nonblocking lockstep: a fast planner supplies one fresh plan per simulation step; if computation exceeds 100 ms,
simulated time slows rather than executing old controls and repeatedly rejecting stale replacements.
A planner is considered too slow when either its completed runtime or the age of its outstanding request exceeds one
simulation timestep.
`LiveWorld` exposes that state as `planner_slow`, and the viewer displays **PLANNER TOO SLOW · WAITING FOR PLAN**.
Native tests and batch measurement can explicitly block for a result; the live viewer never blocks its render thread.

## `Context`

```rust
pub struct Context<'a> {
    pub road: &'a Road,               // centerline + tick length
    pub actors: &'a [State],          // other vehicles, current states only
    pub horizon: usize,               // requested control-trajectory length
    pub latency: Option<&'a Latency>, // recorder; see below
    pub diagnostics: Option<&'a Diagnostics>, // recorder; see below
}
```

Everything a planner needs besides its own state and the ego pose.
Notably:

- **`road` is the current planning window** — the `track::Road` parameter object bundling the track centerline and the
  tick length of the returned controls.
  Planners read `ctx.road.centerline()` and `ctx.road.dt`.
  The live world sizes this window for maximum-acceleration reach over the shared planning horizon for every planner, and
  refreshes it when speed increases beyond the existing window's coverage.
- **`actors` is current-tick only.** Planners see no future information about other vehicles — if they want a prediction,
  they compute one themselves.
  They all go through the shared `prediction::predict`: an actor driving along the route is rolled forward following the
  lane's curve and eased back toward its center (constant-speed, lane-associated kinematics), while oncoming or crossing
  traffic falls back to constant-velocity extrapolation.
- **`horizon` is a request, not a contract.** A planner may return more or fewer controls; the simulator only ever
  consumes the first one during closed-loop simulation.
  The viewer's future-preview feature asks for a larger horizon (up to 100 ticks, `PLANNING_HORIZON_S`) to draw a longer
  plan.
- **`ctx.path()` is a cheap view of the road's prepared geometry.** Arc lengths, reference geometry, and spatial indexes
  are shared with metrics and subsequent plan calls.
  Actor projections remain local to each planning context so stale traffic predictions cannot accumulate in the static
  cache.

## `PlannerKind` and the `PlannerSpec` registry

```rust
pub enum PlannerKind { LeeroyJenkins, BezierToppra, Lattice, Pi2Ddp, RrtStar }

pub struct PlannerSpec {
    pub kind: PlannerKind,
    pub name: &'static str,             // display string
    pub build: fn() -> Box<dyn Planner>, // fresh instance (Factory Method slot)
    pub has_diagnostics: bool,          // records into Diagnostics?
}
```

The selection/comparison seam.
`PlannerKind` is just the key (a `Copy` enum, usable as a hash-map key); everything else about a planner lives in its
row of the `SPECS` table, reached via `kind.spec()` — `.name()`, `.build()`, and `.has_diagnostics()` are thin accessors
over it.
`PlannerKind::ALL` is the definitive list the viewer's dropdown and the batch runner iterate over.
A `specs_align_with_kinds` test pins the table's row order to the enum's discriminants.

**To add another planner:**

1. Create `planning/my_planner/mod.rs` implementing `Planner`.
1. Add `pub mod my_planner;` and `pub use my_planner::MyPlanner;` to `planning/mod.rs`.
1. Add a `PlannerKind::MyPlanner` variant, extend `ALL`, and add one complete `PlannerSpec` row to `SPECS` (name,
   constructor, whether it records diagnostics).

Nothing outside `planning/` needs to change — the viewer, the batch runner, and the metrics evaluator all iterate
`PlannerKind::ALL` or take `Box<dyn Planner>` generically.

## Latency diagnostics

`latency.rs` implements a minimal seam-based timing interface shared by the planner, live simulation, and viewer,
described in full in its module doc.
The short version:

- A **seam** is a named timed span inside one `plan()` call: `ctx.time("name", || { ...work... })`.
  `Context::time` is a no-op wrapper when diagnostics aren't being collected (`ctx.latency` is `None`, as in every test
  and in the future-preview replan), so instrumentation is free outside of `simulate()`.

- **Standardized seam names**, used wherever the phase exists so planners stay comparable across the table in the viewer:

  | Seam            | Meaning                                                                                 | Recorded by                                                     |
  | --------------- | --------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
  | `planner.total` | The whole `plan()` call                                                                 | `LiveWorld`, not the planner — every planner gets this for free |
  | `route`         | Turning `centerline` into the planner's road representation (usually building a `Path`) | the planner                                                     |
  | `optimize`      | Computing the trajectory/decision                                                       | the planner                                                     |
  | `extract`       | Converting the internal solution into `Vec<Control>`                                    | the planner                                                     |

- **Custom seams** are just additional string names a planner chooses for phases only it has.
  Seams may nest (they're independent spans, not a partition of `total`), and a seam recorded more than once inside one
  `plan()` call is summed for that call before being folded into the rollout statistics.

- The live viewer drains the recorder after drawing each frame and accumulates `calls` / `total_ms` / `max_ms` per seam.
  Multiple fixed simulation ticks in one rendered frame are summed.
  Simulation seams use the `simulation.*` namespace and drawing seams use `visualization.*`.

- Every span also accumulates hardware-independent logical `clocks`.
  A clock represents one domain work item (for example an actor, trajectory sample, or rendered plan state); nested seams
  include their children's work.
  Timed spans also include shared geometry work: one clock per indexed segment built or exact segment-distance evaluation,
  including evaluations inside the spatial index.
  A thread-local counter keeps this work attributable to the executing worker; it does not charge the renderer for planner
  work on another thread.
  Long-road tests bound query clocks on 256- and 4,096-segment roads, check index reuse, and compare indexed projection
  against exhaustive scans across every track.
  These catch full-road scans hidden inside a single trajectory sample.
  Separate geometry-construction clocks assert that prepared road windows, contexts, metrics, worker resets, and the
  actual track drawing loop perform zero geometry construction.
  Candidate paths remain dynamic and are built as needed.
  These deterministic totals can be asserted in normal unit tests even though wall milliseconds cannot.

See each planner's README for which custom seams it adds and why.

## Introspection diagnostics

`diagnostics.rs` is the same optional-recorder shape as `latency.rs`, for a different purpose: exposing the search
geometry a planner considered, not timing it.
`ctx.diagnostics` is `Some` only when the viewer's diagnostic overlay is switched on (see
[`src/viewer/README.md`](../viewer/README.md#introspection-diagnostics)) — everywhere else, including `simulate()`'s
closed-loop tick loop, it's `None` and planners record nothing, so there's no cost outside that one on-demand replan.

`DiagnosticsData` stores the recorded geometry and its timing:

- `points: Vec<[f64; 2]>` — standalone samples (the lattice's grid nodes, PI²-DDP's rollout states).
- `trajectories: Vec<Vec<[f64; 2]>>` — polylines (the lattice's DP edges, PI²-DDP's sampled rollouts).
- `trajectory_times: Vec<Vec<f64>>` — seconds from planning start for each polyline point, used to clip candidate
  trajectories to the future preview slider.

The viewer clips both ends to a time window starting at the interpolated ego's simulation time.
Elapsed trajectory prefixes are hidden; candidate geometry stays in its original world coordinates.
The window follows simulation time through pause/resume and planner waits, rather than wall-clock time.

Every search planner records something — `PlannerKind::has_diagnostics()` reports which — including one timed rollout
per Bezier+TOPP-RA path candidate.
Leeroy Jenkins records nothing.
See each planner's README for exactly what it records.

## Test harness

`planning/mod.rs` exposes three `#[cfg(test)]` helpers shared by every planner's tests:

- `test_road(centerline) -> Road` — a `Road` with sane defaults (`dt: 0.1`).
- `test_ctx(&road, actors) -> Context` — a `Context` over that road (`horizon: 10`, no recorders).
- `test_run(planner, ego, actors, ticks) -> Vec<State>` — drives a planner closed-loop through a fixed straight centerline
  for `ticks` steps and returns the ego trace, for assertions like "ends up within 0.5 m of the centerline" or "keeps more
  than 2 m of clearance."

Every planner's own tests are closed-loop in this style rather than single-call unit tests, because a single `plan()`
call proves much less than "the receding-horizon loop actually converges/avoids/stops."

## The shared metric objective

See [`metrics/README.md`](../metrics/README.md#the-shared-metric-objective).

## Shared QMC sampling

`sampling.rs` is the single owner of the quasi-Monte-Carlo low-discrepancy sampling every sampling planner draws from —
the deterministic alternative to a pseudo-random `Rng` that RRT\* already relied on, now shared with the judo-derived
planners.
Two things live here:

- **The QMC sequence, behind one trait.** `van_der_corput` (radical inverse in a prime base) is the building block; the
  `QuasiMonteCarlo` trait, with its single implementor `Halton`, is the *interface* every planner names.
  There is exactly one implementor, so "the whole codebase samples from one QMC construction" is a fact the compiler
  checks — a planner wanting a different sequence would have to name a different type, a compile error at the call site,
  not a silent drift between two hand-maintained radical-inverse loops.
- **The hybrid road-frame sampler.** `road_frame_samples::<Q>` lays down a fixed road-geometry grid over the `(station,
  lateral)` Frenet box (in ascending-station order) and then a Halton QMC pass filling its gaps — the hybrid RRT\* grows
  its tree from, now generic over the same `Q: QuasiMonteCarlo` so the road model and the QMC fill are shared, not copied.

**Parity is enforced at the interface, not by convention.** RRT\* calls `road_frame_samples::<Halton>` for its Frenet
targets; the judo optimizers call `qmc_normals::<Halton>` (Halton coordinates pushed through an inverse-normal-CDF,
`inv_normal_cdf`) for their Gaussian control-knot noise.
Both go through the same `QuasiMonteCarlo` trait, so the parity is *structural* (a type-level share, checked at compile
time).
On top of that, RRT\*'s `rrt_targets_match_shared_sampler` test pins the *numeric* parity — that lifting its old inline
loop into the shared function changed no sample.
Because the sequence is a pure function of the sample index, every planner that samples through this module is a pure
function of the ego state and road context (`plan_is_a_pure_function_of_state`), the property that lets a closed-loop
rollout inherit any single plan's safety margin — PI²-DDP, which keeps a real `Rng` for its rollouts, is now the lone
exception.

## Planner implementations

- [Basic cubic](basic/README.md)
- [Leeroy Jenkins](leeroy_jenkins/README.md)
- [Bezier + TOPP-RA](bezier_toppra/README.md)
- [Frenet lattice](lattice/README.md)
- [PI²-DDP](pi2ddp/README.md)
- [RRT\*](rrt_star/README.md)
- [Sampling MPC](sampling_mpc/README.md)
- [Treetop](treetop/README.md)
