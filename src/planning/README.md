# `planning`

The `Planner` trait, the `Context` planners read, the `PlannerKind` registry
used to select and compare planners, latency diagnostics, and one
subdirectory per planner implementation.

```
planning/
├── mod.rs         Planner trait, Context, PlannerKind + PlannerSpec registry, test harness
├── latency.rs     Latency/LatencyStats/SeamStats — see "Latency diagnostics" below
├── cost.rs        shared trajectory-cost function — see "The shared cost function" below
├── sampling.rs    shared QMC low-discrepancy + road-frame sampler — see "Shared QMC sampling" below
├── straight/      strawman: zero control, always
├── bezier_idm/    cubic Bezier back to the centerline + IDM speed
├── lattice/       Frenet lattice, high-res sampled grid + A* search
├── pi2ddp/        sampling-based DDP (PI²-DDP)
├── rrt_star/      RRT*, cubic-polynomial (differential-flatness) steering
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
    pub road: &'a Road,               // centerline + target speed + tick length
    pub actors: &'a [State],          // other vehicles, current states only
    pub horizon: usize,               // requested control-trajectory length
    pub latency: Option<&'a Latency>, // recorder; see below
    pub diagnostics: Option<&'a Diagnostics>, // recorder; see below
}
```

Everything a planner needs besides its own state and the ego pose. Notably:

- **`road` is the fixed setting of the whole run** — the
  [`scenarios::Road`](../scenarios/README.md#road-the-fixed-setting-of-a-run)
  parameter object bundling the lane centerline, the desired cruise speed,
  and the tick length of the returned controls. Planners read
  `ctx.road.centerline`, `ctx.road.target_speed`, and `ctx.road.dt`.
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
- **`road.centerline` is a raw polyline**, not a `Path`. Every planner that
  needs Frenet operations (arc length, projection, curvature-following)
  builds its own `scenarios::Path` from it — see
  [`src/scenarios/README.md`](../scenarios/README.md#path-the-frenet-helper).

## `PlannerKind` and the `PlannerSpec` registry

```rust
pub enum PlannerKind { Straight, BezierIdm, Lattice, Pi2Ddp, RrtStar }

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

Every search planner records something — `PlannerKind::has_diagnostics()`
reports which — while the strawman and Bezier+IDM planners have no
receding-horizon search to show and record nothing. See each planner's
section below for exactly what it records.

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

## The shared cost function

The search-based planners — the Frenet lattice, PI²-DDP, RRT*, the three
judo-derived sampling-MPC planners (predictive sampling, CEM, MPPI), and
the three treetop-derived planners (RRT, iLQR, treetop) — all
price candidates with the same scalar cost;
`bezier_idm` and `straight` don't (see their own sections below for why
they're out of scope here). Before this module existed, each planner priced
a candidate with its own inline formula, hand-tuned independently, with its
own actor-prediction code, its own collision radius, and its own idea of
"off the road" — several different, undocumented definitions of "good."

`cost.rs` factors the metrics-motivated part of that formula into one
function, `point_cost(sample, target_speed, road_half_width, actors)`, called
by every one of them under the same seam name, `"cost"` (see "Latency
diagnostics" above). It's deliberately grounded in the same quantities
[`crate::metrics`](../metrics/README.md) scores scenario quality by, rather
than inventing new ones:

- **Hard collision and off-road rejection** — `f64::INFINITY` if a sampled
  point is closer than `metrics::collisions`' own threshold
  (`2 × CAR_RADIUS_M`) to any actor's predicted position, or further than
  `road_half_width` from the centerline. That bound is the road's *actual*
  drivable half-width (`Road::half_width`, the same value `drivable_area`
  scores against), passed in per plan rather than read from a fixed
  constant — so on a narrow street the reject fires at the true edge, not at
  the 5.5 m default. A planner should reject these outright, not merely
  disfavor them.
- **Everything else is `WEIGHTS · features`** — the soft terms below are a
  feature vector (`features()`, each term already squared/hinged) dotted
  with one `pub(crate) const WEIGHTS`, so the finite cost is *linear* in the
  weights. That form is what makes the weights *learnable*: the
  [`tuning`](../tuning/README.md) module re-fits them to expert nuPlan
  trajectories with maximum-entropy IRL (`cargo run --release --bin tune`),
  printing a replacement `WEIGHTS` line to paste back here. The hard
  rejection above is a fixed rule of `features()` (it returns `None`), never
  a learned weight.
- **Actor prediction** reuses `metrics::project` — the same constant-
  velocity, constant-heading model `metrics::ttc` scores against — instead
  of each planner reimplementing the same three lines independently.
- **Soft terms** scaled to match: actor-proximity (inverse-square, inside
  the hard collision radius), road-edge proximity, speed tracked against
  `speed_limit::MAX_OVERSPEED_MS`, and comfort (longitudinal and lateral
  accel) tracked against `comfort`'s own empirical bounds. Lateral accel is
  `speed² × curvature` — algebraically the same quantity
  `comfort::Kinematics` measures as `yaw_rate × speed`, since the kinematic
  model defines `yaw_rate = speed × curvature`.

**No analytic derivatives, by construction.** `point_cost` takes
already-known numbers — position, speed, curvature, accel — and returns a
plain `f64`; there's no gradient anywhere in its signature or its callers.
This is a deliberate design constraint, not an oversight: nanoplan never
*provides* a derivative of its cost or dynamics — both are black-box
scalars, and nothing may demand an analytic gradient of either. Most
planners live entirely within that constraint by sampling and comparing
candidates. The one family that genuinely optimizes —
[treetop's iLQR](#ilqr-treetop-finite-differences) — respects it at the
interface: it consumes exactly the same black-box scalars and
differentiates them **numerically** (central finite differences), probing
`point_cost` and `step` a few dozen times per timestep instead of once.
The scalar interface stays the single source of truth for what "good"
means; no second, analytically-differentiated definition of the cost can
drift away from it. Where a planner needs curvature as an input, it gets it
one of two ways, both compatible with that constraint:

- **A closed-form fact about an already-*fixed* candidate curve.** RRT*'s
  `CubicSteer::curvature` evaluates the curvature of a specific Hermite
  polynomial it already committed to — a geometric property of one
  candidate, not a gradient used to choose the next one.
- **A purely numerical estimate off sampled points.** `cost::curvature_of`
  computes the Menger curvature of three points (twice the triangle area
  over the product of the three side lengths) — plain arithmetic, no
  derivative of any parametric formula. The lattice, which has no
  closed-form curve of its own, uses this.

**What stays planner-specific.** Each planner keeps its own structural
terms outside `point_cost` — the pieces that shape *how its search moves*,
not what counts as a good outcome, and that no metric measures:

- The lattice and RRT* both keep their own centerline-pull term (there's no
  "hug the centerline" metric — driving anywhere within the drivable area
  scores the same); the lattice also keeps its DP-layer lateral-smoothness
  term, and RRT*'s edge cost keeps its arc-length accumulation, since that
  *is* what the "star" rewiring optimizes.
- PI²-DDP keeps its own centerline-pull term the same way, plus a
  control-effort quadratic in `running` tied to its own exploration
  covariance (the paper's "linear solvability" trick) — specific to how
  PI²-DDP's sampling distribution is parameterized, not a quality judgment.
- PI²-DDP also converts `point_cost`'s `f64::INFINITY` into a large finite
  `cost::HARD_VIOLATION_PENALTY` instead: its rollout weighting (eq. 12)
  min/max-normalizes cost across rollouts at each timestep, which breaks if
  the range is infinite. The lattice and RRT* have no such statistic and
  propagate the actual infinity, rejecting the candidate outright.
- The road-edge and actor-proximity soft terms inside `point_cost` are
  weighted for PI²-DDP's benefit specifically: the lattice's sampling grid
  never reaches them and RRT* has its own tighter hard bounds
  (`drivable_bound`, `COLLISION_MARGIN_M`), so those two barely feel
  them, but PI²-DDP's continuous search has no hard accept/reject step at
  all — these soft terms are its *only* safety margin, so they're weighted
  to bite hard rather than softly.

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
scenario (`plan_is_a_pure_function_of_state`), the property that lets a
closed-loop rollout inherit any single plan's safety margin — PI²-DDP, which
keeps a real `Rng` for its rollouts, is now the lone exception.

## Sampling MPC (judo)

`sampling_mpc/` — `SamplingPlanner<PredictiveSampling>`,
`SamplingPlanner<Cem>`, `SamplingPlanner<Mppi>`

A port of the three sampling-based optimizers from
[**judo**](https://github.com/rai-opensource/judo)
(`judo/optimizers/{ps,cem,mppi}.py`), kept structurally faithful to judo's
own abstraction and then fitted into the nanoplan framework. The layout
mirrors judo's:

```
sampling_mpc/
├── mod.rs   Optimizer trait + OptimizerConfig (judo base.py), SamplingPlanner<O> driver
├── ps.rs    predictive sampling (judo ps.py)
├── cem.rs   cross-entropy method (judo cem.py)
└── mppi.rs  MPPI (judo mppi.py)
```

**The judo interface, verbatim.** An `Optimizer` is exactly judo's two-method
strategy over control *knots* — `num_nodes` control points of dimension
`nu = 2` (`[accel, curvature]`):

```rust
fn sample_control_knots(&mut self, nominal: &[Knot], sample_base: usize) -> Vec<Vec<Knot>>;
fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot>;
```

The three optimizers are *only* these two methods, matching judo line for
line:

- **Predictive sampling** (`ps.rs`): `sample` = nominal plus fixed-sigma
  noise (first rollout the un-noised nominal); `update` = the single
  best-scoring sample (`argmax` reward).
- **CEM** (`cem.rs`): `sample` = nominal plus an *adaptive per-node* sigma;
  `update` = the elite (top-`num_elites`) mean, with sigma refit to the
  elite std (clipped to `[sigma_min, sigma_max]`), so the distribution
  contracts around whatever keeps scoring well.
- **MPPI** (`mppi.rs`): `sample` like predictive sampling; `update` = a
  Boltzmann reward-weighted average of *all* rollouts,
  `exp(-(cost - min)/temperature)` normalized. The temperature is
  interpreted relative to the rollout cost *spread* (the same min/max
  normalization PI²-DDP applies to its eq.-12 weighting), so it stays a
  scale-free knob rather than tied to a scenario's absolute cost magnitude.

**Everything else is `SamplingPlanner<O>`, the judo→nanoplan adapter.** judo
keeps rollout and reward outside the optimizer; here the generic driver
supplies them the nanoplan way, so each optimizer stays a pure strategy:

- **Knots are deviations from a road-model base policy.** The key
  adaptation. judo's knots *are* the raw controls, applied open-loop over
  the horizon — fine for its short-horizon, feedback-stabilized tasks, but a
  car's lateral dynamics integrate curvature twice, so raw open-loop knots
  diverge metres off-road over a 10 s horizon and every candidate scores as
  garbage (the symptom that drove this design: a nominal rollout ending
  ~20 m off-lane). Instead each interpolated knot is a *deviation* added to a
  **critically-damped PD lane-keeping + speed-hold base policy** evaluated on
  the current rollout state — genuine feedback, so every rollout stays on the
  road and the QMC explores real maneuvers (an obstacle swerve) instead of
  drift. This mirrors PI²-DDP rolling out with its feedback gains rather than
  raw nominal controls, and *is* the "hybrid road model" half of the
  sampling. The nominal starts at zero deviation (the judo-typical zero
  nominal, here meaning "just the base policy").
- **Knots → controls → rollout.** The `num_nodes` deviation knots are spread
  over the `PLANNING_HORIZON_S` horizon and linearly interpolated
  (`control_at`), added to the base policy, clamped to physical actuation
  limits, and rolled out through the shared kinematic `step`.
- **The shared cost function.** Each rolled-out state is priced by
  [`cost::point_cost`](#the-shared-cost-function), with a hard violation made
  finite (`cost::HARD_VIOLATION_PENALTY`) so MPPI's and CEM's reward
  aggregation can't divide by an infinity — exactly PI²-DDP's reasoning. On
  top sit three planner-specific terms, each echoing one PI²-DDP keeps: an
  undiscounted speed-tracking term (the shared cost prices overspeed only
  lightly, which lets MPPI's reward-weighted *average* drift the speed below
  target), a control-effort penalty on the deviation (PI²-DDP's
  "linear-solvability" `½uᵀR⁻¹u`, pulling the deviation back toward the base
  policy unless the cost pays for departing), and a centerline pull.
- **The shared QMC sampler.** The knot noise is drawn from
  [`sampling::qmc_normals`](#shared-qmc-sampling), the *same* low-discrepancy
  sequence RRT* samples targets from — so these planners are deterministic
  pure functions of the ego state (`*_is_a_pure_function_of_state`), unlike
  judo's pseudo-random `np.random.randn`.
- **Warm start.** The winning deviations are carried to the next tick when
  the ego followed the plan, so each 0.1 s replan refines the last.

Each `plan()` runs `iterations` (default 4, echoing PI²-DDP's `GENERATIONS`)
sample→rollout→update passes — a nanoplan adaptation of judo's controller
loop, which runs one optimizer step per control cycle.

**Seams**: `route` (build the `Path`), `warm_start` (reuse or road-informed
re-init), `optimize` (the sample/rollout/update iterations) with `cost` (the
shared cost function, once per rolled-out state) nested inside, `extract`
(sample the winning nominal into `Vec<Control>`).

**Diagnostics**: the final iteration's `num_rollouts` sampled state
sequences, each recorded both as a `trajectory` and flattened into `points`,
mirroring PI²-DDP.

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

An EM/Apollo-style planner. Samples a **high-resolution** grid in the road's
Frenet frame — `STATION_LAYERS = 32` layers spaced evenly out to
`PLANNING_HORIZON_S = 10` s at the assumed cruise speed (`stations_m[i] = v *
PLANNING_HORIZON_S * (i+1) / STATION_LAYERS`) crossed with `LATERALS = 47`
lateral offsets evenly spanning `±3.75` m, i.e. **`32 × 47 = 1504` grid
nodes** — connects consecutive layers with cubic-Hermite lateral segments,
costs every edge (its own lateral-smoothness and centerline-pull terms, plus
the [shared cost function](#the-shared-cost-function) per sampled point —
including its hard `f64::INFINITY` reject on predicted collision or leaving
the drivable area), and finds the cheapest path with **A\* (best-first)
search** over the layered DAG. Curvature at each sampled point, needed for
the shared cost's comfort term, comes from `cost::curvature_of` — the lattice
has no closed-form curve of its own, so it estimates curvature numerically
off the last three sampled points.

**Why A\* rather than the exhaustive DP it used to run.** At this resolution
the old layer-by-layer dynamic program — which prices *every* `L`-to-`L`
inter-layer edge, `O(STATION_LAYERS · LATERALS²)` cost-function evaluations —
would spend almost all its time on large, obviously-bad lateral jumps that no
optimal path uses. Two changes keep the dense grid real-time (p95 well under
10 ms, p100 under 50 ms on the synthetic batch):

- **A\* evaluates edge costs lazily**, only for nodes it actually expands in
  increasing cost-so-far order, and stops the moment it settles a node in the
  final layer. All edge costs are non-negative, so that first final-layer
  node is the global optimum — the path is identical to the DP's, only the
  work to find it is smaller.
- **`NEIGHBOR_SPAN` limits each edge to nearby lateral columns.** A layer is
  only `~horizon / STATION_LAYERS` of travel, so a jump of more than a few
  columns there is a curvature no real car has; never generating those edges
  bounds the branching factor at no cost to path quality (the full lateral
  range is still reachable by ramping over multiple layers).

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

**Seams**: `route`, `optimize` (the A\* search loop) with `cost` (the shared
cost function — nested *inside* `optimize`; it's the hot loop, called once per
sampled point of each edge A\* expands) as a nested seam, then `extract`
(sample the winning path into `xy_to_controls`).

**Diagnostics**: each grid node A\* *expands* as a `point` (plus the tree root
at the ego's current position), and the cubic-Hermite connector of every edge
it evaluates, sampled at `SAMPLES_PER_SEGMENT` points, as a `trajectory` — the
part of the search graph A\* actually explored (which, unlike the old
exhaustive DP, is a small fraction of the full grid), not just the winning
path (that's the separate future-preview line, always drawn regardless of the
diagnostic overlay).

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
distance, rather than an arbitrary constant. The running cost prices the
control just applied against the [shared cost function](#the-shared-cost-function)
— curvature and accel are exactly `u`, the commanded control, since this
kinematic model defines them as the instantaneous curvature and
acceleration — plus PI²-DDP's own centerline-pull term and a control-effort
quadratic tied to the sampling covariance (the paper's "linear solvability"
trick). Unlike the lattice and RRT*, which reject a colliding or off-road
candidate outright, PI²-DDP has no such hard accept/reject step in its
continuous search, so the shared cost's soft road-edge and actor-proximity
terms are weighted heavily for its benefit specifically — see "The shared
cost function" above.

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
expensive part: typically ~85-90% of `total` time) with `cost` (the shared
cost function, called once per rollout per timestep) nested inside it,
`policy_update` (custom — the per-timestep DDP gradient extraction).

**Diagnostics**: the final generation's `ROLLOUTS` sampled state sequences —
each one recorded both as a `trajectory` (the polyline through its `HORIZON`
states) and flattened into `points` (every state along every rollout), so
the overlay can show the sampling distribution's spread either as paths or
as a density of points. Only the last generation is recorded; earlier
generations are refinement steps toward it, not additional information.

## RRT*

`rrt_star/mod.rs` — `RrtStarPlanner`

Rapidly-exploring Random Tree Star: grows a tree of poses from the ego's
current state toward (station, lateral) samples in the road frame, connects
each new node to the cheapest collision-free nearby parent, and rewires
existing nodes when a cheaper path through the new node appears (the "star"
— plain RRT would just keep the first parent found, which isn't
asymptotically optimal).

**Despite the name, the sampling isn't actually random.** `plan()` samples
`GRID_STATIONS × GRID_LATERALS` points from a fixed, road-geometry-informed
grid — the same idea as the [Frenet lattice's](#frenet-lattice) station-
layers-by-laterals grid — then an equal number more from a 2D Halton
low-discrepancy sequence (`van_der_corput`, paired in bases 2 and 3) over
the same domain, filling in what the grid's fixed points miss with
well-distributed rather than clustered coverage. Both are pure functions of
the ego state and scenario (`plan_is_a_pure_function_of_state` pins this
down), so no `Rng` appears anywhere in this module — unlike
[PI²-DDP](#pi2-ddp), which still samples pseudo-randomly for its rollouts.
The grid runs first, in ascending-station order, building a connected
backbone across the full planning horizon before the Halton pass's
arbitrarily-ordered targets are tried, so they almost always land near an
existing node instead of failing for lack of one.

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
at its sample (or matched every node's heading to the lane); either way, two
independently-chosen directions connected by a *short* Hermite tangent needs
far more curvature than any real car has, and nearly every candidate steer
failed the curvature check — measured by instrumenting `feasible`'s own
rejections, under 10 of 300 samples per tick were passing, even on an empty
road. `max_yaw_change(step_len)` inverts this: it caps how
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
[`Path::project_near`](../scenarios/README.md#path-the-frenet-helper) — the
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
tick. (This band replaced an older one-`PROGRESS_TOLERANCE_M`-bucket margin
that the per-tick progress jitter kept crossing.)

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

**Feasibility and edge cost both go through the [shared cost
function](#the-shared-cost-function).** `feasible` additionally enforces its
own tighter margins before ever calling it — `drivable_bound` (the road's
own `half_width` less `DRIVABLE_MARGIN_M` = 0.5 m, so it holds just inside
the shared function's road-edge reject on whatever road is being driven) and
`COLLISION_MARGIN_M` (3.0 m, ahead of the shared 2.5 m collision diameter)
— headroom for the fact that a curve is only checked at `STEER_SAMPLES`
discrete points, so the true closest approach between samples can dip a
little further than what gets tested. `edge_cost` keeps its own arc-length
accumulation (the actual quantity the "star" rewiring minimizes) and
lateral-offset pull, adding the shared cost per sampled point on top;
curvature comes from `CubicSteer::curvature`, a closed-form fact about the
already-fixed candidate curve, not a search gradient.

**Effective progress — not raw distance, and biased toward the side already
committed to — decides the goal.** Ranking on raw station bucketed to
`PROGRESS_TOLERANCE_M` (rather than compared exactly) is most of it: without
bucketing, a node a hair's-breadth further along but squeezing past an
obstacle would beat a node a few centimeters short but giving it a much wider
berth, every single time, since station is compared before cost (which
includes an obstacle-proximity term) ever gets a say. But raw progress alone
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
the term is inert. Tuning `CONTINUITY_WEIGHT` against the synthetic batch,
`0.15` both cut realized lateral-velocity reversals (151→128 over 40
scenarios, worst 15→13) and nudged mean score up (0.5549→0.5761), where a
heavier `0.3` started trading score away.

**Seams**: `route` (build the `Path`), `warm_start` (custom — replaying the
previous winning path), `optimize` (the grid-plus-Halton tree-growing
loop; the deterministic bypass seeding and the final extract step aren't
timed separately since they're comparatively cheap), `extract` (resample the
winning path — itself a `scenarios::Path` built from the tree's polyline —
at `v * dt` intervals and convert to controls via the same technique as the
[Frenet lattice's](#frenet-lattice) `xy_to_controls`). `cost` (the shared
cost function) nests inside all three of `warm_start`, the (untimed)
deterministic bypass seeding, and `optimize` alike, since `feasible` and
`edge_cost` — where it's called, once per sampled point — are shared by
every caller of `try_extend`.

**Diagnostics**: every tree node (after the root) as a `point`, and the
sampled polyline of the edge that added it as a `trajectory` — the whole
search tree considered, not just the winning path, mirroring the lattice's
approach.

## Treetop (RRT / iLQR / RRT+iLQR)

`treetop/` — `RrtPlanner` (`rrt.rs`), `IlqrPlanner` (`ilqr.rs`),
`TreetopPlanner` (`mod.rs`)

A port of [**treetop**](https://github.com/BenGravell/treetop), a
tree-initialized trajectory-optimizing planner: an ego motion sampling tree
provides a strong, collision-aware initial guess at a good path to the
goal, and iLQR (iterative Linear Quadratic Regulator) optimizes that guess
into a smooth trajectory whose solution warm-starts the tree next cycle.
Like the judo port, one upstream codebase yields several registry entries
from one directory — here deliberately three, so the tree and the optimizer
are each measurable *alone* before the coordination glue combines them:

```
treetop/
├── mod.rs   shared OCP core (treetop core/: limits, constrained rollout, goal) + TreetopPlanner glue (treetop planner.h)
├── rrt.rs   the ego motion sampling tree (treetop tree/) — RrtPlanner
└── ilqr.rs  the iLQR solver (treetop ilqr/), finite-difference derivatives — IlqrPlanner
```

nanoplan's kinematic model is *exactly* treetop's `Dynamics::forward` —
same state `[x, y, yaw, speed]`, same action `[accel, curvature]`, same
forward-Euler step — so the dynamics needed no port at all, only the
problem around them. Three adaptations recur throughout (see the module
doc): treetop's fixed user-placed goal pose becomes a **rolling lane
target** (`goal_state`: the centerline pose a planning horizon ahead, at
the target speed); treetop's static circular obstacles become **moving
actors priced through the shared cost function** at the absolute time each
state is reached; and treetop's `std::mt19937` sampling and action jitter
are replaced by the **shared Halton QMC sequence** (jitter dropped
entirely — its purpose is randomized restarts), so all three planners are
pure functions of the ego state, pinned by `*_is_a_pure_function_of_state`
tests.

Shared `mod.rs` core, used by both halves: treetop's vehicle limits
(±3 m/s² longitudinal, ±6 m/s² lateral, |κ| ≤ 0.2) with the **dynamic
curvature bound** `min(CURVATURE_MAX, ACCEL_LAT_MAX / v²)` — at speed, the
car may not steer as sharply as its geometry allows — and the constrained
open-loop rollout that *projects* every action onto those limits
(treetop's key sampling-efficiency trick: projection instead of rejection
keeps nearly every sample usable, at the price of bang-bang edges the
optimizer smooths out). The horizon is `TICKS = 100` ticks (10 s, the
common `PLANNING_HORIZON_S`), split into `SEGMENTS = 10` steering segments
of `STEER_TICKS = 10` ticks.

### RRT (treetop tree)

`treetop/rrt.rs` — `RrtPlanner`

An RRT variant shaped by its downstream job — feeding a trajectory
optimizer — rather than by asymptotic optimality (contrast
[RRT*](#rrt), which rewires toward the shortest path):

- **Time-layered, fixed-depth growth.** The tree has exactly `SEGMENTS`
  layers past the root, each one steering segment later in time, so *any*
  leaf in the final layer closes a full-horizon action sequence of exactly
  `TICKS` controls — precisely the input the iLQR pass wants. Moving
  obstacles come free: a layer's states have a known absolute time, so
  collision checks price actors where they *will be*.
- **Steering in action space.** `steer_actions` fits independent cubics
  `x(t)`, `y(t)` between two states' position+velocity boundary conditions
  (velocity = speed along heading), reads acceleration and curvature off
  the polynomial derivatives — the same differential-flatness idea as
  RRT*'s `CubicSteer`, but emitting *actions* rather than a geometric
  curve — and realizes them through the constrained rollout. A secant
  against the start heading infers forward/reverse. The steer executes
  only its first segment; goal-directed samples steer along a cubic
  spanning the whole remaining horizon and keep just the first second
  of it.
- **Zero-action-point parenting.** A sample attaches to the previous
  layer's node whose coasting endpoint is nearest in `(x, y, yaw, v)` —
  "who reaches me with the least effort" under simplifying kinematic
  assumptions. treetop builds a nanoflann kd-tree per layer for this; a
  layer here holds a few dozen nodes, so a linear scan is simpler *and*
  faster than building the index.
- **Layered sampling, three ways** (treetop's goal 0.1 / warm 0.2 / cold
  0.7 split, drawn against a Halton coordinate instead of an RNG): *goal*
  samples steer toward the goal, *warm* samples perturb around the
  previous solution's trajectory, *cold* samples cover a road-frame
  `(station, lateral, heading error, speed)` box — treetop's axis-aligned
  world-frame box bent into the road frame so it follows a curved road.
- **A zero-action fallback chain** guarantees every layer is non-empty
  (so a full-length path always exists), deliberately ignoring collisions
  — treetop's `growZap`. Such nodes carry a `collides` flag and price
  violating stages at `HARD_VIOLATION_PENALTY`, so they lose to any
  genuine alternative and surface only as a better-than-nothing brace.
- **Edge cost = shared cost + effort + centerline pull.** Every rolled-out
  stage is priced by `point_cost` (hard violations reject the sample
  outright) plus treetop's `softLoss` integrand — the magnitude of total
  (longitudinal, lateral) acceleration — plus this planner's own
  centerline-pull quadratic, the same structural term the lattice, RRT*,
  and PI²-DDP each keep (the shared cost deliberately has no "hug the
  centerline" term; without its own, the tree settles ~2 m off-center,
  since a goal-hitting path there prices almost the same as a centered
  one). Path candidates rank goal-hitters (within
  `GOAL_HIT_TOL` of the goal, treetop's `checkTargetHit` loosened from
  parking precision to lane driving) by cost-to-come, everyone else by
  distance to goal; alternates are the next-best by the same ordering
  where treetop shuffles randomly.

The standalone planner takes the best path candidate as the plan, with its
own warm start (previous plan shifted one tick, replayed as treetop's
"hot" chain and sampled around as "warm") — the plan is exactly what the
treetop planner would hand to iLQR, un-optimized, so the registry can show
what the optimization pass buys.

**Seams**: `route`, `warm_start`, `optimize` (the whole grow), `extract`;
`cost` nests inside wherever an edge is priced.

**Diagnostics**: every tree node as a point and every edge's rollout
polyline as a trajectory — the whole search considered, mirroring RRT*.

### iLQR (treetop, finite differences)

`treetop/ilqr.rs` — `IlqrPlanner`

Iterative LQR: alternate a **backward pass** — dynamic programming over
linearized dynamics and a quadratic cost expansion, producing an affine
policy `u = u_ref + scale·k + K·(x − x_ref)` — with a **forward pass**
rolling that policy out closed-loop and accepting it only if the realized
cost drop is a reasonable fraction of the expansion's prediction
(treetop's feedforward-gain scaling search: backtrack `scale` by 0.2 up to
8 times). Regularization on `Q_uu` is scaled by the gradient norm `|Q_u|`
(treetop's gradient-norm scaling) and adapts by the usual schedule —
decrease on an immediate accept, increase on a backtracked one, surge on a
rejection. The accepted trajectory is finally re-realized under the action
constraints, exactly as treetop re-rollouts its solution.

**Finite differences everywhere, per the port's design brief.** treetop
carries ~200 lines of hand-derived loss gradients/Hessians and a
closed-form dynamics Jacobian; nanoplan deliberately provides neither (see
[the shared cost function](#the-shared-cost-function)). So this solver
differentiates numerically: central differences over the packed
`(x, y, yaw, v, accel, curvature)` vector for the cost gradient and
(symmetrized) Hessian — 73 probes of the black-box scalar per timestep —
and central differences on `simulation::step` for the dynamics Jacobians
`A`, `B` (pinned against the known closed form by
`fd_dynamics_jacobian_matches_the_analytic_one`). FD Hessians of a
piecewise cost are noisy near hinge corners, so where treetop asserts
`Q_uu ≻ 0` "by construction", this port *checks* it and surges
regularization on failure rather than factorizing garbage.

Two cost adaptations make the shared scalar usable under an optimizer that
differentiates it rather than compares it:

- **Hard violations get an escape slope.** `point_cost`'s
  `f64::INFINITY` would poison every difference; PI²-DDP's flat
  `HARD_VIOLATION_PENALTY` substitution is finite but a *flat* plateau has
  zero gradient — a trajectory stuck inside a violation would see no way
  out. This planner prices a violation as
  `HARD_VIOLATION_PENALTY · (1 + depth)`, `depth` being how far inside the
  violation the sample sits (meters past the road edge, meters of overlap
  with an actor) — the same cliff at the boundary, but with a finite-
  difference-visible slope pointing back out.
- **The terminal cost is a quadratic pull to the goal state** (position,
  heading, speed), treetop's terminal loss with its parking-grade
  tolerances swapped for lane-driving quadratics. The structural running
  terms (centerline pull, speed tracking, small control-effort quadratics
  that also keep `l_uu` strictly positive) sit on top of the shared cost,
  scaled by `1/TICKS` as treetop scales by `inverse_traj_length`.

The standalone planner optimizes from a lane-keeping PD initial guess (the
same critically-damped tracker the judo planners use as their base
policy), or its own previous solution shifted one tick. This is trajectory
optimization at its most exposed — a purely local method only as good as
its initial guess, which is precisely the weakness the treetop
coordination exists to fix; it's kept standalone so the registry shows
that difference side by side.

**Seams**: `route`, `warm_start`, `optimize`, `extract`, with `derivs`
(all backward-pass FD work) and `rollout` (forward passes and trajectory
pricing) nested inside `optimize`. Unlike the other search planners there
is no per-call `cost` seam: the FD probes call the shared cost ~10⁵ times
per plan, and timing each call would cost more than the call — `derivs`
and `rollout` are where those calls live.

**Diagnostics**: the optimized trajectory as a polyline and its states as
points.

### treetop (RRT + iLQR)

`treetop/mod.rs` — `TreetopPlanner`

The coordinator glue, treetop's `planner.h` loop: **tree → candidates →
iLQR → best → feed back.**

1. Grow the tree (450 samples), warm-started from last tick's *optimized*
   solution — replayed as the hot chain and sampled around by the warm
   samples, so the tree keeps refining the maneuver the optimizer chose
   rather than rediscovering a different one each tick.
2. Extract the best `num_path_candidates = 2` full-horizon path
   candidates.
3. Run iLQR on each candidate's action sequence (a handful of iterations —
   the tree's near-feasible guess converges fast, where treetop's
   on-demand replans afford up to 200).
4. Select by treetop's two-tier rule: the cheapest solution that still
   *hits the goal*, else the one ending nearest it — a candidate that
   optimized to a low cost by giving up on progress must not beat one that
   gets there.
5. Store the winner's action sequence as next tick's warm start, and
   drive its first control.

The division of labor is the point, and it's the same lesson RRT*'s
warm-start section tells from the other side: the tree contributes global,
discontinuity-crossing search (which side of the obstacle, brake vs.
swerve) that a local optimizer can't do, and iLQR contributes the smooth,
limit-respecting polish (and consistent tick-to-tick refinement) that a
bang-bang sampled tree path lacks. Treetop's action jitter — a third
mechanism for hopping out of local minima — is omitted deliberately: it's
pseudo-random by nature, and determinism (plan as a pure function of
state) is worth more here than its occasional escape.

**Seams**: `route`, `warm_start`, `optimize` split into treetop's own
`TimingInfo` pair — `tree` (grow + candidate extraction) and `traj_opt`
(the iLQR passes) — then `extract`. `cost` nests inside `tree`; the iLQR
passes bury their shared-cost calls in `derivs`/`rollout` as above.

**Diagnostics**: the whole tree (nodes as points, edges as trajectories),
plus the winning candidate's pre-optimization polyline and its post-iLQR
trajectory — the before/after pair that shows what the optimizer bought.
