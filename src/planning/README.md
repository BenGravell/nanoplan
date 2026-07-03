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
└── pi2ddp/       sampling-based DDP (PI²-DDP)
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
pub enum PlannerKind { Straight, BezierIdm, Lattice, Pi2Ddp }
```

The selection/comparison seam. `PlannerKind::ALL` is the definitive list the
viewer's dropdown and the batch runner iterate over; `.name()` gives the
display string; `.build()` returns a fresh `Box<dyn Planner>`.

**To add a fifth planner:**

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

An EM/Apollo-style planner. Samples a grid in the road's Frenet frame — three
station layers spaced evenly out to `PLANNING_HORIZON_S = 10` s at the
assumed cruise speed (`stations_m = v * [1/3, 2/3, 1] * PLANNING_HORIZON_S`)
crossed with five lateral offsets (`LATERALS_M = [-3.5, -1.75, 0, 1.75, 3.5]`
m) — connects consecutive layers with cubic-Hermite lateral segments, costs
every edge
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
`3 stations × 5 laterals × up to 5 predecessors` times per `plan()` call) as
a nested seam, then `extract` (sample the winning path into `xy_to_controls`).

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
