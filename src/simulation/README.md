# `simulation`

The kinematic vehicle model, the receding-horizon `Simulator`, and
`simulate()` — the single function that turns a `Scenario` and a chosen
planner into a `Rollout` (traces + metrics + latency). This is the one
component every other piece of the codebase (the viewer, the batch runner,
and indirectly the metrics evaluator) goes through to actually run
something.

```
simulation/
└── mod.rs   State, Control, step(), Simulator, simulate()/Rollout
```

## The kinematic model

```rust
pub struct State {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub speed: f64,
}

pub struct Control {
    pub accel: f64,
    pub curvature: f64,
}
```

Deliberately not a full bicycle model with a wheelbase parameter — curvature
is used directly as the control, which is what every planner in this repo
actually computes (Frenet curvature, Bezier curvature, sampled curvature).
A wheelbase-based steering-angle mapping is a one-line addition at the
actuator boundary if a specific vehicle's steering limits ever matter; until
then it would be unused generality.

`step()` is one explicit-Euler integration step:

```rust
pub fn step(s: State, u: Control, dt: f64) -> State {
    State {
        x: s.x + s.speed * s.yaw.cos() * dt,
        y: s.y + s.speed * s.yaw.sin() * dt,
        yaw: s.yaw + s.speed * u.curvature * dt,
        speed: s.speed + u.accel * dt,
    }
}
```

`Control::default()` (zero accel, zero curvature) drives straight ahead at
whatever speed the state already has — this is both the strawman planner's
entire plan and the "no plan returned" fallback in `Simulator::tick`.

`step()` is also reused outside the closed loop: actors without a logged
trajectory integrate under a constant `Control` this same way (see
[`src/scenarios/README.md`](../scenarios/README.md#actor-motion)), and the
viewer's future-preview overlay rolls a plan forward into positions with it.

## `Simulator`

```rust
pub struct Simulator {
    pub state: State,
    pub dt: f64,
}

impl Simulator {
    pub fn tick(&mut self, planner: &mut dyn Planner, ctx: &Context) -> State { ... }
}
```

One receding-horizon step: call `planner.plan(self.state, ctx)`, take only
the **first** control (`.first().copied().unwrap_or_default()` — an empty
plan coasts), integrate it, and store + return the new state. The `total`
latency seam (see
[`src/planning/README.md#latency-diagnostics`](../planning/README.md#latency-diagnostics))
is recorded here, around the `plan()` call, so every planner gets it whether
or not it instruments itself — `ctx.time("total", || planner.plan(...))`.

`Simulator` itself is planner-agnostic and scenario-agnostic; it doesn't know
about actors, metrics, or scenario data at all. That separation is what lets
`simulate()` below stay a thin composition rather than its own subsystem.

## `simulate()` and `Rollout`

```rust
pub struct Rollout {
    pub ego: Vec<State>,
    pub actors: Vec<Vec<State>>,
    pub metrics: Metrics,
    pub latency: LatencyStats,
}

pub fn simulate(sc: &Scenario, kind: PlannerKind, duration_s: f64, dt: f64) -> Rollout
```

This is the seam between all four components. In order:

1. Build every actor's trace up front for the full duration
   (`Actor::trace`, in [`scenarios`](../scenarios/README.md#actor-motion)) —
   either replaying a logged trajectory or integrating a constant control.
   Actors are **not** replanned during the loop; only the ego is.
2. Build `kind.build()` into a fresh `Box<dyn Planner>`, a `Simulator`
   seeded at `sc.ego`, and the run's fixed
   [`Road`](../scenarios/README.md#road-the-fixed-setting-of-a-run)
   (`sc.road(dt)`).
3. Step `duration_s / dt` ticks. Each tick builds a `Context` over that
   `Road` with the actors' states *at that tick* (sliced from their
   precomputed traces), calls `sim.tick(...)`, and drains the tick's latency
   spans into a running `LatencyStats` via `LatencyStats::absorb`.
4. Once the full ego trace exists, call
   [`metrics::evaluate`](../metrics/README.md) once over the whole thing
   (the finished traces plus the same `Road`) — metrics are a pure post-hoc
   function of simulation output, not computed incrementally during the
   loop.
5. Package `ego`, `actors`, `metrics`, `latency` into a `Rollout`.

Both consumers — `src/viewer/` (the viewer) and `src/bin/batch.rs` (the
batch runner) — call this one function and nothing else from `simulation`
except the `State`/`Control` types and, in the viewer's future-preview code,
`step()` directly (to roll a *hypothetical* plan forward without re-running
the whole closed loop). Neither consumer duplicates the tick loop.

## Why this design

- **`dt` and `duration_s` are simulate()'s parameters, not the Scenario's.**
  A `Scenario` describes the world; how long and how finely you simulate it
  is a property of the experiment (the viewer and the batch runner both
  currently use `DT = 0.1`, `DURATION_S = 20.0`, but nothing enforces they
  must match).
- **Actors are precomputed, not replanned**, because nothing in this
  codebase currently models multi-agent interactive prediction — actors are
  either scripted (constant control) or logged (replay). If interactive
  actor behavior is ever added, it would change step 1 into something that
  runs alongside the ego loop rather than before it.
- **Metrics run once, after the fact**, deliberately: see
  [`src/metrics/README.md`](../metrics/README.md) for why this keeps metrics
  a pure function of simulation output with no access to planner internals.
