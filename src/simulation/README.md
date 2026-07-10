# `simulation`

The kinematic vehicle model, the receding-horizon `Simulator`, and
`simulate()` — the single function that turns a `Scenario` and a chosen
planner into a `Rollout` (traces + metrics + latency). This is the one
component every other piece of the codebase (the viewer, the batch runner,
and indirectly the metrics evaluator) goes through to actually run
something.

```
simulation/
├── collision.rs       actor footprint collision response
├── integration.rs     private world plant step and command limiter
├── physics.rs         formulas around limits, drag, and clamping
├── state_control.rs   State and Control
└── mod.rs             Simulator, IncrementalSim, simulate()/Rollout
```

Roadside barrier entities live at crate level in `crate::barrier`; simulation
re-exports them for existing callers.

Vehicle capability constants live in `crate::vehicle`; `simulation` consumes
them and re-exports the public ones for existing callers.

## The kinematic model

```rust
pub struct State {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub speed: f64,
}

pub struct Control {
    pub acceleration: f64,
    pub curvature: f64,
}
```

Deliberately not a full bicycle model with a wheelbase parameter — curvature
is used directly, which is what every planner in this repo already computes
(Frenet curvature, Bezier curvature, sampled curvature). The planner-facing
action is just longitudinal acceleration and curvature.

The planner-internal model lives in `crate::planning::model`: pure kinematics
plus state/control-space vehicle limits. `simulation` owns the higher-fidelity
world plant. Its private `world_step()` clamps acceleration and curvature,
subtracts passive rolling resistance and aerodynamic drag, then integrates
pose and speed:

```rust
fn world_step(s: State, u: Control, dt: f64) -> State {
    let u = clamp_control(u, s.speed);
    let net_accel = u.acceleration - longitudinal_resistance_accel(s.speed);
    State {
        x: s.x + s.speed * s.yaw.cos() * dt,
        y: s.y + s.speed * s.yaw.sin() * dt,
        yaw: s.yaw + s.speed * u.curvature * dt,
        speed: s.speed + net_accel * dt,
    }
}
```

In the world model, `Control::default()` applies no throttle/brake input and
coasts straight, losing speed to those passive losses. Under a constant
positive acceleration command, drag grows with `speed²`, so the model has a
finite terminal speed. Negative speed is allowed; passive losses oppose
reverse motion too.

`world_step()` deliberately has no actuator memory. Closed-loop simulation and
live world use `CommandLimiter` before calling it; planner rollouts cannot call
it and instead use `planning::model::step`.

### Vehicle capability

These global constants live in `src/vehicle.rs` and are capability limits,
*not* comfort limits. The public planner command is acceleration/curvature;
the simulator privately slews the applied command toward it so impossible
command jumps are softened without making planners model jerk or curvature
rate.

- **longitudinal acceleration** `MAX_LON_ACCEL = 4.0` m/s² and **braking**
  `MIN_LON_ACCEL = -9.0` m/s²;
- **passive losses** from rolling resistance
  (`ROLLING_RESISTANCE_COEFF = 0.012`) and air drag
  (`0.5 * AIR_DENSITY_KG_M3 * DRAG_AREA_M2 * v² / EGO_MASS_KG`);
- **steering angle** `MAX_ABS_CURVATURE = 0.2` /m (a ~5 m turning radius);
- **lateral acceleration** `MAX_ABS_LAT_ACCEL = 9.0` m/s² (~0.9 g skidpad
  grip), which tightens the curvature limit as speed rises so the car can't
  hold a hairpin at highway speed;
- **longitudinal jerk** and **curvature rate** are private `Simulator`
  guard rails inside `CommandLimiter`, deliberately not planner inputs.

### Two Forward Models

`planning::model::step(state, control, dt) -> State` is the direct kinematic
model used for planner rollouts. It has no drag dynamics, no actuator memory,
and no collision response; speed is only clamped to the static terminal-speed
envelope exported by `physics.rs`.

`Simulator::tick`, live world ego stepping, and scenario actors without a
logged trajectory use the world model: command slew in `CommandLimiter`,
passive resistance in `world_step()`, and collision/barrier handling around
the resulting state.

Roadside barriers are first-class physics entities in `crate::barrier`:

```rust
pub struct Barrier {
    pub a: [f64; 2],
    pub b: [f64; 2],
    pub normal: [f64; 2],
    pub restitution: f64,
}
```

`Road::new` generates barrier segments from the road centerline and
half-width. `collide_with_barriers(prev, next, barriers)` sweeps the ego
footprint against those segments from either side, clamps to the first crossed
barrier, slides along the remaining tangent motion, and reflects the velocity
component through it.

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
plan coasts), integrate it, collide the ego with the current road-side
barriers, and store + return the new state. The `total`
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
   either replaying a logged trajectory or integrating a constant
   acceleration/curvature command through the same private command limiter.
   Actors are **not** replanned during the loop; only the ego is.
2. Build `kind.build()` into a fresh `Box<dyn Planner>`, a `Simulator`
   seeded at `sc.ego`, and the run's fixed
   [`Road`](../scenarios/README.md#road-the-fixed-setting-of-a-run)
   (`sc.road(dt)`).
3. Step `duration_s / dt` ticks. Each tick builds a `Context` over that
   `Road` with the actors' states *at that tick* (sliced from their
   precomputed traces), calls `sim.tick(...)`, and drains the tick's latency
   spans into a running `LatencyStats` via `LatencyStats::absorb`. The target
   speed the planner sees is tapered into a comfortable stop at the route end
   (`GOAL_DECEL_MS2`), so the ego arrives and holds the goal pose rather than
   sailing off the end of its reference and spinning about it; where the route
   outlasts the horizon this never binds. Scoring in step 4 uses the
   scenario's own speed limit, not the tapered value.
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
  either scripted (constant acceleration/curvature command) or logged
  (replay). If interactive
  actor behavior is ever added, it would change step 1 into something that
  runs alongside the ego loop rather than before it.
- **Metrics run once, after the fact**, deliberately: see
  [`src/metrics/README.md`](../metrics/README.md) for why this keeps metrics
  a pure function of simulation output with no access to planner internals.
