# 0001 — Bundle `(centerline, target_speed, dt)` into a `Road` value

- **Status**: Accepted
- **Pattern**: Introduce Parameter Object
  ([refactoring.guru](https://refactoring.guru/introduce-parameter-object))
- **Affects**: `src/scenarios/`, `src/planning/`, `src/simulation/`, `src/metrics/`, `src/viewer/`

## Context

Three values describe the fixed setting of one closed-loop run: the route
centerline the ego follows, the target/cruise speed (which doubles as the
metrics' speed limit), and the tick length `dt` everything is sampled at.
They are not scenario data — `dt` in particular is a property of the
*experiment*, chosen by the caller of `simulate()`, not by the `Scenario`
(see the simulation README's "Why this design").

Before this change the trio traveled as a **data clump**: an unnamed,
recurring parameter list threaded through the whole system.

- `metrics::evaluate(ego, actors, centerline, speed_limit, dt)` — a
  five-argument function, three of them the clump.
- `IncrementalSim` stored `centerline`, `target_speed`, and `dt` as three
  separate loose fields.
- `planning::Context` carried `centerline`, `target_speed`, and `dt` as three
  separate fields, re-assembled by hand at every construction site
  (`simulation`, `viewer::draw`, the test harness).

The clump had no name, so nothing signalled that the three move together;
each new consumer re-plumbed all three, and a five-argument evaluate() is
exactly the shape that invites transposed-argument bugs (`speed_limit` and
`dt` are both bare `f64`).

## Decision

Introduce `scenarios::Road`:

```rust
pub struct Road {
    pub centerline: Vec<[f64; 2]>,
    pub target_speed: f64,
    pub dt: f64,
}

impl Scenario {
    pub fn road(&self, dt: f64) -> Road { /* pair scenario fields with run dt */ }
}
```

- `Context` now embeds `road: &'a Road`; planners read `ctx.road.centerline`,
  `ctx.road.target_speed`, `ctx.road.dt`.
- `IncrementalSim` holds one `Road` for the run instead of three fields.
- `metrics::evaluate(ego, actors, &road)` — three arguments, the clump named.

`Road` is derived per run (`sc.road(dt)`), keeping `dt` out of the
serialized `Scenario` format.

## Consequences

**Good**

- One name for a concept that was previously implicit. The three `Context`
  construction sites collapse to passing one reference.
- `evaluate()`'s signature can no longer transpose `speed_limit` and `dt`.
- Sets up ADR 0002: `metrics::evaluate` builds its per-rollout `TickCtx`
  straight off the `Road`, so the parameter object and the metric table
  compose cleanly.

**Costs / trade-offs**

- `Road` owns its `centerline` (a `Vec`), so `sc.road(dt)` clones the
  polyline once per run. Runs are coarse-grained (one per scenario×planner),
  so this is negligible; a borrowing `Road<'a>` was rejected as lifetime
  noise for no measurable gain.
- One more type in the public surface (`Road` is re-exported from the crate
  root alongside `Path`/`Scenario`).

## Alternatives considered

- **Leave it as loose parameters.** Rejected: the clump was already threaded
  through four modules and every new metric/planner/consumer re-plumbed it.
- **A Builder for `Road`/`Scenario`/`IncrementalSim`.** Rejected as
  over-engineering: `Road` has three fields with no optional/telescoping
  construction, and `Scenario` already gets its defaults from serde. Builder
  solves a problem this code doesn't have.
- **Borrowing `Road<'a>` to avoid the centerline clone.** Rejected: adds a
  lifetime parameter to `Road` and everything that stores it (`IncrementalSim`)
  to save one clone per multi-tick run.

## Verification

Full test suite green; `cargo run --bin batch -- --count 4 --seed 42`
produces byte-identical CSV before and after (behavior-preserving).
