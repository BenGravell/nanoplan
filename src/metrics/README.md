# `metrics`

nuPlan-derived closed-loop planner quality metrics, computed strictly
tickwise as **pure functions of simulation output**. Nothing in this
component sees a `Planner`, a `Context`, or planner internals of any
kind — only the finished ego trace, actor traces, and the `Road` (centerline,
speed limit, tick length) that `simulate()` already produced. That boundary
is enforced by the type signature of `evaluate()`, not just convention.

Each metric is one row of the `METRICS` spec table in `mod.rs` — its display
label, per-tick score function, aggregation rule, and composite role, all in
one place. A metric's position in the score arrays is the position of its
row; nothing indexes scores by hand-synchronized magic number.

```
metrics/
├── mod.rs               Metrics struct, METRICS spec table, evaluate() orchestrator, composite formula
├── collisions/          no at-fault collisions          (event-driven, min)
├── ttc/                 time to collision within bound  (event-driven, min)
├── drivable_area/       drivable area compliance        (event-driven, min)
├── driving_direction/   driving direction compliance    (event-driven, min)
├── making_progress/     making-progress boolean          (threshold on progress)
├── progress/            progress ratio                   (smooth, average)
├── speed_limit/         speed limit compliance           (smooth, average)
└── comfort/             comfort (accel/yaw bounds)        (smooth, average)
```

Threshold values throughout are taken from the vendored
[`scenarios/nuplan/metrics_description.md`](../../scenarios/nuplan/metrics_description.md)
(nuPlan's own metric definitions). Where this implementation's *aggregation*
deliberately differs from nuPlan's, it's called out below and in the module
doc comment.

## The tickwise model

Every metric produces a score in `[0, 1]` **at every simulation tick**, not
just once per scenario. `Metrics::per_tick[tick]` is an 8-element array (one
score per metric, in the row order of the `METRICS` table), and
`Metrics::score_per_tick[tick]` is their composite at that instant. This is
what lets the viewer's metrics table show a collision (or a hard brake, or a
moment of leaving the road) as a dip at an exact scrubbed time, rather than
only as a scenario-wide verdict.

```rust
pub struct Metrics {
    pub per_tick: Vec<[f64; N_METRICS]>,
    pub score_per_tick: Vec<f64>,
    pub aggregate: [f64; N_METRICS],
    pub score: f64,
}

impl Metrics {
    pub fn at(&self, tick: usize) -> ([f64; N_METRICS], f64) { ... } // clamped to the rollout
}
```

## Aggregation: two rules, chosen per metric

Scenario-level values (`aggregate`, and the scenario `score`) fold the
per-tick series down with the aggregation function in each metric's
`METRICS` row — `agg::min` or `agg::avg` for almost every metric, chosen by
what kind of failure the metric represents:

- **Event-driven → worst case (`agg::min`)**: collisions, drivable area, driving
  direction, TTC. One bad tick is a real violation of the scenario — a
  planner that collides once and drives perfectly the rest of the time did
  not have a good run, so the *minimum* over all ticks (not the average) is
  what the scenario aggregate reports. A single collision tick zeroes the
  entire scenario's score, exactly matching what you'd want from a safety
  metric.
- **Smooth → average (`agg::avg`)**: progress ratio, speed limit compliance,
  comfort. These represent ongoing quality-of-driving quantities where one
  bad instant (a brief hard brake, a moment slightly over the limit)
  shouldn't zero out an otherwise-good rollout the way a collision does; the
  average reflects magnitude and duration proportionally.
- **`making_progress`** is a special case with its own aggregation function
  (`making_progress::aggregate`): a boolean *threshold* on the progress
  ratio (`> 0.2`), applied both per-tick (for the `@t` column) and to the
  already-averaged aggregate progress ratio (for the scenario column) —
  matching nuPlan's `min_progress_threshold` semantics, which is a
  scenario-level threshold rather than a tickwise one. It's the reason the
  aggregation slot in `MetricSpec` is a function, not a two-variant enum.

## The composite score

Each metric's `METRICS` row declares its `CompositeRole`:

```rust
pub enum CompositeRole {
    Multiplier,    // gates the score: a 0 zeroes everything
    Weighted(f64), // blends into the weighted average with this weight
}
```

`composite()` folds over the table: the product of every `Multiplier`
metric times the weighted average of the `Weighted` ones. This reproduces
nuPlan's structure — the four multiplier metrics (collisions, drivable
area, driving direction, making progress) gate the score entirely, and the
four weighted metrics (TTC, progress, speed limit, comfort at 5/5/4/2,
matching nuPlan's published weighting) blend
smoothly. This same formula is applied twice per rollout: once per tick
(`score_per_tick`, for the `@t` column and future use) and once to the
aggregates (`Metrics::score`, the number everything else — the viewer's
"closed-loop score" and the batch runner's CSV `score` column — actually
reports).

## The eight metrics

| # | Metric | Module | Rule | What it checks | Key thresholds |
|---|---|---|---|---|---|
| 0 | No at-fault collisions | `collisions` | min | Ego stays farther than `2 × CAR_RADIUS_M` (2.5 m) from every actor. | `CAR_RADIUS_M = 1.25` (circle approximation of the vehicle footprint, shared with the planners' own 2.5 m collision spacing) |
| 1 | Drivable area | `drivable_area` | min | Ego's signed Frenet lateral offset stays within the road's half-width (`Road::half_width`, from the scenario's `map.road_half_width` or the live street geometry). | `ROAD_HALF_WIDTH_M = 5.5` (default when a scenario sets none) |
| 2 | Driving direction | `driving_direction` | min | Ego doesn't move backward along the route by more than a threshold over a trailing 1 s window. | `2.0` m → full credit, `2.0`-`6.0` m → half credit, `>6.0` m → zero |
| 3 | Making progress | `making_progress` | threshold | The (aggregated or per-tick) progress ratio exceeds a minimum. | `MIN_PROGRESS_RATIO = 0.2` |
| 4 | TTC within bound | `ttc` | min | Constant-velocity projections of ego and every actor, sampled every `0.1` s out to `3.0` s, never come within `2 × CAR_RADIUS_M`. | `LEAST_MIN_TTC_S = 0.95` |
| 5 | Progress ratio | `progress` | average | Station rate at this tick relative to driving at the speed limit, clamped to `[0, 1]`. | — (no expert trajectory available, so the speed limit stands in for it — see the `ponytail:` comment in `progress/mod.rs`) |
| 6 | Speed limit | `speed_limit` | average | Overspeed above the limit, normalized. | `MAX_OVERSPEED_MS = 2.23` |
| 7 | Comfort | `comfort` | average | Longitudinal accel, lateral accel, and yaw rate all within smooth-driving bounds. | see `comfort/mod.rs` |
| 8 | Lane keeping | `lane_keeping` | average | Ego holds the center of its lane: penalizes a *sustained* one-sided bias (mean offset over a trailing window) and *instantaneous* straddling of the lane line into the next lane. | `LANE_HALF_WIDTH_M = 1.75`, `CENTER_TOLERANCE_M = 0.5` |

The `METRICS` table holds the same nine rows in this order — its `label`
strings are what the viewer's table and the batch CSV header display; index
`i` into `per_tick[tick]` and `aggregate` always means the metric in row `i`
above.

## `evaluate()` and `TickCtx`

```rust
pub fn evaluate(ego: &[State], actors: &[Vec<State>], road: &Road) -> Metrics
```

Builds a `TickCtx` once per rollout — the precomputed series every metric
scores from: the ego trace, per-tick actor states, the ego's Frenet station
and lateral-offset series (via one `scenarios::Path` projection pass), and
the `comfort::Kinematics` (forward-differenced accel/yaw-rate)
— then loops over ticks calling each `METRICS` row's
`score(&TickCtx, tick)` function and composing. The per-metric aggregation
(each row's `aggregate` function over its score column) and the final
`composite()` call happen once, after the tickwise loop.

`actors[i]` must be sampled at the same ticks as `ego` — `simulate()`
guarantees this by construction (see
[`src/simulation/README.md`](../simulation/README.md#simulate-and-rollout)).

## Adding a metric

1. Create `metrics/my_metric/mod.rs` with a
   `pub fn score(ctx: &TickCtx, i: usize) -> f64` returning a value in
   `[0, 1]`, plus a module doc comment stating its aggregation rule
   (event-driven/min or smooth/average) and its thresholds' provenance.
2. Add `pub mod my_metric;` to `metrics/mod.rs`.
3. Add one `MetricSpec` row to the `METRICS` table: label, `my_metric::score`,
   an aggregation function (`agg::min`, `agg::avg`, or your own), and a
   `CompositeRole`. `N_METRICS`, the viewer table, and the batch CSV header
   all follow the table automatically.
4. If the metric needs a per-rollout series `TickCtx` doesn't carry yet,
   precompute it in `evaluate()` and add a field — metrics must stay pure
   functions of simulation output.
5. If nuPlan defines this metric, cite the threshold values from
   [`scenarios/nuplan/metrics_description.md`](../../scenarios/nuplan/metrics_description.md)
   rather than guessing.

## Testing

`metrics/mod.rs`'s tests are tick-exact: e.g.
`collision_zeroes_its_ticks_and_the_scenario_by_min_agg` asserts a collision
scores `0` at the exact colliding tick, `1` on an untouched tick, and `0`
for both the aggregate and the final scenario score (proving the min-vs-avg
distinction actually holds). `harsh_braking_is_uncomfortable_only_while_braking`
checks the opposite: comfort dips only during the braking window but the
*aggregate* stays strictly between `0` and `1`, proving average aggregation
doesn't over-punish a brief event the way min aggregation would.
