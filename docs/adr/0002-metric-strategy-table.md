# 0002 — Replace metric magic indices with a `METRICS` spec table

- **Status**: Accepted
- **Pattern**: Strategy, table-driven
  ([refactoring.guru](https://refactoring.guru/design-patterns/strategy))
- **Affects**: `src/metrics/` (all modules), `src/viewer/ui.rs`, `src/bin/batch.rs`

## Context

There are eight nuPlan-derived quality metrics. Each was already its own
module with a `score()` function — a strategy in spirit. But a metric's
*identity* was its **position in a set of parallel arrays**, and that
position had to be kept in sync by hand across six places:

1. `N_METRICS: usize = 8` — the arity.
2. `Metrics::LABELS` — the display strings, in order.
3. The ordered `scores = [...]` literal in `evaluate()`'s tick loop.
4. `composite()`'s hardcoded indices and weights.
5. The aggregation block, readable only with index comments: `min_of(0), // collisions`.
6. Every consumer that zipped `LABELS` against a score array (viewer table,
   batch CSV header).

To answer "what is `m[4]`?" you counted entries in `LABELS`. The compiler
enforced only the array *arity* (all `[f64; 8]`), not the *ordering*: reorder
two entries in the `scores` literal without also reordering `composite()`'s
indices and every downstream number is silently wrong, with no error. This is
the dangerous failure mode — a reorder produces plausible-but-wrong scores.

## Decision

Make each metric one row of a spec table (Strategy, table-driven). Every
metric module exposes a uniform `score(&TickCtx, tick) -> f64`, and the table
row carries everything else about it:

```rust
pub struct MetricSpec {
    pub label: &'static str,
    pub score: fn(&TickCtx, usize) -> f64,
    pub aggregate: fn(&TickCtx, &[f64]) -> f64,   // agg::min | agg::avg | custom
    pub role: CompositeRole,                      // Multiplier | Weighted(f64)
}

pub const METRICS: [MetricSpec; 8] = [ /* one row per metric */ ];
pub const N_METRICS: usize = METRICS.len();
```

- `TickCtx` holds the per-rollout series `evaluate()` precomputes once (ego
  trace, per-tick actor states, Frenet station/lateral series, comfort
  `Kinematics`, physical speed envelope, dt). A metric reads the tick it's
  given and nothing else, preserving the "pure function of simulation output" boundary.
- `evaluate()` fills `per_tick[i][m] = (METRICS[m].score)(&ctx, i)`, then
  aggregates each column with `METRICS[m].aggregate`.
- `composite()` folds over the table: product of the `Multiplier` metrics
  times the weighted average of the `Weighted` ones. The weights now
  live on the metrics they describe.
- The viewer table and batch CSV header iterate `METRICS` for labels.

The one genuinely irregular metric — `making_progress`, which thresholds
another metric's *aggregate* — becomes an explicit `making_progress::aggregate`
function in its row, visible rather than buried as a mid-array special case.
That case is exactly why the `aggregate` slot is a `fn`, not a two-variant
enum.

## Consequences

**Good**

- A reorder is no longer silently wrong: the row *is* the ordering, and label,
  score, aggregation, and composite role for a metric all sit on one line.
- Adding a metric goes from "edit six synchronized places" to "write the
  module, add one row." `N_METRICS`, the viewer, and the batch CSV follow the
  table automatically.
- `composite()` and the aggregation loop no longer contain magic indices.

**Costs / trade-offs**

- `MetricSpec` uses function pointers (`fn(&TickCtx, usize) -> f64`), an
  indirect call per metric per tick. Negligible next to the metric bodies
  (TTC projects every actor 30 steps), and metrics run once post-hoc, not in
  the planner hot loop.
- `making_progress::aggregate` recomputes the progress column from `TickCtx`
  rather than receiving the already-computed progress scores — a small
  redundancy that keeps the `aggregate` interface uniform (every metric
  aggregates its *own* score column) instead of special-casing cross-metric
  dependencies in `evaluate()`.
- `TickCtx` must carry any series a metric needs; a metric wanting a new
  derived quantity adds a field there.

## Alternatives considered

- **Const index names** (`const TTC: usize = 4; …`). Rejected: names the
  indices but still leaves score/label/weight/aggregation in four separate
  synchronized structures — the ordering hazard survives.
- **A trait object per metric** (`Box<dyn Metric>`). Rejected: heavier than
  needed. The strategies are stateless free functions; a `fn` pointer in a
  `const` table is the minimal form and keeps everything in one screenful.
- **Enum-valued aggregation** (`enum Aggregation { Min, Avg }`). Rejected:
  `making_progress` doesn't fit two variants (it thresholds a cross-metric
  aggregate), so a `fn` slot is strictly more general at no extra cost.

## Verification

Full test suite green (the tick-exact metric tests are unchanged); batch CSV
byte-identical before/after on `--count 4 --seed 42`.
