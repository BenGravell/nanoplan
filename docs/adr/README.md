# Architecture Decision Records

Short records of design decisions that shaped the codebase: the context, the
decision, and the trade-offs weighed. Each ADR is self-contained — read the
one that covers the part of the code you're touching.

## Format

Each record follows a light [MADR](https://adr.github.io/madr/)-style
template: **Context** (the problem and forces), **Decision** (what we did),
**Consequences** (what it bought and cost), and **Alternatives considered**.
ADRs are immutable once accepted — if a decision is revisited, add a new ADR
that supersedes the old one rather than editing history.

## Index

| ADR | Decision | Pattern | Status |
|---|---|---|---|
| [0001](0001-road-parameter-object.md) | Bundle `(centerline, target_speed, dt)` into a `Road` value | Introduce Parameter Object | Accepted |
| [0002](0002-metric-strategy-table.md) | Replace metric magic indices with a `METRICS` spec table | Strategy (table-driven) | Accepted |
| [0003](0003-planner-spec-registry.md) | Collapse `PlannerKind`'s parallel `match`es into one `PlannerSpec` table | Factory Method (table-driven) | Accepted |
| [0004](0004-scenario-source-strategy.md) | Unify the desktop/web scenario loaders behind a `ScenarioSource` trait | Strategy / Adapter | Accepted |

## Why these four

All four came out of a single design-patterns pass over the codebase
([refactoring.guru](https://refactoring.guru/design-patterns) as the
reference catalog). The common thread: behavior that *belonged to* an
abstraction was scattered across parallel arrays, synchronized `match`
arms, or duplicated `cfg` blocks, so the compiler couldn't catch a
desynchronized edit. Each ADR names the specific silent-failure mode the
pattern closes.

Patterns deliberately **not** adopted (and why) are recorded at the bottom
of the relevant ADR, so the same tempting-but-wrong ideas don't get
relitigated: Template Method for `plan()` (0003), Observer for the
latency/diagnostics recorders (0003), Builder for `Scenario`/`IncrementalSim`
(0001).
