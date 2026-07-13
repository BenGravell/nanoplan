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
| [0002](0002-metric-strategy-table.md) | Replace metric magic indices with a `METRICS` spec table | Strategy (table-driven) | Accepted |
| [0003](0003-planner-spec-registry.md) | Collapse `PlannerKind`'s parallel `match`es into one `PlannerSpec` table | Factory Method (table-driven) | Accepted |
| [0005](0005-shared-qmc-sampling-interface.md) | Share RRT*'s road-frame QMC sampling with the judo optimizers behind one `QuasiMonteCarlo` trait | Strategy / dependency inversion | Accepted |

## Why these three

The first two came out of a single design-patterns pass over the codebase
([refactoring.guru](https://refactoring.guru/design-patterns) as the
reference catalog). The common thread: behavior that *belonged to* an
abstraction was scattered across parallel arrays, synchronized `match`
arms, or duplicated `cfg` blocks, so the compiler couldn't catch a
desynchronized edit. Each ADR names the specific silent-failure mode the
pattern closes. ADR 0005 came out of porting judo's sampling
optimizers: the same silent-failure lens applied to sampling code that
would otherwise be copy-pasted between RRT* and the new planners, closed by
making the shared QMC sequence a compile-time interface both name.

Patterns deliberately **not** adopted (and why) are recorded at the bottom
of the relevant ADR, so the same tempting-but-wrong ideas don't get
relitigated: Template Method for `plan()` (0003), Observer for the
latency/diagnostics recorders (0003).
