# 0003 — Collapse `PlannerKind`'s parallel `match`es into a `PlannerSpec` registry

- **Status**: Accepted
- **Pattern**: Factory Method, table-driven
  ([refactoring.guru](https://refactoring.guru/design-patterns/factory-method))
- **Affects**: `src/planning/mod.rs`

## Context

`PlannerKind` is the selection/comparison seam: a `Copy` enum used as the key
that the viewer dropdown, the batch runner, and the `RolloutCache` hash map
all pivot on. Keeping it an enum is right.

The problem was that everything *else* about a planner was spread across three
separate `match self` blocks plus a standalone allowlist:

- `name(self)` — a `match` returning the display string.
- `build(self)` — a `match` returning `Box<dyn Planner>`.
- `has_diagnostics(self)` — a `matches!(self, Lattice | Pi2Ddp | RrtStar)`
  allowlist.

Adding a planner meant editing all of them. The `has_diagnostics` allowlist
was the real trap: it's the one piece the compiler does **not** force you to
update. Add a sixth planner that records diagnostics and forget the
`matches!` arm, and there is no error — the viewer's diagnostic-overlay
checkboxes just silently never appear for it. Since "add a planner and
compare it" is the project's core workflow, a silent gap on exactly that path
is the wrong failure mode.

## Decision

Keep the `PlannerKind` enum as the key, but move its metadata into a single
registry table indexed by the enum's discriminant. The `build` field is the
Factory Method slot — a constructor per row:

```rust
pub struct PlannerSpec {
    pub kind: PlannerKind,
    pub name: &'static str,
    pub build: fn() -> Box<dyn Planner>,
    pub has_diagnostics: bool,
}

const SPECS: [PlannerSpec; 5] = [ /* one complete row per planner */ ];

impl PlannerKind {
    pub fn spec(self) -> &'static PlannerSpec { &SPECS[self as usize] }
    pub fn name(self) -> &'static str { self.spec().name }
    pub fn build(self) -> Box<dyn Planner> { (self.spec().build)() }
    pub fn has_diagnostics(self) -> bool { self.spec().has_diagnostics }
}
```

`spec()` indexes `SPECS` by `self as usize`, so row order must match the
enum's variant order. A `specs_align_with_kinds` unit test pins that down by
asserting `kind.spec().kind == kind` for every kind — a misordered or missing
row fails the build.

## Consequences

**Good**

- One `match`/index instead of three; a new planner is one enum variant plus
  one complete `SPECS` row (name, constructor, capability flags).
- It is now structurally impossible to add the row while forgetting the
  diagnostics flag or the display name — the struct requires every field, and
  the alignment test guards ordering. The silent-gap failure mode is gone.

**Costs / trade-offs**

- Row order is coupled to enum discriminant order. This is the one thing that
  could still go wrong, and it's exactly what the alignment test exists to
  catch (turning a silent bug into a failing test).
- `build` is a `fn()` pointer, so the constructors must be nullary. Every
  planner is either a unit struct or has a `Default`, so `|| Box::new(...)`
  closures cover all cases; a planner needing constructor arguments would
  need a richer slot (not a concern today).

## Alternatives considered

- **Keep the three matches.** Rejected: that's the status quo whose
  `has_diagnostics` allowlist silently rots.
- **Derive metadata via a macro** (e.g. `strum`-style). Rejected: a new
  dependency and macro machinery for five planners, against the project's
  minimalist ethos. A plain `const` array is more legible and has no build
  cost.
- **Template Method on the `Planner` trait** (a `route → optimize → extract`
  skeleton method). Considered and rejected — recorded here so it isn't
  retried: each planner's intermediate types differ, and the trait must stay
  `dyn`-compatible for `Box<dyn Planner>`, which associated types would
  break. The shared phase *names* are a documented latency-seam convention
  (`planning/latency.rs`), which is the right weight for what's actually
  shared.

## Related non-decisions

- **Observer for `Latency`/`Diagnostics`.** These are already a lean
  optional-recorder pair reachable from `Context`, costing nothing when
  absent. Formalizing them behind a subscriber interface would add
  indirection and save nothing, so it was not done.

## Verification

Full test suite green, including the new `specs_align_with_kinds` test; batch
CSV byte-identical before/after.
