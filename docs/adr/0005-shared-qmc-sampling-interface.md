# 0005 — Share RRT*'s road-frame QMC sampling behind a compile-time interface

- **Status**: Accepted
- **Pattern**: Strategy over a shared interface, dependency inversion
  ([refactoring.guru](https://refactoring.guru/design-patterns/strategy))
- **Affects**: `src/planning/sampling.rs`, `src/planning/rrt_star/mod.rs`,
  `src/planning/sampling_mpc/`

## Context

Porting judo's three sampling optimizers (predictive sampling, CEM, MPPI)
into nanoplan raised a sampling question. judo samples control-knot noise
with a pseudo-random `np.random.randn`. RRT*, by contrast, deliberately
samples from a *quasi*-Monte-Carlo low-discrepancy sequence (a Halton grid
built on `van_der_corput`), because at the few-dozen-samples-per-tick budget
a real-time planner can afford, a low-discrepancy set covers the domain more
evenly than an RNG, and — being a pure function of the sample index — it
makes `plan()` a pure function of the ego state. That purity is load-bearing:
it is what lets a closed-loop rollout, stitched from one control per tick
across many replans, inherit any single plan's safety margin instead of
chattering between differently-shaped detours (see RRT*'s
`swerves_around_stopped_obstacle` and the warm-start notes).

We wanted the judo planners to inherit that same property, which meant they
had to draw from the *same* QMC construction RRT* uses — not a second copy of
`van_der_corput` that could drift. The `van_der_corput` function lived
private inside `rrt_star/mod.rs`.

## Decision

Lift the sampling into one module, `planning/sampling.rs`, and express the
share as a **compile-time interface** rather than a shared free function two
callers happen to call:

```rust
pub(crate) trait QuasiMonteCarlo {
    fn coordinate(index: usize, dim: usize) -> f64; // radical inverse in the dim-th prime
}
pub(crate) struct Halton; // the one and only implementor
impl QuasiMonteCarlo for Halton { /* van_der_corput(index, PRIMES[dim]) */ }
```

The shared entry points are generic over `Q: QuasiMonteCarlo`, so the
interface appears literally at every call site:

- `road_frame_samples::<Q>(...)` — the hybrid road-geometry grid + QMC pass
  over the `(station, lateral)` Frenet box. RRT* calls it with `Halton`.
- `qmc_normals::<Q>(...)` — Halton coordinates pushed through an
  inverse-normal-CDF (`inv_normal_cdf`), i.e. *low-discrepancy Gaussian*
  noise. The judo optimizers call it with `Halton` for their knot noise,
  replacing judo's `np.random.randn`.

With one trait and one implementor, "every sampling planner draws from the
same QMC sequence" is checked by the type system: a planner wanting a
different sequence would have to name a different type, a compile error at
the call, not a silent divergence between two hand-maintained radical-inverse
loops.

## Consequences

**Good**

- Sampling parity is structural. RRT* and the judo optimizers cannot drift
  apart without a type error. The numeric contract (that lifting RRT*'s old
  inline loop changed no sample) is additionally pinned by
  `rrt_targets_match_shared_sampler`.
- The judo planners inherit RRT*'s determinism: they are pure functions of
  the ego state (`plan_is_a_pure_function_of_state`), with no `Rng`. PI²-DDP
  remains the lone RNG-based planner, by its own design.
- `van_der_corput` has one home instead of being copy-pasted into a fourth
  sampling planner.

**Costs / trade-offs**

- `QuasiMonteCarlo::coordinate` is an associated function (no `&self`): a
  Halton coordinate is a pure function of `(index, dim)` with no state, so
  the trait carries no data. This is deliberate but means the trait is used
  purely as a compile-time seam, not for runtime polymorphism (there is only
  ever one implementor).
- Generic-over-`Q` functions with a single instantiation are, strictly, more
  ceremony than a plain shared `fn`. The ceremony *is* the point here — it
  puts the interface in the signature so the parity is enforced where the
  sampling happens.

## Alternatives considered

- **A shared free `van_der_corput` fn, called by both.** Rejected as the
  weaker share: it makes the *primitive* common but not the *interface*, so
  nothing stops a caller from re-deriving its own sequence around it. The
  brief specifically asked for parity "at a compile-time interface level."
- **Give the judo planners their own `Rng`, like PI²-DDP.** Rejected: it
  would forfeit the state-purity property and re-introduce the tick-to-tick
  chatter QMC + warm start exist to prevent.
- **A full runtime-polymorphic sampler object** (`dyn QuasiMonteCarlo`).
  Rejected: there is exactly one sequence in this codebase; boxing it would
  add indirection and an allocation for no gain. The zero-sized `Halton` with
  associated functions is the right weight.

## Verification

Full test suite green, including `rrt_targets_match_shared_sampler` (numeric
parity), the `sampling.rs` unit tests (radical inverse, inverse-normal-CDF,
QMC determinism), and each judo planner's closed-loop tests
(centerline tracking, obstacle avoidance, state-purity, diagnostics). RRT*'s
own behavior is unchanged — its batch scores are byte-identical before/after
the lift.
