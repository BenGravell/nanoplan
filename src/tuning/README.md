# `tuning`

Autotuning of the [shared cost function's](../planning/README.md#the-shared-cost-function)
soft weights from expert human demonstrations, via **maximum-entropy inverse
reinforcement learning** — the [DriveIRL](https://arxiv.org/abs/2206.03004)
recipe (Phan-Minh et al., *"Driving in Real Life with Inverse Reinforcement
Learning"*) at nanoplan scale.

```
tuning/
└── mod.rs   candidate generator, featurization, MaxEnt IRL fit, tune()/TuneResult
```

The CLI is `src/bin/tune.rs`; usage lives in
[docs/USAGE.md#autotuning-the-cost-weights](../../docs/USAGE.md#autotuning-the-cost-weights).

## The premise

nuPlan logs contain the trajectory the human driver actually drove — the
`expert` field of a [`Scenario`](../scenarios/README.md#field-reference),
exported by `tools/export_nuplan_scenarios.py`. Assume that driver was
optimizing a well-tuned cost of exactly the linear form every search-based
planner here shares: `WEIGHTS · features` per sampled point
([`cost.rs`](../planning/cost.rs)). Then the hand-set `WEIGHTS` are just a
guess at something the data can measure, and IRL is the measurement: find the
weights under which the human's choices look best.

## The model

Per scenario, a candidate set of trajectories is generated (below) and the
expert is modeled as choosing among them with Boltzmann/maximum-entropy
probability

```
P(τ) ∝ exp(−w · φ(τ))        φ(τ) = Σ_ticks features(sample_t)
```

Fitting `w` is maximum likelihood: minimize the negative log-likelihood of
the expert choice per scenario. This is convex, and its gradient is the
classic MaxEnt IRL feature-matching residual `φ(expert) − E_P[φ]` — at the
optimum, the model's expected features match the expert's.

**The hard rules are not learned.** Collision and leaving the drivable area
are infinite cost / zero score by fiat, hardcoded in `cost::features`
(it returns `None`); no amount of data adjusts them. In the IRL fit this
shows up twice:

- a **candidate** that hard-violates is dropped from the distribution's
  support entirely (DriveIRL's safety filter plays the same role), and
- a **scenario whose expert hard-violates under the model** is skipped — a
  demonstration the model already calls infinitely bad carries no usable
  preference signal about the *soft* weights.

## The candidate set

A deterministic grid of Frenet maneuvers from the ego's starting state,
mirroring DriveIRL's lattice generator: target lateral offsets (±3 m) ×
maneuver durations (3/6 s) × longitudinal profiles, each rolled out over the
shared `PLANNING_HORIZON_S` at the simulator's 0.1 s tick. Longitudinal
profiles are **exponential approaches to a grid of settled cruise speeds**
(relative to the scenario's target speed), not constant accelerations —
human speed profiles settle, and against candidates that never do, a real
expert is cheaper than everything by a wide margin, the softmax saturates,
and the likelihood gradient vanishes.

The **expert itself is always candidate 0**, resampled to the same tick
grid, so the demonstration is in the distribution's support. Every
trajectory, expert included, goes through the *same* featurization code
path: project onto the centerline for lateral offset and heading error,
Menger curvature off consecutive points (the same estimate the
[lattice planner](../planning/README.md#frenet-lattice) uses), speed
difference quotients for accel, then sum `cost::features` over the ticks.

## The fit

Projected gradient descent (weights clamped ≥ 0 — they're penalties) with a
backtracking line search, on the mean per-scenario NLL plus a small L2 pull
toward the *current* hand weights — a Gaussian prior centered on them, so a
feature the data expresses no preference about (e.g. `road_edge` when no
expert or candidate ever nears the edge) keeps its hand-tuned value instead
of drifting to zero. Features are internally rescaled to unit mean magnitude
(raw magnitudes span several orders) and the learned weights mapped back to
raw units at the end. Everything is deterministic: same scenarios in, same
weights out.

`TuneResult` reports, besides the learned weights: the NLL before/after, and
in how many scenarios the expert is the *minimum-cost* candidate
before/after — the interpretable "would a planner using this cost have
picked what the human did" number. `TuneResult::report()` ends with the
exact `WEIGHTS` line to paste into `src/planning/cost.rs`; the weights stay
a compile-time constant there (every planner keeps its zero-plumbing read of
one `const`), so "applying" a tune is a one-line, reviewable diff.

## Limitations worth knowing

- **Weights only.** The feature *set* is fixed; IRL here re-balances the
  existing terms, it doesn't invent new ones. If the expert cares about
  something no feature measures (e.g. centerline-hugging — deliberately not
  a shared-cost term), the fit can't express it.
- **Model-consistent, not reality-consistent.** Features price actors via
  the same `metrics::predict` model the planners use (lane-following where an
  actor drives the route, constant-velocity otherwise). An expert who avoided
  a car that *actually* swerved may look unmotivated (or hard-violating)
  under the prediction; such scenarios are skipped rather than half-learned.
- **The candidate grid is coarse.** MaxEnt IRL only learns from cost
  *contrasts* among candidates; preferences the grid can't express (e.g.
  double lane changes) generate no signal. Enrich the grids in `mod.rs` if
  your logs exercise them.
- Scenarios whose expert is shorter than the planning horizon are skipped —
  export with `--horizon` ≥ 10 s (the default 20 s is fine).

## Testing

`recovers_a_shifted_preference_from_demonstrations` builds synthetic
demonstrations chosen as min-cost under ground-truth weights with a 10×
heading-error preference, and checks the fit moves decisively toward the
truth (NLL near its floor, heading weight more than doubled).
`hard_violating_expert_is_skipped_and_weights_stay_put` pins the hardcoded
infinite-cost rule: an expert driven through a parked car is skipped and the
weights come back unchanged, as do scenarios with no or too-short experts
(`scenarios_without_or_with_short_experts_are_skipped`).
