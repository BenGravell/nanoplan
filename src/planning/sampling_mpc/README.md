# Sampling MPC (judo)

`sampling_mpc/` — `SamplingPlanner<PredictiveSampling>`, `SamplingPlanner<Cem>`, `SamplingPlanner<Mppi>`

A port of the three sampling-based optimizers from [**judo**](https://github.com/rai-opensource/judo)
(`judo/optimizers/{ps,cem,mppi}.py`), kept structurally faithful to judo's own abstraction and then fitted into the
nanoplan framework.
The layout mirrors judo's:

```
sampling_mpc/
├── mod.rs   Optimizer trait + OptimizerConfig (judo base.py), SamplingPlanner<O> driver
├── ps.rs    predictive sampling (judo ps.py)
├── cem.rs   cross-entropy method (judo cem.py)
└── mppi.rs  MPPI (judo mppi.py)
```

**The judo interface, verbatim.** An `Optimizer` is exactly judo's two-method strategy over control *knots* —
`num_nodes` control points of dimension `nu = 2` (`[acceleration, curvature]`):

```rust
fn sample_control_knots(&mut self, nominal: &[Knot], sample_base: usize) -> Vec<Vec<Knot>>;
fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot>;
```

The three optimizers are *only* these two methods, matching judo line for line:

- **Predictive sampling** (`ps.rs`): `sample` = nominal plus fixed-sigma noise (first rollout the un-noised nominal);
  `update` = the single best-scoring sample (`argmax` reward).
- **CEM** (`cem.rs`): `sample` = nominal plus an *adaptive per-node* sigma; `update` = the elite (top-`num_elites`) mean,
  with sigma refit to the elite std (clipped to `[sigma_min, sigma_max]`), so the distribution contracts around whatever
  keeps scoring well.
- **MPPI** (`mppi.rs`): `sample` like predictive sampling; `update` = a Boltzmann reward-weighted average of *all*
  rollouts, `exp(-(cost - min)/temperature)` normalized.
  The temperature is interpreted relative to the rollout cost *spread* (the same min/max normalization PI²-DDP applies to
  its eq.-12 weighting), so it stays a scale-free knob rather than tied to a rollout's absolute cost magnitude.

**Everything else is `SamplingPlanner<O>`, the judo→nanoplan adapter.** judo keeps rollout and reward outside the
optimizer; here the generic driver supplies them the nanoplan way, so each optimizer stays a pure strategy:

- **Knots are deviations from a road-model base policy.** The key adaptation.
  judo's knots *are* the raw controls, applied open-loop over the horizon — fine for its short-horizon,
  feedback-stabilized tasks, but a car's lateral dynamics integrate curvature twice, so raw open-loop knots diverge metres
  off-road over a 10 s horizon and every candidate scores as garbage (the symptom that drove this design: a nominal
  rollout ending ~20 m off-lane).
  Instead each interpolated knot is a *deviation* added to a **critically-damped PD lane-keeping + speed-hold base
  policy** evaluated on the current rollout state — genuine feedback, so every rollout stays on the road and the QMC
  explores real maneuvers (an obstacle swerve) instead of drift.
  This mirrors PI²-DDP rolling out with its feedback gains rather than raw nominal controls, and *is* the "hybrid road
  model" half of the sampling.
  The nominal starts at zero deviation (the judo-typical zero nominal, here meaning "just the base policy").
- **Knots → controls → rollout.** The `num_nodes` deviation knots are spread over the `PLANNING_HORIZON_S` horizon and
  linearly interpolated (`control_at`), added to the base policy, clamped to physical actuation limits, and rolled out
  through the shared kinematic `step`.
- **The shared metric objective.** Each rolled-out state is priced by `HardConstraints::point_cost`, with a hard violation
  made finite (`constraints::HARD_VIOLATION_PENALTY`) so MPPI's and CEM's reward aggregation can't divide by an infinity —
  exactly PI²-DDP's reasoning.
  No planner-local outcome terms are added.
- **The shared QMC sampler.** The knot noise is drawn from [`sampling::qmc_normals`](../README.md#shared-qmc-sampling),
  the *same* low-discrepancy sequence RRT\* samples targets from — so these planners are deterministic pure functions of
  the ego state (`*_is_a_pure_function_of_state`), unlike judo's pseudo-random `np.random.randn`.
- **Warm start.** The winning deviations are carried to the next tick when the ego followed the plan, so each 0.1 s replan
  refines the last.

Each `plan()` runs `iterations` (default 4, echoing PI²-DDP's `GENERATIONS`) sample→rollout→update passes — a nanoplan
adaptation of judo's controller loop, which runs one optimizer step per control cycle.

**Seams**: `route` (build the `Path`), `warm_start` (reuse or road-informed re-init), `optimize` (the
sample/rollout/update iterations) with `cost` (the shared metric objective, once per rolled-out state) nested inside,
`extract` (sample the winning nominal into `Vec<Control>`).

**Diagnostics**: the final iteration's `num_rollouts` sampled state sequences, each recorded both as a `trajectory` and
flattened into `points`, mirroring PI²-DDP.
