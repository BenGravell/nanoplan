# PI2-DDP

`pi2ddp/mod.rs` — `Pi2DdpPlanner`

Sampling-based Differential Dynamic Programming, implementing Algorithm 2 of
Lefebvre & Crevecoeur, *"Path Integral Policy Improvement with Differential
Dynamic Programming"* (PI²-DDP). `HORIZON = 100` ticks, i.e.
`PLANNING_HORIZON_S = 10` s at the simulator's 0.1 s tick rate. Each `plan()`
call runs `GENERATIONS = 4` generations; each generation samples
`ROLLOUTS = 32` perturbed control
sequences around a nominal trajectory (with feedback), weights them by
exponentiated normalized cost-to-go (paper eq. 12), and extracts a DDP-style
update from the reward-weighted rollout statistics:

- feedforward `k = Σₖ pₖ(δu − Kδx)`
- feedback `K = Σᵤₓ Σₓₓ⁺`
- perturbation covariance `Σᵤ = Σᵤᵤ − ΣᵤₓΣₓₓ⁺Σₓᵤ + λ_exp R⁻¹` (eq. 37)

with the eq. 38 trust-region rule on the exploration magnitude `λ_exp` (the
paper's "adaptive v2" variant: a generation that makes the noise-free cost
worse is discarded outright rather than blended in).

**Road-model-informed sampling** (the point of the exercise): the initial
nominal control sequence isn't zero, it's a pure-pursuit tracker toward the
centerline plus proportional speed hold (`init_policy`); the initial
curvature exploration variance `σ_κ` is sized so sampled trajectories span
roughly the lane half-width (`LANE_HALF_M = 1.75` m) by the preview
distance, rather than an arbitrary constant. The running cost prices the
rolled-out state against the [shared metric objective](../../metrics/README.md#the-shared-metric-objective)
— `State` is just `(x, y, yaw, speed)`, while `u` is direct
acceleration/curvature. Unlike
the lattice and RRT*, which reject a colliding or off-road
candidate outright, PI²-DDP has no such hard accept/reject step in its
continuous search, so violations use the finite depth-scaled escape penalty.

The policy **warm-starts** across ticks: if the ego ended up close to where
the previous plan predicted (`expected_next`, within 1 m), the policy shifts
one step and continues refining; otherwise it re-initializes from scratch.

**Stability guards**, added after closed-loop testing surfaced real
failures (see the `stays_finite_and_safe_over_long_rollout` regression
test):

- `clamp_control` bounds direct acceleration and curvature commands, including
  the speed-dependent lateral-acceleration limit — near-stationary rollouts have
  little state diversity, which makes the `Σₓₓ` inverse in the gain computation
  nearly singular and can otherwise blow the policy up.
- A PSD guard on the perturbation covariance: if `Σᵤ`'s Schur complement
  loses positive-definiteness (noisy statistics), it's replaced with the
  road-informed prior scaled by `λ_exp` rather than propagated.

**Seams**: `route` (build the `Path`), `warm_start` (custom — includes the
occasional full road-informed re-init when the shift check misses),
`rollouts` (custom — the `ROLLOUTS × HORIZON` sampling loop, by far the most
expensive part: typically ~85-90% of `total` time) with `cost` (the shared
cost function, called once per rollout per timestep) nested inside it,
`policy_update` (custom — the per-timestep DDP gradient extraction).

**Diagnostics**: the final generation's `ROLLOUTS` sampled state sequences —
each one recorded both as a `trajectory` (the polyline through its `HORIZON`
states) and flattened into `points` (every state along every rollout), so
the overlay can show the sampling distribution's spread either as paths or
as a density of points. Only the last generation is recorded; earlier
generations are refinement steps toward it, not additional information.
