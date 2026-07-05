# nanoplan

Ultra minimalist motion planner for car-like vehicles, written in Rust.

Eight planners share one kinematic model, one closed-loop simulator, and one
set of nuPlan-derived quality metrics, so they can be scrubbed side by side
in an interactive [Bevy](https://bevy.org) viewer or swept over hundreds of
scenarios from the command line.

**Try it live**: https://bengravell.github.io/nanoplan/ (builds automatically
from `main`; see [Web deploy](docs/USAGE.md#web-deploy)).

## Start here

| I want to... | Read |
|---|---|
| Install the toolchain and get a build running | [docs/SETUP.md](docs/SETUP.md) |
| Run the viewer, the batch tool, or export real nuPlan scenarios | [docs/USAGE.md](docs/USAGE.md) |
| Understand or extend a specific part of the code | the component READMEs below |
| Know *why* a core abstraction is shaped the way it is | [docs/adr/](docs/adr/) |

## Architecture

```
src/
├── planning/     Planner trait + Context + one directory per planner
├── simulation/   kinematic model, closed-loop Simulator, simulate()/Rollout
├── metrics/      nuPlan closed-loop quality metrics, one directory per metric
├── scenarios/    Scenario/Actor/Path types, JSON loading, synthetic generation
└── tuning/       MaxEnt IRL cost-weight autotuner over expert nuPlan trajectories
```

Every planner implements the same `Planner` trait and reads the same
`Context` (road centerline, actor states, target speed, horizon). `simulate()`
wires a `Scenario` through a chosen planner and the `Simulator` tick loop,
producing a `Rollout`: the ego and actor traces, the tickwise `Metrics`, and
per-planner `LatencyStats`. The viewer (`src/viewer/`) and the batch runner
(`src/bin/batch.rs`) are both thin consumers of `simulate()` — neither owns
planning, simulation, or scoring logic itself.

| Component | README | What it owns |
|---|---|---|
| Planning | [src/planning/README.md](src/planning/README.md) | The `Planner` trait, `PlannerKind` registry, latency diagnostics, the shared QMC sampler, and the eight planners: strawman, Bezier+IDM, Frenet lattice, PI²-DDP, RRT*, and the judo-derived predictive sampling, CEM, and MPPI |
| Simulation | [src/simulation/README.md](src/simulation/README.md) | `State`/`Control`, the kinematic bicycle-free step, `Simulator`, and `simulate()`/`Rollout` |
| Metrics | [src/metrics/README.md](src/metrics/README.md) | Tickwise nuPlan closed-loop quality metrics, one module per metric, with their aggregation rules |
| Scenarios | [src/scenarios/README.md](src/scenarios/README.md) | `Scenario`/`Actor`/`Waypoint` data model, the Frenet `Path`, JSON loading, trajectory replay, synthetic generation |
| Tuning | [src/tuning/README.md](src/tuning/README.md) | Maximum-entropy IRL autotuning of the shared cost function's soft weights from expert human trajectories (the `tune` binary); collision and off-road stay infinite cost by fiat |

A few more directories hold data rather than code:

- [`scenarios/nuplan/`](scenarios/nuplan/) — reference material vendored from
  [nuplan-devkit](https://github.com/motional/nuplan-devkit) (log schema,
  vehicle parameters, metric definitions). Source of truth for the metric
  thresholds in `src/metrics/`.
- `scenarios/json/` — example scenario JSON files bundled into the viewer
  binary at compile time (see [docs/USAGE.md](docs/USAGE.md#scenario-sources)).
- `scenarios/web/` (not checked in by default) — scenarios for the *web*
  build to fetch at startup instead, combined by `tools/bundle_web_scenarios.py`
  into `scenarios/web_bundle.json` (see
  [docs/USAGE.md](docs/USAGE.md#scenario-sources)).

## Planners at a glance

| Planner | Idea | Detail |
|---|---|---|
| Strawman | Zero acceleration, zero curvature, always | [src/planning/README.md#strawman](src/planning/README.md#strawman) |
| Bezier + IDM | Cubic Bezier back to the lane centerline; Intelligent Driver Model for speed | [src/planning/README.md#bezier--idm](src/planning/README.md#bezier--idm) |
| Frenet lattice | EM/Apollo-style: sample a high-res station×lateral grid, A* over the layered DAG | [src/planning/README.md#frenet-lattice](src/planning/README.md#frenet-lattice) |
| PI²-DDP | Sampling-based DDP (Lefebvre & Crevecoeur), road-informed exploration | [src/planning/README.md#pi2-ddp](src/planning/README.md#pi2-ddp) |
| RRT* | Rapidly-exploring random tree with rewiring; steers between poses with a cubic polynomial via differential flatness | [src/planning/README.md#rrt](src/planning/README.md#rrt) |
| Predictive sampling (judo) | Fixed-sigma sampling around a nominal, take the best rollout | [src/planning/README.md#sampling-mpc-judo](src/planning/README.md#sampling-mpc-judo) |
| CEM (judo) | Cross-entropy method: refit an adaptive per-node Gaussian to the elite rollouts | [src/planning/README.md#sampling-mpc-judo](src/planning/README.md#sampling-mpc-judo) |
| MPPI (judo) | Model Predictive Path Integral: Boltzmann reward-weighted average of the rollouts | [src/planning/README.md#sampling-mpc-judo](src/planning/README.md#sampling-mpc-judo) |

## Provenance

- **PI²-DDP**: Lefebvre & Crevecoeur, "Path Integral Policy Improvement with
  Differential Dynamic Programming."
- **Predictive sampling / CEM / MPPI**: ported from
  [judo](https://github.com/rai-opensource/judo) (Robotics and AI Institute),
  keeping its `Optimizer` sample/update interface and adapting the rollout to
  nanoplan's shared cost, road-model base policy, and QMC sampler — see
  [src/planning/README.md#sampling-mpc-judo](src/planning/README.md#sampling-mpc-judo).
- **Frenet lattice**: the EM-planner family (Apollo, and the lattice planners
  surveyed in the sampling-based-DDP literature).
- **RRT***: Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion
  Planning" (the asymptotically-optimal rewiring step over plain RRT); the
  steering function between poses is a cubic polynomial chosen via
  differential flatness of the unicycle/bicycle model's flat outputs
  `(x, y)`, a standard technique in nonholonomic motion planning.
- **IDM**: the Intelligent Driver Model, standard car-following model.
- **DriveIRL**: Phan-Minh et al., "Driving in Real Life with Inverse
  Reinforcement Learning" ([arXiv:2206.03004](https://arxiv.org/abs/2206.03004))
  — the maximum-entropy IRL recipe (lattice candidates, safety filter, linear
  reward over trajectory features) behind the cost-weight autotuner in
  `src/tuning/`.
- **nuPlan**: scenario schema, vehicle parameters, and closed-loop metric
  definitions vendored from [nuplan-devkit](https://github.com/motional/nuplan-devkit)
  (Apache-2.0) — see [`scenarios/nuplan/README.md`](scenarios/nuplan/README.md).
- **ponytail**: the [ponytail](https://github.com/DietrichGebert/ponytail)
  agent skills (MIT) are vendored in [`.claude/skills/`](.claude/skills/) to
  keep AI-assisted contributions minimal, in line with this project's ethos.

## License

MIT — see [LICENSE](LICENSE).
