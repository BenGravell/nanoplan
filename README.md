# nanoplan

Ultra minimalist motion planner for car-like vehicles, written in Rust.

Eleven planners share one kinematic model, one closed-loop simulator, and one
set of nuPlan-derived quality metrics, so they can be scrubbed side by side
in an interactive [Bevy](https://bevy.org) viewer or swept over hundreds of
scenarios from the command line.

**Try it live**: https://bengravell.github.io/nanoplan/ (builds automatically
from `main`; see [Web deploy](docs/USAGE.md#web-deploy)).

## Start here

| I want to... | Read |
|---|---|
| Install the toolchain and get a build running | [docs/SETUP.md](docs/SETUP.md) |
| Run the viewer, the batch tool, or convert CommonRoad scenarios | [docs/USAGE.md](docs/USAGE.md) |
| Understand or extend a specific part of the code | the component READMEs below |
| Know *why* a core abstraction is shaped the way it is | [docs/adr/](docs/adr/) |

## Architecture

```
src/
├── planning/     Planner trait + Context + one directory per planner
├── simulation/   kinematic model, closed-loop Simulator, simulate()/Rollout
├── metrics/      nuPlan closed-loop quality metrics, one directory per metric
├── scenarios/    Scenario/Actor/Path types, JSON loading, synthetic generation
├── world/        infinite chunked procedural street world, mixed IDM traffic, realtime LiveWorld
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
| Planning | [src/planning/README.md](src/planning/README.md) | The `Planner` trait, `PlannerKind` registry, latency diagnostics, the shared QMC sampler, and the eleven planners: strawman, Bezier+IDM, Frenet lattice, PI²-DDP, RRT*, the judo-derived predictive sampling, CEM, and MPPI, and the treetop-derived RRT, iLQR, and treetop (RRT+iLQR) |
| Simulation | [src/simulation/README.md](src/simulation/README.md) | `State`/`Control`, the kinematic bicycle-free step, `Simulator`, and `simulate()`/`Rollout` |
| Metrics | [src/metrics/README.md](src/metrics/README.md) | Tickwise nuPlan closed-loop quality metrics, one module per metric, with their aggregation rules |
| Scenarios | [src/scenarios/README.md](src/scenarios/README.md) | `Scenario`/`Actor`/`Waypoint` data model, the Frenet `Path`, JSON loading, trajectory replay, synthetic generation |
| World | [src/world/README.md](src/world/README.md) | The viewer's realtime "open world" mode: an infinite procedural street network (a pure function of the seed, chunked Minecraft-style around the ego), click-to-place goal routing with junction lane connectors, mixed IDM traffic (cars, trucks, bikes, pedestrians), and the `LiveWorld` tick loop that replans and steps the ego continuously (judo/treetop style) |
| Tuning | [src/tuning/README.md](src/tuning/README.md) | Maximum-entropy IRL autotuning of the shared cost function's soft weights from expert human trajectories (the `tune` binary); collision and off-road stay infinite cost by fiat |

A few more directories hold data rather than code:

- [`scenarios/commonroad/`](scenarios/commonroad/) — the scenario corpus
  this repo ships, in the open [CommonRoad](https://commonroad.in.tum.de)
  2020a XML format (original to this repo, MIT — redistributable, unlike
  nuPlan data). `tools/export_commonroad_scenarios.py` converts these (or
  any real CommonRoad scenario) into nanoplan's JSON.
- [`scenarios/nuplan/`](scenarios/nuplan/) — reference material vendored from
  [nuplan-devkit](https://github.com/motional/nuplan-devkit) (log schema,
  vehicle parameters, metric definitions). Source of truth for the metric
  thresholds in `src/metrics/`; scenario data exported from real nuPlan logs
  stays local (tuning only — see [docs/USAGE.md](docs/USAGE.md#exporting-real-nuplan-scenarios-local-only)).
- `scenarios/json/` — bundled conversions of two CommonRoad scenarios,
  compiled into the viewer binary (see [docs/USAGE.md](docs/USAGE.md#scenario-sources)).
- `scenarios/web/` (not checked in) — staging directory for the *web*
  build's scenario set, combined by `tools/bundle_web_scenarios.py`
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
| RRT (treetop) | Time-layered motion sampling tree; steers in action space via cubic flat outputs, projected onto the actuation limits | [src/planning/README.md#rrt-treetop-tree](src/planning/README.md#rrt-treetop-tree) |
| iLQR (treetop) | Iterative LQR trajectory optimization, with every cost and dynamics derivative taken by finite differences | [src/planning/README.md#ilqr-treetop-finite-differences](src/planning/README.md#ilqr-treetop-finite-differences) |
| treetop (RRT+iLQR) | The tree seeds iLQR with collision-free path candidates; the optimized solution warm-starts the tree next tick | [src/planning/README.md#treetop-rrt--ilqr](src/planning/README.md#treetop-rrt--ilqr) |

## Provenance

- **PI²-DDP**: Lefebvre & Crevecoeur, "Path Integral Policy Improvement with
  Differential Dynamic Programming."
- **Predictive sampling / CEM / MPPI**: ported from
  [judo](https://github.com/rai-opensource/judo) (Robotics and AI Institute),
  keeping its `Optimizer` sample/update interface and adapting the rollout to
  nanoplan's shared cost, road-model base policy, and QMC sampler — see
  [src/planning/README.md#sampling-mpc-judo](src/planning/README.md#sampling-mpc-judo).
- **RRT / iLQR / treetop**: ported from
  [treetop](https://github.com/BenGravell/treetop), a tree-initialized
  trajectory-optimizing planner (ego motion sampling tree + iLQR), split
  into its two halves plus the coordinator so each is comparable on its
  own; the iLQR port takes all cost and dynamics derivatives by finite
  differences since nanoplan deliberately provides no analytic ones — see
  [src/planning/README.md#treetop-rrt--ilqr](src/planning/README.md#treetop-rrt--ilqr).
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
- **CommonRoad**: the scenario corpus in [`scenarios/commonroad/`](scenarios/commonroad/)
  uses the [CommonRoad](https://commonroad.in.tum.de) 2020a XML format
  (Althoff, Koschi & Manzinger, "CommonRoad: Composable benchmarks for
  motion planning on roads", IV 2017) so scenarios interoperate with the
  CommonRoad ecosystem; the files themselves are original to this repo.
- **nuPlan**: scenario schema, vehicle parameters, and closed-loop metric
  definitions vendored from [nuplan-devkit](https://github.com/motional/nuplan-devkit)
  (Apache-2.0) — see [`scenarios/nuplan/README.md`](scenarios/nuplan/README.md).
- **ponytail**: the [ponytail](https://github.com/DietrichGebert/ponytail)
  agent skills (MIT) are vendored in [`.claude/skills/`](.claude/skills/) to
  keep AI-assisted contributions minimal, in line with this project's ethos.

## License

MIT — see [LICENSE](LICENSE).
