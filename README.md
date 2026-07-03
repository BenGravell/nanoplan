# nanoplan

Ultra minimalist motion planner for car-like vehicles, written in Rust.

Four planners share one kinematic model, one closed-loop simulator, and one
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

## Architecture

```
src/
├── planning/     Planner trait + Context + one directory per planner
├── simulation/   kinematic model, closed-loop Simulator, simulate()/Rollout
├── metrics/      nuPlan closed-loop quality metrics, one directory per metric
└── scenarios/    Scenario/Actor/Path types, JSON loading, synthetic generation
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
| Planning | [src/planning/README.md](src/planning/README.md) | The `Planner` trait, `PlannerKind` registry, latency diagnostics, and the four planners: strawman, Bezier+IDM, Frenet lattice, PI²-DDP |
| Simulation | [src/simulation/README.md](src/simulation/README.md) | `State`/`Control`, the kinematic bicycle-free step, `Simulator`, and `simulate()`/`Rollout` |
| Metrics | [src/metrics/README.md](src/metrics/README.md) | Tickwise nuPlan closed-loop quality metrics, one module per metric, with their aggregation rules |
| Scenarios | [src/scenarios/README.md](src/scenarios/README.md) | `Scenario`/`Actor`/`Waypoint` data model, the Frenet `Path`, JSON loading, trajectory replay, synthetic generation |

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
| Frenet lattice | EM/Apollo-style: sample a station×lateral grid, DP over layers | [src/planning/README.md#frenet-lattice](src/planning/README.md#frenet-lattice) |
| PI²-DDP | Sampling-based DDP (Lefebvre & Crevecoeur), road-informed exploration | [src/planning/README.md#pi2-ddp](src/planning/README.md#pi2-ddp) |

## Provenance

- **PI²-DDP**: Lefebvre & Crevecoeur, "Path Integral Policy Improvement with
  Differential Dynamic Programming."
- **Frenet lattice**: the EM-planner family (Apollo, and the lattice planners
  surveyed in the sampling-based-DDP literature).
- **IDM**: the Intelligent Driver Model, standard car-following model.
- **nuPlan**: scenario schema, vehicle parameters, and closed-loop metric
  definitions vendored from [nuplan-devkit](https://github.com/motional/nuplan-devkit)
  (Apache-2.0) — see [`scenarios/nuplan/README.md`](scenarios/nuplan/README.md).
- **ponytail**: the [ponytail](https://github.com/DietrichGebert/ponytail)
  agent skills (MIT) are vendored in [`.claude/skills/`](.claude/skills/) to
  keep AI-assisted contributions minimal, in line with this project's ethos.

## License

MIT — see [LICENSE](LICENSE).
