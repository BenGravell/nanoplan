# Usage

See [SETUP.md](SETUP.md) first if you haven't built the project yet.

## The viewer

```sh
cargo run
```

opens an interactive window with a control panel in the top-left corner and
a top-down view of the scenario. The panel's two tabs switch between the
viewer's modes:

- **scenarios** — scrub through precomputed 20-second rollouts of the
  bundled/loaded scenarios (everything below in [Controls](#controls));
- **open world** — a realtime interactive sandbox: click anywhere on a
  procedurally generated street map and the ego routes there, replanning
  every tick while IDM traffic wanders around it (see
  [Open world mode](#open-world-mode)).

### Controls

| Control | Effect |
|---|---|
| **scenario** dropdown | Selects which `Scenario` to simulate. See [Scenario sources](#scenario-sources) below for what's in the list. |
| **planner** dropdown | Selects which planner drives the ego vehicle. If this (scenario, planner) combo hasn't been simulated yet, a "Simulate" button (then a progress bar while it runs) appears in place of the metrics table below — simulating doesn't block the window, so an expensive planner like PI²-DDP or RRT* never freezes the UI. Re-selecting a combo already simulated is instant. |
| **time [s]** slider | Scrubs through the 20-second rollout. The camera follows the ego; the metrics and latency tables below update to the scrubbed tick. |
| **future preview [s]** slider | When above zero, frames the screen in an accent color and overlays: the ego's plan from the scrubbed state (replanned live, pink line + ghost car at the horizon) and every actor's constant-velocity prediction (dimmer pink). Use this to see a planner's plan diverge from reality — e.g. select "curving lead" or the bundled "ZAM_CutIn-1_1_T-1" scenario and watch the prediction miss. |
| **diagnostic points** / **diagnostic trajectories** checkboxes | Only shown for planners with search geometry to show (all but the strawman and Bezier+IDM). Overlays what the planner considered during the future-preview replan above: for the lattice, its (station, lateral) sample grid (yellow points) and every DP edge it costed (cyan trajectories); for PI²-DDP, its sampled rollout states (points) and rollouts (trajectories); for RRT*, every tree node (points) and the sampled polyline of the edge that added it (trajectories). Needs **future preview [s]** above zero — that replan is what gets recorded. |

### Reading the metrics table

Two columns per metric:

- **@t** — the metric's score at the exact scrubbed tick. This is where you
  see the moment a collision happens, or comfort briefly drops during a hard
  brake.
- **agg** — the metric's aggregated value over the whole scenario (worst-case
  for event-driven metrics like collisions, average for smooth ones like
  comfort — see [src/metrics/README.md](../src/metrics/README.md) for the
  full rule).

**closed-loop score** at the bottom is the nuPlan-style composite: the
product of the multiplier metrics (collisions, drivable area, driving
direction, making progress) times the weighted average of TTC, progress,
speed limit, and comfort. It's `0` the instant any multiplier metric fails
anywhere in the scenario.

### Reading the latency table

One row per timing *seam* the running planner recorded (see
[src/planning/README.md#latency-diagnostics](../src/planning/README.md#latency-diagnostics)
for what a seam is). `mean [ms]` and `max [ms]` are computed across every
`plan()` call in the 20-second rollout (200 calls at the default 10 Hz).
`total` is present for every planner; the rest vary — the strawman planner
has only `total`, PI²-DDP shows `route`, `warm_start`, `rollouts`, and
`policy_update`.

### Open world mode

The **open world** tab runs the planner and simulator continuously in
realtime (judo/treetop style) instead of scrubbing a precomputed rollout:
every 100 ms tick, the traffic actors step, the selected planner replans
from the live ego state, and the first planned control is applied. The pink
line is the plan *that was just executed* — watch it re-shape as traffic
moves.

- **Click** anywhere on the map to place the goal (green rings). The route
  (dim green line) snaps to the street network, prefers not to U-turn, and
  can be re-placed at any time, including mid-drive. The ego tapers its
  speed to arrive stopped; with no goal it brakes and waits (faint ring).
- **Scroll** to zoom.
- **planner** switches the live planner mid-drive (fresh instance). It
  defaults to Bezier + IDM; the sampling MPC planners (predictive sampling
  / CEM / MPPI) are the most fun to watch replan. A planner slower than
  realtime (PI²-DDP) doesn't freeze the UI — the world just advances slower
  than wall clock (see **plan latency** in the readout).
- **cruise speed** sets the route's target speed, live.
- **pause** freezes the world; **new map** regenerates the street network
  with a fresh seed; **clear goal** drops the current goal.

The map is an *infinite* jittered grid of two-way streets — multi-lane
arterials, one-lane locals, Perlin-noise urbanization driving density and
lane counts, and junction furniture (left-turn pockets that flare the
approach, occasional slip-lane right turns, crosswalks where pedestrians
sometimes cross) — generated as a pure function of the seed and
materialized in Minecraft-style chunks around the ego, so you can drive as
far as you like. Traffic is mixed (cars, trucks, curb-hugging bikes, sidewalk
pedestrians), spawned per chunk and despawned with hysteresis as chunks
unload: actors wander the network in their own right-hand lane, hold speed
with the same IDM the Bezier planner uses, queue behind slower traffic
(including the ego), and brake for anything crossing their bumper. See
[src/world/README.md](../src/world/README.md) for how it all works.

### Scenario sources

The dropdown offers scenarios from up to five sources, concatenated in this
order:

1. **Six built-in scenarios** hardcoded in `src/viewer/scenarios.rs`: a straight road
   with an offset start, an s-curve road, a stopped lead, oncoming traffic,
   crossing traffic, and a curving lead (chosen to make constant-velocity
   prediction error visible).
2. **Bundled JSON scenarios** from [`scenarios/json/`](../scenarios/json/),
   embedded into the binary at compile time with `include_str!` — they ship
   in the web build too, with no filesystem access needed. Currently a
   braking lead (`ZAM_BrakingLead-1_1_T-1`) and a lane cut-in
   (`ZAM_CutIn-1_1_T-1`), converted from the CommonRoad corpus in
   [`scenarios/commonroad/`](../scenarios/commonroad/) and driven by logged
   trajectory replay
   (see [src/scenarios/README.md#trajectory-replay](../src/scenarios/README.md#trajectory-replay)).
3. **Desktop only**: any files or directories passed as command-line
   arguments, each loaded with `nanoplan::scenarios::load_path`:

   ```sh
   cargo run -- path/to/exported_scenarios/    # a directory of *.json files
   cargo run -- path/to/one_scenario.json      # a single scenario file
   ```

   Bad paths are skipped with a warning on stderr rather than crashing the
   viewer.
4. **Desktop only, live**: the "scenario path" text field + "Load" button in
   the viewer window itself — type a path to a `*.json` file or a directory
   of them and click Load. Newly loaded scenarios are appended to the
   dropdown and the first one is selected immediately; a status line reports
   how many scenarios loaded, or why loading failed. This is the same
   `load_path` used for CLI args, just without needing to relaunch — the way
   to browse scenarios converted from CommonRoad files (see
   [below](#converting-commonroad-scenarios)) or exported from a local
   nuPlan log during a live session.
5. **Web only**: `scenarios/web_bundle.json`, fetched automatically at
   startup — the filesystem-based sources above don't exist in a browser.
   It's a single compact JSON array (built by
   `tools/bundle_web_scenarios.py` from a directory of scenario files, one
   HTTP request instead of one per scenario) that Trunk copies into `dist/`
   at build time. Ships with the converted CommonRoad corpus from
   [`scenarios/commonroad/`](../scenarios/commonroad/) — freely
   redistributable, unlike nuPlan data (see
   [Generating the scenario corpus](#generating-the-scenario-corpus)
   below). A failed or empty fetch degrades silently (a warning in the
   browser console, zero extra scenarios) rather than breaking the viewer.
6. **Web only, live**: the "Load scenario file(s)…" button in the viewer —
   opens the browser's native file picker (via the `rfd` crate) so a visitor
   can browse to their own exported `*.json` scenario file(s) — a single
   scenario, or a bundle array like `web_bundle.json`'s format — and load
   them into the running app without a maintainer having to bake them into
   the deployed bundle first. This is the web equivalent of desktop's
   "scenario path" widget above: same "append to the dropdown, select the
   first newly loaded one" behavior, same status line, just picking files
   through the browser's dialog instead of typing a filesystem path.

## Batch evaluation

```sh
cargo run --release --bin batch -- --count 100          # synthetic scenario sweep
cargo run --release --bin batch -- --count 0 --dir DIR  # only scenario JSON files
cargo run --release --bin batch -- --count 50 --dir DIR --seed 7   # both, combined
```

Runs **every planner over every scenario** and prints:

- **stdout**: one CSV row per (scenario, planner) pair — `scenario,planner,score,` followed
  by all eight metric aggregates (see [src/metrics/README.md](../src/metrics/README.md)
  for what each column means):

  ```
  scenario,planner,score,no_at-fault_collisions,drivable_area,driving_direction,making_progress,TTC_within_bound,progress_ratio,speed_limit,comfort
  lead-000,straight_(strawman),0.0000,0.0000,0.0000,1.0000,1.0000,0.0000,0.9928,0.8439,1.0000
  lead-000,bezier_+_IDM,0.7421,1.0000,1.0000,1.0000,1.0000,0.9820,0.9908,0.9962,0.9900
  ...
  ```

- **stderr**: a mean-score-per-planner summary, useful for a quick leaderboard:

  ```
  mean score over 100 scenarios:
    straight (strawman)    0.2668
    bezier + IDM           0.7425
    frenet lattice         0.8982
    PI2-DDP                0.8567
  ```

Redirect stdout to a file to analyze in a spreadsheet or notebook:

```sh
cargo run --release --bin batch -- --count 200 > results.csv
```

Flags:

| Flag | Default | Meaning |
|---|---|---|
| `--count N` | `20` | Number of synthetic scenarios to generate (see [`synthetic_batch`](../src/scenarios/README.md#synthetic-generation)). Set to `0` to run only `--dir` scenarios. |
| `--seed S` | `42` | Seed for the synthetic generator. Same seed + count always produces the same scenarios. |
| `--dir PATH` | (none) | A directory of `*.json` scenario files to include. Repeatable — pass `--dir a --dir b` to combine multiple directories. |

## Latency profiling

```sh
cargo run --release --bin profile_latency -- --count 20
cargo run --release --bin profile_latency -- --count 0 --dir DIR
cargo run --release --bin profile_latency -- --mode world --duration 30
```

By default, runs the same planner/scenario sweep as `batch`, but aggregates
the existing `simulate()` latency diagnostics instead of metric scores.
`--mode world` instead runs the procedural `LiveWorld` loop offline with the
viewer defaults: seed `1`, `64` traffic actors, `0.1 s` ticks, and a
deterministic clicked-style route goal ahead of the ego.

- **stdout**: one CSV row per `(planner, seam)` across the whole batch:

  ```
  planner,seam,calls,total_ms,mean_ms,max_ms
  straight_(strawman),total,4000,12.341,0.003,0.029
  frenet_lattice,route,4000,44.120,0.011,0.041
  ...
  ```

- **stderr**: a compact mean `total` latency summary per planner, useful for
  quick comparisons.

Scenario-mode flags are the same as [Batch evaluation](#batch-evaluation):
`--count`, `--seed`, and repeatable `--dir`. Shared/world flags:

| Flag | Default | Meaning |
|---|---|---|
| `--mode scenarios\|world` | `scenarios` | Profile scenario rollouts or the procedural live world. |
| `--duration S` | `20` | Simulated seconds per scenario or world run. |
| `--world-seed S` | `1` | Procedural world seed, matching the viewer's default. |
| `--max-actors N` | `64` | Live traffic cap for `--mode world`, matching the viewer's default. |

## Converting CommonRoad scenarios

`tools/export_commonroad_scenarios.py` reads [CommonRoad](https://commonroad.in.tum.de)
2020a XML scenarios with only the Python standard library — no
`commonroad-io` install required — and writes one nanoplan-format JSON file
per scenario: the route through the lanelet network becomes the
`centerline`, the planning problem's initial state becomes the `ego`, every
obstacle becomes an actor (dynamic ones replay their logged trajectory), and
the road half-width / lane divider / crosswalks are derived from the
lanelets. The mapping details are documented in
[src/scenarios/README.md#commonroad-export-mapping](../src/scenarios/README.md#commonroad-export-mapping).

```sh
python3 tools/export_commonroad_scenarios.py scenarios/commonroad out_dir
python3 tools/export_commonroad_scenarios.py path/to/ONE_Scenario-1_1_T-1.xml out_dir
```

| Flag | Default | Meaning |
|---|---|---|
| `src` (positional) | — | A CommonRoad `*.xml` file, or a directory of them (non-recursive). |
| `out` (positional) | — | Output directory; created if it doesn't exist. |

This works on the corpus shipped in
[`scenarios/commonroad/`](../scenarios/commonroad/) and on real scenarios
downloaded from the [CommonRoad database](https://commonroad.in.tum.de/scenarios)
alike. Once converted, feed the directory into either tool:

```sh
cargo run --release --bin batch -- --count 0 --dir out_dir   # score every planner on them
cargo run -- out_dir                                          # browse them in the viewer
```

To make them available in the **web build** too (the deployed viewer has no
filesystem, so `out_dir` above only works on desktop), convert straight into
`scenarios/web/` and bundle it into the single file the web build fetches at
startup (see [Scenario sources](#scenario-sources) above):

```sh
python3 tools/export_commonroad_scenarios.py scenarios/commonroad scenarios/web
python3 tools/bundle_web_scenarios.py            # writes scenarios/web_bundle.json
trunk build --release --public-url /nanoplan/    # copies it into dist/
```

## Exporting real nuPlan scenarios (local only)

`tools/export_nuplan_scenarios.py` reads a nuPlan log database directly with
Python's standard-library `sqlite3` — no `nuplan-devkit` install required —
and writes one nanoplan-format JSON file per tagged scenario in the log.
Its main purpose now is the `expert` (human) trajectory each export carries:
the demonstration data the [cost-weight autotuner](#autotuning-the-cost-weights)
learns from, which CommonRoad scenarios don't include.

> **Keep nuPlan exports local.** The nuPlan dataset is registration-gated
> and its license does not permit redistribution — don't commit exported
> scenarios to this repo or bundle them into the deployed web build. That's
> exactly why the corpus this repo *ships* is CommonRoad-format instead
> (see [Converting CommonRoad scenarios](#converting-commonroad-scenarios)).

```sh
python3 tools/export_nuplan_scenarios.py path/to/log.db out_dir
python3 tools/export_nuplan_scenarios.py path/to/log.db out_dir \
    --horizon 15 --max 50 --types stopping_with_lead,starting_left_turn
```

| Flag | Default | Meaning |
|---|---|---|
| `db` (positional) | — | Path to the nuPlan log `.db` (SQLite) file. |
| `out` (positional) | — | Output directory; created if it doesn't exist. |
| `--horizon` | `20.0` | Seconds of log to capture per scenario, starting at the tagged frame. |
| `--max` | `100` | Stop after writing this many scenario files. |
| `--types` | (all) | Comma-separated nuPlan scenario type names to keep (see `scenario_tag.type` in [`scenarios/nuplan/nuplan_schema.md`](../scenarios/nuplan/nuplan_schema.md) — there are ~70). |

What ends up in each JSON file (and why) is documented in
[src/scenarios/README.md#nuplan-export-mapping](../src/scenarios/README.md#nuplan-export-mapping).
Exported directories work everywhere converted CommonRoad ones do (batch
`--dir`, viewer CLI args, the in-app "scenario path" widget) — just keep
them out of `scenarios/web/`.

> **Note:** the export script is written strictly against the vendored
> [`nuplan_schema.md`](../scenarios/nuplan/nuplan_schema.md) but has not been
> exercised against a real nuPlan log in this repository (the dataset
> requires registration to download). Treat the first run against your own
> log as a shakedown, and please report schema mismatches.

## Autotuning the cost weights

```sh
cargo run --release --bin tune -- --dir out_dir
```

Fits the [shared cost function's](../src/planning/README.md#the-shared-cost-function)
soft weights to the expert (human) trajectories in exported nuPlan scenarios,
with maximum-entropy inverse reinforcement learning (DriveIRL-style — see
[src/tuning/README.md](../src/tuning/README.md) for the model and its
assumptions). Scenarios need an `expert` field, which
`tools/export_nuplan_scenarios.py` includes (from nuPlan data you've
licensed and exported locally — see
[above](#exporting-real-nuplan-scenarios-local-only)); CommonRoad scenarios
carry no expert trajectory, and scenarios without a usable expert are
counted and skipped. Collision and driving off-road stay
**infinite cost by fiat** — only the soft weights are learned.

The output ends with the learned `WEIGHTS` line to paste into
`src/planning/cost.rs` (the weights are a compile-time constant, so applying
a tune is a one-line diff), plus before/after diagnostics to judge it by:

```
maxent-irl cost-weight autotune
  scenarios: 42 used, 3 skipped (no expert, expert shorter than the 10 s horizon, or expert hard-violating)
  mean NLL/scenario:      4.3948 -> 0.9611
  expert is min-cost in:  12/42 -> 33/42 scenarios

  feature             current    learned
  actor_proximity    200.0000   181.2034
  ...

paste into src/planning/cost.rs:
pub(crate) const WEIGHTS: [f64; N_FEATURES] = [181.2034, ...];
```

After pasting, re-run the [batch evaluation](#batch-evaluation) to see what
the new weights do to closed-loop scores.

| Flag | Default | Meaning |
|---|---|---|
| `--dir PATH` | (required) | A directory of scenario JSON files with `expert` trajectories. Repeatable. |
| `--iters N` | `500` | Gradient-descent iterations. |
| `--l2 X` | `1e-3` | Strength of the pull toward the current hand weights (a prior); lower it to let the data dominate. |

## Generating the scenario corpus

`tools/generate_diverse_scenarios.py` procedurally generates the CommonRoad
XML corpus checked into [`scenarios/commonroad/`](../scenarios/commonroad/):
one scenario per category (stopping with a lead, traversing an intersection,
near multiple vehicles, congested stop-and-go, cutting in, merging onto a
highway, and more — see the script for the full list) plus the two fixed
classics compiled into the viewer binary, spanning a wide range of speeds
(residential to highway), road geometries (straight, sine curves, arcs,
S-curves, roundabout-like loops), and agent interactions. Each file is a
complete CommonRoad 2020a scenario: lanelet network, obstacles with scripted
20 s / 10 Hz trajectories, and a planning problem.

```sh
python3 tools/generate_diverse_scenarios.py --clean                          # scenarios/commonroad/*.xml
python3 tools/export_commonroad_scenarios.py scenarios/commonroad scenarios/web
python3 tools/bundle_web_scenarios.py                                        # scenarios/web_bundle.json
```

| Flag | Default | Meaning |
|---|---|---|
| `--out` | `scenarios/commonroad` | Output directory. |
| `--variations` | `1` | How many randomized variations to generate per scenario category. |
| `--seed` | `1` | RNG seed; same seed + variations always produces byte-identical scenarios. |
| `--clean` | off | Remove the output directory's `*.xml` files first, instead of adding to whatever's already there. |

A random subset of scenarios additionally gets CommonRoad environment
conditions (light rain, night, fog) and a correspondingly derated
goal-velocity interval, which the converter surfaces as a reduced
`target_speed` and a `[light_rain]`-style name suffix — nanoplan has no
weather/visibility model, so treat it as difficulty/flavor variation, not a
claim that the viewer renders weather.

## Web deploy

Every push to `main` runs [`.github/workflows/deploy.yml`](../.github/workflows/deploy.yml),
which builds the viewer with Trunk for `wasm32-unknown-unknown` and deploys
`dist/` to GitHub Pages. To enable it on a fork: repo Settings → Pages →
Source → "GitHub Actions". You can also trigger a deploy manually from the
Actions tab (`workflow_dispatch`) without pushing a commit.

For local wasm development, see
[docs/SETUP.md#web-wasm-build](SETUP.md#web-wasm-build).

## Running tests

```sh
cargo test                                        # everything
cargo test -p nanoplan --lib planning::pi2ddp     # one planner
cargo test -p nanoplan --lib metrics::            # the metrics evaluator
```

Most tests are closed-loop: they drive a planner through the `Simulator` for
many ticks and assert on the resulting trace (e.g. "stays within 0.5 m of
the centerline", "keeps more than 2 m of clearance from the obstacle").
A few are pure unit tests of formulas (Frenet round-trip, metric thresholds,
JSON round-trips). Individual metrics (`metrics::comfort`, `metrics::ttc`,
etc.) are pure `score()` functions with no tests of their own — they're
exercised through `metrics::evaluate()`'s tick-exact tests in
`metrics/mod.rs` instead (see
[src/metrics/README.md#testing](../src/metrics/README.md#testing)). See each
component's README for what its tests check and why.
