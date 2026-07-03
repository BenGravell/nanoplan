# Usage

See [SETUP.md](SETUP.md) first if you haven't built the project yet.

## The viewer

```sh
cargo run
```

opens an interactive window with a control panel in the top-left corner and
a top-down view of the scenario.

### Controls

| Control | Effect |
|---|---|
| **scenario** dropdown | Selects which `Scenario` to simulate. See [Scenario sources](#scenario-sources) below for what's in the list. |
| **planner** dropdown | Selects which of the four planners drives the ego vehicle. If this (scenario, planner) combo hasn't been simulated yet, a "Simulate" button (then a progress bar while it runs) appears in place of the metrics table below — simulating doesn't block the window, so an expensive planner like PI²-DDP never freezes the UI. Re-selecting a combo already simulated is instant. |
| **time [s]** slider | Scrubs through the 20-second rollout. The camera follows the ego; the metrics and latency tables below update to the scrubbed tick. |
| **future preview [s]** slider | When above zero, frames the screen in an accent color and overlays: the ego's plan from the scrubbed state (replanned live, pink line + ghost car at the horizon) and every actor's constant-velocity prediction (dimmer pink). Use this to see a planner's plan diverge from reality — e.g. select "curving lead" or the bundled "nuplan: cut-in" scenario and watch the prediction miss. |
| **diagnostic points** / **diagnostic trajectories** checkboxes | Only shown for planners with search geometry to show (Frenet lattice, PI²-DDP). Overlays what the planner considered during the future-preview replan above: for the lattice, its (station, lateral) sample grid (yellow points) and every DP edge it costed (cyan trajectories); for PI²-DDP, its sampled rollout states (points) and rollouts (trajectories). Needs **future preview [s]** above zero — that replan is what gets recorded. |

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
   braking lead and a lane cut-in, both driven by logged trajectory replay
   (see [src/scenarios/README.md#trajectory-replay](../src/scenarios/README.md#trajectory-replay)).
3. **Desktop only**: any files or directories passed as command-line
   arguments, each loaded with `nanoplan::scenarios::load_path`:

   ```sh
   cargo run -- path/to/exported_scenarios/    # a directory of *.json files
   cargo run -- path/to/one_scenario.json      # a single scenario file
   ```

   Bad paths are skipped with a warning on stderr rather than crashing the
   viewer.
4. **Desktop only, live**: the "nuPlan path" text field + "Load" button in
   the viewer window itself — type a path to a `*.json` file or a directory
   of them and click Load. Newly loaded scenarios are appended to the
   dropdown and the first one is selected immediately; a status line reports
   how many scenarios loaded, or why loading failed. This is the same
   `load_path` used for CLI args, just without needing to relaunch — the way
   to browse scenarios exported from a real nuPlan log (see
   [below](#exporting-real-nuplan-scenarios)) during a live session.
5. **Web only**: `scenarios/web_bundle.json`, fetched automatically at
   startup — the filesystem-based sources above don't exist in a browser.
   It's a single compact JSON array (built by
   `tools/bundle_web_scenarios.py` from a directory of scenario files, one
   HTTP request instead of one per scenario) that Trunk copies into `dist/`
   at build time. Ships empty by default (no real nuPlan corpus is checked
   into this repo — see the note in
   [Exporting real nuPlan scenarios](#exporting-real-nuplan-scenarios)); a
   maintainer populates it by exporting into `scenarios/web/` and rerunning
   the bundler before deploying. A failed or empty fetch degrades silently
   (a warning in the browser console, zero extra scenarios) rather than
   breaking the viewer.
6. **Web only, live**: the "Load scenario file(s)…" button in the viewer —
   opens the browser's native file picker (via the `rfd` crate) so a visitor
   can browse to their own exported `*.json` scenario file(s) — a single
   scenario, or a bundle array like `web_bundle.json`'s format — and load
   them into the running app without a maintainer having to bake them into
   the deployed bundle first. This is the web equivalent of desktop's
   "nuPlan path" widget above: same "append to the dropdown, select the
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

## Exporting real nuPlan scenarios

`tools/export_nuplan_scenarios.py` reads a nuPlan log database directly with
Python's standard-library `sqlite3` — no `nuplan-devkit` install required —
and writes one nanoplan-format JSON file per tagged scenario in the log.

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
Once exported, feed the directory into either tool:

```sh
cargo run --release --bin batch -- --count 0 --dir out_dir   # score every planner on them
cargo run -- out_dir                                          # browse them in the viewer
```

To make them available in the **web build** too (the deployed viewer has no
filesystem, so `out_dir` above only works on desktop), export straight into
`scenarios/web/` and bundle it into the single file the web build fetches at
startup (see [Scenario sources](#scenario-sources) above):

```sh
python3 tools/export_nuplan_scenarios.py path/to/log.db scenarios/web
python3 tools/bundle_web_scenarios.py            # writes scenarios/web_bundle.json
trunk build --release --public-url /nanoplan/    # copies it into dist/
```

> **Note:** the export script is written strictly against the vendored
> [`nuplan_schema.md`](../scenarios/nuplan/nuplan_schema.md) but has not been
> exercised against a real nuPlan log in this repository (the dataset
> requires registration to download). Treat the first run against your own
> log as a shakedown, and please report schema mismatches.

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
