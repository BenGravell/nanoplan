# nanoplan

Ultra minimalist motion planner for car-like vehicles, written in Rust with zero dependencies.

## Approach

- **Trajectory trees**: candidate motions are grown as a tree of dynamically feasible trajectories.
- **Sampling-based DDP**: tree expansion is guided by a sampling-based variant of differential dynamic programming.
- **Kinematic ego model**: state (x, y, yaw, speed), controls (acceleration, curvature).
- **Planners**: strawman straight-line, Bezier-to-centerline with IDM speed, an
  EM-style Frenet lattice, and a sampling-based DDP planner following PI²-DDP
  (Lefebvre & Crevecoeur, "Path Integral Policy Improvement with Differential
  Dynamic Programming") with a road-model-informed sampling distribution —
  selectable in the viewer for comparison.
- **nuPlan scenarios**: scenario definitions are vendored from
  [nuplan-devkit](https://github.com/motional/nuplan-devkit) in [`scenarios/nuplan/`](scenarios/nuplan/).
- **Bevy app**: a lightweight interactive viewer to inspect simulator results and steer the planner.

## Usage

```sh
cargo run    # interactive viewer: scenario selector, time scrubber, future preview
cargo test
```

## Batch evaluation

```sh
cargo run --release --bin batch -- --count 100          # synthetic scenario sweep
cargo run --release --bin batch -- --count 0 --dir DIR  # scenario JSON files
```

Runs every planner over every scenario: per-scenario metric rows as CSV on
stdout, mean score per planner on stderr. Scenarios are plain JSON
(`nanoplan::scenario::Scenario`); actors with a logged `trajectory` are
replayed (interpolated, constant velocity past the end) instead of
constant-velocity extrapolation. To run real nuPlan log scenarios, export
them first (standard library only, no devkit install):

```sh
python3 tools/export_nuplan_scenarios.py path/to/log.db out_dir
cargo run --release --bin batch -- --count 0 --dir out_dir
cargo run --release -- out_dir    # or browse them in the viewer
```

The viewer's scenario dropdown offers the built-ins, the bundled JSONs in
[`scenarios/json/`](scenarios/json/) (which also ship in the web build), and
any scenario directories passed as arguments on desktop.

## Web deploy

Pushes to `main` build the viewer for `wasm32-unknown-unknown` with
[Trunk](https://trunkrs.dev) and deploy it to GitHub Pages (enable Pages with
source "GitHub Actions" in the repo settings). To build locally:

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk
trunk serve
```

## Agent skills

The [ponytail](https://github.com/DietrichGebert/ponytail) skills (MIT) are vendored in
[`.claude/skills/`](.claude/skills/) to keep AI-assisted contributions minimal, in line with
the project's ethos.
