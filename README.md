# nanoplan

Ultra minimalist motion planner for car-like vehicles, written in Rust with zero dependencies.

## Approach

- **Trajectory trees**: candidate motions are grown as a tree of dynamically feasible trajectories.
- **Sampling-based DDP**: tree expansion is guided by a sampling-based variant of differential dynamic programming.
- **Kinematic ego model**: state (x, y, yaw, speed), controls (acceleration, curvature).
- **Planners**: strawman straight-line, Bezier-to-centerline with IDM speed, and an
  EM-style Frenet lattice — selectable in the viewer for comparison.
- **nuPlan scenarios**: scenario definitions are vendored from
  [nuplan-devkit](https://github.com/motional/nuplan-devkit) in [`scenarios/nuplan/`](scenarios/nuplan/).
- **Bevy app**: a lightweight interactive viewer to inspect simulator results and steer the planner.

## Usage

```sh
cargo run    # interactive viewer: scenario selector, time scrubber, future preview
cargo test
```

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
