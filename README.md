# nanoplan

Ultra minimalist motion planner for car-like vehicles, written in Rust with zero dependencies.

## Approach

- **Trajectory trees**: candidate motions are grown as a tree of dynamically feasible trajectories.
- **Sampling-based DDP**: tree expansion is guided by a sampling-based variant of differential dynamic programming.
- **Kinematic bicycle model**: the vehicle model shared by both.
- **nuPlan scenarios**: scenario definitions are vendored from
  [nuplan-devkit](https://github.com/motional/nuplan-devkit) in [`scenarios/nuplan/`](scenarios/nuplan/).
- **Bevy app**: a lightweight interactive viewer to inspect simulator results and steer the planner.

## Usage

```sh
cargo build
cargo test
```

## Agent skills

The [ponytail](https://github.com/DietrichGebert/ponytail) skills (MIT) are vendored in
[`.claude/skills/`](.claude/skills/) to keep AI-assisted contributions minimal, in line with
the project's ethos.
