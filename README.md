# nanoplan

An interactive motion-planning demo for car-like vehicles. Traffic cars share
the road and the selected planner replans the ego every tick.

```bash
cargo run --release
```

The UI selects a planner, pauses the simulation, or generates a new track seed.
Scroll to zoom.

## Layout

```text
src/
├── track/         track geometry, loading, and generation
├── world/         realtime ego, traffic, and planning loop
├── planning/      motion planners
├── simulation/    vehicle dynamics and collision handling
├── metrics/       shared cost primitives
└── viewer/        Bevy rendering and controls
```

See [setup](docs/SETUP.md), [usage](docs/USAGE.md), and the
[track documentation](src/track/README.md).

Nanoplan's source code is licensed under the [MIT License](LICENSE).
