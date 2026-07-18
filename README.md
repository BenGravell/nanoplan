# nanoplan

An interactive motion-planning app for car racing.

```bash
cargo run --release
```

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

See [setup](docs/SETUP.md) and [usage](docs/USAGE.md).
