# nanoplan

An interactive motion-planning demo on one endless procedural track.

The track is a deterministic function of distance and seed. Its curvature and
width vary continuously, so the viewer can sample only the region around the
ego without loading maps, building road topology, choosing lanes, or planning
a route. Traffic cars follow the same track and the selected planner replans
the ego every tick.

```bash
cargo run --release
```

The UI selects a planner and target speed, pauses the simulation, or generates
a new track seed. Scroll to zoom.

## Layout

```text
src/
├── track.rs       endless centerline/width functions and Path/Road geometry
├── world/         realtime ego, traffic, and planning loop
├── planning/      motion planners
├── simulation/    vehicle dynamics and collision handling
├── metrics/       shared cost primitives
└── viewer/        Bevy rendering and controls
```

See [setup](docs/SETUP.md) and [usage](docs/USAGE.md).

Licensed under the [MIT License](LICENSE).
