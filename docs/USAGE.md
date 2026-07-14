# Usage

Run the desktop demo:

```bash
cargo run --release
```

The ego and traffic race around a procedurally generated, unbounded single
track. Forward progress makes the ego accelerate; there is no cruise-speed
control.

- **planner** changes the active motion planner.
- **future preview** sets how many seconds of the current plan are drawn;
  zero hides the preview without stopping the ego.
- **diagnostic points/trajectories** show the selected planner's sampled
  search geometry when that planner records diagnostics.
- **pause** freezes the simulation.
- **new track** increments the seed and regenerates curvature, width, traffic,
  and the ego start.
- **scroll** zooms the camera.

For a browser build:

```bash
trunk serve --release
```
