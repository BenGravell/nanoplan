# Usage

Run the desktop demo:

```bash
cargo run --release
```

The ego and traffic race on either a generated or selected real race circuit.

The viewer opens after all real tracks have been loaded. The first run needs a
network connection; subsequent runs use the platform cache.

- **track** selects the generated circuit or a downloaded circuit.
- **planner** changes the active motion planner.
- **future preview** sets how many seconds of the current plan are drawn;
  zero hides the preview without stopping the ego.
- **diagnostic points/trajectories** show the selected planner's sampled
  search geometry when that planner records diagnostics.
- **pause** freezes the simulation.
- **new track** increments the seed and spectrally regenerates a simple closed
  circuit, its width, traffic, and the ego start.
- **scroll** zooms the camera.

For a browser build:

```bash
trunk serve --release
```
