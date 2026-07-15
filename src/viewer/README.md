# Viewer

The viewer contains only the endless-track mode, split by responsibility:

- `live/` owns simulation updates, camera input, and world rendering, with
  atomic scene drawing in `live/drawing/`;
- `ui/` owns layout, controls, the driving HUD, visual style, and reusable
  widgets;
- `mod.rs` wires those features into the Bevy app.

Planner diagnostics are captured by the realtime world only while an overlay
is enabled. The latency table accumulates per-seam mean/max timings for the
active planner and resets on planner or track changes.
