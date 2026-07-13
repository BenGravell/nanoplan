# Viewer

The viewer contains only the endless-track mode:

- `live.rs` advances and draws `LiveWorld`;
- `ui.rs` exposes planner, speed, preview, diagnostics, pause, and seed controls;
- `draw.rs` contains the small vehicle drawing helpers.

There are no scenario loaders, rollout scrubbers, route controls, or
platform-specific data-loading paths. Planner diagnostics are captured by the
realtime world only while an overlay is enabled. The latency table accumulates
per-seam mean/max timings for the active planner and resets on planner or track
changes.
