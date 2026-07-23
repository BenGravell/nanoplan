# Viewer

The viewer contains one live driving mode, split by responsibility:

- `live/` owns simulation updates, camera input, and world rendering, with
  atomic scene drawing in `live/drawing/`;
- `ui/` owns layout, controls, the driving HUD, visual style, and reusable
  widgets;
- `mod.rs` wires those features into the Bevy app.
