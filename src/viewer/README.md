# `viewer`

The interactive viewer binary: an egui control panel over a Bevy 2D scene
that scrubs through a simulated scenario and previews the planner's future
plan. `src/main.rs` is a two-line entry point (`mod viewer; fn main() {
viewer::run(); }`) — everything else lives here.

```
viewer/
├── mod.rs        run(), shared resources (Scenarios, UiState) and constants
├── scenarios.rs  where scenarios come from: built-ins, bundled JSON, CLI args
├── rollouts.rs   RolloutCache and the chunked async simulation job
├── ui.rs         the egui control panel system
├── draw.rs       gizmo rendering: map, cars, future-preview overlay
└── web.rs        wasm only — fetches the nuPlan scenario bundle at startup
```

## Why simulation is chunked

Simulation logic itself — `simulate()`, `IncrementalSim` — lives entirely in
[`nanoplan::simulation`](../simulation/README.md); `rollouts.rs` only
schedules and caches it for the viewer, it doesn't implement any of it.

An expensive planner (PI²-DDP) can take seconds to run a full closed-loop
rollout. Running that synchronously in the `ui` system would freeze the
window on every scenario/planner change. `rollouts.rs`'s `ActiveJob` instead
holds an `IncrementalSim`, stepped a fixed wall-clock budget per frame
(`FRAME_BUDGET_MS`) until done, at which point the result moves into
`RolloutCache` (keyed by `(scenario, planner)`, so re-selecting a combo
already simulated is instant). `ui.rs` shows a "Simulate" button when the
current selection is neither cached nor in-flight, and a progress bar while
it's running.

## Desktop vs. web scenario sources

Both are additive to the six built-in scenarios and the two bundled via
`include_str!` in `scenarios.rs`:

- **Desktop** (`ui.rs`'s `ScenarioLoader`, `scenarios.rs`'s CLI-arg loop):
  the wasm build has no filesystem, so these are `#[cfg(not(target_arch =
  "wasm32"))]`. Both go through `nanoplan::scenarios::load_path`.
- **Web** (`web.rs`): the wasm build fetches `scenarios/web_bundle.json` —
  a single static file, built by `tools/bundle_web_scenarios.py` and copied
  into `dist/` by Trunk — once at startup via `gloo-net`, using the same
  "spawn async, poll each frame, merge into state when ready" pattern as
  `ActiveJob`.

See [`docs/USAGE.md`](../../docs/USAGE.md#scenario-sources) for the
user-facing description of all four sources.

## Introspection diagnostics

The "diagnostic points" / "diagnostic trajectories" checkboxes in `ui.rs`
(shown only when `state.planner.has_diagnostics()`, i.e. the Frenet lattice
or PI²-DDP is selected — see
[`src/planning/README.md#introspection-diagnostics`](../planning/README.md#introspection-diagnostics))
render whatever the planner recorded into a `nanoplan::planning::Diagnostics`
during the *future-preview* replan in `draw.rs`. That replan already runs
every frame while `preview_s > 0` (it's how the pink plan-preview line gets
drawn); when either checkbox is on, `draw.rs` additionally passes a
`Diagnostics` recorder into that same `plan()` call and draws its points
(yellow) and trajectories (cyan) as gizmos. Nothing is recorded, and the
checkboxes have no effect, while `preview_s == 0` — there's no replan
happening to record from.
