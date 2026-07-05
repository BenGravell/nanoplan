# `viewer`

The interactive viewer binary: an egui control panel over a Bevy 2D scene
with two modes — **scenarios** (scrub through a simulated scenario and
preview the planner's future plan) and **open world** (a realtime
interactive sandbox over [`nanoplan::world`](../world/README.md)).
`src/main.rs` is a two-line entry point (`mod viewer; fn main() {
viewer::run(); }`) — everything else lives here.

```
viewer/
├── mod.rs        run(), Mode, shared resources (Scenarios, UiState) and constants
├── scenarios.rs  where scenarios come from: built-ins, bundled JSON, CLI args
├── rollouts.rs   RolloutCache and the chunked async simulation job
├── loader.rs     ScenarioSource trait + Loader resource; desktop path-loading source
├── ui.rs         the egui control panel system (mode tabs, scrub + live panels)
├── draw.rs       gizmo rendering: map, cars, future-preview overlay (scrub mode)
├── live.rs       open-world mode: realtime pacing, click-to-goal input, map/traffic drawing
└── web.rs        wasm only — fetches the startup bundle; web file-picker ScenarioSource
```

## Open world mode

All open-world logic — street map generation, routing, traffic, the
realtime `LiveWorld::tick` loop — lives in
[`nanoplan::world`](../world/README.md), not here. `live.rs` is only the
Bevy plumbing: a fixed-timestep accumulator that ticks the world at `DT`
(capped at `MAX_TICKS_PER_FRAME` per frame, so a slower-than-realtime
planner lags gracefully instead of freezing the UI), mouse handling (left
click → `set_goal`, guarded by `UiState::pointer_over_ui` so clicks on the
egui panel don't place goals; scroll → zoom), and gizmo drawing of the
street network, route, goal, live plan, and cars. `Live` is a `NonSend`
resource for the same reason `ActiveJob` is: `LiveWorld` holds a
`Box<dyn Planner>`. Both modes' systems run every frame and early-return
on the inactive mode's state (`UiState::mode`).

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
`include_str!` in `scenarios.rs`.

The in-app loading widget goes through one seam: `loader.rs`'s
`ScenarioSource` trait (Strategy — one platform implementation each,
selected in `Loader::default()`). `ui.rs` only calls
`loader.source.widget(ui)` and handles the returned scenarios; merging them
into the list, selecting the first new one, and the green/red status line
are platform-independent and exist once, in `ui.rs`/`Loader`. The platform
sources are:

- **Desktop** (`loader.rs`'s `DesktopLoader`, plus `scenarios.rs`'s CLI-arg
  loop): the wasm build has no filesystem, so these are
  `#[cfg(not(target_arch = "wasm32"))]`. Both go through
  `nanoplan::scenarios::load_path`.
- **Web** (`web.rs`), two independent mechanisms, both using the same
  "spawn async, poll each frame, merge into state when ready" pattern as
  `ActiveJob`:
  - `WebScenarioFetch`/`spawn_fetch`/`absorb_fetch`: fetches
    `scenarios/web_bundle.json` — a single static file, built by
    `tools/bundle_web_scenarios.py` and copied into `dist/` by Trunk — once
    at startup via `gloo-net`. Ships with 115 procedurally generated
    scenarios by default (see
    [`tools/generate_diverse_scenarios.py`](../../tools/generate_diverse_scenarios.py)),
    since there's no real nuPlan corpus checked into this repo.
  - `WebScenarioLoader`, the web `ScenarioSource`: opens the browser's
    native file picker via `rfd::AsyncFileDialog` (wasm backend: a hidden
    `<input type="file">`, so it's automatable in tests via a filechooser
    handler) when its "Load scenario file(s)…" button is clicked, reads
    each picked file's bytes, and parses it as either a single `Scenario`
    or a `Vec<Scenario>`, handing the result back through `widget()`. The
    web equivalent of `DesktopLoader` — the one way a visitor to the
    deployed site can bring their own nuPlan-exported scenarios into the
    running app, since nothing short of a page reload can add to what
    `WebScenarioFetch` grabbed at startup.

See [`docs/USAGE.md`](../../docs/USAGE.md#scenario-sources) for the
user-facing description of all sources.

## Introspection diagnostics

The "diagnostic points" / "diagnostic trajectories" checkboxes in `ui.rs`
(shown only when `state.planner.has_diagnostics()`, i.e. the Frenet lattice,
PI²-DDP, or RRT* is selected — see
[`src/planning/README.md#introspection-diagnostics`](../planning/README.md#introspection-diagnostics))
render whatever the planner recorded into a `nanoplan::planning::Diagnostics`
during the *future-preview* replan in `draw.rs`. That replan already runs
every frame while `preview_s > 0` (it's how the pink plan-preview line gets
drawn); when either checkbox is on, `draw.rs` additionally passes a
`Diagnostics` recorder into that same `plan()` call and draws its points
(yellow) and trajectories (cyan) as gizmos. Nothing is recorded, and the
checkboxes have no effect, while `preview_s == 0` — there's no replan
happening to record from.
