# `scenarios`

The `Scenario`/`Actor`/`Waypoint`/`MapData` data model, the Frenet-frame
`Path` helper shared by planners and metrics, scenario JSON loading, logged
trajectory replay, and the synthetic scenario generator.

```
scenarios/
└── mod.rs   Path, Waypoint, Actor (+trace/replay), MapData, Scenario, load_dir, load_path, synthetic_batch
```

> **Naming note**: this is the Rust module `src/scenarios/`. There are also
> repo-root *data* directories with similar names — don't confuse them:
> [`scenarios/nuplan/`](../../scenarios/nuplan/) holds vendored nuPlan
> reference documents (schema, vehicle parameters, metric definitions),
> `scenarios/json/` holds example scenario JSON files bundled into the
> viewer binary at compile time, and `scenarios/web/` (not checked in by
> default) is the source directory `tools/bundle_web_scenarios.py` reads to
> build `scenarios/web_bundle.json`, fetched by the web build at startup —
> see [docs/USAGE.md#scenario-sources](../../docs/USAGE.md#scenario-sources).
> None of these three contain code.

## `Scenario`, the JSON format

```rust
pub struct Scenario {
    pub name: String,
    pub ego: State,
    pub actors: Vec<Actor>,       // defaults to empty
    pub centerline: Vec<[f64; 2]>,
    pub target_speed: f64,        // defaults to 10.0
    pub map: MapData,             // defaults to MapData::default()
}
```

Everything derives `Serialize`/`Deserialize` via `serde`, with `#[serde(default)]`
on every optional field, so a scenario file can be as small as:

```json
{
  "name": "minimal",
  "ego": { "x": 0.0, "y": 0.0, "speed": 8.0 },
  "centerline": [[-10.0, 0.0], [100.0, 0.0]]
}
```

A fuller example, with an actor replaying a logged trajectory and explicit
map data (this is a trimmed version of [`scenarios/json/braking_lead.json`](../../scenarios/json/braking_lead.json)):

```json
{
  "name": "nuplan: braking lead",
  "ego": { "x": 0.0, "y": 0.0, "speed": 8.0 },
  "actors": [
    {
      "init": { "x": 40.0, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
      "trajectory": [
        { "t": 0.0, "x": 40.0, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
        { "t": 0.1, "x": 40.8, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
        { "t": 2.0, "x": 56.0, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
        { "t": 6.0, "x": 76.0, "y": 0.0, "yaw": 0.0, "speed": 0.0 }
      ]
    }
  ],
  "centerline": [[-50.0, 0.0], [400.0, 0.0]]
}
```

### Field reference

| Field | Type | Default | Notes |
|---|---|---|---|
| `name` | `String` | required | Display name; shown verbatim in the viewer's dropdown and the batch CSV's `scenario` column. |
| `ego` | `State` | required | `{x, y, yaw, speed}`; `yaw` and `speed` each default to `0.0` if omitted. Starting pose/speed of the planned vehicle. |
| `actors` | `Vec<Actor>` | `[]` | Other vehicles. See [Actor motion](#actor-motion) below. |
| `centerline` | `Vec<[f64; 2]>` | required | Polyline of `[x, y]` points describing the lane the ego should follow. At least 2 points. Order matters — arc length increases along it. |
| `target_speed` | `f64` | `10.0` | Desired cruise speed (m/s), used by planners' speed control and by the [`progress`](../metrics/README.md) and [`speed_limit`](../metrics/README.md) metrics as the reference speed limit. |
| `map` | `MapData` | see below | Cosmetic/road-geometry data — see [MapData](#mapdata). |

### `Actor`

```rust
pub struct Actor {
    pub init: State,
    pub control: Control,             // defaults to Control::default() (drives straight)
    pub trajectory: Vec<Waypoint>,     // defaults to []; overrides control when non-empty
}
```

| Field | Type | Default | Notes |
|---|---|---|---|
| `init` | `State` | required | The actor's state at scenario time `0`. |
| `control` | `Control` | `{accel: 0, curvature: 0}` | Constant control the actor drives under, if `trajectory` is empty. |
| `trajectory` | `Vec<Waypoint>` | `[]` | A logged path to replay instead of integrating `control`. Must be sorted by `t`. See [Trajectory replay](#trajectory-replay). |

### `Waypoint`

```rust
pub struct Waypoint {
    pub t: f64,       // seconds since scenario start
    #[serde(flatten)]
    pub state: State, // x, y, yaw, speed — flattened into the same JSON object as t
}
```

`#[serde(flatten)]` means a waypoint serializes as one flat JSON object —
`{"t": 0.5, "x": 32.5, "y": 0.0, "yaw": 0.0, "speed": 5.0}` — not a nested
`state` object.

### `MapData`

```rust
pub struct MapData {
    pub road_half_width: f64,     // default 5.5 — matches the drivable-area metric's threshold
    pub divider_d: Option<f64>,   // Some(offset) draws a dashed lane divider at that Frenet offset
    pub crosswalk_s: Vec<f64>,    // stations along the centerline where a crosswalk band is drawn
}
```

This is purely descriptive: the viewer draws it (road boundary lines, dashed
divider, crosswalk stripes), and `road_half_width` happens to match the
constant the [`drivable_area`](../metrics/README.md) metric uses — but
metrics read their own constant, not this field, so changing `road_half_width`
in a scenario file does not change what counts as off-road for scoring
purposes. If you need that coupled, the metric constant is the one to edit.

## `Path`, the Frenet helper

```rust
pub struct Path { /* polyline + cumulative arc length */ }

impl Path {
    pub fn new(pts: &[[f64; 2]]) -> Self;
    pub fn length(&self) -> f64;
    pub fn pose_at(&self, s: f64) -> ([f64; 2], f64);      // position + heading at arc length s
    pub fn project(&self, p: [f64; 2]) -> (f64, f64);      // xy -> (arc length, signed lateral offset)
    pub fn frenet_to_xy(&self, s: f64, d: f64) -> [f64; 2]; // inverse of project
}
```

Built once from a `centerline` and reused by everything that needs to
reason about "where along the lane" and "how far off to the side": every
planner that steers relative to the road ([`bezier_idm`](../planning/README.md#bezier--idm),
[`lattice`](../planning/README.md#frenet-lattice),
[`pi2ddp`](../planning/README.md#pi2-ddp)), every event-driven
[metric](../metrics/README.md) that needs a lateral offset or a station
(collisions don't, but `drivable_area`, `driving_direction`, and `progress`
do), and the viewer's `draw_map` function for rendering road boundaries that
follow curved roads.

`project`'s sign convention: **positive lateral offset is left of the path**
(found via the cross product of the segment direction and the offset
vector). `pose_at` and `project` both clamp to the path's extent rather than
extrapolating or panicking past its ends.

## Actor motion

`Actor::trace(steps, dt) -> Vec<State>` produces the actor's state at every
simulation tick, one of two ways:

- **No trajectory** (the common/default case, and what every hand-authored
  built-in scenario in `src/viewer/scenarios.rs` and every `synthetic_batch` scenario
  uses): integrate `init` under the constant `control` with
  [`step()`](../simulation/README.md#the-kinematic-model), the same Euler
  step the ego uses.
- **Non-empty trajectory**: replay the logged waypoints (see below). The
  presence of any waypoints overrides `control` entirely — there's no
  partial mixing.

### Trajectory replay

```rust
fn replay(trajectory: &[Waypoint], t: f64) -> State
```

Three regimes, in order:

1. **Before the log starts** (`t <= trajectory[0].t`): hold the first
   waypoint's state.
2. **Within the log**: linear interpolation between the two bracketing
   waypoints, with **shortest-arc interpolation for `yaw`**
   (`wrap_angle(b.yaw - a.yaw) * u`, not a naive linear blend, which would
   go the long way around when a heading crosses ±π).
3. **After the log ends** (`t >= trajectory.last().t`): constant-velocity
   extrapolation from the last waypoint's state — the same model the
   planners themselves use to predict actors.

This is why trajectory replay is worth having at all, rather than just
constant-velocity extrapolation everywhere: an actor can **brake, swerve, or
cut in** exactly as logged, while a *planner's own prediction* of that actor
(built only from `Context::actors`, the current-tick state) is still just
constant-velocity — so a replayed scenario can genuinely test how a planner
degrades when reality diverges from its prediction. The bundled
[`cut_in.json`](../../scenarios/json/cut_in.json) scenario and the future
preview's dimmer "predicted" ghost car in the viewer make this gap visible.

## Loading and generating scenarios

### `load_dir` and `load_path`

```rust
pub fn load_dir(dir: &Path) -> std::io::Result<Vec<Scenario>>
pub fn load_path(path: &Path) -> std::io::Result<Vec<Scenario>>
```

`load_dir` reads every `*.json` file directly in `dir` (not recursive),
sorted by file name, deserializing each with `serde_json`. A malformed file
surfaces as an `io::Error` naming the offending path, not a panic. Used by
the batch runner's `--dir` flag.

`load_path` is the same, generalized to accept either a directory (delegates
to `load_dir`) or a single scenario file. It's what the viewer uses for both
its CLI-argument scenario sources and its in-app "nuPlan path" loading
widget (desktop only — see
[`docs/USAGE.md`](../../docs/USAGE.md#scenario-sources)).

### Synthetic generation

```rust
pub fn synthetic_batch(count: usize, seed: u64) -> Vec<Scenario>
```

Generates `count` scenarios deterministically from `seed` (same seed + count
always produces byte-identical scenarios — see
`synthetic_batch_is_deterministic`), standing in for real nuPlan logs when
you want a large batch without exporting one. Cycles through four families
by index (`i % 4`):

| `i % 4` | Family | Actor placed |
|---|---|---|
| 0 | `lead` | Ahead in-lane, station `[25, 80)` m, speed `[0, 6)` m/s |
| 1 | `oncoming` | Opposing lane, station `[120, 200)` m, facing backward, speed `[4, 10)` m/s |
| 2 | `crossing` | Perpendicular approach from off-road, speed `[3, 8)` m/s |
| 3 | `open` | No actors |

Every scenario also gets a randomized sinusoidal centerline (amplitude
`[0, 10)` m, wavelength `[60, 140)` m) and a randomized ego starting speed
and lateral offset, so a batch exercises curvature-tracking as well as the
actor-response families above. Randomness comes from the crate-level `Rng`
(deterministic xorshift* + Box-Muller, defined in `lib.rs` — chosen over
pulling in the `rand` crate specifically so batches and their expected
scores are reproducible byte-for-byte across machines and CI runs).

## nuPlan export mapping

`tools/export_nuplan_scenarios.py` (documented for *usage* in
[`docs/USAGE.md`](../../docs/USAGE.md#exporting-real-nuplan-scenarios))
converts nuPlan log rows into this JSON format. What maps to what, and why:

| nanoplan field | nuPlan source | Rationale |
|---|---|---|
| `centerline` | The expert's own driven route over the horizon, downsampled to ≥2 m spacing | nuPlan's actual lane/map geometry isn't in the log database (see [`scenarios/nuplan/nuplan_schema.md`](../../scenarios/nuplan/nuplan_schema.md)) — only the map name is. The driven route is the closest available stand-in for "the lane," and it's what nuPlan's own metrics do too (see `Ego progress along the expert's route ratio` in [`metrics_description.md`](../../scenarios/nuplan/metrics_description.md)). |
| `ego` | The ego pose/velocity at the tagged scenario's anchor frame | Direct mapping; yaw comes from the ego pose quaternion. |
| `actors[].init` / `.trajectory` | Every `vehicle`-category tracked box present at the anchor frame, followed across the horizon | Real logged trajectories, not scripted — this is the whole point of replay (see [above](#trajectory-replay)). Downsampled to the simulation's 10 Hz tick rate. |
| `target_speed` | 85th percentile of the expert's speed over the horizon | A single scalar target; nuPlan's actual speed limit isn't in the log database either, so the expert's own typical cruising speed stands in. |
| coordinates (`ego`, `centerline`, actor positions) | Translated to a local origin at the ego's anchor pose | nuPlan logs use global map coordinates (hundreds of kilometers from an arbitrary map origin); leaving them un-translated would put every position outside `f32` precision the moment it hits the viewer's Bevy transforms, causing visible jitter. |

The exporter is written strictly against the vendored schema doc and has
**not been run against a real nuPlan log** in this repository (the dataset
requires registration); treat a first run against your own log file as a
shakedown of the query logic, not a guarantee.

## Testing

`frenet_roundtrip` checks `project`/`frenet_to_xy` are inverses on a simple
straight path. `replay_interpolates_and_extrapolates` checks all three replay
regimes at once (interpolated midpoint, extrapolated tail) against a
two-waypoint trajectory. `scenario_json_round_trip_with_defaults` and
`trajectory_json_round_trip` check the serde contracts — that a minimal JSON
file gets sane defaults, and that a full round trip through `serde_json`
preserves a trajectory exactly.
