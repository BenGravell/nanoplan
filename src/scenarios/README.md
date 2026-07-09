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
> [`scenarios/commonroad/`](../../scenarios/commonroad/) holds the CommonRoad
> XML scenario corpus this repo ships, [`scenarios/nuplan/`](../../scenarios/nuplan/)
> holds vendored nuPlan reference documents (schema, vehicle parameters,
> metric definitions), `scenarios/json/` holds two converted CommonRoad
> scenarios bundled into the viewer binary at compile time, and
> `scenarios/web/` (not checked in) is the staging directory
> `tools/bundle_web_scenarios.py` reads to build `scenarios/web_bundle.json`,
> fetched by the web build at startup — see
> [docs/USAGE.md#scenario-sources](../../docs/USAGE.md#scenario-sources).
> None of these contain code.

## `Scenario`, the JSON format

```rust
pub struct Scenario {
    pub name: String,
    pub ego: State,
    pub actors: Vec<Actor>,       // defaults to empty
    pub centerline: Vec<[f64; 2]>,
    pub target_speed: f64,        // defaults to 10.0
    pub map: MapData,             // defaults to MapData::default()
    pub expert: Vec<Waypoint>,    // defaults to empty
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
map data (this is a trimmed version of
[`scenarios/json/braking_lead.json`](../../scenarios/json/braking_lead.json),
which `tools/export_commonroad_scenarios.py` converted from
[`scenarios/commonroad/ZAM_BrakingLead-1_1_T-1.xml`](../../scenarios/commonroad/ZAM_BrakingLead-1_1_T-1.xml)):

```json
{
  "name": "ZAM_BrakingLead-1_1_T-1",
  "ego": { "x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
  "actors": [
    {
      "init": { "x": 40.0, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
      "trajectory": [
        { "t": 0.0, "x": 40.0, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
        { "t": 0.1, "x": 40.8, "y": 0.0, "yaw": 0.0, "speed": 8.0 },
        { "t": 6.0, "x": 71.6, "y": 0.0, "yaw": 0.0, "speed": 0.0 }
      ]
    }
  ],
  "centerline": [[-50.0, 0.0], [-45.0, 0.0], [400.0, 0.0]],
  "target_speed": 10.0,
  "map": { "road_half_width": 5.5, "divider_d": null, "crosswalk_s": [], "cross_streets": [] }
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
| `expert` | `Vec<Waypoint>` | `[]` | The expert (human) ego trajectory logged over the same horizon, when the scenario comes from a real log. Not used in simulation — the ego is planned, not replayed — but it is the demonstration data the cost-weight autotuner learns from (see [src/tuning/README.md](../tuning/README.md)). |

### `Actor`

```rust
pub struct Actor {
    pub init: State,
    pub control: Control,             // defaults to Control::default() (holds actuators)
    pub trajectory: Vec<Waypoint>,     // defaults to []; overrides control when non-empty
}
```

| Field | Type | Default | Notes |
|---|---|---|---|
| `init` | `State` | required | The actor's state at scenario time `0`. |
| `control` | `Control` | `{acceleration: 0, curvature: 0}` | Constant command the actor drives under from `init`, if `trajectory` is empty. Actor tracing uses the simulator's private command limiter before stepping. |
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
    pub road_half_width: f64,      // default 5.5 — the drivable half-width the planner and metric use
    pub divider_d: Option<f64>,    // Some(offset) draws a dashed lane divider at that Frenet offset
    pub crosswalk_s: Vec<f64>,     // stations along the centerline where a crosswalk band is drawn
    pub cross_streets: Vec<f64>,   // stations where a perpendicular street is drawn crossing the road
}
```

The viewer draws it (road boundary lines, dashed divider, crosswalk stripes,
cross streets), and `road_half_width` is also the *actual* drivable
half-width the run enforces: `Scenario::road` copies it into
[`Road::half_width`](#road-the-fixed-setting-of-a-run), which the
[`drivable_area`](../metrics/README.md) metric scores against and every
planner's shared cost function rejects trajectories outside of. So a
narrower `road_half_width` genuinely tightens what counts as off-road, for
both scoring and planning, on that scenario. The `ROAD_HALF_WIDTH_M = 5.5`
constant in `metrics::drivable_area` is now only the *default* used when a
scenario doesn't set its own (and `MapData::default`'s value).

`cross_streets` exists because the `Scenario` format has only ever modeled a
single road (`centerline`) — an intersection/crossing actor's trajectory
takes it well off that road (e.g. approaching from `d = ±60`), and with
nothing else drawn, it looks like it's driving through empty space. Each
station in `cross_streets` draws a straight perpendicular road through the
main one, purely as a visual explanation of where such an actor is coming
from — it isn't itself a lane the planner or metrics know anything about.

## `Road`, the fixed setting of a run

```rust
pub struct Road {
    pub centerline: Vec<[f64; 2]>, // route reference path
    pub target_speed: f64,         // cruise speed; doubles as the metrics' speed limit
    pub half_width: f64,           // drivable half-width; the off-road bound planner + metric enforce
    pub dt: f64,                   // tick length everything is sampled at
}

impl Scenario {
    pub fn road(&self, dt: f64) -> Road; // one run's Road at tick length dt
}
```

Not part of the JSON format — a `Road` is derived per *run*, pairing the
scenario's `centerline`/`target_speed`/`map.road_half_width` with the
caller-chosen `dt` (see
[`src/simulation/README.md`](../simulation/README.md#why-this-design) for
why `dt` belongs to the experiment, not the scenario). These three values
always travel together — the planning `Context` embeds a `&Road`, the
simulator holds one for the whole run, and `metrics::evaluate` scores
against one — so they move as a single parameter object instead of a
recurring three-argument list.

## `Path`, the Frenet helper

```rust
pub struct Path { /* polyline + cumulative arc length */ }

impl Path {
    pub fn new(pts: &[[f64; 2]]) -> Self;
    pub fn length(&self) -> f64;
    pub fn pose_at(&self, s: f64) -> ([f64; 2], f64);      // position + heading at arc length s
    pub fn project(&self, p: [f64; 2]) -> (f64, f64);      // xy -> (arc length, signed lateral offset)
    pub fn project_near(&self, p: [f64; 2], s_hint: f64, window_m: f64) -> (f64, f64); // windowed project
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

`project_near` is `project` restricted to the centerline segments within
`window_m` arc length of a caller-supplied `s_hint`, so it scans a handful of
segments instead of all of them. It's exact whenever the nearest segment is
inside the window (the caller sizes the window generously). RRT* uses it in
its hot loop — it projects every sampled point of every candidate edge and
already knows each segment's rough station — where the full `O(n)` scan was
the dominant cost once its spatial index removed the linear neighbor scans
(see [`rrt_star`](../planning/README.md#rrt)). A
`project_near_matches_full_projection_within_window` test pins the exactness.

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
   extrapolation from the last waypoint's state.

This is why trajectory replay is worth having at all, rather than just
constant-velocity extrapolation everywhere: an actor can **brake, swerve, or
cut in** exactly as logged, while a *planner's own prediction* of that actor
(built only from `Context::actors`, the current-tick state, via
`metrics::predict`) at best follows the lane and eases back to its center —
so a replayed scenario can genuinely test how a planner degrades when reality
diverges from its prediction. The bundled
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
its CLI-argument scenario sources and its in-app "scenario path" loading
widget (desktop only — see
[`docs/USAGE.md`](../../docs/USAGE.md#scenario-sources)).

### Synthetic generation

```rust
pub fn synthetic_batch(count: usize, seed: u64) -> Vec<Scenario>
```

Generates `count` scenarios deterministically from `seed` (same seed + count
always produces byte-identical scenarios — see
`synthetic_batch_is_deterministic`), supplementing the checked-in CommonRoad
corpus when you want a large batch without converting one. Cycles through
four families by index (`i % 4`):

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

A separate, much larger generator lives outside the crate:
[`tools/generate_diverse_scenarios.py`](../../tools/generate_diverse_scenarios.py)
writes the CommonRoad XML corpus in `scenarios/commonroad/` across ~20
scenario categories (turns, intersections, cut-ins, stop-and-go traffic,
and more), each obstacle with a fully scripted trajectory rather than a
constant `Control` — see
[docs/USAGE.md#generating-the-scenario-corpus](../../docs/USAGE.md#generating-the-scenario-corpus).
Converted, it's what populates `scenarios/web_bundle.json`;
`synthetic_batch` above is the batch runner's own generator and is
unrelated (Rust, not Python; used by `--count`, not the web build).

## CommonRoad export mapping

`tools/export_commonroad_scenarios.py` (documented for *usage* in
[`docs/USAGE.md`](../../docs/USAGE.md#converting-commonroad-scenarios))
converts CommonRoad 2020a XML into this JSON format. What maps to what, and
why:

| nanoplan field | CommonRoad source | Rationale |
|---|---|---|
| `name` | The `benchmarkID`, plus a `[fog]`-style suffix when the scenario declares environment conditions | Benchmark IDs are CommonRoad's own unique scenario names. |
| `ego` | The planning problem's `initialState` (position, orientation, velocity) | Direct mapping — CommonRoad separates the planned vehicle from the obstacles the same way nanoplan does. |
| `centerline` | Concatenated centers (midpoint of left/right bounds) of the lanelets on the route from the ego's lanelet to the goal, following `successor` edges | nanoplan models one reference route, not a full lanelet network; the successor chain to the goal is that route. Without a reachable position goal, the deepest successor chain stands in. |
| `actors[].init` / `.trajectory` | Every static and dynamic obstacle; dynamic ones carry their full state trajectory (time step × `timeStepSize`) | Logged trajectories, replayed exactly — see [Trajectory replay](#trajectory-replay). Static obstacles become speed-0 actors. |
| `target_speed` | Midpoint of the goal state's `velocity` interval, else the ego's initial speed | CommonRoad 2020a has no per-lanelet speed limit element (that moved to traffic signs); the goal velocity interval is the scenario's own statement of the intended speed. |
| `map.road_half_width` | Max lateral extent of the route lanelets' bounds, including one adjacent lanelet per side | The drawn road edge spans every lane, not just the ego's. |
| `map.divider_d` | Signed lateral offset of the bound shared with an `adjacentLeft`/`adjacentRight` lanelet | Where the viewer draws the dashed lane divider. |
| `map.crosswalk_s` / `map.cross_streets` | Stations (projected onto the route) of `crosswalk`-type lanelets, and of other roads whose centerline crosses the route at a steep angle | Purely descriptive markers the viewer draws — nanoplan's planner and metrics only know the single route. |

## nuPlan export mapping

`tools/export_nuplan_scenarios.py` (documented for *usage* in
[`docs/USAGE.md`](../../docs/USAGE.md#exporting-real-nuplan-scenarios-local-only);
exports stay local — nuPlan data is not redistributable) converts nuPlan
log rows into this JSON format. Its main remaining role is producing
`expert` trajectories for the cost-weight autotuner, which CommonRoad
scenarios don't carry. What maps to what, and why:

| nanoplan field | nuPlan source | Rationale |
|---|---|---|
| `centerline` | The expert's own driven route over the horizon, downsampled to ≥2 m spacing | nuPlan's actual lane/map geometry isn't in the log database (see [`scenarios/nuplan/nuplan_schema.md`](../../scenarios/nuplan/nuplan_schema.md)) — only the map name is. The driven route is the closest available stand-in for "the lane," and it's what nuPlan's own metrics do too (see `Ego progress along the expert's route ratio` in [`metrics_description.md`](../../scenarios/nuplan/metrics_description.md)). |
| `ego` | The ego pose/velocity at the tagged scenario's anchor frame | Direct mapping; yaw comes from the ego pose quaternion. |
| `actors[].init` / `.trajectory` | Every `vehicle`-category tracked box present at the anchor frame, followed across the horizon | Real logged trajectories, not scripted — this is the whole point of replay (see [above](#trajectory-replay)). Downsampled to the simulation's 10 Hz tick rate. |
| `target_speed` | 85th percentile of the expert's speed over the horizon | A single scalar target; nuPlan's actual speed limit isn't in the log database either, so the expert's own typical cruising speed stands in. |
| `expert` | The expert ego trajectory over the horizon, downsampled to the 10 Hz simulation rate | The human demonstration itself, kept alongside the derived `centerline`/`target_speed` so the cost-weight autotuner ([src/tuning/README.md](../tuning/README.md)) can learn from what the expert actually did, not just the route they took. |
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
