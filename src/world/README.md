# `world`

The interactive open world behind the viewer's **open world** mode: an
*infinite* procedurally generated street network, chunked Minecraft-style
around the ego, routing to a user-placed goal, mixed IDM traffic (cars,
trucks, bikes, pedestrians), and `LiveWorld` — the realtime closed-loop
stepper. Unlike the scenario pipeline (`simulate()`/`Rollout`, which
precomputes a whole rollout and lets the viewer scrub it), `LiveWorld::tick`
replans and steps the ego *every tick* while it's being watched,
judo/treetop style — the plan on screen is the plan that was just executed.

Everything here is plain library code (no Bevy): the viewer's
`src/viewer/live.rs` is only pacing, mouse input, and drawing on top of it,
and the whole thing is exercised by ordinary `cargo test`.

## Procedural generation — a pure function of the seed

The street network is defined over the infinite grid of coordinates
`[i64; 2]` by pure functions of `(seed, coords)` — hashes plus a Perlin
noise field — so any part of the world can be queried at any time and
always answers the same. Nothing is ever "generated and stored"; chunks are
just materializations.

- **Nodes** sit on a `GRID_SPACING_M` lattice with ±22% hashed jitter.
- An **urbanization** Perlin field (~350 m scale) makes downtown areas
  dense and semi-rural areas sparse: it drives street density, lane
  counts, actor counts, and the traffic mix.
- **Arterials** are whole grid rows/columns (~1 in 4, hashed per line):
  multi-lane roads that run for blocks and are never dropped.
- Every node keeps one hashed **parent street** (west or south), which
  keeps the network connected without any global connectivity check; the
  remaining streets survive a density draw weighted by urbanization.
- **Lanes per direction** (`edge_lanes`): locals are 1 (sometimes 2
  downtown), arterials 2 — with occasional per-block promotions (3) and
  demotions (1), so roads gain and lose lanes along their length and lane
  topology shifts across junctions. A street's half-width is
  `lanes × LANE_W_M`.

## `StreetMap` — the active window

`StreetMap::window(seed, center)` materializes the 3×3 chunks
(`CHUNK_NODES`² grid nodes each) around a center chunk: node positions,
edges, per-edge lane counts, and adjacency, for drawing, snapping, and
routing. Windows over the same seed always agree wherever they overlap.

`route(from, yaw, to)` snaps both endpoints to the network, runs Dijkstra
over the loaded intersections (seeded with both ends of the start street),
and returns a *drivable lane centerline*: corners cut with a quadratic
Bezier of radius `CORNER_RADIUS_M`, then each leg offset into a chosen
lane — the rightmost by default, the innermost (turning) lane on
multi-lane approaches to a left turn. Offsets blend across corners, so
lane gains/losses taper smoothly and junction turns come out as lane
connectors from the departure lane to the arrival lane. Routes that would
start against `yaw` pay `U_TURN_PENALTY_M`, so the ego prefers rounding a
block over flipping in place.

## `SmartActor` — mixed traffic

Each actor wanders the *infinite* map on a random node walk (extended
street by street as it's consumed, via the pure adjacency — no window
needed), following its own right-hand corridor by `ActorKind`:

- **Car / Truck**: lane driving with the same turn-lane choice as the ego's
  routes; trucks are longer and slower.
- **Bike**: hugs the curb (1 m inside the road edge), slower still.
- **Pedestrian**: walks the sidewalk just outside the roadway at walking
  speed.

Speed control is the same IDM the Bezier planner uses
(`planning::bezier_idm::idm_accel`), against the nearer of: the closest
road user ahead **in its corridor**, or anything close **in front of its
bumper** regardless of lane — the crossing/intersection guard, which is
what makes actors yield to the ego and to each other at junctions. No lane
changes, no traffic lights, no negotiation: predictable-but-live traffic
for the ego's planners (which see them as plain `State`s and predict
constant velocity, same as everywhere else in nanoplan).

## Chunking — near-infinite driving

`LiveWorld` keeps the window centered on the ego with a full ring of
preloaded buffer chunks around it, so the drivable world is effectively
infinite:

- **Recentering** happens once the ego is `RECENTER_HYST_M` past the
  center chunk's bounds (spatial hysteresis: cruising along a chunk line
  doesn't thrash regeneration).
- **Spawning**: each chunk owns a deterministic traffic set
  (`spawn_chunk`, a pure function of `(seed, chunk)`); a chunk is
  populated when it enters the active window and has no actors of its own
  alive, so revisiting a chunk whose traffic still exists never
  double-spawns. A global `max_actors` cap bounds the whole world.
- **Despawning**: an actor that stays outside the active bounds (plus
  `DESPAWN_MARGIN_M`) for `DESPAWN_GRACE_S` is dropped — the temporal
  hysteresis that keeps traffic from flickering when the ego darts out of
  and back into a chunk.

## `LiveWorld` — the realtime loop

Owns the window, the ego (a persistent planner instance — sampling
planners keep their warm starts across ticks), the actors, and the current
goal/route. Each `tick()`: the window recenters and traffic
spawns/despawns as above; every actor steps against a snapshot of all
traffic (ego included); the ego replans on the current route `Road` and
applies the first control — or, with no goal, brakes to a stop and waits.
The route's `target_speed` is tapered to `√(2·a·remaining)` near the goal
so the ego *arrives stopped*; once within a car length and nearly stopped,
the goal is cleared.

`set_goal()` re-routes from the ego's current pose at any time — including
mid-drive — and `plan`/`last_plan_ms` expose the just-computed plan and its
wall-clock cost for the viewer's overlay and readout. The planner runs
under the exact same `Planner`/`Context` interface as everywhere else;
nothing in `planning/` knows live mode exists.

## Testing

`cargo test --lib world::` covers: window determinism and seam agreement
between overlapping windows, road width and traffic-kind variety, routes
staying on-road and reaching the snapped goal, actors cruising at their
target speed and stopping behind a blocker, chunk-churn hysteresis (darting
across a chunk line neither despawns nor double-spawns traffic; leaving for
good prunes it), a long multi-seam drive staying numerically sane, driving
a `LiveWorld` to a clicked goal and stopping there, and the goalless
brake-to-stop.
