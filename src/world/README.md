# `world`

The interactive open world behind the viewer's **open world** mode: a
procedurally generated street map, routing to a user-placed goal, basic
IDM traffic actors, and `LiveWorld` — the realtime closed-loop stepper.
Unlike the scenario pipeline (`simulate()`/`Rollout`, which precomputes a
whole rollout and lets the viewer scrub it), `LiveWorld::tick` replans and
steps the ego *every tick* while it's being watched, judo/treetop style —
the plan on screen is the plan that was just executed.

Everything here is plain library code (no Bevy): the viewer's
`src/viewer/live.rs` is only pacing, mouse input, and drawing on top of it,
and the whole thing is exercised by ordinary `cargo test`.

## `StreetMap` — procedural street network

A jittered `6×6` grid of intersections (`GRID_SPACING_M` apart, ±22%
jitter), 4-connected, with ~25% of the streets randomly removed — each
removal is skipped if it would disconnect the graph, so every point stays
reachable. Deterministic in the seed. Streets are two-way: lane centers sit
`LANE_OFFSET_M` right of the road axis (right-hand traffic), and the road
spans `ROAD_HALF_WIDTH_M` each side of the axis — the same drivable-area
bound the metrics and shared cost function use.

`route(from, yaw, to)` snaps both endpoints to the network, runs Dijkstra
over the intersections (seeded with both ends of the start street), and
returns a *drivable lane centerline*: corners cut with a quadratic Bezier
of radius `CORNER_RADIUS_M` (a 90° kink is not a followable centerline),
then the whole polyline offset into the right-hand lane. Routes that would
start against `yaw` pay `U_TURN_PENALTY_M`, so the ego prefers rounding a
block over flipping in place — but a goal behind the ego is never
unreachable, just discouraged.

## `SmartActor` — basic smart traffic

Each actor wanders the map on a random node walk (extended street by
street as it's consumed, avoiding immediate backtracks), following its own
right-hand lane. Speed control is the same IDM the Bezier planner uses
(`planning::bezier_idm::idm_accel`), against the nearer of:

- the closest vehicle ahead **in its lane** (projected onto its path), or
- anything close **in front of its bumper** regardless of lane — the
  crossing/intersection guard, which is what makes actors yield to the ego
  and to each other at junctions.

"Smart" deliberately stops there: no lane changes, no traffic lights, no
negotiation. Actors are meant to be predictable-but-live traffic for the
ego's planners (which see them as plain `State`s and predict constant
velocity, same as everywhere else in nanoplan).

## `LiveWorld` — the realtime loop

Owns the map, the ego (a persistent planner instance — sampling planners
keep their warm starts across ticks), the actors, and the current
goal/route. Each `tick()`:

1. every actor steps against a snapshot of all traffic (ego included);
2. the ego replans on the current route `Road` and applies the first
   control — or, with no goal, brakes to a stop and waits;
3. the route's `target_speed` is tapered to `√(2·a·remaining)` near the
   goal so the ego *arrives stopped* instead of sailing through; once
   within a car length and nearly stopped, the goal is considered reached
   and cleared.

`set_goal()` re-routes from the ego's current pose at any time — including
mid-drive — and `plan`/`last_plan_ms` expose the just-computed plan and its
wall-clock cost for the viewer's overlay and readout. The planner runs
under the exact same `Planner`/`Context` interface as everywhere else;
nothing in `planning/` knows live mode exists.

## Testing

`cargo test --lib world::` covers: map determinism + connectivity, routes
staying on-road and reaching the snapped goal, actors cruising at their
target speed and stopping behind a blocker, the ego driving a `LiveWorld`
to a clicked goal and stopping there, and the goalless brake-to-stop.
