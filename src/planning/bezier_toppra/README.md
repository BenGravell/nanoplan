# Bezier + TOPP-RA

`bezier_toppra/mod.rs` — `BezierToppraPlanner`

Searches road-following paths with two independent lateral targets: an intermediate station and a terminal station.
Cubic lateral interpolation starts at the ego offset and heading, reaching each target with zero lateral slope.
The existing 100-interval grid supplies route-following anchors, joined by short cubic Bezier segments with shared
tangent directions.
Handles scale with each interval, avoiding curvature amplification at unequal grid spacing.
This retains intervening bends instead of cutting across them with two long world-space cubics.
Targets stay inside the local road bounds, with vehicle-width clearance.
The first three candidates return to the centerline over the full, half, and quarter intermediate distance.
Two more candidates explore left and right passing paths at half distance.
Remaining candidates follow a nested deterministic grid: hold an offset, return to center, or depart from center, at
half, quarter, and full intermediate distance.
The grid explores full-width offsets first, then half-width offsets, and finally opposite-side targets; each maneuver is
paired with its left/right mirror.
Increasing the budget extends the same grid prefix, avoiding a full Cartesian product of independent lateral targets.

The maximum intermediate distance uses the midpoint of the braking/acceleration reachable interval at 5 seconds.
The terminal station covers maximum acceleration over the full 10-second horizon, plus one control step of integration
margin.
Both bounds include rolling resistance and drag.
Stations are clipped to the available road window while retaining two distinct segments.
Every live planner now receives a road window sized to full acceleration reach; it also grows when increasing speed
would exhaust that window before the next normal road update.

The compute-budget slider caps new path evaluations: 5 at minimum budget, 9 at nominal, and 45 at maximum.
The five-candidate floor preserves centerline recovery and both passing directions even at minimum budget.
If every new candidate is infeasible, the remaining controls of the previous selected plan provide one additional
candidate without another TOPP-RA solve.
Alignment uses the nearest predicted state, allowing skipped live ticks; the suffix is extended with its last command
and checked over the entire horizon from the actual current ego against current road and actor constraints.
It is discarded if infeasible.
This preserves a safe continuation when a fresh sparse search misses it.

Each path gets its own scalar [TOPP-RA](https://arxiv.org/abs/1707.07239) speed profile: squared speed is propagated
backward through controllable intervals and forward under maximum acceleration.
Grid spacing uses sampled arc length along the curve chain.
Curvature, lateral grip, target speed, acceleration and braking limits bound the profile.
The preview endpoint permits nonzero speed; it does not introduce an artificial stop into an otherwise clear
acceleration path.

Extraction follows the profile's arrival-time speed ramp, with geometric curvature and the shared lateral/heading
feedback tracking the path.
Predicted actor footprints tighten the speed envelope to stop before obstacles.
When the actual vehicle rollout contacts a road barrier, refinement halves the squared-speed cap from just before
contact onward, allowing a slower traversal instead of immediately forcing a permanent stop.
Backward propagation applies the required braking before that point.
Refinement is capped at eight passes per path to bound the latency tail; the final feasibility check still rejects any
remaining violation.
Every final trajectory is checked over at least the full 10-second planning horizon, even when fewer controls are
requested.
Road checks reserve 10 cm at the sides and front to tolerate the boundary-sampling difference between a rolling window
and the full circuit after a lap wrap.
The shared progress cost ranks feasible trajectories; rectangular actor collisions and road-barrier contact reject a
candidate.
If none is feasible, the planner applies a braking fallback that holds at standstill.

**Seams**: `route` (project ego), `bezier_fit` (fit each road-following curve chain), `optimize` (TOPP-RA and bound
tightening), `extract` (path profile to controls), and `cost` (full-rollout feasibility and shared objective).
Diagnostics record one timed trajectory per candidate, including the previous-plan continuation and rejected candidates.

Historical browser calibration (before the road-following fit) at 100% budget (9 paths), using the optimized `web`
profile in a Chrome 153 WebAssembly worker on an Intel Xeon 6737P: 1,080 measured calls after warmup gave p99 **66.4
ms**, maximum **68.5 ms**, and no calls above 100 ms. Worker round-trip p99 was 67.0 ms. The corpus covered straight
roads, constant-radius bends and S-bends, ego speeds of 0–80 m/s, and 0, 5 or 15 actors, with diagnostics disabled.
These historical measurements are not a calibration of the current geometry; latency depends on browser, hardware and
workload.
