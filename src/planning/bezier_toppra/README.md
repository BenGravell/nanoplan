# Bezier + TOPP-RA

`bezier_toppra/mod.rs` — `BezierToppraPlanner`

Searches paths made from exactly two cubic Bezier segments: ego to an intermediate station, then to a terminal station.
The segments share their joining position and tangent.
Each station independently samples lateral offsets inside the local road bounds, with vehicle-width clearance.
Zero offset is always included at both stations, and the centerline candidate is evaluated first.

The intermediate station uses the midpoint of the braking/acceleration reachable interval at 5 seconds.
The terminal station covers maximum acceleration over the full 10-second horizon, plus one control step of integration
margin.
Both bounds include rolling resistance and drag.
Stations are clipped to the available road window while retaining two distinct segments.
Every live planner now receives a road window sized to full acceleration reach; it also grows when increasing speed
would exhaust that window before the next normal road update.

The compute-budget slider scales the lateral grid's total path allowance.
The largest odd number of offsets whose Cartesian product fits that allowance is used: 1 path at minimum budget, 9 at
nominal, and 25 at maximum, on a road wide enough for offsets on both sides.

Each path gets its own scalar [TOPP-RA](https://arxiv.org/abs/1707.07239) speed profile: squared speed is propagated
backward through controllable intervals and forward under maximum acceleration.
Grid spacing uses sampled arc length along both curves.
Curvature, lateral grip, target speed, acceleration and braking limits bound the profile.
The preview endpoint permits nonzero speed; it does not introduce an artificial stop into an otherwise clear
acceleration path.

Extraction follows the profile's arrival-time speed ramp, with geometric curvature and the shared lateral/heading
feedback tracking the path.
Predicted actor footprints and actual vehicle rollouts tighten the speed envelope to stop before obstacles or road
barriers.
Refinement is capped at eight passes per path to bound the latency tail; the final feasibility check still rejects any
remaining violation.
Every final trajectory is checked over at least the full 10-second planning horizon, even when fewer controls are
requested.
The shared progress/comfort cost ranks feasible trajectories; rectangular actor collisions and road-barrier contact
reject a candidate.
If none is feasible, the planner applies a braking fallback that holds at standstill.

**Seams**: `route` (project ego), `bezier_fit` (fit and sample each two-segment path), `optimize` (TOPP-RA and bound
tightening), `extract` (path profile to controls), and `cost` (full-rollout feasibility and shared objective).
Diagnostics record one timed trajectory per path, including rejected candidates.

Browser calibration at 100% budget (9 paths), using the optimized `web` profile in a Chrome 153 WebAssembly worker on an
Intel Xeon 6737P: 1,080 measured calls after warmup gave p99 **66.4 ms**, maximum **68.5 ms**, and no calls above 100
ms. Worker round-trip p99 was 67.0 ms. The corpus covered straight roads, constant-radius bends and S-bends, ego speeds
of 0–80 m/s, and 0, 5 or 15 actors, with diagnostics disabled.
These measurements leave headroom for the 100 ms allowance on that machine; latency depends on browser, hardware and
workload.
