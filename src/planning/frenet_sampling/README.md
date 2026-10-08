# Frenet sampling planner

`frenet_sampling/mod.rs` — `FrenetSamplingPlanner`

Generates independent cubic polynomials for Frenet station `s(t)` and lateral offset `d(t)`, matching position and
velocity at both endpoints.
Fitting and evaluation reuse `common::polynomial::CubicPolynomial`, also used by Cartesian steering.
Vector operations, angle differences, and finite-position checks reuse the shared `common` helpers.
Terminal station and speed are sampled independently around a zero-thrust rollout, bounded by braking and acceleration
rollouts including resistance.
Squared spacing above the nominal rollout retains slow rolling candidates on tight bends.

At nominal budget there are 18 station samples, 10 speed samples, and 11 lateral offsets across the usable road width.
Each offset is crossed with lateral velocities −0.5, 0, and +0.5 m/s.
Each station/speed pair also includes a reflected lateral endpoint for stronger centerline recovery.
The three variable sample counts scale with the cube root of the compute budget; lateral counts stay odd to include the
centerline.

Below 2 m/s, a straight acceleration prefix establishes the vehicle heading.
Cubics start from the projected state after that prefix and use the remaining duration of `PLANNING_HORIZON_S`.
The sampling rollout uses that same remaining duration.

Frenet position, velocity, and acceleration are transformed into Cartesian state and controls using reference heading,
curvature, and curvature derivative.
Longitudinal commands compensate resistance.
Vehicle limits are rejection filters: commands are never clipped to make a sampled trajectory feasible.

Evaluation follows the [FRENETIX paper's Section III-B](https://arxiv.org/html/2402.01443v2#S3.SS2) order, with the
project's existing vehicle limits and progress objective:

1. Reject invalid Frenet transforms and kinodynamically infeasible commands.
   Cache the Cartesian vehicle-model rollout, aborting at the first invalid sample.
1. Assign surviving candidates their negative shared progress score using the realized terminal position.
   Sort by increasing cost, preserving sampling order for ties.
1. Check cached Cartesian rollouts against road boundaries, swept footprint barriers, and predicted actors in cost order.
   Return the first feasible candidate; do not collision-check the remaining candidates.

Collision checks cover the full planning horizon, independently of the requested control prefix.
Road checks retain footprint clearance at rolling-window seams, and actor checks include supplied poses at the first
step.
Reverse motion is rejected.
Diagnostics show all scored candidates that passed the kinodynamic filter, reusing their cached Cartesian rollouts.
Displayed candidates may still fail road or collision checks; only the selected plan has passed those checks.
Enabling diagnostics does not change selection or trigger additional collision checks.
Validated fallback rollouts are also recorded.

If no sampled candidate passes, the planner brakes along the centerline.
If that rollout is infeasible, it tries the remaining previous plan, revalidated against the current road and actors,
before falling back to braking.

Select **Frenet sampling** in the viewer or use `--planner frenet-sampling` with the profiler.
This is the viewer default.

**Seams:** `route`, `fit`, `kinodynamic`, `cost`, and `collision`.

## Sources

This implementation draws on these sources, using cubic polynomials and the project’s vehicle model and progress
objective:

- [FRENETIX: A High-Performance and Modular Motion Planning Framework for Autonomous Driving](https://arxiv.org/abs/2402.01443).
- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://ieeexplore.ieee.org/abstract/document/5509799).
