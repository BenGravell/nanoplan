# FRENETIX polynomial planner

Adapts the velocity-sampling scheme of Section III.B of the [FRENETIX paper](https://arxiv.org/pdf/2402.01443) and
[the reference implementation](https://github.com/TUM-AVS/Frenetix-Motion-Planner/blob/main/frenetix_motion_planner/polynomial_trajectory.py).

Lateral cubics match initial displacement and velocity and finish at a sampled offset and lateral speed.
Terminal lateral speeds are limited to −0.5, 0, and +0.5 m/s.
Longitudinal cubics match initial position and velocity and finish at a sampled position and speed.
Terminal states form a Cartesian grid with 11 evenly spaced values per axis at nominal compute budget: longitudinal
position ahead of the current station, lateral offset across the usable road width, and longitudinal speed from zero to
the road's target speed.
The station range includes acceleration over the horizon: `speed * T + 0.5 * MAX_LON_ACCEL * T²`, capped at the end of
the available road.
The live world supplies FRENETIX with the existing speed-dependent road window, sized for maximum acceleration with
drag, instead of the fixed 250 m window.
Each combination is crossed with all three terminal lateral speeds.
Neither fit constrains initial or terminal acceleration: acceleration follows from the position/velocity conditions.
This reduces the paper's quintic/quartic degrees and avoids restarting an acceleration ramp at every replan.
Every polynomial ends at `PLANNING_HORIZON_S`.
Per-axis density scales with the cube root of the compute budget, rounded to whole samples, so the total trajectory
count is approximately proportional to the budget.
The three terminal lateral speeds stay fixed.
Position and speed are sampled independently.

The existing four-dimensional vehicle state supplies initial Frenet position and velocity.
Reference curvature is estimated from the shared polyline.
Polynomial derivatives produce acceleration and curvature commands, with drag compensation and the existing vehicle
limits.
Each candidate is rolled out through `world_step` and ranked by the existing `HardConstraints::point_cost`, including
predicted actors, local road bounds, progress, and comfort.
The returned controls are truncated to `Context::horizon`; evaluation always covers `PLANNING_HORIZON_S`.
If every candidate is rejected, the existing stopping controls are returned.

Select **FRENETIX** in the viewer or use `--planner frenetix` with the profiler.
This implements polynomial motion generation only: no CommonRoad integration, upstream cost weights, risk assessment, or
other FRENETIX modules.
