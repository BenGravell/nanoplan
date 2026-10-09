# CenterlineFollower

`centerline_follower/mod.rs` — `CenterlineFollower`

Samples merge distances along the road centerline.
Each candidate contains two cubic Bezier primitives in Frenet coordinates `(s, d)`: a curve from the ego offset and
heading to `d = 0` with zero lateral slope, followed by a centerline-following Bezier with `d = 0` throughout.
The primitive evaluates its four control points and analytic first and second derivatives.
There are no lateral passing targets or previous-plan candidates.

The first three merge distances use the full, half, and quarter intermediate reach.
Additional candidates bisect the remaining intervals; increasing the compute budget retains the same grid prefix.
The budget allows 3 paths at minimum, 9 at nominal, and 45 at maximum.
The intermediate reach is the midpoint of braking and acceleration reach over 5 seconds.
The continuation covers maximum acceleration over the 10-second horizon plus one control step, clipped to the road
window.
Reach calculations include rolling resistance and drag.

Before speed planning, both Beziers are sampled and transformed into Cartesian positions, headings, and curvatures using
the shared Frenet derivative transformation.
The 100-interval grid includes the merge point and approximates Cartesian arc length with chord lengths.
Road curvature and its derivative contribute to the transformed geometry, so the continuation follows road bends.
Invalid Frenet charts and degenerate Cartesian samples are discarded.

Each Cartesian path gets a scalar TOPP-RA speed profile: squared speed propagates backward through controllable
intervals, then forward under maximum acceleration.
Curvature, lateral grip, terminal speed, acceleration, braking, and resistance bound the profile.
The preview endpoint permits nonzero speed.
Actor predictions tighten the profile to stop before obstacles, and road-contact rollouts tighten speed limits to allow
slower traversal; refinement is capped at eight passes per path.
Control extraction tracks the timed speed profile with geometric curvature and shared lateral/heading feedback.

Every rollout is checked with the shared trajectory feasibility checker over at least the full 10-second horizon, even
when fewer controls are requested.
The lowest-cost feasible trajectory wins, using the shared progress metric.
If none is feasible, the planner returns braking controls that hold at standstill.
Diagnostics record each evaluated rollout, including rejected candidates.

**Seams**: `route` (project ego), `bezier_fit` (Frenet curves to Cartesian samples), `optimize` (TOPP-RA and bound
refinement), `extract` (profile to controls), and `cost` (full-rollout feasibility and shared objective).
