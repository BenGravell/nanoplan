# Bezier + TOPP-RA

`bezier_toppra/mod.rs` — `BezierToppraPlanner`

Steers back to the lane by fitting a cubic Bezier curve from the ego's
current pose to a lookahead point on the centerline. Speed uses the scalar
special case of [TOPP-RA](https://arxiv.org/abs/1707.07239): squared path
speed is propagated over a station grid by a backward controllable-set pass
and a maximum-acceleration forward pass. Commanded longitudinal acceleration,
geometric curvature, lateral grip, target speed, and predicted actor clearance
are hard bounds. Extraction adds the shared centerline feedback to the
geometric curvature, then rolls out the full vehicle footprint and tightens
the speed envelope until it stays between the road barriers.

**Seams**: `route` (build the `Path`, project the ego), `bezier_fit` (compute
the four Bezier control points), `optimize` (TOPP-RA backward/forward passes
and collision-bound tightening), and `extract` (convert the path profile to
controls).

Because path parameterization cannot steer around an obstacle, predicted
collision occupancy imposes a zero-speed station and the backward pass builds
the braking profile needed to stop before it. The collision bound uses the
shared lane-aware actor prediction and therefore also covers crossing traffic.
