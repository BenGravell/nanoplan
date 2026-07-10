//! Global ego vehicle capability and resistance constants.

/// Strongest requested forward acceleration before rolling/aero losses, m/s2.
/// Traction/engine limited: a typical passenger car reaches 100 km/h
/// (27.8 m/s) in ~7-11 s, with a higher launch peak; 4.0 (~0.41 g) is a
/// representative peak for a brisk passenger car.
pub const MAX_LON_ACCEL: f64 = 4.0;
/// Hardest braking deceleration, m/s2. Tyre-grip limited on dry asphalt with
/// ABS; -9.0 (~0.9 g) is a conservative dry-road capability, consistent with
/// the lateral grip limit.
pub const MIN_LON_ACCEL: f64 = -9.0;
/// Simulator-internal longitudinal jerk capability, m/s3.
/// Deliberately permissive: this is a plant guard rail, not a planner model.
pub(crate) const MAX_ABS_LON_JERK: f64 = 80.0;
/// Tightest steer the plant will execute, a ~5 m turning radius.
/// Only binds at low speed; above it the lateral-grip cap is tighter.
pub const MAX_ABS_CURVATURE: f64 = 0.2;
/// Lateral-acceleration (tyre-grip) limit, m/s2.
pub const MAX_ABS_LAT_ACCEL: f64 = 9.0;
/// Simulator-internal curvature-rate capability, in 1/(m*s).
/// Deliberately permissive: this is a plant guard rail, not a planner model.
pub(crate) const MAX_ABS_CURVATURE_RATE: f64 = 2.0;

/// Rolling-resistance coefficient for ordinary passenger-car tyres.
pub const ROLLING_RESISTANCE_COEFF: f64 = 0.012;
/// Effective ego mass used for aerodynamic drag.
pub const EGO_MASS_KG: f64 = 2_000.0;
/// Sea-level air density.
pub const AIR_DENSITY_KG_M3: f64 = 1.225;
/// Effective drag area, Cd*A, for a blunt minivan/crossover-sized ego.
pub const DRAG_AREA_M2: f64 = 0.9;
pub(crate) const GRAVITY_MS2: f64 = 9.81;
