//! Generic production sports-car capability and resistance constants.
//!
//! The 2017 Ford GT is the primary calibration point, softened into a generic
//! track-focused road car.
//! c.f. <https://fastestlaps.com/models/ford-gt-2017>

/// 2017 Ford GT exterior dimensions, m.
pub(crate) const BODY_LENGTH_M: f64 = 4.779;
pub(crate) const BODY_WIDTH_M: f64 = 2.003;
pub(crate) const WHEELBASE_M: f32 = 2.710;
pub(crate) const FRONT_TRACK_M: f32 = 1.694;
pub(crate) const REAR_TRACK_M: f32 = 1.662;

/// 245/35 R20 front and 325/30 R20 rear tire dimensions, m.
pub(crate) const FRONT_TIRE_DIAMETER_M: f32 = 0.6795;
pub(crate) const FRONT_TIRE_WIDTH_M: f32 = 0.245;
pub(crate) const REAR_TIRE_DIAMETER_M: f32 = 0.703;
pub(crate) const REAR_TIRE_WIDTH_M: f32 = 0.325;

/// Strongest requested forward acceleration before rolling/aero losses, m/s2.
pub(crate) const MAX_LON_ACCEL: f64 = 6.5;
/// Hardest braking deceleration, m/s2.
pub(crate) const MIN_LON_ACCEL: f64 = -9.0;
/// Tightest steer the plant will execute, a 5 m turning radius.
pub(crate) const MAX_ABS_CURVATURE: f64 = 0.2;
/// Lateral-acceleration limit, m/s2.
pub(crate) const MAX_ABS_LAT_ACCEL: f64 = 11.0;

/// Rolling-resistance coefficient for warm performance tires or racing slicks.
pub(crate) const ROLLING_RESISTANCE_COEFF: f64 = 0.012;
/// Effective mass matching a tested 2017 Ford GT with fluids and driver.
pub(crate) const EGO_MASS_KG: f64 = 1_480.0;
/// Sea-level air density.
pub(crate) const AIR_DENSITY_KG_M3: f64 = 1.225;
/// Effective high-speed resistance area. This is larger than is physically accurate
/// because the constant-thrust model has no gears or power falloff; it yields
/// the Ford GT's 347 km/h claimed top speed.
pub(crate) const DRAG_AREA_M2: f64 = 1.66;
pub(crate) const GRAVITY_MS2: f64 = 9.81;
