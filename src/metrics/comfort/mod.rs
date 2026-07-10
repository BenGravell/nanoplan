//! Comfort: 1 while every kinematic quantity at this tick is within nuPlan's
//! empirical expert bounds. Smooth — aggregates by average.

use crate::math::wrap_angle;
use crate::metrics::TickCtx;
use crate::simulation::State;

/// Comfort at tick `i`: reads the [`Kinematics`] precomputed for the whole
/// rollout (forward differences need the neighboring ticks, so they can't
/// be built tickwise).
pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    ctx.kinematics.score(i)
}

// comfort thresholds (empirical expert bounds from nuPlan). The longitudinal
// and lateral accel bounds are also shared with the planners' cost function
// (`planning::cost`) so its comfort term penalizes exactly what this metric
// scores as uncomfortable.
pub(crate) const MIN_LON_ACCEL: f64 = -4.05;
pub(crate) const MAX_LON_ACCEL: f64 = 2.40;
pub(crate) const MAX_ABS_LAT_ACCEL: f64 = 4.89;
const MAX_ABS_YAW_RATE: f64 = 0.95;

/// Forward difference, padded by repeating the last value so the result has
/// the same length as the input.
fn diff(v: &[f64], dt: f64) -> Vec<f64> {
    let mut d: Vec<f64> = v.windows(2).map(|w| (w[1] - w[0]) / dt).collect();
    d.push(d.last().copied().unwrap_or(0.0));
    d
}

/// Tickwise kinematics of an ego trace (forward differences, padded).
pub struct Kinematics {
    lon_accel: Vec<f64>,
    lat_accel: Vec<f64>,
    yaw_rate: Vec<f64>,
}

impl Kinematics {
    pub fn new(ego: &[State], dt: f64) -> Self {
        let speed: Vec<f64> = ego.iter().map(|s| s.speed).collect();
        let lon_accel = diff(&speed, dt);
        let yaw_rate = {
            let mut r: Vec<f64> = ego
                .windows(2)
                .map(|w| wrap_angle(w[1].yaw - w[0].yaw) / dt)
                .collect();
            r.push(r.last().copied().unwrap_or(0.0));
            r
        };
        let lat_accel: Vec<f64> = yaw_rate.iter().zip(&speed).map(|(r, v)| r * v).collect();
        Kinematics {
            lon_accel,
            lat_accel,
            yaw_rate,
        }
    }

    pub fn score(&self, i: usize) -> f64 {
        if (MIN_LON_ACCEL..=MAX_LON_ACCEL).contains(&self.lon_accel[i])
            && self.lat_accel[i].abs() <= MAX_ABS_LAT_ACCEL
            && self.yaw_rate[i].abs() <= MAX_ABS_YAW_RATE
        {
            1.0
        } else {
            0.0
        }
    }
}
