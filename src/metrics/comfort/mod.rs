//! Race-car comfort: a continuous score of longitudinal and lateral
//! acceleration. Vehicle limits are free; only nonlinear exceedance costs.

use crate::metrics::TickCtx;
use crate::simulation::State;
use crate::vehicle::{MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};

/// Comfort at tick `i`: reads the [`Kinematics`] precomputed for the whole
/// rollout (forward differences need the neighboring ticks, so they can't
/// be built tickwise).
pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    ctx.kinematics.score(i)
}

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
}

impl Kinematics {
    pub fn new(ego: &[State], dt: f64) -> Self {
        let speed: Vec<f64> = ego.iter().map(|s| s.speed).collect();
        let lon_accel = diff(&speed, dt);
        let mut lat_accel: Vec<f64> = ego
            .windows(2)
            .map(|w| {
                let v0 = [w[0].speed * w[0].yaw.cos(), w[0].speed * w[0].yaw.sin()];
                let v1 = [w[1].speed * w[1].yaw.cos(), w[1].speed * w[1].yaw.sin()];
                let dv = [(v1[0] - v0[0]) / dt, (v1[1] - v0[1]) / dt];
                -w[0].yaw.sin() * dv[0] + w[0].yaw.cos() * dv[1]
            })
            .collect();
        lat_accel.push(lat_accel.last().copied().unwrap_or(0.0));
        Kinematics {
            lon_accel,
            lat_accel,
        }
    }

    pub fn score(&self, i: usize) -> f64 {
        accel_score(self.lon_accel[i], self.lat_accel[i])
    }
}

fn accel_score(lon: f64, lat: f64) -> f64 {
    let lon_excess = if lon < MIN_LON_ACCEL {
        (MIN_LON_ACCEL - lon) / -MIN_LON_ACCEL
    } else if lon > MAX_LON_ACCEL {
        (lon - MAX_LON_ACCEL) / MAX_LON_ACCEL
    } else {
        0.0
    };
    let lat_excess = ((lat.abs() - MAX_ABS_LAT_ACCEL) / MAX_ABS_LAT_ACCEL).max(0.0);
    1.0 / (1.0 + lon_excess.powi(4) + lat_excess.powi(4))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vehicle_acceleration_limits_are_fully_acceptable() {
        for lon in [MIN_LON_ACCEL, 0.0, MAX_LON_ACCEL] {
            for lat in [-MAX_ABS_LAT_ACCEL, 0.0, MAX_ABS_LAT_ACCEL] {
                assert_eq!(accel_score(lon, lat), 1.0);
            }
        }
    }

    #[test]
    fn score_is_continuous_at_each_limit() {
        let scores = [
            accel_score(MAX_LON_ACCEL * 1.001, 0.0),
            accel_score(MIN_LON_ACCEL * 1.001, 0.0),
            accel_score(0.0, MAX_ABS_LAT_ACCEL * 1.001),
        ];
        assert!(scores.into_iter().all(|s| s < 1.0 && s > 0.999_999_999_99));
    }

    #[test]
    fn fourth_power_shaper_ignores_small_excess_but_bites_extremes() {
        let mild = accel_score(1.5 * MAX_LON_ACCEL, 0.0);
        let severe = accel_score(2.0 * MAX_LON_ACCEL, 0.0);
        let extreme = accel_score(3.0 * MAX_LON_ACCEL, 0.0);
        assert!(mild > 0.9);
        assert_eq!(severe, 0.5);
        assert!(extreme < 0.1);
        assert!(mild > severe && severe > extreme);
    }

    #[test]
    fn acceleration_and_braking_use_their_own_vehicle_limits() {
        assert_eq!(accel_score(2.0 * MAX_LON_ACCEL, 0.0), 0.5);
        assert_eq!(accel_score(2.0 * MIN_LON_ACCEL, 0.0), 0.5);
    }

    #[test]
    fn combined_longitudinal_and_lateral_extremes_compound() {
        assert_eq!(
            accel_score(2.0 * MAX_LON_ACCEL, 2.0 * MAX_ABS_LAT_ACCEL),
            1.0 / 3.0
        );
    }

    #[test]
    fn fast_heading_change_is_fine_when_lateral_acceleration_is_low() {
        let kinematics = Kinematics::new(
            &[
                State {
                    speed: 1.0,
                    ..Default::default()
                },
                State {
                    speed: 1.0,
                    yaw: 0.2,
                    ..Default::default()
                },
            ],
            0.1,
        );
        assert_eq!(kinematics.score(0), 1.0);
    }
}
