//! nuPlan closed-loop planner quality metrics.
//!
//! Thresholds and aggregation follow scenarios/nuplan/metrics_description.md.
//! Everything here is a pure function of simulation outputs (ego trace, actor
//! traces, centerline, speed limit, dt) — planner internals are off limits.

use crate::{Path, State};

/// Circle approximation of a car footprint for collision/TTC checks.
// ponytail: nuPlan intersects oriented boxes; a circle matches the planners'
// 2.5 m spacing and misses only bumper-to-bumper geometry
const CAR_RADIUS_M: f64 = 1.25;
const TTC_HORIZON_S: f64 = 3.0;
const TTC_STEP_S: f64 = 0.1;
const LEAST_MIN_TTC_S: f64 = 0.95;
const MAX_OVERSPEED_MS: f64 = 2.23;
const MIN_PROGRESS_RATIO: f64 = 0.2;
const ROAD_HALF_WIDTH_M: f64 = 5.5;
const DIRECTION_WINDOW_S: f64 = 1.0;
const DIRECTION_COMPLIANCE_M: f64 = 2.0;
const DIRECTION_VIOLATION_M: f64 = 6.0;
// comfort thresholds (empirical expert bounds from nuPlan)
const MIN_LON_ACCEL: f64 = -4.05;
const MAX_LON_ACCEL: f64 = 2.40;
const MAX_ABS_LAT_ACCEL: f64 = 4.89;
const MAX_ABS_YAW_ACCEL: f64 = 1.93;
const MAX_ABS_YAW_RATE: f64 = 0.95;
const MAX_ABS_LON_JERK: f64 = 4.13;
const MAX_ABS_MAG_JERK: f64 = 8.37;

/// Per-scenario closed-loop scores, each in [0, 1].
#[derive(Debug, Clone, Copy, Default)]
pub struct Metrics {
    // multipliers
    pub no_at_fault_collisions: f64,
    pub drivable_area_compliance: f64,
    pub driving_direction_compliance: f64,
    pub making_progress: f64,
    // weighted terms
    pub ttc_within_bound: f64,
    pub progress_ratio: f64,
    pub speed_limit_compliance: f64,
    pub comfort: f64,
    /// nuPlan closed-loop score: product of multipliers times the weighted
    /// average of (TTC, progress, speed limit, comfort) with weights 5/5/4/2.
    pub score: f64,
}

impl Metrics {
    pub const LABELS: [&'static str; 8] = [
        "no at-fault collisions",
        "drivable area",
        "driving direction",
        "making progress",
        "TTC within bound",
        "progress ratio",
        "speed limit",
        "comfort",
    ];

    pub fn values(&self) -> [f64; 8] {
        [
            self.no_at_fault_collisions,
            self.drivable_area_compliance,
            self.driving_direction_compliance,
            self.making_progress,
            self.ttc_within_bound,
            self.progress_ratio,
            self.speed_limit_compliance,
            self.comfort,
        ]
    }
}

fn gap(a: &State, b: &State) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

/// Constant-speed, constant-heading projection (nuPlan's TTC model).
fn project(s: &State, t: f64) -> State {
    State {
        x: s.x + s.speed * s.yaw.cos() * t,
        y: s.y + s.speed * s.yaw.sin() * t,
        ..*s
    }
}

fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Evaluate all metrics over a finished rollout. `actors[i]` must be sampled
/// at the same ticks as `ego`.
pub fn evaluate(
    ego: &[State],
    actors: &[Vec<State>],
    centerline: &[[f64; 2]],
    speed_limit: f64,
    dt: f64,
) -> Metrics {
    let path = Path::new(centerline);
    let n = ego.len();
    let duration = (n.saturating_sub(1)) as f64 * dt;
    let frenet: Vec<(f64, f64)> = ego.iter().map(|s| path.project([s.x, s.y])).collect();

    // --- safety ---
    let collided = (0..n).any(|i| {
        actors
            .iter()
            .any(|a| gap(&ego[i], &a[i]) < 2.0 * CAR_RADIUS_M)
    });
    let no_at_fault_collisions = if collided { 0.0 } else { 1.0 };

    let min_ttc = (0..n)
        .flat_map(|i| {
            actors.iter().filter_map(move |a| {
                let mut t = TTC_STEP_S;
                while t <= TTC_HORIZON_S {
                    if gap(&project(&ego[i], t), &project(&a[i], t)) < 2.0 * CAR_RADIUS_M {
                        return Some(t);
                    }
                    t += TTC_STEP_S;
                }
                None
            })
        })
        .fold(f64::INFINITY, f64::min);
    let ttc_within_bound = if min_ttc >= LEAST_MIN_TTC_S { 1.0 } else { 0.0 };

    // --- road compliance ---
    let drivable_area_compliance = if frenet.iter().any(|&(_, d)| d.abs() > ROAD_HALF_WIDTH_M) {
        0.0
    } else {
        1.0
    };

    // worst backward movement along the route over any 1 s window
    let window = (DIRECTION_WINDOW_S / dt) as usize;
    let worst_backward = (0..n.saturating_sub(window))
        .map(|i| frenet[i].0 - frenet[i + window].0)
        .fold(0.0, f64::max);
    let driving_direction_compliance = if worst_backward <= DIRECTION_COMPLIANCE_M {
        1.0
    } else if worst_backward <= DIRECTION_VIOLATION_M {
        0.5
    } else {
        0.0
    };

    // --- progress ---
    // ponytail: no expert trajectory; the route driven at the speed limit
    // stands in for expert progress
    let ego_progress = frenet.last().map_or(0.0, |f| f.0) - frenet[0].0;
    let expert_progress = speed_limit * duration;
    let progress_ratio = if ego_progress < -0.1 {
        0.0
    } else {
        (ego_progress.max(0.1) / expert_progress.max(0.1)).min(1.0)
    };
    let making_progress = if progress_ratio > MIN_PROGRESS_RATIO {
        1.0
    } else {
        0.0
    };

    // --- speed limit ---
    let avg_violation = ego
        .iter()
        .map(|s| (s.speed - speed_limit).max(0.0))
        .sum::<f64>()
        / n as f64;
    let speed_limit_compliance = (1.0 - avg_violation / MAX_OVERSPEED_MS).clamp(0.0, 1.0);

    // --- comfort ---
    let lon_accel: Vec<f64> = ego
        .windows(2)
        .map(|w| (w[1].speed - w[0].speed) / dt)
        .collect();
    let yaw_rate: Vec<f64> = ego
        .windows(2)
        .map(|w| wrap_angle(w[1].yaw - w[0].yaw) / dt)
        .collect();
    let lat_accel: Vec<f64> = yaw_rate.iter().zip(ego).map(|(r, s)| r * s.speed).collect();
    let diff = |v: &[f64]| -> Vec<f64> { v.windows(2).map(|w| (w[1] - w[0]) / dt).collect() };
    let lon_jerk = diff(&lon_accel);
    let lat_jerk = diff(&lat_accel);
    let yaw_accel = diff(&yaw_rate);
    let within = |v: &[f64], lo: f64, hi: f64| v.iter().all(|&x| x >= lo && x <= hi);
    let comfort = if within(&lon_accel, MIN_LON_ACCEL, MAX_LON_ACCEL)
        && within(&lat_accel, -MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL)
        && within(&yaw_rate, -MAX_ABS_YAW_RATE, MAX_ABS_YAW_RATE)
        && within(&yaw_accel, -MAX_ABS_YAW_ACCEL, MAX_ABS_YAW_ACCEL)
        && within(&lon_jerk, -MAX_ABS_LON_JERK, MAX_ABS_LON_JERK)
        && lon_jerk
            .iter()
            .zip(&lat_jerk)
            .all(|(lo, la)| lo.hypot(*la) <= MAX_ABS_MAG_JERK)
    {
        1.0
    } else {
        0.0
    };

    let weighted = (5.0 * ttc_within_bound
        + 5.0 * progress_ratio
        + 4.0 * speed_limit_compliance
        + 2.0 * comfort)
        / 16.0;
    let score = no_at_fault_collisions
        * drivable_area_compliance
        * driving_direction_compliance
        * making_progress
        * weighted;

    Metrics {
        no_at_fault_collisions,
        drivable_area_compliance,
        driving_direction_compliance,
        making_progress,
        ttc_within_bound,
        progress_ratio,
        speed_limit_compliance,
        comfort,
        score,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CENTERLINE: [[f64; 2]; 2] = [[-20.0, 0.0], [400.0, 0.0]];
    const DT: f64 = 0.1;

    fn cruise(speed: f64, ticks: usize) -> Vec<State> {
        (0..=ticks)
            .map(|i| State {
                x: speed * DT * i as f64,
                speed,
                ..Default::default()
            })
            .collect()
    }

    #[test]
    fn perfect_cruise_scores_one() {
        let m = evaluate(&cruise(10.0, 200), &[], &CENTERLINE, 10.0, DT);
        assert_eq!(m.values(), [1.0; 8]);
        assert!((m.score - 1.0).abs() < 1e-9);
    }

    #[test]
    fn collision_zeroes_the_score() {
        let ego = cruise(10.0, 200);
        let parked = vec![
            State {
                x: 100.0,
                ..Default::default()
            };
            201
        ];
        let m = evaluate(&ego, &[parked], &CENTERLINE, 10.0, DT);
        assert_eq!(m.no_at_fault_collisions, 0.0);
        assert_eq!(m.ttc_within_bound, 0.0);
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn overspeed_reduces_compliance() {
        let m = evaluate(&cruise(11.0, 200), &[], &CENTERLINE, 10.0, DT);
        assert!(m.speed_limit_compliance < 1.0 && m.speed_limit_compliance > 0.0);
    }

    #[test]
    fn harsh_braking_is_uncomfortable() {
        let mut ego = cruise(10.0, 200);
        for (i, s) in ego.iter_mut().enumerate().skip(100) {
            s.speed = (10.0 - 6.0 * DT * (i - 100) as f64).max(0.0);
        }
        let m = evaluate(&ego, &[], &CENTERLINE, 10.0, DT);
        assert_eq!(m.comfort, 0.0);
    }

    #[test]
    fn driving_backwards_is_noncompliant() {
        let mut ego = cruise(10.0, 200);
        ego.reverse();
        let m = evaluate(&ego, &[], &CENTERLINE, 10.0, DT);
        assert_eq!(m.driving_direction_compliance, 0.0);
        assert_eq!(m.progress_ratio, 0.0);
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn leaving_the_road_violates_drivable_area() {
        let mut ego = cruise(10.0, 200);
        ego[50].y = 7.0;
        let m = evaluate(&ego, &[], &CENTERLINE, 10.0, DT);
        assert_eq!(m.drivable_area_compliance, 0.0);
    }
}
