//! nuPlan closed-loop planner quality metrics, built up strictly tickwise.
//!
//! Every metric is a per-tick score in [0, 1]. Scenario values aggregate the
//! per-tick scores with a per-metric rule: event-driven metrics (collisions,
//! drivable area, driving direction, TTC) take the worst case (min) — one bad
//! tick is a violation — while smooth quantities (progress, speed limit,
//! comfort) take the average; making-progress thresholds the aggregated
//! progress ratio, as in nuPlan. The composite score applies the nuPlan
//! structure (multipliers times the 5/5/4/2 weighted average) to the
//! aggregates, and per tick for the scrubber display. Thresholds follow
//! scenarios/nuplan/metrics_description.md. Everything here is a pure
//! function of simulation outputs (ego trace, actor traces, centerline,
//! speed limit, dt) — planner internals are off limits.

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

pub const N_METRICS: usize = 8;

/// Per-tick metric scores for a rollout, plus their scenario aggregates.
#[derive(Debug, Clone, Default)]
pub struct Metrics {
    /// Per-tick score of each metric, `per_tick[tick][metric]`.
    pub per_tick: Vec<[f64; N_METRICS]>,
    /// Per-tick composite score.
    pub score_per_tick: Vec<f64>,
    /// Scenario value of each metric, aggregated per its rule (min or avg).
    pub aggregate: [f64; N_METRICS],
    /// Scenario score: the nuPlan composite applied to the aggregates.
    pub score: f64,
}

impl Metrics {
    pub const LABELS: [&'static str; N_METRICS] = [
        "no at-fault collisions",
        "drivable area",
        "driving direction",
        "making progress",
        "TTC within bound",
        "progress ratio",
        "speed limit",
        "comfort",
    ];

    /// Metric scores and composite score at a tick (clamped to the rollout).
    pub fn at(&self, tick: usize) -> ([f64; N_METRICS], f64) {
        let i = tick.min(self.per_tick.len().saturating_sub(1));
        (self.per_tick[i], self.score_per_tick[i])
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

/// Forward difference, padded by repeating the last value so the result has
/// the same length as the input.
fn diff(v: &[f64], dt: f64) -> Vec<f64> {
    let mut d: Vec<f64> = v.windows(2).map(|w| (w[1] - w[0]) / dt).collect();
    d.push(d.last().copied().unwrap_or(0.0));
    d
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
    if n == 0 {
        return Metrics::default();
    }
    let frenet: Vec<(f64, f64)> = ego.iter().map(|s| path.project([s.x, s.y])).collect();
    let station: Vec<f64> = frenet.iter().map(|f| f.0).collect();

    // tickwise kinematics (forward differences, padded to length n)
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
    let lon_jerk = diff(&lon_accel, dt);
    let lat_jerk = diff(&lat_accel, dt);
    let yaw_accel = diff(&yaw_rate, dt);

    let window = ((DIRECTION_WINDOW_S / dt) as usize).max(1);
    let mut per_tick: Vec<[f64; N_METRICS]> = Vec::with_capacity(n);
    let mut score_per_tick = Vec::with_capacity(n);

    for i in 0..n {
        // safety: collision-free and TTC at this tick
        let no_collision = if actors
            .iter()
            .any(|a| gap(&ego[i], &a[i]) < 2.0 * CAR_RADIUS_M)
        {
            0.0
        } else {
            1.0
        };
        let min_ttc = actors
            .iter()
            .filter_map(|a| {
                let mut t = TTC_STEP_S;
                while t <= TTC_HORIZON_S {
                    if gap(&project(&ego[i], t), &project(&a[i], t)) < 2.0 * CAR_RADIUS_M {
                        return Some(t);
                    }
                    t += TTC_STEP_S;
                }
                None
            })
            .fold(f64::INFINITY, f64::min);
        let ttc = if min_ttc >= LEAST_MIN_TTC_S { 1.0 } else { 0.0 };

        // road compliance at this tick
        let drivable = if frenet[i].1.abs() > ROAD_HALF_WIDTH_M {
            0.0
        } else {
            1.0
        };
        // backward movement along the route over the trailing window
        let backward = station[i.saturating_sub(window)] - station[i];
        let direction = if backward <= DIRECTION_COMPLIANCE_M {
            1.0
        } else if backward <= DIRECTION_VIOLATION_M {
            0.5
        } else {
            0.0
        };

        // progress at this tick: station rate vs. driving at the speed limit
        // ponytail: no expert trajectory; the speed limit stands in
        let ds = if i + 1 < n {
            station[i + 1] - station[i]
        } else if i > 0 {
            station[i] - station[i - 1]
        } else {
            0.0
        };
        let progress = (ds / dt / speed_limit.max(0.1)).clamp(0.0, 1.0);
        let making_progress = if progress > MIN_PROGRESS_RATIO {
            1.0
        } else {
            0.0
        };

        // speed limit compliance at this tick
        let overspeed = (speed[i] - speed_limit).max(0.0);
        let speed_score = (1.0 - overspeed / MAX_OVERSPEED_MS).clamp(0.0, 1.0);

        // comfort at this tick: all kinematic bounds hold
        let comfort = if (MIN_LON_ACCEL..=MAX_LON_ACCEL).contains(&lon_accel[i])
            && lat_accel[i].abs() <= MAX_ABS_LAT_ACCEL
            && yaw_rate[i].abs() <= MAX_ABS_YAW_RATE
            && yaw_accel[i].abs() <= MAX_ABS_YAW_ACCEL
            && lon_jerk[i].abs() <= MAX_ABS_LON_JERK
            && lon_jerk[i].hypot(lat_jerk[i]) <= MAX_ABS_MAG_JERK
        {
            1.0
        } else {
            0.0
        };

        let scores = [
            no_collision,
            drivable,
            direction,
            making_progress,
            ttc,
            progress,
            speed_score,
            comfort,
        ];
        // nuPlan structure at tick granularity: multipliers times the
        // weighted average of (TTC, progress, speed limit, comfort), 5/5/4/2
        let weighted = (5.0 * ttc + 5.0 * progress + 4.0 * speed_score + 2.0 * comfort) / 16.0;
        let score = no_collision * drivable * direction * making_progress * weighted;

        per_tick.push(scores);
        score_per_tick.push(score);
    }

    // per-metric aggregation: worst case for event-driven metrics, average
    // for smooth quantities; making-progress thresholds aggregated progress
    let min_of = |m: usize| per_tick.iter().map(|t| t[m]).fold(1.0, f64::min);
    let avg_of = |m: usize| per_tick.iter().map(|t| t[m]).sum::<f64>() / n as f64;
    let progress = avg_of(5);
    let aggregate = [
        min_of(0), // collisions
        min_of(1), // drivable area
        min_of(2), // driving direction
        if progress > MIN_PROGRESS_RATIO {
            1.0
        } else {
            0.0
        },
        min_of(4), // TTC
        progress,
        avg_of(6), // speed limit
        avg_of(7), // comfort
    ];
    let weighted =
        (5.0 * aggregate[4] + 5.0 * aggregate[5] + 4.0 * aggregate[6] + 2.0 * aggregate[7]) / 16.0;
    let score = aggregate[0] * aggregate[1] * aggregate[2] * aggregate[3] * weighted;

    Metrics {
        per_tick,
        score_per_tick,
        aggregate,
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
    fn perfect_cruise_scores_one_every_tick() {
        let m = evaluate(&cruise(10.0, 200), &[], &CENTERLINE, 10.0, DT);
        assert!(
            m.per_tick
                .iter()
                .all(|t| t.iter().all(|s| (s - 1.0).abs() < 1e-9))
        );
        assert!(m.aggregate.iter().all(|a| (a - 1.0).abs() < 1e-9));
        assert!((m.score - 1.0).abs() < 1e-9);
    }

    #[test]
    fn collision_zeroes_its_ticks_and_the_scenario_by_min_agg() {
        let ego = cruise(10.0, 200);
        let parked = vec![
            State {
                x: 100.0,
                ..Default::default()
            };
            201
        ];
        let m = evaluate(&ego, &[parked], &CENTERLINE, 10.0, DT);
        // tick 100 is exactly at the parked car
        let (scores, score) = m.at(100);
        assert_eq!(scores[0], 0.0);
        assert_eq!(score, 0.0);
        // far away the tick is untouched, but the event zeroes the scenario
        assert_eq!(m.at(0).0[0], 1.0);
        assert_eq!(m.aggregate[0], 0.0);
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn overspeed_reduces_compliance_each_tick() {
        let m = evaluate(&cruise(11.0, 200), &[], &CENTERLINE, 10.0, DT);
        let expected = 1.0 - 1.0 / MAX_OVERSPEED_MS;
        assert!(m.per_tick.iter().all(|t| (t[6] - expected).abs() < 1e-9));
        assert!((m.aggregate[6] - expected).abs() < 1e-9);
    }

    #[test]
    fn harsh_braking_is_uncomfortable_only_while_braking() {
        let mut ego = cruise(10.0, 200);
        for (i, s) in ego.iter_mut().enumerate().skip(100) {
            s.speed = (10.0 - 6.0 * DT * (i - 100) as f64).max(0.0);
        }
        let m = evaluate(&ego, &[], &CENTERLINE, 10.0, DT);
        assert_eq!(m.at(50).0[7], 1.0);
        assert_eq!(m.at(110).0[7], 0.0);
        // comfort is a smooth quantity: averaged, not zeroed by the event
        assert!(m.aggregate[7] > 0.0 && m.aggregate[7] < 1.0);
    }

    #[test]
    fn driving_backwards_is_noncompliant() {
        let mut ego = cruise(10.0, 200);
        ego.reverse();
        let m = evaluate(&ego, &[], &CENTERLINE, 10.0, DT);
        // once the trailing window fills, direction is fully violated
        assert_eq!(m.at(50).0[2], 0.0);
        assert_eq!(m.aggregate[2], 0.0);
        assert_eq!(m.aggregate[3], 0.0); // no forward progress either
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn leaving_the_road_once_zeroes_drivable_area() {
        let mut ego = cruise(10.0, 200);
        ego[50].y = 7.0;
        let m = evaluate(&ego, &[], &CENTERLINE, 10.0, DT);
        assert_eq!(m.at(50).0[1], 0.0);
        assert_eq!(m.at(51).0[1], 1.0);
        assert_eq!(m.aggregate[1], 0.0); // min aggregation: one bad tick
        assert_eq!(m.score, 0.0);
    }
}
