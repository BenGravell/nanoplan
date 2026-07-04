//! nuPlan closed-loop planner quality metrics, built up strictly tickwise,
//! one module per metric.
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

pub mod collisions;
pub mod comfort;
pub mod drivable_area;
pub mod driving_direction;
pub mod making_progress;
pub mod progress;
pub mod speed_limit;
pub mod ttc;

use crate::scenarios::{Path, Road};
use crate::simulation::State;

/// Circle approximation of a car footprint for collision/TTC checks.
// ponytail: nuPlan intersects oriented boxes; a circle matches the planners'
// 2.5 m spacing and misses only bumper-to-bumper geometry
pub(crate) const CAR_RADIUS_M: f64 = 1.25;

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

pub(crate) fn gap(a: &State, b: &State) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

/// Constant-speed, constant-heading projection (nuPlan's TTC model).
pub(crate) fn project(s: &State, t: f64) -> State {
    State {
        x: s.x + s.speed * s.yaw.cos() * t,
        y: s.y + s.speed * s.yaw.sin() * t,
        ..*s
    }
}

/// The nuPlan composite: multipliers times the 5/5/4/2 weighted average of
/// (TTC, progress, speed limit, comfort).
fn composite(m: &[f64; N_METRICS]) -> f64 {
    let weighted = (5.0 * m[4] + 5.0 * m[5] + 4.0 * m[6] + 2.0 * m[7]) / 16.0;
    m[0] * m[1] * m[2] * m[3] * weighted
}

/// Evaluate all metrics over a finished rollout. `actors[i]` must be sampled
/// at the same ticks as `ego`.
pub fn evaluate(ego: &[State], actors: &[Vec<State>], road: &Road) -> Metrics {
    let (speed_limit, dt) = (road.target_speed, road.dt);
    let path = Path::new(&road.centerline);
    let n = ego.len();
    if n == 0 {
        return Metrics::default();
    }
    let frenet: Vec<(f64, f64)> = ego.iter().map(|s| path.project([s.x, s.y])).collect();
    let station: Vec<f64> = frenet.iter().map(|f| f.0).collect();
    let kinematics = comfort::Kinematics::new(ego, dt);

    let mut per_tick: Vec<[f64; N_METRICS]> = Vec::with_capacity(n);
    let mut score_per_tick = Vec::with_capacity(n);
    for i in 0..n {
        let actors_now: Vec<State> = actors.iter().map(|a| a[i]).collect();
        let progress = progress::score(&station, i, dt, speed_limit);
        let scores = [
            collisions::score(&ego[i], &actors_now),
            drivable_area::score(frenet[i].1),
            driving_direction::score(&station, i, dt),
            making_progress::score(progress),
            ttc::score(&ego[i], &actors_now),
            progress,
            speed_limit::score(ego[i].speed, speed_limit),
            kinematics.score(i),
        ];
        score_per_tick.push(composite(&scores));
        per_tick.push(scores);
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
        making_progress::score(progress),
        min_of(4), // TTC
        progress,
        avg_of(6), // speed limit
        avg_of(7), // comfort
    ];
    let score = composite(&aggregate);

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

    fn road() -> Road {
        Road {
            centerline: CENTERLINE.to_vec(),
            target_speed: 10.0,
            dt: DT,
        }
    }

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
        let m = evaluate(&cruise(10.0, 200), &[], &road());
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
        let m = evaluate(&ego, &[parked], &road());
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
        let m = evaluate(&cruise(11.0, 200), &[], &road());
        let expected = 1.0 - 1.0 / speed_limit::MAX_OVERSPEED_MS;
        assert!(m.per_tick.iter().all(|t| (t[6] - expected).abs() < 1e-9));
        assert!((m.aggregate[6] - expected).abs() < 1e-9);
    }

    #[test]
    fn harsh_braking_is_uncomfortable_only_while_braking() {
        let mut ego = cruise(10.0, 200);
        for (i, s) in ego.iter_mut().enumerate().skip(100) {
            s.speed = (10.0 - 6.0 * DT * (i - 100) as f64).max(0.0);
        }
        let m = evaluate(&ego, &[], &road());
        assert_eq!(m.at(50).0[7], 1.0);
        assert_eq!(m.at(110).0[7], 0.0);
        // comfort is a smooth quantity: averaged, not zeroed by the event
        assert!(m.aggregate[7] > 0.0 && m.aggregate[7] < 1.0);
    }

    #[test]
    fn driving_backwards_is_noncompliant() {
        let mut ego = cruise(10.0, 200);
        ego.reverse();
        let m = evaluate(&ego, &[], &road());
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
        let m = evaluate(&ego, &[], &road());
        assert_eq!(m.at(50).0[1], 0.0);
        assert_eq!(m.at(51).0[1], 1.0);
        assert_eq!(m.aggregate[1], 0.0); // min aggregation: one bad tick
        assert_eq!(m.score, 0.0);
    }
}
