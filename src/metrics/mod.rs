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
//! function of simulation outputs (ego trace, actor traces, and the
//! [`Road`]) — planner internals are off limits.
//!
//! Everything a metric *is* — its display label, how it scores a tick, how
//! its ticks aggregate to a scenario value, and its role in the composite —
//! lives in one row of the [`METRICS`] spec table (the Strategy pattern,
//! table-driven). A metric's position in the per-tick score arrays is the
//! position of its row, nothing more: adding a metric means writing its
//! module and adding one row here, and no consumer indexes scores by magic
//! number.

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

/// Precomputed, per-rollout series every metric scores from. Built once by
/// [`evaluate`]; a metric's score function reads the tick it's given and
/// nothing else, so metrics stay pure functions of simulation outputs.
pub struct TickCtx<'a> {
    /// Ego state at every tick.
    pub ego: &'a [State],
    /// Every actor's state at every tick: `actors_at[tick][actor]`.
    pub actors_at: &'a [Vec<State>],
    /// Ego arc length along the route at every tick.
    pub station: &'a [f64],
    /// Ego signed lateral offset from the centerline at every tick.
    pub lateral: &'a [f64],
    /// Tickwise ego kinematics (accels, jerks, yaw rates).
    pub kinematics: &'a comfort::Kinematics,
    pub speed_limit: f64,
    pub dt: f64,
}

/// A metric's role in the nuPlan composite score: hard gate or weighted term.
pub enum CompositeRole {
    /// Multiplies the composite directly — a 0 zeroes the whole score.
    Multiplier,
    /// Contributes to the weighted average with this weight.
    Weighted(f64),
}

/// One metric, whole: everything [`evaluate`] and the score consumers (the
/// viewer's metrics table, the batch CSV) need to know about it.
pub struct MetricSpec {
    pub label: &'static str,
    /// Score of one tick, in [0, 1].
    pub score: fn(&TickCtx, usize) -> f64,
    /// Scenario value from this metric's per-tick score column ([`agg::min`]
    /// for event-driven metrics, [`agg::avg`] for smooth quantities, or a
    /// metric-specific rule like [`making_progress::aggregate`]).
    pub aggregate: fn(&TickCtx, &[f64]) -> f64,
    pub role: CompositeRole,
}

/// The metric registry: row order defines score-array order everywhere.
pub const METRICS: [MetricSpec; 8] = [
    MetricSpec {
        label: "no at-fault collisions",
        score: collisions::score,
        aggregate: agg::min,
        role: CompositeRole::Multiplier,
    },
    MetricSpec {
        label: "drivable area",
        score: drivable_area::score,
        aggregate: agg::min,
        role: CompositeRole::Multiplier,
    },
    MetricSpec {
        label: "driving direction",
        score: driving_direction::score,
        aggregate: agg::min,
        role: CompositeRole::Multiplier,
    },
    MetricSpec {
        label: "making progress",
        score: making_progress::score,
        aggregate: making_progress::aggregate,
        role: CompositeRole::Multiplier,
    },
    MetricSpec {
        label: "TTC within bound",
        score: ttc::score,
        aggregate: agg::min,
        role: CompositeRole::Weighted(5.0),
    },
    MetricSpec {
        label: "progress ratio",
        score: progress::score,
        aggregate: agg::avg,
        role: CompositeRole::Weighted(5.0),
    },
    MetricSpec {
        label: "speed limit",
        score: speed_limit::score,
        aggregate: agg::avg,
        role: CompositeRole::Weighted(4.0),
    },
    MetricSpec {
        label: "comfort",
        score: comfort::score,
        aggregate: agg::avg,
        role: CompositeRole::Weighted(2.0),
    },
];

pub const N_METRICS: usize = METRICS.len();

/// The two standard aggregation rules (see the module doc): worst case for
/// event-driven metrics, average for smooth quantities.
pub mod agg {
    use super::TickCtx;

    pub fn min(_: &TickCtx, per_tick: &[f64]) -> f64 {
        per_tick.iter().copied().fold(1.0, f64::min)
    }

    pub fn avg(_: &TickCtx, per_tick: &[f64]) -> f64 {
        per_tick.iter().sum::<f64>() / per_tick.len().max(1) as f64
    }
}

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

/// The nuPlan composite: the product of every [`CompositeRole::Multiplier`]
/// metric times the weighted average of the [`CompositeRole::Weighted`]
/// ones, both read straight from the [`METRICS`] table.
fn composite(scores: &[f64; N_METRICS]) -> f64 {
    let (mut product, mut weighted, mut total_weight) = (1.0, 0.0, 0.0);
    for (spec, s) in METRICS.iter().zip(scores) {
        match spec.role {
            CompositeRole::Multiplier => product *= s,
            CompositeRole::Weighted(w) => {
                weighted += w * s;
                total_weight += w;
            }
        }
    }
    product * weighted / total_weight
}

/// Evaluate all metrics over a finished rollout. `actors[i]` must be sampled
/// at the same ticks as `ego`.
pub fn evaluate(ego: &[State], actors: &[Vec<State>], road: &Road) -> Metrics {
    let n = ego.len();
    if n == 0 {
        return Metrics::default();
    }
    let path = Path::new(&road.centerline);
    let frenet: Vec<(f64, f64)> = ego.iter().map(|s| path.project([s.x, s.y])).collect();
    let station: Vec<f64> = frenet.iter().map(|f| f.0).collect();
    let lateral: Vec<f64> = frenet.iter().map(|f| f.1).collect();
    let actors_at: Vec<Vec<State>> = (0..n)
        .map(|i| actors.iter().map(|a| a[i]).collect())
        .collect();
    let kinematics = comfort::Kinematics::new(ego, road.dt);
    let ctx = TickCtx {
        ego,
        actors_at: &actors_at,
        station: &station,
        lateral: &lateral,
        kinematics: &kinematics,
        speed_limit: road.target_speed,
        dt: road.dt,
    };

    let per_tick: Vec<[f64; N_METRICS]> = (0..n)
        .map(|i| std::array::from_fn(|m| (METRICS[m].score)(&ctx, i)))
        .collect();
    let score_per_tick: Vec<f64> = per_tick.iter().map(composite).collect();
    let aggregate: [f64; N_METRICS] = std::array::from_fn(|m| {
        let column: Vec<f64> = per_tick.iter().map(|t| t[m]).collect();
        (METRICS[m].aggregate)(&ctx, &column)
    });
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
