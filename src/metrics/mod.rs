//! Closed-loop planner quality metrics, built up strictly tickwise,
//! one module per metric.
//!
//! Every metric is a per-tick score in [0, 1]. Rollout values aggregate the
//! per-tick scores with a per-metric rule: event-driven TTC takes the worst
//! case (min) — one bad tick is a violation — while smooth quantities
//! (progress and comfort)
//! take the average. The composite score multiplies hard gates by a weighted
//! average of the remaining race-quality signals. Everything here is a pure
//! function of simulation outputs (ego trace, actor traces, and the
//! [`Road`] geometry) — planner internals are off limits.
//!
//! Everything a metric *is* — its display label, how it scores a tick, how
//! its ticks aggregate to a rollout value, and its role in the composite —
//! lives in one row of the [`METRICS`] spec table (the Strategy pattern,
//! table-driven). A metric's position in the per-tick score arrays is the
//! position of its row, nothing more: adding a metric means writing its
//! module and adding one row here, and no consumer indexes scores by magic
//! number.

pub(crate) mod comfort;
pub(crate) mod progress;
pub(crate) mod safety;

pub(crate) mod aggregation;

use crate::geometry::CAR_FOOTPRINT;
use crate::simulation::{Control, State};
use crate::track::{Path, Road};

pub(crate) use aggregation as agg;

/// Center clearance used by point-sample planners that do not carry boxes.
pub(crate) const COLLISION_CLEARANCE_M: f64 = CAR_FOOTPRINT.width;

/// Precomputed, per-rollout series every metric scores from. Built once by
/// [`evaluate`]; a metric's score function reads the tick it's given and
/// nothing else, so metrics stay pure functions of simulation outputs.
pub(crate) struct TickCtx<'a> {
    /// Ego state at every tick.
    pub(crate) ego: &'a [State],
    /// Every actor's state at every tick: `actors_at[tick][actor]`.
    pub(crate) actors_at: &'a [Vec<State>],
    /// Ego arc length along the route at every tick.
    pub(crate) station: &'a [f64],
    /// Applied ego control at every scored tick.
    pub(crate) controls: &'a [Control],
    /// Road geometry, including its static side barriers.
    pub(crate) road: &'a Road,
    pub(crate) dt: f64,
}

/// A metric's role in the composite score: hard gate or weighted term.
pub(crate) enum CompositeRole {
    /// Multiplies the composite directly — a 0 zeroes the whole score.
    Multiplier,
    /// Contributes to the weighted average with this weight.
    Weighted(f64),
}

/// One metric's scoring and aggregation behavior.
pub(crate) struct MetricSpec {
    /// Score of one tick, in [0, 1].
    pub(crate) score: fn(&TickCtx, usize) -> f64,
    /// Rollout value from this metric's per-tick score column
    /// ([`aggregation::min`] for event-driven metrics, [`aggregation::avg`]
    /// for smooth quantities).
    pub(crate) aggregate: fn(&TickCtx, &[f64]) -> f64,
    pub(crate) role: CompositeRole,
}

/// The metric registry: row order defines score-array order everywhere.
pub(crate) const METRICS: [MetricSpec; 3] = [
    MetricSpec {
        score: safety::score,
        aggregate: agg::min,
        role: CompositeRole::Multiplier,
    },
    MetricSpec {
        score: progress::score,
        aggregate: agg::avg,
        role: CompositeRole::Weighted(5.0),
    },
    MetricSpec {
        score: comfort::score,
        aggregate: agg::avg,
        role: CompositeRole::Weighted(0.01),
    },
];

pub(crate) const N_METRICS: usize = METRICS.len();

/// Per-tick metric scores for a rollout, plus their aggregates.
#[derive(Debug, Clone, Default)]
pub(crate) struct Metrics {
    /// Per-tick score of each metric, `per_tick[tick][metric]`.
    pub(crate) per_tick: Vec<[f64; N_METRICS]>,
    /// Per-tick composite score.
    pub(crate) score_per_tick: Vec<f64>,
    /// Rollout value of each metric, aggregated per its rule (min or avg).
    pub(crate) aggregate: [f64; N_METRICS],
    /// Rollout score: the composite applied to the aggregates.
    pub(crate) score: f64,
}

/// Evaluate all metrics over a finished rollout. `controls[i]` and
/// `actors[*][i]` must be sampled at the same ticks as `ego[i]`.
pub(crate) fn evaluate(
    ego: &[State],
    controls: &[Control],
    actors: &[Vec<State>],
    road: &Road,
) -> Metrics {
    let n = ego.len();
    if n == 0 {
        return Metrics::default();
    }
    assert_eq!(controls.len(), n);
    let path = Path::new(road.centerline());
    let station: Vec<f64> = ego.iter().map(|s| path.project(s.position()).0).collect();
    let actors_at: Vec<Vec<State>> = (0..n)
        .map(|i| actors.iter().map(|a| a[i]).collect())
        .collect();
    let ctx = TickCtx {
        ego,
        actors_at: &actors_at,
        station: &station,
        controls,
        road,
        dt: road.dt,
    };

    let per_tick: Vec<[f64; N_METRICS]> = (0..n)
        .map(|i| std::array::from_fn(|m| (METRICS[m].score)(&ctx, i)))
        .collect();
    let score_per_tick: Vec<f64> = per_tick
        .iter()
        .map(|scores| aggregation::composite(&METRICS, scores))
        .collect();
    let aggregate: [f64; N_METRICS] = std::array::from_fn(|m| {
        let column: Vec<f64> = per_tick.iter().map(|t| t[m]).collect();
        (METRICS[m].aggregate)(&ctx, &column)
    });
    let score = aggregation::composite(&METRICS, &aggregate);

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
    use crate::simulation::MAX_TERMINAL_SPEED_MPS;
    use crate::simulation::{Control, world_step};
    use crate::vehicle::MAX_LON_ACCEL;

    const CENTERLINE: [[f64; 2]; 2] = [[-20.0, 0.0], [400.0, 0.0]];
    const DT: f64 = 0.1;
    const TEST_HALF_WIDTH_M: f64 = 5.5;

    fn road() -> Road {
        Road::new(CENTERLINE.to_vec(), 10.0, TEST_HALF_WIDTH_M, DT)
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

    fn evaluate_coasting(ego: &[State], actors: &[Vec<State>], road: &Road) -> Metrics {
        evaluate(ego, &vec![Control::default(); ego.len()], actors, road)
    }

    #[test]
    fn perfect_cruise_scores_one_every_tick() {
        let ego = cruise(*MAX_TERMINAL_SPEED_MPS, 20);
        let m = evaluate_coasting(&ego, &[], &road());
        assert!(
            m.per_tick
                .iter()
                .all(|t| t.iter().all(|s| (s - 1.0).abs() < 1e-9))
        );
        assert!(m.aggregate.iter().all(|a| (a - 1.0).abs() < 1e-9));
        assert!((m.score - 1.0).abs() < 1e-9);
    }

    #[test]
    fn progress_uses_full_acceleration_from_the_initial_speed() {
        let initial = *MAX_TERMINAL_SPEED_MPS / 2.0;
        let mut trace = vec![State {
            speed: initial,
            ..Default::default()
        }];
        for _ in 0..20 {
            trace.push(world_step(
                *trace.last().unwrap(),
                Control {
                    acceleration: MAX_LON_ACCEL,
                    ..Default::default()
                },
                DT,
            ));
        }
        let controls = vec![
            Control {
                acceleration: MAX_LON_ACCEL,
                ..Default::default()
            };
            trace.len()
        ];
        let accelerated = evaluate(&trace, &controls, &[], &road());
        assert!((accelerated.aggregate[1] - 1.0).abs() < 1e-9);

        let held_trace = cruise(initial, 20);
        let held = evaluate_coasting(&held_trace, &[], &road());
        assert!(held.aggregate[1] < accelerated.aggregate[1]);
    }

    #[test]
    fn collision_zeroes_the_rollout_by_min_aggregation() {
        let ego = cruise(10.0, 200);
        let parked = vec![
            State {
                x: 100.0,
                ..Default::default()
            };
            201
        ];
        let m = evaluate_coasting(&ego, &[parked], &road());
        // tick 100 is exactly at the parked car
        assert_eq!(m.per_tick[100][0], 0.0);
        assert_eq!(m.score_per_tick[100], 0.0);
        // far away the tick is untouched, but the event zeroes the rollout
        assert_eq!(m.per_tick[0][0], 1.0);
        assert_eq!(m.aggregate[0], 0.0);
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn changing_acceleration_is_penalized_but_steady_braking_is_not() {
        let mut ego = cruise(10.0, 200);
        let mut controls = vec![Control::default(); ego.len()];
        for (i, s) in ego.iter_mut().enumerate().skip(100) {
            s.speed = (10.0 - 6.0 * DT * (i - 100) as f64).max(0.0);
            controls[i].acceleration = -6.0;
        }
        let m = evaluate(&ego, &controls, &[], &road());
        assert_eq!(m.per_tick[50][2], 1.0);
        assert_eq!(m.per_tick[99][2], 0.0);
        assert_eq!(m.per_tick[110][2], 1.0);
        assert!(m.aggregate[2] < 1.0);
    }

    #[test]
    fn comfort_only_breaks_tiny_ties() {
        let comfortable_mid_safety = aggregation::composite(&METRICS, &[0.5, 1.0, 1.0]);
        let comfortable_mid_progress = aggregation::composite(&METRICS, &[1.0, 0.5, 1.0]);

        let uncomfortable_tiny_improve_safety = aggregation::composite(&METRICS, &[0.51, 1.0, 0.0]);
        let uncomfortable_tiny_improve_progress =
            aggregation::composite(&METRICS, &[1.0, 0.51, 0.0]);

        // Tiny safety improvement must dominate perfect comfort improvement.
        assert!(uncomfortable_tiny_improve_safety > comfortable_mid_safety);

        // Tiny progress improvement must dominate perfect comfort improvement.
        assert!(uncomfortable_tiny_improve_progress > comfortable_mid_progress);

        // Safety score zero must collapse overall score to zero.
        assert_eq!(aggregation::composite(&METRICS, &[0.0, 1.0, 1.0]), 0.0);
    }

    #[test]
    fn progress_clamps_backward_motion_to_zero() {
        let mut ego = cruise(10.0, 200);
        ego.reverse();
        let m = evaluate_coasting(&ego, &[], &road());
        assert_eq!(m.per_tick[50][1], 0.0);
        assert_eq!(m.aggregate[1], 0.0);
    }

    #[test]
    fn leaving_the_road_once_zeroes_ttc() {
        let mut ego = cruise(10.0, 200);
        ego[50].y = 7.0;
        let m = evaluate_coasting(&ego, &[], &road());
        assert_eq!(m.per_tick[50][0], 0.0);
        assert_eq!(m.per_tick[51][0], 1.0);
        assert_eq!(m.aggregate[0], 0.0); // min aggregation: one bad tick
        assert_eq!(m.score, 0.0);
    }

    #[test]
    fn ttc_tracks_the_road_half_width() {
        // the same 4 m lateral excursion is on a wide road but off a narrow
        // one: the barrier geometry follows the road's own half-width, not a
        // fixed constant
        let mut ego = cruise(10.0, 200);
        ego[50].y = 4.0;
        let wide = Road::new(CENTERLINE.to_vec(), 10.0, 5.5, DT);
        let narrow = Road::new(CENTERLINE.to_vec(), 10.0, 3.5, DT);
        assert_eq!(evaluate_coasting(&ego, &[], &wide).aggregate[0], 1.0);
        assert_eq!(evaluate_coasting(&ego, &[], &narrow).aggregate[0], 0.0);
    }
}
