//! Time-colored swept ego footprint.

use bevy::gizmos::config::GizmoConfigStore;
use bevy::prelude::*;
use colorgrad::Gradient;

use crate::common::math::wrap_angle;
use crate::geometry::EGO_FOOTPRINT;
use crate::metrics::Metrics;
use crate::simulation::{MAX_TERMINAL_SPEED_MPS, State};
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};
use crate::viewer::{
    CarpetVisualization,
    colors::{GUPPY, GUPPY_BLUE, GUPPY_ORANGE},
};

use super::super::Live;
use super::super::screen::{PX_PER_M, px};

const BAND_M: f64 = 0.35;
const ALPHA: f32 = 0.72;
const FOOTPRINT_EPSILON_M: f64 = 1e-9;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct EgoCarpetGizmos;

#[derive(Clone, Copy)]
struct TimedState {
    state: State,
    time: f64,
}

pub(crate) fn configure(live: NonSend<Live>, mut configs: ResMut<GizmoConfigStore>) {
    configs.config_mut::<EgoCarpetGizmos>().0.line.width =
        BAND_M as f32 * PX_PER_M * live.camera.zoom * 1.05;
}

pub(crate) fn draw(
    gizmos: &mut Gizmos<EgoCarpetGizmos>,
    ego: State,
    plan: &[State],
    dt: f64,
    visualization: CarpetVisualization,
    metrics: Option<&Metrics>,
) {
    let footprints = sample_footprints(ego, plan, dt);
    let bands = carpet_bands(&footprints);
    let values = visualization_values(ego, plan, dt, visualization, metrics);
    let colormap = match visualization {
        CarpetVisualization::Time => &*GUPPY_BLUE,
        CarpetVisualization::Speed => &*GUPPY_BLUE,
        _ => &*GUPPY,
    };

    for band in bands {
        let index = (band.time / dt).round() as usize;
        let color = colormap.at(values[index.min(values.len() - 1)] as f32);
        let [red, green, blue, _] = color.to_rgba8();
        let color = Color::Srgba(Srgba::new(
            red as f32 / 255.0,
            green as f32 / 255.0,
            blue as f32 / 255.0,
            ALPHA,
        ));
        let forward = Vec2::new(band.state.yaw.cos() as f32, band.state.yaw.sin() as f32);
        let left = Vec2::new(-forward.y, forward.x);
        let center = px(&band.state);
        let half_width = 0.5 * EGO_FOOTPRINT.width as f32 * PX_PER_M;
        gizmos.line_2d(
            center - left * half_width,
            center + left * half_width,
            color,
        );
    }
}

fn visualization_values(
    ego: State,
    plan: &[State],
    dt: f64,
    visualization: CarpetVisualization,
    metrics: Option<&Metrics>,
) -> Vec<f64> {
    if let Some(metrics) = metrics {
        let values = match visualization {
            CarpetVisualization::Safety => metrics.per_tick.iter().map(|v| v[0]).collect(),
            CarpetVisualization::Progress => metrics.per_tick.iter().map(|v| v[1]).collect(),
            CarpetVisualization::Comfort => metrics.per_tick.iter().map(|v| v[2]).collect(),
            CarpetVisualization::Overall => metrics.score_per_tick.clone(),
            _ => vec![],
        };
        if !values.is_empty() {
            return values;
        }
    }

    let states: Vec<_> = std::iter::once(ego)
        .chain(plan.iter().skip(1).copied())
        .collect();
    let raw = match visualization {
        CarpetVisualization::Speed => states.iter().map(|state| state.speed).collect(),
        CarpetVisualization::Time => (0..states.len()).map(|i| i as f64 * dt).collect(),
        CarpetVisualization::LongitudinalAcceleration => {
            padded_forward(&states, |a, b| (b.speed - a.speed) / dt)
        }
        CarpetVisualization::LateralAcceleration => padded_forward(&states, |a, b| {
            let dvx = (b.speed * b.yaw.cos() - a.speed * a.yaw.cos()) / dt;
            let dvy = (b.speed * b.yaw.sin() - a.speed * a.yaw.sin()) / dt;
            -a.yaw.sin() * dvx + a.yaw.cos() * dvy
        }),
        CarpetVisualization::Curvature => padded_forward(&states, |a, b| {
            wrap_angle(b.yaw - a.yaw) / (a.speed.abs().max(0.1) * dt)
        }),
        _ => vec![0.0; states.len()],
    };
    let range = match visualization {
        CarpetVisualization::Speed => (0.0, *MAX_TERMINAL_SPEED_MPS),
        CarpetVisualization::Time => (
            0.0,
            (states.len().saturating_sub(1) as f64 * dt).max(f64::EPSILON),
        ),
        CarpetVisualization::LongitudinalAcceleration => (MIN_LON_ACCEL, MAX_LON_ACCEL),
        CarpetVisualization::LateralAcceleration => (-MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL),
        CarpetVisualization::Curvature => (-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE),
        _ => (0.0, 1.0),
    };
    raw.into_iter()
        .map(|value| ((value - range.0) / (range.1 - range.0)).clamp(0.0, 1.0))
        .collect()
}

fn padded_forward(states: &[State], f: impl Fn(&State, &State) -> f64) -> Vec<f64> {
    let mut result: Vec<_> = states
        .windows(2)
        .map(|pair| f(&pair[0], &pair[1]))
        .collect();
    result.push(result.last().copied().unwrap_or(0.0));
    result
}

fn sample_footprints(ego: State, plan: &[State], dt: f64) -> Vec<TimedState> {
    let max_dt = (0.5 * EGO_FOOTPRINT.length / *MAX_TERMINAL_SPEED_MPS).min(dt);
    let steps = (dt / max_dt).ceil() as usize;
    let mut samples = Vec::with_capacity(plan.len() * steps + 1);
    samples.push(TimedState {
        state: ego,
        time: 0.0,
    });
    let mut previous = ego;
    for (i, &next) in plan.iter().enumerate() {
        for step in 1..=steps {
            let alpha = step as f64 / steps as f64;
            samples.push(TimedState {
                state: interpolate_state(previous, next, alpha),
                time: (i as f64 + alpha) * dt,
            });
        }
        previous = next;
    }
    samples
}

fn carpet_bands(footprints: &[TimedState]) -> Vec<TimedState> {
    let Some(&first) = footprints.first() else {
        return vec![];
    };
    let mut centers = Vec::new();
    let extensions = (EGO_FOOTPRINT.length / BAND_M).ceil() as usize;
    centers.push(first);
    let mut traversed = 0.0;
    let mut next_band = BAND_M;
    for pair in footprints.windows(2) {
        let distance = (pair[1].state.x - pair[0].state.x).hypot(pair[1].state.y - pair[0].state.y);
        while distance > f64::EPSILON && next_band <= traversed + distance {
            let alpha = (next_band - traversed) / distance;
            centers.push(TimedState {
                state: interpolate_state(pair[0].state, pair[1].state, alpha),
                time: pair[0].time + (pair[1].time - pair[0].time) * alpha,
            });
            next_band += BAND_M;
        }
        traversed += distance;
    }
    let last = *footprints.last().unwrap();
    if traversed > 0.0 && next_band - traversed < BAND_M * 0.5 {
        centers.push(last);
    }
    for i in 1..=extensions {
        centers.push(offset_state(
            last,
            (i as f64 * BAND_M).min(EGO_FOOTPRINT.length),
        ));
    }

    centers
        .into_iter()
        .filter_map(|mut band| {
            band.time = mean_occupancy_time(band.state, footprints)?;
            Some(band)
        })
        .collect()
}

fn offset_state(mut timed: TimedState, distance: f64) -> TimedState {
    timed.state.x += distance * timed.state.yaw.cos();
    timed.state.y += distance * timed.state.yaw.sin();
    timed
}

fn mean_occupancy_time(point: State, footprints: &[TimedState]) -> Option<f64> {
    let mut total = 0.0;
    let mut count = 0;
    for sample in footprints {
        let dx = point.x - sample.state.x;
        let dy = point.y - sample.state.y;
        let c = sample.state.yaw.cos();
        let s = sample.state.yaw.sin();
        let longitudinal = dx * c + dy * s;
        let lateral = -dx * s + dy * c;
        if longitudinal >= -FOOTPRINT_EPSILON_M
            && longitudinal <= EGO_FOOTPRINT.length + FOOTPRINT_EPSILON_M
            && lateral.abs() <= EGO_FOOTPRINT.width * 0.5 + FOOTPRINT_EPSILON_M
        {
            total += sample.time;
            count += 1;
        }
    }
    (count > 0).then_some(total / count as f64)
}

fn interpolate_state(previous: State, current: State, alpha: f64) -> State {
    let yaw_delta = (current.yaw - previous.yaw + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    State {
        x: previous.x + (current.x - previous.x) * alpha,
        y: previous.y + (current.y - previous.y) * alpha,
        yaw: previous.yaw + yaw_delta * alpha,
        speed: previous.speed + (current.speed - previous.speed) * alpha,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viewer::DT;

    #[test]
    fn samples_overlap_even_at_terminal_speed() {
        let speed = *MAX_TERMINAL_SPEED_MPS;
        let ego = State {
            speed,
            ..Default::default()
        };
        let samples = sample_footprints(
            ego,
            &[State {
                x: speed * DT,
                speed,
                ..Default::default()
            }],
            DT,
        );

        assert!(samples.windows(2).all(|pair| {
            (pair[1].state.x - pair[0].state.x).hypot(pair[1].state.y - pair[0].state.y)
                <= EGO_FOOTPRINT.length * 0.5 + 1e-12
        }));
    }

    #[test]
    fn uses_mean_time_for_repeated_occupancy() {
        let point = State::default();
        let footprints = [
            TimedState {
                state: point,
                time: 0.0,
            },
            TimedState {
                state: point,
                time: 2.0,
            },
        ];

        assert_eq!(mean_occupancy_time(point, &footprints), Some(1.0));
    }

    #[test]
    fn keeps_the_terminal_footprint_band_at_rotated_headings() {
        let state = State {
            yaw: 0.7,
            ..Default::default()
        };
        let nose = offset_state(TimedState { state, time: 0.0 }, EGO_FOOTPRINT.length);

        assert_eq!(
            mean_occupancy_time(nose.state, &[TimedState { state, time: 0.0 }]),
            Some(0.0)
        );
    }

    #[test]
    fn every_signal_visualization_is_normalized_for_each_planned_tick() {
        let ego = State::default();
        let plan = [State { speed: 2.0, ..ego }, State { speed: 3.0, ..ego }];
        for visualization in [
            CarpetVisualization::Speed,
            CarpetVisualization::Time,
            CarpetVisualization::LongitudinalAcceleration,
            CarpetVisualization::LateralAcceleration,
            CarpetVisualization::Curvature,
        ] {
            let values = visualization_values(ego, &plan, DT, visualization, None);
            assert_eq!(values.len(), plan.len());
            assert!(values.iter().all(|value| (0.0..=1.0).contains(value)));
        }
        assert_eq!(
            visualization_values(ego, &plan, DT, CarpetVisualization::Time, None),
            [0.0, 1.0]
        );
    }

    #[test]
    fn carpet_colormaps_match_metric_signedness() {
        assert_eq!(GUPPY.at(0.0).to_rgba8()[..3], [250, 145, 79]);
        assert_eq!(GUPPY.at(1.0).to_rgba8()[..3], [30, 204, 191]);
        assert_eq!(GUPPY_ORANGE.at(0.0).to_rgba8()[..3], [250, 145, 79]);
        assert_eq!(GUPPY_BLUE.at(0.0).to_rgba8()[..3], [30, 204, 191]);
        assert_eq!(GUPPY_ORANGE.at(1.0), GUPPY.at(0.5));
        assert_eq!(GUPPY_BLUE.at(1.0), GUPPY.at(0.5));
    }
}
