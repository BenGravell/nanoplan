//! Time-colored swept ego footprint.

use std::sync::LazyLock;

use bevy::gizmos::config::GizmoConfigStore;
use bevy::prelude::*;
use colorgrad::{BlendMode, Gradient};

use nanoplan::simulation::MAX_TERMINAL_SPEED_MPS;
use nanoplan::{EGO_FOOTPRINT, State};

use super::draw::{PX_PER_M, px};
use super::live::Live;

const BAND_M: f64 = 0.35;
const ALPHA: f32 = 0.72;

static GRADIENT: LazyLock<colorgrad::LinearGradient> = LazyLock::new(|| {
    let blue: Srgba = Oklcha::new(0.72, 0.14, 255.0, 1.0).into();
    let orange: Srgba = Oklcha::new(0.72, 0.14, 55.0, 1.0).into();
    colorgrad::GradientBuilder::new()
        .colors(&[
            colorgrad::Color::new(blue.red, blue.green, blue.blue, 1.0),
            colorgrad::Color::new(orange.red, orange.green, orange.blue, 1.0),
        ])
        .mode(BlendMode::Oklab)
        .build()
        .expect("two colors form a valid gradient")
});

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

pub(crate) fn draw(gizmos: &mut Gizmos<EgoCarpetGizmos>, ego: State, plan: &[State], dt: f64) {
    let footprints = sample_footprints(ego, plan, dt);
    let bands = carpet_bands(&footprints);
    let horizon = footprints.last().map_or(1.0, |s| s.time).max(f64::EPSILON);

    for band in bands {
        let color = GRADIENT.at((band.time / horizon) as f32);
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
    let half_length = EGO_FOOTPRINT.length * 0.5;
    let extensions = (half_length / BAND_M).ceil() as usize;
    for i in (1..=extensions).rev() {
        centers.push(offset_state(first, -(i as f64 * BAND_M).min(half_length)));
    }
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
        centers.push(offset_state(last, (i as f64 * BAND_M).min(half_length)));
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
        if longitudinal.abs() <= EGO_FOOTPRINT.length * 0.5
            && lateral.abs() <= EGO_FOOTPRINT.width * 0.5
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
}
