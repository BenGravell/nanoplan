//! Deterministic, procedurally constructed test circuits.

use std::f64::consts::PI;

use super::model::GeneratedTrack;
use crate::vehicle::{
    AIR_DENSITY_KG_M3, DRAG_AREA_M2, EGO_MASS_KG, GRAVITY_MS2, MAX_ABS_CURVATURE, MAX_LON_ACCEL,
    ROLLING_RESISTANCE_COEFF,
};

const STRAIGHT_HALF_WIDTH_M: f64 = 10.0;
const CURVED_HALF_WIDTH_M: f64 = 3.5;
const HALF_SEPARATION_M: f64 = 110.0;
const END_CAP_REACH_M: f64 = 180.0;
const SUPERELLIPSE_POWER: f64 = 4.0;
const SAMPLE_STEP_M: f64 = 2.0;
const TERMINAL_SPEED_FRACTION: f64 = 0.95;
const CHIRP_CYCLES: f64 = 16.0;
const CHIRP_START_FREQUENCY_RATIO: f64 = 0.1;
const CHIRP_EASE_FRACTION: f64 = 0.06;
const CHIRP_TARGET_CURVATURE: f64 = 0.5 * MAX_ABS_CURVATURE;

#[derive(Clone, Copy)]
pub(crate) struct PresetInfo {
    pub(crate) name: &'static str,
}

pub(crate) const TRACK_PRESETS: [PresetInfo; 1] = [PresetInfo { name: "Test Track" }];

/// Distance needed to reach a fraction of drag-limited terminal speed from rest
/// under maximum requested acceleration.
fn acceleration_distance(speed_fraction: f64) -> f64 {
    let rolling_accel = ROLLING_RESISTANCE_COEFF * GRAVITY_MS2;
    let aero_accel_per_speed_squared = 0.5 * AIR_DENSITY_KG_M3 * DRAG_AREA_M2 / EGO_MASS_KG;
    assert!(MAX_LON_ACCEL > rolling_accel);
    -(1.0 - speed_fraction * speed_fraction).ln() / (2.0 * aero_accel_per_speed_squared)
}

pub(crate) fn generate(index: usize) -> GeneratedTrack {
    let _preset = TRACK_PRESETS[index];
    let straight_length = acceleration_distance(TERMINAL_SPEED_FRACTION).ceil();
    let half_length = straight_length / 2.0;
    let mut points = Vec::new();

    // The top straight is deliberately unobstructed: it is the acceleration run.
    sample(&mut points, straight_length, |u| {
        [-half_length + straight_length * u, HALF_SEPARATION_M]
    });
    let acceleration_straight_samples = points.len();
    sample(
        &mut points,
        PI * (END_CAP_REACH_M + HALF_SEPARATION_M) / 2.0,
        |u| superellipse_cap(half_length, PI / 2.0 - PI * u, 1.0),
    );
    let right_cap_end = points.len();

    // A continuous chirped sine contracts its wavelength along the return
    // straight. Smoothstep envelopes join it tangent to both end caps.
    let amplitude = chirp_amplitude(straight_length);
    sample(&mut points, straight_length, |u| {
        let along = straight_length * u;
        [
            half_length - along,
            -HALF_SEPARATION_M + chirp_offset(along, straight_length, amplitude),
        ]
    });
    let return_straight_end = points.len();
    sample(
        &mut points,
        PI * (END_CAP_REACH_M + HALF_SEPARATION_M) / 2.0,
        |u| superellipse_cap(-half_length, -PI / 2.0 + PI * u, -1.0),
    );

    let mut widths = vec![CURVED_HALF_WIDTH_M; points.len()];
    widths[..acceleration_straight_samples].fill(STRAIGHT_HALF_WIDTH_M);
    transition_widths(
        &mut widths,
        &points,
        acceleration_straight_samples,
        right_cap_end,
        points[right_cap_end],
        STRAIGHT_HALF_WIDTH_M,
        CURVED_HALF_WIDTH_M,
    );
    transition_widths(
        &mut widths,
        &points,
        return_straight_end,
        points.len(),
        points[0],
        CURVED_HALF_WIDTH_M,
        STRAIGHT_HALF_WIDTH_M,
    );
    GeneratedTrack {
        points,
        right: widths.clone(),
        left: widths,
    }
}

fn transition_widths(
    widths: &mut [f64],
    points: &[[f64; 2]],
    start: usize,
    end: usize,
    end_point: [f64; 2],
    from: f64,
    to: f64,
) {
    let arc_length = points[start..end]
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .sum::<f64>()
        + point_distance(points[end - 1], end_point);
    let mut traveled = 0.0;
    for i in start..end {
        if i > start {
            traveled += point_distance(points[i - 1], points[i]);
        }
        let u = traveled / arc_length;
        let smoothstep = u * u * (3.0 - 2.0 * u);
        widths[i] = from + (to - from) * smoothstep;
    }
}

fn point_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

fn chirp_offset(along: f64, length: f64, amplitude: f64) -> f64 {
    let u = (along / length).clamp(0.0, 1.0);
    let frequency_ramp =
        CHIRP_START_FREQUENCY_RATIO * u + (1.0 - CHIRP_START_FREQUENCY_RATIO) * u * u;
    let phase = 2.0 * PI * CHIRP_CYCLES * frequency_ramp;
    amplitude * chirp_envelope(u) * phase.sin()
}

fn chirp_envelope(u: f64) -> f64 {
    if u < CHIRP_EASE_FRACTION {
        smoothstep(u / CHIRP_EASE_FRACTION)
    } else if u > 1.0 - CHIRP_EASE_FRACTION {
        smoothstep((1.0 - u) / CHIRP_EASE_FRACTION)
    } else {
        1.0
    }
}

fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

fn chirp_amplitude(length: f64) -> f64 {
    let (mut lower, mut upper) = (0.0, 20.0);
    for _ in 0..60 {
        let middle = (lower + upper) / 2.0;
        if sampled_chirp_peak(length, middle) < CHIRP_TARGET_CURVATURE {
            lower = middle;
        } else {
            upper = middle;
        }
    }
    (lower + upper) / 2.0
}

fn sampled_chirp_peak(length: f64, amplitude: f64) -> f64 {
    let count = (length / SAMPLE_STEP_M).ceil() as usize;
    (0..=count)
        .map(|i| {
            let along = length * i as f64 / count as f64;
            [along, chirp_offset(along, length, amplitude)]
        })
        .collect::<Vec<_>>()
        .windows(3)
        .map(|points| polyline_curvature(points[0], points[1], points[2]))
        .fold(0.0, f64::max)
}

fn polyline_curvature(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    let ab = point_distance(a, b);
    let bc = point_distance(b, c);
    let ac = point_distance(a, c);
    let cross = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs();
    2.0 * cross / (ab * bc * ac).max(1e-9)
}

fn sample(points: &mut Vec<[f64; 2]>, approximate_length: f64, point: impl Fn(f64) -> [f64; 2]) {
    let count = (approximate_length / SAMPLE_STEP_M).ceil() as usize;
    points.extend((0..count).map(|i| point(i as f64 / count as f64)));
}

fn superellipse_cap(center_x: f64, angle: f64, side: f64) -> [f64; 2] {
    let exponent = 2.0 / SUPERELLIPSE_POWER;
    [
        center_x + side * END_CAP_REACH_M * angle.cos().abs().powf(exponent),
        HALF_SEPARATION_M * angle.sin().signum() * angle.sin().abs().powf(exponent),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::physics::terminal_speed_for_accel;

    #[test]
    fn acceleration_straight_reaches_ninety_five_percent_terminal_speed() {
        let terminal = terminal_speed_for_accel(MAX_LON_ACCEL).unwrap();
        let distance = acceleration_distance(TERMINAL_SPEED_FRACTION);
        let aero = 0.5 * AIR_DENSITY_KG_M3 * DRAG_AREA_M2 / EGO_MASS_KG;
        let net_accel = MAX_LON_ACCEL - ROLLING_RESISTANCE_COEFF * GRAVITY_MS2;
        let speed = (net_accel / aero * (1.0 - (-2.0 * aero * distance).exp())).sqrt();
        assert!((speed / terminal - TERMINAL_SPEED_FRACTION).abs() < 1e-12);
    }

    #[test]
    fn preset_has_the_required_straight_width_transitions_and_chirp() {
        let required = acceleration_distance(TERMINAL_SPEED_FRACTION);
        for (index, _preset) in TRACK_PRESETS.iter().enumerate() {
            let generated = generate(index);
            let top = generated
                .points
                .iter()
                .filter(|point| (point[1] - HALF_SEPARATION_M).abs() < 1e-12)
                .map(|point| point[0]);
            let (min_x, max_x) = top.fold((f64::INFINITY, f64::NEG_INFINITY), |bounds, x| {
                (bounds.0.min(x), bounds.1.max(x))
            });
            assert!(max_x - min_x >= required);
            let straight_samples = (required.ceil() / SAMPLE_STEP_M).ceil() as usize;
            assert!(
                generated.right[..straight_samples]
                    .iter()
                    .chain(&generated.left[..straight_samples])
                    .all(|&width| width == STRAIGHT_HALF_WIDTH_M)
            );

            let cap_samples =
                (PI * (END_CAP_REACH_M + HALF_SEPARATION_M) / 2.0 / SAMPLE_STEP_M).ceil() as usize;
            let right_cap = &generated.right[straight_samples..straight_samples + cap_samples];
            assert_eq!(right_cap[0], STRAIGHT_HALF_WIDTH_M);
            assert!(right_cap.windows(2).all(|pair| pair[1] < pair[0]));
            assert!(right_cap.last().unwrap() > &CURVED_HALF_WIDTH_M);
            assert_eq!(
                generated.right[straight_samples + cap_samples],
                CURVED_HALF_WIDTH_M
            );

            let left_cap = &generated.right[generated.right.len() - cap_samples..];
            assert_eq!(left_cap[0], CURVED_HALF_WIDTH_M);
            assert!(left_cap.windows(2).all(|pair| pair[1] > pair[0]));
            assert!(left_cap.last().unwrap() < &STRAIGHT_HALF_WIDTH_M);

            let length = required.ceil();
            let amplitude = chirp_amplitude(length);
            let peak = sampled_chirp_peak(length, amplitude);
            assert!((peak - CHIRP_TARGET_CURVATURE).abs() < 1e-9);
            assert_eq!(chirp_offset(0.0, length, amplitude), 0.0);
            assert!(chirp_offset(length, length, amplitude).abs() < 1e-12);
            let epsilon = 1e-3;
            assert!((chirp_offset(epsilon, length, amplitude) / epsilon).abs() < 1e-6);
            assert!((chirp_offset(length - epsilon, length, amplitude) / epsilon).abs() < 1e-6);

            let initial_frequency = CHIRP_START_FREQUENCY_RATIO;
            let final_frequency = 2.0 - CHIRP_START_FREQUENCY_RATIO;
            assert!(final_frequency > initial_frequency);

            let count = (length / SAMPLE_STEP_M).ceil() as usize;
            let chirp = (0..=count)
                .map(|i| {
                    let along = length * i as f64 / count as f64;
                    [along, chirp_offset(along, length, amplitude)]
                })
                .collect::<Vec<_>>();
            let ramp_end = 1.0 - CHIRP_EASE_FRACTION;
            let bin_peaks = (0..8)
                .map(|bin| {
                    let from = ramp_end * bin as f64 / 8.0;
                    let to = ramp_end * (bin + 1) as f64 / 8.0;
                    chirp
                        .windows(3)
                        .enumerate()
                        .filter(|(i, _)| {
                            let u = (*i + 1) as f64 / count as f64;
                            (from..to).contains(&u)
                        })
                        .map(|(_, points)| polyline_curvature(points[0], points[1], points[2]))
                        .fold(0.0, f64::max)
                })
                .collect::<Vec<_>>();
            assert!(
                bin_peaks.windows(2).all(|pair| pair[1] > pair[0]),
                "curvature peaks must ramp up: {bin_peaks:?}"
            );
            assert!(bin_peaks[0] < 0.1 * CHIRP_TARGET_CURVATURE);
            assert!(bin_peaks[7] > 0.9 * CHIRP_TARGET_CURVATURE);
        }
    }
}
