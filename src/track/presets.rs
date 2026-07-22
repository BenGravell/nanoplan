//! Deterministic, procedurally constructed test circuits.

use std::f64::consts::PI;

use super::model::GeneratedTrack;
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_LON_ACCEL, ROLLING_RESISTANCE_ACCEL,
};

const STRAIGHT_HALF_WIDTH_M: f64 = 10.0;
const CURVED_HALF_WIDTH_M: f64 = 5.0;
const LARGE_HALF_SEPARATION_M: f64 = 320.0;
const LARGE_END_CAP_REACH_M: f64 = 480.0;
const SMALL_HALF_SEPARATION_M: f64 = 45.0;
const SMALL_END_CAP_REACH_M: f64 = 60.0;
const SMALL_STRAIGHT_LENGTH_M: f64 = 187.0;
const SUPERELLIPSE_EXPONENT: f64 = 0.75;
#[cfg(test)]
const SMALL_REFERENCE_SPEED_FRACTION: f64 = 0.25;
#[cfg(test)]
const SMALL_TARGET_LAP_TIME_S: f64 = 30.0;
const SAMPLE_STEP_M: f64 = 2.0;
const TERMINAL_SPEED_FRACTION: f64 = 0.95;
const CHIRP_CYCLES: f64 = 16.0;
const CHIRP_START_FREQUENCY_RATIO: f64 = 0.1;
const CHIRP_PERIOD_EASE_FRACTION: f64 = 0.30;
const CHIRP_TARGET_CURVATURE: f64 = 0.75 * MAX_ABS_CURVATURE;

#[derive(Clone, Copy)]
pub(crate) struct PresetInfo {
    pub(crate) name: &'static str,
}

pub(crate) const TRACK_PRESETS: [PresetInfo; 2] = [
    PresetInfo {
        name: "Test Track (large)",
    },
    PresetInfo {
        name: "Test Track (small)",
    },
];

/// Distance needed to reach a fraction of drag-limited terminal speed from rest
/// under maximum thrust acceleration.
fn acceleration_distance(speed_fraction: f64) -> f64 {
    let rolling_accel = ROLLING_RESISTANCE_ACCEL;
    assert!(MAX_LON_ACCEL > rolling_accel);
    -(1.0 - speed_fraction * speed_fraction).ln() / (2.0 * AERO_DRAG_ACCEL_COEFFICIENT)
}

pub(crate) fn generate(index: usize) -> GeneratedTrack {
    let _preset = TRACK_PRESETS[index];
    match index {
        0 => generate_large(),
        1 => generate_small(),
        _ => unreachable!(),
    }
}

fn generate_large() -> GeneratedTrack {
    let straight_length = acceleration_distance(TERMINAL_SPEED_FRACTION).ceil();
    let half_length = straight_length / 2.0;
    let mut points = Vec::new();

    // The top straight is deliberately unobstructed: it is the acceleration run.
    sample(&mut points, straight_length, |u| {
        [-half_length + straight_length * u, LARGE_HALF_SEPARATION_M]
    });
    let acceleration_straight_samples = points.len();
    sample(
        &mut points,
        cap_length_estimate(LARGE_END_CAP_REACH_M, LARGE_HALF_SEPARATION_M),
        |u| {
            superellipse_cap(
                half_length,
                PI / 2.0 - PI * u,
                1.0,
                LARGE_END_CAP_REACH_M,
                LARGE_HALF_SEPARATION_M,
                SUPERELLIPSE_EXPONENT,
            )
        },
    );
    let right_cap_end = points.len();

    // Alternating chirp periods are straight, providing room to accelerate and
    // brake between increasingly tight corners. Smootherstep envelopes make each
    // sine period tangent to the neighboring straights and end caps.
    let amplitude = chirp_amplitude(straight_length);
    sample(&mut points, straight_length, |u| {
        let along = straight_length * u;
        [
            half_length - along,
            -LARGE_HALF_SEPARATION_M + chirp_offset(along, straight_length, amplitude),
        ]
    });
    let return_straight_end = points.len();
    sample(
        &mut points,
        cap_length_estimate(LARGE_END_CAP_REACH_M, LARGE_HALF_SEPARATION_M),
        |u| {
            superellipse_cap(
                -half_length,
                -PI / 2.0 + PI * u,
                -1.0,
                LARGE_END_CAP_REACH_M,
                LARGE_HALF_SEPARATION_M,
                SUPERELLIPSE_EXPONENT,
            )
        },
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

fn generate_small() -> GeneratedTrack {
    let straight_length = SMALL_STRAIGHT_LENGTH_M;
    let half_length = straight_length / 2.0;
    let cap_length = cap_length_estimate(SMALL_END_CAP_REACH_M, SMALL_HALF_SEPARATION_M);
    let mut points = Vec::new();

    sample(&mut points, straight_length, |u| {
        [-half_length + straight_length * u, SMALL_HALF_SEPARATION_M]
    });
    sample(&mut points, cap_length, |u| {
        superellipse_cap(
            half_length,
            PI / 2.0 - PI * u,
            1.0,
            SMALL_END_CAP_REACH_M,
            SMALL_HALF_SEPARATION_M,
            SUPERELLIPSE_EXPONENT,
        )
    });
    sample(&mut points, straight_length, |u| {
        [half_length - straight_length * u, -SMALL_HALF_SEPARATION_M]
    });
    sample(&mut points, cap_length, |u| {
        superellipse_cap(
            -half_length,
            -PI / 2.0 + PI * u,
            -1.0,
            SMALL_END_CAP_REACH_M,
            SMALL_HALF_SEPARATION_M,
            SUPERELLIPSE_EXPONENT,
        )
    });

    let widths = vec![CURVED_HALF_WIDTH_M; points.len()];
    GeneratedTrack {
        points,
        right: widths.clone(),
        left: widths,
    }
}

fn cap_length_estimate(reach: f64, half_separation: f64) -> f64 {
    PI * (reach + half_separation) / 2.0
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
    let phase_cycles = CHIRP_CYCLES * frequency_ramp;
    let period = phase_cycles.floor() as usize;
    if period % 2 == 1 || phase_cycles >= CHIRP_CYCLES {
        return 0.0;
    }

    let period_u = phase_cycles.fract();
    let period_envelope = smootherstep(period_u.min(1.0 - period_u) / CHIRP_PERIOD_EASE_FRACTION);
    let phase = 2.0 * PI * phase_cycles;
    amplitude * period_envelope * phase.sin()
}

fn smootherstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
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

fn superellipse_cap(
    center_x: f64,
    angle: f64,
    side: f64,
    reach: f64,
    half_separation: f64,
    exponent: f64,
) -> [f64; 2] {
    [
        center_x + side * reach * angle.cos().abs().powf(exponent),
        half_separation * angle.sin().signum() * angle.sin().abs().powf(exponent),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::{MAX_ABS_LAT_ACCEL, MAX_TERMINAL_SPEED_MPS};

    #[test]
    fn acceleration_straight_reaches_ninety_five_percent_terminal_speed() {
        let terminal = *MAX_TERMINAL_SPEED_MPS;
        let distance = acceleration_distance(TERMINAL_SPEED_FRACTION);
        let aero = AERO_DRAG_ACCEL_COEFFICIENT;
        let net_accel = MAX_LON_ACCEL - ROLLING_RESISTANCE_ACCEL;
        let speed = (net_accel / aero * (1.0 - (-2.0 * aero * distance).exp())).sqrt();
        assert!((speed / terminal - TERMINAL_SPEED_FRACTION).abs() < 1e-12);
    }

    #[test]
    fn large_track_has_the_required_straight_width_transitions_and_chirp() {
        let required = acceleration_distance(TERMINAL_SPEED_FRACTION);
        let generated = generate(0);
        let top = generated
            .points
            .iter()
            .filter(|point| (point[1] - LARGE_HALF_SEPARATION_M).abs() < 1e-12)
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

        let cap_samples = (cap_length_estimate(LARGE_END_CAP_REACH_M, LARGE_HALF_SEPARATION_M)
            / SAMPLE_STEP_M)
            .ceil() as usize;
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

        let u_for_phase_cycles = |phase_cycles: f64| {
            let ramp = phase_cycles / CHIRP_CYCLES;
            (-(CHIRP_START_FREQUENCY_RATIO)
                + (CHIRP_START_FREQUENCY_RATIO.powi(2)
                    + 4.0 * (1.0 - CHIRP_START_FREQUENCY_RATIO) * ramp)
                    .sqrt())
                / (2.0 * (1.0 - CHIRP_START_FREQUENCY_RATIO))
        };
        for period in 0..CHIRP_CYCLES as usize {
            let u = u_for_phase_cycles(period as f64 + 0.25);
            let offset = chirp_offset(u * length, length, amplitude);
            if period % 2 == 0 {
                assert!(offset.abs() > 0.1 * amplitude);
            } else {
                assert_eq!(offset, 0.0, "period {period} must be straight");
            }
        }
        for boundary in 1..CHIRP_CYCLES as usize {
            let along = u_for_phase_cycles(boundary as f64) * length;
            let epsilon = 1e-3;
            assert!((chirp_offset(along - epsilon, length, amplitude) / epsilon).abs() < 1e-6);
            assert!((chirp_offset(along + epsilon, length, amplitude) / epsilon).abs() < 1e-6);
        }

        let count = (length / SAMPLE_STEP_M).ceil() as usize;
        let chirp = (0..=count)
            .map(|i| {
                let along = length * i as f64 / count as f64;
                [along, chirp_offset(along, length, amplitude)]
            })
            .collect::<Vec<_>>();
        let active_period_peaks = (0..CHIRP_CYCLES as usize)
            .step_by(2)
            .map(|period| {
                let from = period as f64 / CHIRP_CYCLES;
                let to = (period + 1) as f64 / CHIRP_CYCLES;
                chirp
                    .windows(3)
                    .enumerate()
                    .filter(|(i, _)| {
                        let u = (*i + 1) as f64 / count as f64;
                        let ramp = CHIRP_START_FREQUENCY_RATIO * u
                            + (1.0 - CHIRP_START_FREQUENCY_RATIO) * u * u;
                        (from..to).contains(&ramp)
                    })
                    .map(|(_, points)| polyline_curvature(points[0], points[1], points[2]))
                    .fold(0.0, f64::max)
            })
            .collect::<Vec<_>>();
        assert!(
            active_period_peaks.windows(2).all(|pair| pair[1] > pair[0]),
            "curvature peaks must ramp up: {active_period_peaks:?}"
        );
        assert!(
            active_period_peaks[0] < 0.15 * CHIRP_TARGET_CURVATURE,
            "curvature peaks: {active_period_peaks:?}"
        );
        assert!(active_period_peaks[7] > 0.9 * CHIRP_TARGET_CURVATURE);
    }

    #[test]
    fn large_end_caps_support_fifty_percent_of_terminal_speed() {
        let generated = generate(0);
        let straight_samples =
            (acceleration_distance(TERMINAL_SPEED_FRACTION).ceil() / SAMPLE_STEP_M).ceil() as usize;
        let cap_samples = (cap_length_estimate(LARGE_END_CAP_REACH_M, LARGE_HALF_SEPARATION_M)
            / SAMPLE_STEP_M)
            .ceil() as usize;
        let peak_curvature = generated.points[straight_samples..straight_samples + cap_samples]
            .windows(3)
            .map(|points| polyline_curvature(points[0], points[1], points[2]))
            .fold(0.0, f64::max);
        let sustainable_speed = (MAX_ABS_LAT_ACCEL / peak_curvature).sqrt();
        let fraction = sustainable_speed / *MAX_TERMINAL_SPEED_MPS;
        assert!(
            (0.50..=0.55).contains(&fraction),
            "speed fraction: {fraction}"
        );
    }

    #[test]
    fn small_track_is_two_straights_with_a_thirty_second_reference_lap() {
        let generated = generate(1);
        let straight_samples = (SMALL_STRAIGHT_LENGTH_M / SAMPLE_STEP_M).ceil() as usize;
        let cap_samples = (cap_length_estimate(SMALL_END_CAP_REACH_M, SMALL_HALF_SEPARATION_M)
            / SAMPLE_STEP_M)
            .ceil() as usize;
        assert_eq!(generated.points.len(), 2 * (straight_samples + cap_samples));
        assert!(
            generated.points[..straight_samples]
                .iter()
                .all(|point| point[1] == SMALL_HALF_SEPARATION_M)
        );
        let lower_start = straight_samples + cap_samples;
        assert!(
            generated.points[lower_start..lower_start + straight_samples]
                .iter()
                .all(|point| point[1] == -SMALL_HALF_SEPARATION_M)
        );

        let lap_length = generated
            .points
            .iter()
            .zip(generated.points.iter().cycle().skip(1))
            .map(|(&a, &b)| point_distance(a, b))
            .sum::<f64>();
        let lap_time = lap_length / (SMALL_REFERENCE_SPEED_FRACTION * *MAX_TERMINAL_SPEED_MPS);
        assert!(
            (lap_time - SMALL_TARGET_LAP_TIME_S).abs() < 0.1,
            "lap time: {lap_time}"
        );
    }
}
