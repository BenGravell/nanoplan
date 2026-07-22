//! Pretrained spectral model for generated closed circuits.

use std::f64::consts::TAU;

use crate::common::rng::Rng;
use crate::geometry::{RoadPolygon, polygons_overlap, segments_intersect};
use serde::{Deserialize, Serialize};

use super::loader::{REVISION, SOURCE};

pub(crate) const SAMPLE_COUNT: usize = 256;
const MAX_ATTEMPTS: usize = 64;
const CURVATURE_WIDTH_BUFFER_M: f64 = 0.25;
const MAX_WIDTH_SLOPE: f64 = 0.25;
const MIN_GENERATED_HALF_WIDTH_M: f64 = 2.5;
const COEFFICIENT_COUNT: usize = SAMPLE_COUNT / 2 + 1;
const MODEL: &str = include_str!("trained_model.json");

#[cfg(test)]
pub(crate) struct TrainingTrack {
    pub(crate) length: f64,
    pub(crate) points: Vec<[f64; 2]>,
    pub(crate) right: Vec<f64>,
    pub(crate) left: Vec<f64>,
}

pub(crate) struct GeneratedTrack {
    pub(crate) points: Vec<[f64; 2]>,
    pub(crate) right: Vec<f64>,
    pub(crate) left: Vec<f64>,
}

pub(crate) struct TrackModel {
    profiles: Vec<Profile>,
}

#[derive(Clone, Serialize, Deserialize)]
struct Profile {
    length: f64,
    turning: Vec<Coeff>,
    right: Vec<Coeff>,
    left: Vec<Coeff>,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct Coeff {
    re: f64,
    im: f64,
}

#[derive(Serialize, Deserialize)]
struct StoredModel {
    format_version: u32,
    source: String,
    revision: String,
    sample_count: usize,
    profiles: Vec<Profile>,
}

impl TrackModel {
    pub(crate) fn pretrained() -> Self {
        Self::from_json(MODEL).expect("invalid bundled track model")
    }

    fn from_json(json: &str) -> Result<Self, String> {
        let stored: StoredModel = serde_json::from_str(json).map_err(|error| error.to_string())?;
        if stored.format_version != 1
            || stored.source != SOURCE
            || stored.revision != REVISION
            || stored.sample_count != SAMPLE_COUNT
            || stored.profiles.len() != 24
            || stored.profiles.iter().any(|profile| {
                !profile.length.is_finite()
                    || profile.length <= 0.0
                    || [&profile.turning, &profile.right, &profile.left]
                        .into_iter()
                        .any(|coefficients| {
                            coefficients.len() != COEFFICIENT_COUNT
                                || coefficients.iter().any(|coefficient| {
                                    !coefficient.re.is_finite() || !coefficient.im.is_finite()
                                })
                        })
            })
        {
            return Err("invalid spectral model value".to_owned());
        }
        Ok(Self {
            profiles: stored.profiles,
        })
    }

    #[cfg(test)]
    pub(crate) fn train(tracks: &[TrainingTrack]) -> Result<Self, String> {
        let profiles = tracks
            .iter()
            .map(|track| {
                if track.points.len() != SAMPLE_COUNT
                    || track.right.len() != SAMPLE_COUNT
                    || track.left.len() != SAMPLE_COUNT
                    || !track.length.is_finite()
                    || track.length <= 0.0
                {
                    return Err("invalid spectral training track".to_owned());
                }
                let mut turning = signed_curvature(&track.points)
                    .into_iter()
                    .map(|curvature| curvature * track.length)
                    .collect::<Vec<_>>();
                let winding = if turning.iter().sum::<f64>() < 0.0 {
                    -TAU
                } else {
                    TAU
                };
                let correction = winding - turning.iter().sum::<f64>() / SAMPLE_COUNT as f64;
                for value in &mut turning {
                    *value += correction;
                }
                Ok(Profile {
                    length: track.length,
                    turning: spectrum(&turning),
                    right: spectrum(&track.right),
                    left: spectrum(&track.left),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        if profiles.is_empty() {
            return Err("track model needs training data".to_owned());
        }
        Ok(Self { profiles })
    }

    #[cfg(test)]
    fn to_json(&self) -> String {
        serde_json::to_string_pretty(&StoredModel {
            format_version: 1,
            source: SOURCE.to_owned(),
            revision: REVISION.to_owned(),
            sample_count: SAMPLE_COUNT,
            profiles: self.profiles.clone(),
        })
        .unwrap()
    }

    pub(crate) fn generate(&self, seed: u64) -> Option<GeneratedTrack> {
        let mut rng = Rng(seed ^ 0xd1b5_4a32_d192_ed03);
        for attempt in 0..MAX_ATTEMPTS {
            let profile = &self.profiles
                [(rng.uniform() * self.profiles.len() as f64) as usize % self.profiles.len()];
            let exact = attempt % 16 == 15;
            let phases = phases(&mut rng, exact);
            let mut turning = reconstruct(&profile.turning, &phases);
            let length = profile.length
                * if exact {
                    1.0
                } else {
                    (1.0 + 0.05 * rng.normal()).clamp(0.85, 1.15)
                };
            let Some(points) = close_curve(&mut turning, length) else {
                continue;
            };
            let mut right = reconstruct(&profile.right, &phases);
            let mut left = reconstruct(&profile.left, &phases);
            limit_widths_for_curvature(&points, &mut right, &mut left);
            if right
                .iter()
                .chain(&left)
                .all(|width| *width >= MIN_GENERATED_HALF_WIDTH_M)
                && road_is_simple(&points, &right, &left)
            {
                return Some(GeneratedTrack {
                    points: centered(points),
                    right,
                    left,
                });
            }
        }
        None
    }
}

fn signed_curvature(points: &[[f64; 2]]) -> Vec<f64> {
    (0..points.len())
        .map(|i| {
            let a = points[(i + points.len() - 1) % points.len()];
            let b = points[i];
            let c = points[(i + 1) % points.len()];
            let ab = distance(a, b);
            let bc = distance(b, c);
            let ac = distance(a, c);
            let cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            2.0 * cross / (ab * bc * ac).max(1e-9)
        })
        .collect()
}

pub(super) fn limit_widths_for_curvature(points: &[[f64; 2]], right: &mut [f64], left: &mut [f64]) {
    for ((curvature, right), left) in signed_curvature(points)
        .into_iter()
        .zip(right.iter_mut())
        .zip(left.iter_mut())
    {
        let inner_limit = 1.0 / curvature.abs().max(1e-9) - CURVATURE_WIDTH_BUFFER_M;
        if curvature > 0.0 {
            *left = left.min(inner_limit);
        } else if curvature < 0.0 {
            *right = right.min(inner_limit);
        }
    }
    limit_width_slope(points, right);
    limit_width_slope(points, left);
}

fn limit_width_slope(points: &[[f64; 2]], widths: &mut [f64]) {
    let n = widths.len();
    let mut smooth = (0..3 * n).map(|i| widths[i % n]).collect::<Vec<_>>();
    for i in 1..smooth.len() {
        let step = distance(points[(i - 1) % n], points[i % n]);
        smooth[i] = smooth[i].min(smooth[i - 1] + MAX_WIDTH_SLOPE * step);
    }
    for i in (0..smooth.len() - 1).rev() {
        let step = distance(points[i % n], points[(i + 1) % n]);
        smooth[i] = smooth[i].min(smooth[i + 1] + MAX_WIDTH_SLOPE * step);
    }
    widths.copy_from_slice(&smooth[n..2 * n]);
}

#[cfg(test)]
fn spectrum(values: &[f64]) -> Vec<Coeff> {
    (0..=values.len() / 2)
        .map(|k| {
            let (re, im) = values
                .iter()
                .enumerate()
                .fold((0.0, 0.0), |(re, im), (i, value)| {
                    let angle = TAU * k as f64 * i as f64 / values.len() as f64;
                    (re + value * angle.cos(), im - value * angle.sin())
                });
            Coeff {
                re: re / values.len() as f64,
                im: im / values.len() as f64,
            }
        })
        .collect()
}

fn phases(rng: &mut Rng, exact: bool) -> Vec<f64> {
    let half = SAMPLE_COUNT / 2;
    let shift = TAU * rng.uniform();
    let warp = if exact { 0.0 } else { 0.22 * rng.normal() };
    (0..=half)
        .map(|k| {
            if k == 0 || k == half {
                0.0
            } else {
                k as f64 * shift
                    + warp * (TAU * k as f64 / half as f64).sin()
                    + if exact { 0.0 } else { 0.035 * rng.normal() }
            }
        })
        .collect()
}

fn reconstruct(coefficients: &[Coeff], phases: &[f64]) -> Vec<f64> {
    let n = (coefficients.len() - 1) * 2;
    (0..n)
        .map(|i| {
            coefficients
                .iter()
                .enumerate()
                .map(|(k, coefficient)| {
                    let phase = TAU * k as f64 * i as f64 / n as f64 + phases[k];
                    let value = coefficient.re * phase.cos() - coefficient.im * phase.sin();
                    if k == 0 || k == n / 2 {
                        value
                    } else {
                        2.0 * value
                    }
                })
                .sum()
        })
        .collect()
}

fn close_curve(turning: &mut [f64], length: f64) -> Option<Vec<[f64; 2]>> {
    let winding = if turning.iter().sum::<f64>() < 0.0 {
        -TAU
    } else {
        TAU
    };
    let mean_correction = winding - turning.iter().sum::<f64>() / turning.len() as f64;
    for value in turning.iter_mut() {
        *value += mean_correction;
    }

    let base = turning.to_vec();
    let (mut a, mut b) = (0.0, 0.0);
    for _ in 0..12 {
        apply_closure_harmonics(turning, &base, a, b);
        let (points, error) = integrate(turning, length);
        if error[0].hypot(error[1]) < 0.05 {
            return Some(points);
        }
        let h = 1e-4;
        apply_closure_harmonics(turning, &base, a + h, b);
        let error_a = integrate(turning, length).1;
        apply_closure_harmonics(turning, &base, a, b + h);
        let error_b = integrate(turning, length).1;
        let j00 = (error_a[0] - error[0]) / h;
        let j10 = (error_a[1] - error[1]) / h;
        let j01 = (error_b[0] - error[0]) / h;
        let j11 = (error_b[1] - error[1]) / h;
        let determinant = j00 * j11 - j01 * j10;
        if determinant.abs() < 1e-9 {
            return None;
        }
        a += (-error[0] * j11 + error[1] * j01) / determinant;
        b += (-j00 * error[1] + j10 * error[0]) / determinant;
        if !a.is_finite() || !b.is_finite() || a.abs().max(b.abs()) > 20.0 {
            return None;
        }
    }
    None
}

fn apply_closure_harmonics(values: &mut [f64], base: &[f64], a: f64, b: f64) {
    let length = values.len() as f64;
    for (i, value) in values.iter_mut().enumerate() {
        let angle = TAU * i as f64 / length;
        *value = base[i] + a * angle.cos() + b * angle.sin();
    }
}

fn integrate(turning: &[f64], length: f64) -> (Vec<[f64; 2]>, [f64; 2]) {
    let step = length / turning.len() as f64;
    let (mut point, mut heading) = ([0.0, 0.0], 0.0);
    let mut points = Vec::with_capacity(turning.len());
    for &turn in turning {
        points.push(point);
        let midpoint = heading + 0.5 * turn / turning.len() as f64;
        point[0] += step * midpoint.cos();
        point[1] += step * midpoint.sin();
        heading += turn / turning.len() as f64;
    }
    (points, point)
}

fn centered(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    let center = points.iter().fold([0.0, 0.0], |sum, point| {
        [sum[0] + point[0], sum[1] + point[1]]
    });
    let center = [
        center[0] / points.len() as f64,
        center[1] / points.len() as f64,
    ];
    for point in &mut points {
        point[0] -= center[0];
        point[1] -= center[1];
    }
    points
}

pub(crate) fn is_simple(points: &[[f64; 2]]) -> bool {
    for i in 0..points.len() {
        let (a, b) = (points[i], points[(i + 1) % points.len()]);
        for j in i + 2..points.len() {
            if i == 0 && j == points.len() - 1 {
                continue;
            }
            let (c, d) = (points[j], points[(j + 1) % points.len()]);
            if segments_intersect(a, b, c, d) {
                return false;
            }
        }
    }
    true
}

pub(super) fn road_is_simple(points: &[[f64; 2]], right: &[f64], left: &[f64]) -> bool {
    if !is_simple(points) || points.len() != right.len() || points.len() != left.len() {
        return false;
    }
    let n = points.len();
    let Some(road) = RoadPolygon::new(points.to_vec(), right.to_vec(), left.to_vec(), true) else {
        return false;
    };
    let quads = road.quads().collect::<Vec<_>>();
    quads.iter().all(|quad| is_simple(quad))
        && (0..n).all(|i| {
            (i + 1..n).all(|j| {
                j == i + 1 || (i == 0 && j == n - 1) || !polygons_overlap(&quads[i], &quads[j])
            })
        })
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_model_is_valid() {
        let model = TrackModel::pretrained();
        assert_eq!(model.profiles.len(), 24);
        for seed in 0..32 {
            let track = model.generate(seed).unwrap();
            assert!(road_is_simple(&track.points, &track.right, &track.left));
        }
        assert!(TrackModel::from_json(&model.to_json()).is_ok());
    }

    #[test]
    #[cfg(not(target_family = "wasm"))]
    #[ignore = "maintenance command: downloads training data and rewrites the model"]
    fn regenerate_bundled_model() {
        use crate::track::{circuit::Circuit, loader::download_tracks};

        let tracks = download_tracks()
            .unwrap()
            .iter()
            .map(|csv| Circuit::parse(csv).unwrap().training_track())
            .collect::<Vec<_>>();
        let json = TrackModel::train(&tracks).unwrap().to_json() + "\n";
        std::fs::write(
            concat!(env!("CARGO_MANIFEST_DIR"), "/src/track/trained_model.json"),
            json,
        )
        .unwrap();
    }

    #[test]
    fn a_common_phase_preserves_power_and_cross_spectrum() {
        let a = Coeff { re: 2.0, im: -3.0 };
        let b = Coeff { re: -1.0, im: 4.0 };
        let rotate = |value: Coeff, phase: f64| Coeff {
            re: value.re * phase.cos() - value.im * phase.sin(),
            im: value.re * phase.sin() + value.im * phase.cos(),
        };
        let cross = |x: Coeff, y: Coeff| (x.re * y.re + x.im * y.im, x.im * y.re - x.re * y.im);
        let (rotated_a, rotated_b) = (rotate(a, 1.7), rotate(b, 1.7));
        assert!((a.re.hypot(a.im) - rotated_a.re.hypot(rotated_a.im)).abs() < 1e-12);
        let before = cross(a, b);
        let after = cross(rotated_a, rotated_b);
        assert!((before.0 - after.0).abs() < 1e-12);
        assert!((before.1 - after.1).abs() < 1e-12);
    }

    #[test]
    fn curvature_limits_only_the_inside_width() {
        let points = (0..8)
            .map(|i| {
                let angle = TAU * i as f64 / 8.0;
                [10.0 * angle.cos(), 10.0 * angle.sin()]
            })
            .collect::<Vec<_>>();
        let (mut right, mut left) = (vec![20.0; 8], vec![20.0; 8]);

        limit_widths_for_curvature(&points, &mut right, &mut left);

        assert_eq!(right, vec![20.0; 8]);
        assert!(left.iter().all(|width| *width < 10.0));
    }

    #[test]
    fn curvature_width_caps_change_smoothly_between_stations() {
        let points = (0..8)
            .map(|i| {
                let angle = TAU * i as f64 / 8.0;
                [10.0 * angle.cos(), 10.0 * angle.sin()]
            })
            .collect::<Vec<_>>();
        let mut widths = vec![10.0; points.len()];
        widths[3] = 2.0;

        limit_width_slope(&points, &mut widths);

        assert!((0..points.len()).all(|i| {
            let next = (i + 1) % points.len();
            (widths[next] - widths[i]).abs()
                <= MAX_WIDTH_SLOPE * distance(points[i], points[next]) + 1e-12
        }));
    }

    #[test]
    fn full_road_geometry_rejects_nonlocal_overlap() {
        let points = [
            [0.0, 0.0],
            [5.0, 0.0],
            [10.0, 0.0],
            [10.0, 1.0],
            [5.0, 1.0],
            [0.0, 1.0],
        ];

        assert!(is_simple(&points));
        assert!(!road_is_simple(&points, &[0.75; 6], &[0.75; 6]));
        assert!(road_is_simple(&points, &[0.1; 6], &[0.1; 6]));
    }
}
