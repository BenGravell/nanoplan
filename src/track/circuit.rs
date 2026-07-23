//! Closed-circuit samples, parsing, interpolation, and projection.

use super::model::{GeneratedTrack, limit_widths_for_curvature, road_is_simple};
#[cfg(test)]
use super::model::{SAMPLE_COUNT, TrainingTrack};
use crate::geometry::distance::dist;
use splinefit::{ClosedCubicSplineFit2D, evaluate::evaluate};

const SAMPLE_SPACING_M: f64 = 1.0;
const SPLINE_ARC_STEP_M: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub(super) struct Sample {
    pub(super) point: [f64; 2],
    right: f64,
    left: f64,
}

#[derive(Debug)]
pub(super) struct Circuit {
    pub(super) samples: Vec<Sample>,
    distance: Vec<f64>,
    pub(super) length: f64,
}

impl Circuit {
    pub(super) fn parse(csv: &str) -> Result<Self, String> {
        let samples = csv
            .lines()
            .filter(|line| !line.is_empty() && !line.starts_with('#'))
            .enumerate()
            .map(|(index, line)| {
                let fields = line
                    .split(',')
                    .map(str::parse::<f64>)
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|error| format!("line {}: {error}", index + 2))?;
                if fields.len() != 4 || !fields.iter().all(|value| value.is_finite()) {
                    return Err(format!("line {}: expected four finite numbers", index + 2));
                }
                if fields[2] <= 0.0 || fields[3] <= 0.0 {
                    return Err(format!("line {}: track widths must be positive", index + 2));
                }
                Ok(Sample {
                    point: [fields[0], fields[1]],
                    right: fields[2],
                    left: fields[3],
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        if samples.len() < 3 {
            return Err("track needs at least three samples".to_owned());
        }
        let circuit = Self::from_samples(samples);
        if !circuit.length.is_finite() || circuit.length <= 0.0 {
            return Err("track length must be finite and positive".to_owned());
        }
        if circuit
            .samples
            .iter()
            .any(|sample| sample.right <= 0.0 || sample.left <= 0.0)
        {
            return Err("track curvature is too tight for a positive width".to_owned());
        }
        Ok(circuit)
    }

    pub(super) fn generated(track: GeneratedTrack) -> Self {
        Self::from_samples(
            track
                .points
                .into_iter()
                .zip(track.right)
                .zip(track.left)
                .map(|((point, right), left)| Sample { point, right, left })
                .collect(),
        )
    }

    fn from_samples(mut samples: Vec<Sample>) -> Self {
        samples = resample_spline(&samples, SAMPLE_SPACING_M);
        let points = samples
            .iter()
            .map(|sample| sample.point)
            .collect::<Vec<_>>();
        let mut right = samples
            .iter()
            .map(|sample| sample.right)
            .collect::<Vec<_>>();
        let mut left = samples.iter().map(|sample| sample.left).collect::<Vec<_>>();
        limit_widths_for_curvature(&points, &mut right, &mut left);
        for ((sample, right), left) in samples.iter_mut().zip(right).zip(left) {
            sample.right = right;
            sample.left = left;
        }
        let mut distance = vec![0.0];
        for pair in samples.windows(2) {
            distance.push(distance.last().unwrap() + dist(pair[0].point, pair[1].point));
        }
        let length =
            distance.last().unwrap() + dist(samples.last().unwrap().point, samples[0].point);
        Self {
            samples,
            distance,
            length,
        }
    }

    #[cfg(test)]
    pub(super) fn training_track(&self) -> TrainingTrack {
        let mut points = Vec::with_capacity(SAMPLE_COUNT);
        let mut right = Vec::with_capacity(SAMPLE_COUNT);
        let mut left = Vec::with_capacity(SAMPLE_COUNT);
        for i in 0..SAMPLE_COUNT {
            let progress = self.length * i as f64 / SAMPLE_COUNT as f64;
            points.push(self.pose(progress).0);
            let widths = self.widths(progress);
            right.push(widths.0);
            left.push(widths.1);
        }
        TrainingTrack {
            length: self.length,
            points,
            right,
            left,
        }
    }

    fn segment(&self, progress: f64) -> (usize, usize, f64) {
        let progress = progress.rem_euclid(self.length);
        let next = self.distance.partition_point(|&s| s <= progress);
        let a = next.saturating_sub(1);
        let b = next % self.samples.len();
        let start = self.distance[a];
        let length = if b == 0 {
            self.length - start
        } else {
            self.distance[b] - start
        };
        (a, b, (progress - start) / length.max(1e-9))
    }

    pub(super) fn pose(&self, progress: f64) -> ([f64; 2], f64) {
        let (a, b, u) = self.segment(progress);
        let (a, b) = (self.samples[a].point, self.samples[b].point);
        (
            [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u],
            (b[1] - a[1]).atan2(b[0] - a[0]),
        )
    }

    pub(super) fn widths(&self, progress: f64) -> (f64, f64) {
        let (a, b, u) = self.segment(progress);
        let (a, b) = (self.samples[a], self.samples[b]);
        (
            a.right + (b.right - a.right) * u,
            a.left + (b.left - a.left) * u,
        )
    }

    pub(super) fn project(&self, point: [f64; 2], hint: f64) -> f64 {
        let mut best = (0.0, f64::INFINITY);
        for a in 0..self.samples.len() {
            let b = (a + 1) % self.samples.len();
            let (p, q) = (self.samples[a].point, self.samples[b].point);
            let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
            let length_squared = (dx * dx + dy * dy).max(1e-12);
            let u = (((point[0] - p[0]) * dx + (point[1] - p[1]) * dy) / length_squared)
                .clamp(0.0, 1.0);
            let candidate = [p[0] + dx * u, p[1] + dy * u];
            let error = dist(point, candidate);
            if error < best.1 {
                best = (self.distance[a] + length_squared.sqrt() * u, error);
            }
        }
        best.0 + ((hint - best.0) / self.length).round() * self.length
    }

    pub(super) fn is_simple(&self) -> bool {
        road_is_simple(
            &self
                .samples
                .iter()
                .map(|sample| sample.point)
                .collect::<Vec<_>>(),
            &self
                .samples
                .iter()
                .map(|sample| sample.right)
                .collect::<Vec<_>>(),
            &self
                .samples
                .iter()
                .map(|sample| sample.left)
                .collect::<Vec<_>>(),
        )
    }
}

/// Fit a closed, periodic cubic spline through the source stations and return
/// a nearly arc-length-uniform polyline.
fn resample_spline(anchors: &[Sample], spacing: f64) -> Vec<Sample> {
    #[derive(Clone, Copy)]
    struct Station {
        distance: f64,
        parameter: f64,
    }

    let mut anchor_parameters = vec![0.0];
    for segment in 0..anchors.len() {
        let chord = dist(
            anchors[segment].point,
            anchors[(segment + 1) % anchors.len()].point,
        )
        .max(1e-9);
        anchor_parameters.push(anchor_parameters.last().unwrap() + chord);
    }
    let coordinates = anchors
        .iter()
        .chain(std::iter::once(&anchors[0]))
        .flat_map(|anchor| anchor.point)
        .collect();
    let spline = ClosedCubicSplineFit2D::new(anchor_parameters.clone(), coordinates)
        .and_then(ClosedCubicSplineFit2D::interpolating_spline)
        .expect("valid track anchors must produce a periodic cubic spline");
    let evaluate_points = |parameters: &[f64]| {
        evaluate(&spline, parameters)
            .expect("spline evaluation parameters must lie inside the track domain")
            .chunks_exact(2)
            .map(|point| [point[0], point[1]])
            .collect::<Vec<_>>()
    };

    let mut dense_parameters = vec![0.0];
    for segment in 0..anchors.len() {
        let chord = anchor_parameters[segment + 1] - anchor_parameters[segment];
        let steps = (chord / SPLINE_ARC_STEP_M).ceil().max(8.0) as usize;
        for step in 1..=steps {
            let u = step as f64 / steps as f64;
            dense_parameters.push(anchor_parameters[segment] + u * chord);
        }
    }
    let dense_points = evaluate_points(&dense_parameters);
    let mut traveled = 0.0;
    let mut stations = Vec::with_capacity(dense_parameters.len());
    stations.push(Station {
        distance: 0.0,
        parameter: 0.0,
    });
    for i in 1..dense_parameters.len() {
        traveled += dist(dense_points[i - 1], dense_points[i]);
        stations.push(Station {
            distance: traveled,
            parameter: dense_parameters[i],
        });
    }

    let count = (traveled / spacing).ceil().max(3.0) as usize;
    let parameters = (0..count)
        .map(|i| {
            let target = traveled * i as f64 / count as f64;
            let next = stations.partition_point(|station| station.distance < target);
            let b = next.clamp(1, stations.len() - 1);
            let a = b - 1;
            let span = stations[b].distance - stations[a].distance;
            let fraction = (target - stations[a].distance) / span.max(1e-12);
            stations[a].parameter + fraction * (stations[b].parameter - stations[a].parameter)
        })
        .collect::<Vec<_>>();
    evaluate_points(&parameters)
        .into_iter()
        .zip(parameters)
        .map(|(point, parameter)| {
            let segment = anchor_parameters
                .partition_point(|&anchor| anchor <= parameter)
                .saturating_sub(1)
                .min(anchors.len() - 1);
            let u = (parameter - anchor_parameters[segment])
                / (anchor_parameters[segment + 1] - anchor_parameters[segment]);
            let next_anchor = (segment + 1) % anchors.len();
            Sample {
                point,
                right: anchors[segment].right
                    + u * (anchors[next_anchor].right - anchors[segment].right),
                left: anchors[segment].left
                    + u * (anchors[next_anchor].left - anchors[segment].left),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circuit_construction_limits_catalog_widths_for_curvature() {
        let samples = (0..8)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / 8.0;
                Sample {
                    point: [10.0 * angle.cos(), 10.0 * angle.sin()],
                    right: 20.0,
                    left: 20.0,
                }
            })
            .collect();

        let circuit = Circuit::from_samples(samples);

        assert!(circuit.samples.iter().all(|sample| sample.right == 20.0));
        assert!(circuit.samples.iter().all(|sample| sample.left < 20.0));
    }

    #[test]
    fn coarse_anchors_become_a_fine_smooth_centerline() {
        let anchors = (0..8)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / 8.0;
                Sample {
                    point: [20.0 * angle.cos(), 20.0 * angle.sin()],
                    right: 4.0 + i as f64,
                    left: 5.0,
                }
            })
            .collect::<Vec<_>>();

        let samples = resample_spline(&anchors, SAMPLE_SPACING_M);

        assert!(samples.len() > 120);
        assert!(samples.iter().enumerate().all(|(i, sample)| {
            dist(sample.point, samples[(i + 1) % samples.len()].point) <= 1.01
        }));
        assert!(
            samples
                .iter()
                .all(|sample| sample.point[0].hypot(sample.point[1]) > 19.0)
        );
    }
}
