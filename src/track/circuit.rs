//! Closed-circuit samples, parsing, interpolation, and projection.

#[cfg(feature = "track-pregeneration")]
use super::presets::PresetTrack;
use crate::common::interp::lerp;
use crate::geometry::distance::dist;
#[cfg(any(test, feature = "track-pregeneration"))]
use crate::geometry::{RoadPolygon, polygons_overlap, segments_intersect};
use crate::simulation::Position;
#[cfg(any(test, feature = "track-pregeneration"))]
use splinefit::{ClosedCubicSplineFit2D, evaluate::evaluate};

#[cfg(any(test, feature = "track-pregeneration"))]
const SAMPLE_SPACING_M: f64 = 1.0;
#[cfg(any(test, feature = "track-pregeneration"))]
const SPLINE_ARC_STEP_M: f64 = 0.25;
#[cfg(any(test, feature = "track-pregeneration"))]
const CURVATURE_WIDTH_BUFFER_M: f64 = 0.25;
#[cfg(any(test, feature = "track-pregeneration"))]
const MAX_WIDTH_SLOPE: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub(super) struct Sample {
    pub(super) point: Position,
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
    #[cfg(feature = "track-pregeneration")]
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
                    point: Position::new(fields[0], fields[1]),
                    right: fields[2],
                    left: fields[3],
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        if samples.len() < 3 {
            return Err("track needs at least three samples".to_owned());
        }
        let circuit = Self::processed(samples);
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

    #[cfg(feature = "track-pregeneration")]
    pub(super) fn baked_csv(&self) -> String {
        use std::fmt::Write;

        let mut csv = "# x_m,y_m,w_tr_right_m,w_tr_left_m\n".to_owned();
        for sample in &self.samples {
            writeln!(
                csv,
                "{},{},{},{}",
                sample.point.x as f32, sample.point.y as f32, sample.right as f32, sample.left as f32
            )
            .unwrap();
        }
        csv
    }

    #[cfg(feature = "track-pregeneration")]
    pub(super) fn preset(track: PresetTrack) -> Self {
        Self::processed(
            track
                .points
                .into_iter()
                .zip(track.right)
                .zip(track.left)
                .map(|((point, right), left)| Sample { point, right, left })
                .collect(),
        )
    }

    pub(super) fn baked(csv: &str) -> Self {
        let samples = csv
            .lines()
            .filter(|line| !line.is_empty() && !line.starts_with('#'))
            .map(|line| {
                let mut fields = line.split(',');
                let mut value = || fields.next().unwrap().parse::<f32>().unwrap() as f64;
                Sample {
                    point: Position::new(value(), value()),
                    right: value(),
                    left: value(),
                }
            })
            .collect();
        Self::finish(samples)
    }

    #[cfg(any(test, feature = "track-pregeneration"))]
    fn processed(samples: Vec<Sample>) -> Self {
        Self::from_samples(resample_spline(&samples, SAMPLE_SPACING_M))
    }

    #[cfg(any(test, feature = "track-pregeneration"))]
    fn from_samples(mut samples: Vec<Sample>) -> Self {
        let points = samples.iter().map(|sample| sample.point).collect::<Vec<_>>();
        let mut right = samples.iter().map(|sample| sample.right).collect::<Vec<_>>();
        let mut left = samples.iter().map(|sample| sample.left).collect::<Vec<_>>();
        limit_widths_for_curvature(&points, &mut right, &mut left);
        for ((sample, right), left) in samples.iter_mut().zip(right).zip(left) {
            sample.right = right;
            sample.left = left;
        }
        Self::finish(samples)
    }

    fn finish(samples: Vec<Sample>) -> Self {
        let mut distance = vec![0.0];
        for pair in samples.windows(2) {
            distance.push(distance.last().unwrap() + dist(pair[0].point, pair[1].point));
        }
        let length = distance.last().unwrap() + dist(samples.last().unwrap().point, samples[0].point);
        Self {
            samples,
            distance,
            length,
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

    pub(super) fn pose(&self, progress: f64) -> (Position, f64) {
        let (a, b, u) = self.segment(progress);
        let (a, b) = (self.samples[a].point, self.samples[b].point);
        (lerp(a, b, u), (b.y - a.y).atan2(b.x - a.x))
    }

    pub(super) fn widths(&self, progress: f64) -> (f64, f64) {
        let (a, b, u) = self.segment(progress);
        let (a, b) = (self.samples[a], self.samples[b]);
        (a.right + (b.right - a.right) * u, a.left + (b.left - a.left) * u)
    }

    pub(super) fn project(&self, point: Position, hint: f64) -> f64 {
        let mut best = (0.0, f64::INFINITY);
        for a in 0..self.samples.len() {
            let b = (a + 1) % self.samples.len();
            let (p, q) = (self.samples[a].point, self.samples[b].point);
            let (dx, dy) = (q.x - p.x, q.y - p.y);
            let length_squared = (dx * dx + dy * dy).max(1e-12);
            let u = (((point.x - p.x) * dx + (point.y - p.y) * dy) / length_squared).clamp(0.0, 1.0);
            let candidate = Position::new(p.x + dx * u, p.y + dy * u);
            let error = dist(point, candidate);
            if error < best.1 {
                best = (self.distance[a] + length_squared.sqrt() * u, error);
            }
        }
        best.0 + ((hint - best.0) / self.length).round() * self.length
    }

    #[cfg(any(test, feature = "track-pregeneration"))]
    pub(super) fn is_simple(&self) -> bool {
        road_is_simple(
            &self.samples.iter().map(|sample| sample.point).collect::<Vec<_>>(),
            &self.samples.iter().map(|sample| sample.right).collect::<Vec<_>>(),
            &self.samples.iter().map(|sample| sample.left).collect::<Vec<_>>(),
        )
    }
}

#[cfg(any(test, feature = "track-pregeneration"))]
fn limit_widths_for_curvature(points: &[Position], right: &mut [f64], left: &mut [f64]) {
    for i in 0..points.len() {
        let a = points[(i + points.len() - 1) % points.len()];
        let b = points[i];
        let c = points[(i + 1) % points.len()];
        let ab = dist(a, b);
        let bc = dist(b, c);
        let ac = dist(a, c);
        let curvature = 2.0 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / (ab * bc * ac).max(1e-9);
        let inner_limit = 1.0 / curvature.abs().max(1e-9) - CURVATURE_WIDTH_BUFFER_M;
        if curvature > 0.0 {
            left[i] = left[i].min(inner_limit);
        } else if curvature < 0.0 {
            right[i] = right[i].min(inner_limit);
        }
    }
    limit_width_slope(points, right);
    limit_width_slope(points, left);
}

#[cfg(any(test, feature = "track-pregeneration"))]
fn limit_width_slope(points: &[Position], widths: &mut [f64]) {
    let n = widths.len();
    let mut smooth = (0..3 * n).map(|i| widths[i % n]).collect::<Vec<_>>();
    for i in 1..smooth.len() {
        smooth[i] = smooth[i].min(smooth[i - 1] + MAX_WIDTH_SLOPE * dist(points[(i - 1) % n], points[i % n]));
    }
    for i in (0..smooth.len() - 1).rev() {
        smooth[i] = smooth[i].min(smooth[i + 1] + MAX_WIDTH_SLOPE * dist(points[i % n], points[(i + 1) % n]));
    }
    widths.copy_from_slice(&smooth[n..2 * n]);
}

#[cfg(any(test, feature = "track-pregeneration"))]
fn is_simple(points: &[Position]) -> bool {
    for i in 0..points.len() {
        let (a, b) = (points[i], points[(i + 1) % points.len()]);
        for j in i + 2..points.len() {
            if (i != 0 || j != points.len() - 1) && segments_intersect(a, b, points[j], points[(j + 1) % points.len()])
            {
                return false;
            }
        }
    }
    true
}

#[cfg(any(test, feature = "track-pregeneration"))]
fn road_is_simple(points: &[Position], right: &[f64], left: &[f64]) -> bool {
    if !is_simple(points) || points.len() != right.len() || points.len() != left.len() {
        return false;
    }
    let Some(road) = RoadPolygon::new(points.to_vec(), right.to_vec(), left.to_vec(), true) else {
        return false;
    };
    let quads = road.quads().collect::<Vec<_>>();
    quads.iter().all(|quad| is_simple(quad))
        && (0..quads.len()).all(|i| {
            (i + 1..quads.len())
                .all(|j| j == i + 1 || (i == 0 && j == quads.len() - 1) || !polygons_overlap(&quads[i], &quads[j]))
        })
}

/// Fit a closed, periodic cubic spline through the source stations and return
/// a nearly arc-length-uniform polyline.
#[cfg(any(test, feature = "track-pregeneration"))]
fn resample_spline(anchors: &[Sample], spacing: f64) -> Vec<Sample> {
    #[derive(Clone, Copy)]
    struct Station {
        distance: f64,
        parameter: f64,
    }

    let mut anchor_parameters = vec![0.0];
    for segment in 0..anchors.len() {
        let chord = dist(anchors[segment].point, anchors[(segment + 1) % anchors.len()].point).max(1e-9);
        anchor_parameters.push(anchor_parameters.last().unwrap() + chord);
    }
    let coordinates = anchors
        .iter()
        .chain(std::iter::once(&anchors[0]))
        .flat_map(|anchor| anchor.point.xy())
        .collect();
    let spline = ClosedCubicSplineFit2D::new(anchor_parameters.clone(), coordinates)
        .and_then(ClosedCubicSplineFit2D::interpolating_spline)
        .expect("valid track anchors must produce a periodic cubic spline");
    let evaluate_points = |parameters: &[f64]| {
        evaluate(&spline, parameters)
            .expect("spline evaluation parameters must lie inside the track domain")
            .chunks_exact(2)
            .map(|point| Position::new(point[0], point[1]))
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
                right: anchors[segment].right + u * (anchors[next_anchor].right - anchors[segment].right),
                left: anchors[segment].left + u * (anchors[next_anchor].left - anchors[segment].left),
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
                let unit = Position::from_angle(angle);
                Sample {
                    point: Position::new(10.0 * unit.x, 10.0 * unit.y),
                    right: 20.0,
                    left: 20.0,
                }
            })
            .collect();

        let circuit = Circuit::processed(samples);

        assert!(circuit.samples.iter().all(|sample| sample.right == 20.0));
        assert!(circuit.samples.iter().all(|sample| sample.left < 20.0));
    }

    #[test]
    fn coarse_anchors_become_a_fine_smooth_centerline() {
        let anchors = (0..8)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / 8.0;
                let unit = Position::from_angle(angle);
                Sample {
                    point: Position::new(20.0 * unit.x, 20.0 * unit.y),
                    right: 4.0 + i as f64,
                    left: 5.0,
                }
            })
            .collect::<Vec<_>>();

        let samples = resample_spline(&anchors, SAMPLE_SPACING_M);

        assert!(samples.len() > 120);
        assert!(
            samples
                .iter()
                .enumerate()
                .all(|(i, sample)| { dist(sample.point, samples[(i + 1) % samples.len()].point) <= 1.01 })
        );
        assert!(samples.iter().all(|sample| sample.point.x.hypot(sample.point.y) > 19.0));
    }
}
