//! Canonical road strip derived from centerline stations and side widths.

use crate::common::types::Position;

/// A sampled road represented by its source stations and two continuous
/// boundary polylines. All rendered and physical road geometry is derived from
/// this type so corners use the same miter joins everywhere.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RoadPolygon {
    centerline: Vec<Position>,
    right_widths: Vec<f64>,
    left_widths: Vec<f64>,
    right_boundary: Vec<Position>,
    left_boundary: Vec<Position>,
    closed: bool,
}

impl RoadPolygon {
    pub(crate) fn new(
        centerline: Vec<Position>,
        right_widths: Vec<f64>,
        left_widths: Vec<f64>,
        closed: bool,
    ) -> Option<Self> {
        if centerline.len() != right_widths.len()
            || centerline.len() != left_widths.len()
            || centerline.len() < 2
            || centerline.iter().any(|point| !point.is_finite())
            || right_widths
                .iter()
                .chain(&left_widths)
                .any(|width| !width.is_finite() || *width <= 0.0)
        {
            return None;
        }

        let segment_count = if closed { centerline.len() } else { centerline.len() - 1 };
        let normals = (0..segment_count)
            .map(|i| {
                let next = (i + 1) % centerline.len();
                let tangent = [
                    centerline[next].x - centerline[i].x,
                    centerline[next].y - centerline[i].y,
                ];
                let length = tangent[0].hypot(tangent[1]);
                (length >= 1e-9).then_some([-tangent[1] / length, tangent[0] / length])
            })
            .collect::<Option<Vec<_>>>()?;

        let boundaries = (0..centerline.len())
            .map(|i| {
                let (miter, denominator) = if !closed && i == 0 {
                    (normals[0], 1.0)
                } else if !closed && i == centerline.len() - 1 {
                    (normals[i - 1], 1.0)
                } else {
                    let previous = normals[(i + normals.len() - 1) % normals.len()];
                    let next = normals[i % normals.len()];
                    (
                        [previous[0] + next[0], previous[1] + next[1]],
                        1.0 + previous[0] * next[0] + previous[1] * next[1],
                    )
                };
                (denominator > 1e-9).then_some((
                    Position::new(
                        centerline[i].x - right_widths[i] * miter[0] / denominator,
                        centerline[i].y - right_widths[i] * miter[1] / denominator,
                    ),
                    Position::new(
                        centerline[i].x + left_widths[i] * miter[0] / denominator,
                        centerline[i].y + left_widths[i] * miter[1] / denominator,
                    ),
                ))
            })
            .collect::<Option<Vec<_>>>()?;
        let (right_boundary, left_boundary) = boundaries.into_iter().unzip();

        Some(Self {
            centerline,
            right_widths,
            left_widths,
            right_boundary,
            left_boundary,
            closed,
        })
    }

    #[cfg(test)]
    pub(crate) fn uniform<P: Into<Position>>(centerline: Vec<P>, half_width: f64) -> Option<Self> {
        let widths = vec![half_width; centerline.len()];
        Self::new(
            centerline.into_iter().map(Into::into).collect(),
            widths.clone(),
            widths,
            false,
        )
    }

    pub(crate) fn centerline(&self) -> &[Position] {
        &self.centerline
    }

    pub(crate) fn right_widths(&self) -> &[f64] {
        &self.right_widths
    }

    pub(crate) fn left_widths(&self) -> &[f64] {
        &self.left_widths
    }

    pub(crate) fn right_boundary(&self) -> &[Position] {
        &self.right_boundary
    }

    pub(crate) fn left_boundary(&self) -> &[Position] {
        &self.left_boundary
    }

    pub(crate) fn segment_count(&self) -> usize {
        self.centerline.len() - usize::from(!self.closed)
    }

    pub(crate) fn is_closed(&self) -> bool {
        self.closed
    }

    pub(crate) fn quads(&self) -> impl Iterator<Item = [Position; 4]> + '_ {
        (0..self.segment_count()).map(|i| {
            let next = (i + 1) % self.centerline.len();
            [
                self.right_boundary[i],
                self.right_boundary[next],
                self.left_boundary[next],
                self.left_boundary[i],
            ]
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn high_curvature_corner_has_continuous_mitered_boundaries() {
        let road = RoadPolygon::uniform(
            vec![
                Position::new(0.0, 0.0),
                Position::new(10.0, 0.0),
                Position::new(10.0, 10.0),
            ],
            2.0,
        )
        .unwrap();

        assert_eq!(
            road.right_boundary(),
            &[
                Position::new(0.0, -2.0),
                Position::new(12.0, -2.0),
                Position::new(12.0, 10.0)
            ]
        );
        assert_eq!(
            road.left_boundary(),
            &[
                Position::new(0.0, 2.0),
                Position::new(8.0, 2.0),
                Position::new(8.0, 10.0)
            ]
        );
        assert_eq!(road.quads().count(), 2);
    }

    #[test]
    fn each_side_uses_its_own_station_width() {
        let road = RoadPolygon::new(
            vec![Position::new(0.0, 0.0), Position::new(10.0, 0.0)],
            vec![1.0, 2.0],
            vec![3.0, 4.0],
            false,
        )
        .unwrap();

        assert_eq!(
            road.right_boundary(),
            &[Position::new(0.0, -1.0), Position::new(10.0, -2.0)]
        );
        assert_eq!(
            road.left_boundary(),
            &[Position::new(0.0, 3.0), Position::new(10.0, 4.0)]
        );
    }
}
