//! Cubic Bezier geometry, independent of the coordinate system and timing.

use crate::common::types::Position;

/// Four control points in a plane (Cartesian x/y or Frenet s/d).
pub(crate) struct CubicBezier(pub(crate) [Position; 4]);

impl CubicBezier {
    /// Position and first two derivatives with respect to t, for t in [0, 1].
    pub(crate) fn at(&self, t: f64) -> [Position; 3] {
        let [a, b, c, d] = self.0;
        let u = 1.0 - t;
        [
            a * u.powi(3) + b * (3.0 * u * u * t) + c * (3.0 * u * t * t) + d * t.powi(3),
            (b - a) * (3.0 * u * u) + (c - b) * (6.0 * u * t) + (d - c) * (3.0 * t * t),
            (c - b * 2.0 + a) * (6.0 * u) + (d - c * 2.0 + b) * (6.0 * t),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluates_control_points_and_analytic_derivatives() {
        let curve = CubicBezier([
            [0.0, 0.0].into(),
            [1.0, 2.0].into(),
            [3.0, 2.0].into(),
            [4.0, 0.0].into(),
        ]);
        assert_eq!(
            curve.at(0.0),
            [[0.0, 0.0].into(), [3.0, 6.0].into(), [6.0, -12.0].into()]
        );
        assert_eq!(
            curve.at(0.5),
            [[2.0, 1.5].into(), [4.5, 0.0].into(), [0.0, -12.0].into()]
        );
        assert_eq!(
            curve.at(1.0),
            [[4.0, 0.0].into(), [3.0, -6.0].into(), [-6.0, -12.0].into()]
        );
    }
}
