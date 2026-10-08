//! Polynomial helpers.

/// Cubic Hermite polynomial in one coordinate, with coefficients in ascending order.
pub(crate) struct CubicPolynomial(pub(crate) [f64; 4]);

impl CubicPolynomial {
    /// Fit position and velocity at both ends of a positive-duration interval.
    pub(crate) fn from_boundary(p0: f64, v0: f64, p1: f64, v1: f64, duration: f64) -> Self {
        let (t2, t3) = (duration * duration, duration * duration * duration);
        Self([
            p0,
            v0,
            (3.0 * (p1 - p0) - (2.0 * v0 + v1) * duration) / t2,
            (2.0 * (p0 - p1) + (v0 + v1) * duration) / t3,
        ])
    }

    /// Position, velocity, and acceleration at `t`, without time clamping.
    pub(crate) fn at(&self, t: f64) -> [f64; 3] {
        let [c0, c1, c2, c3] = self.0;
        [
            c0 + t * (c1 + t * (c2 + t * c3)),
            c1 + t * (2.0 * c2 + t * 3.0 * c3),
            2.0 * c2 + t * 6.0 * c3,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cubic_matches_boundaries_and_derivatives() {
        for duration in [0.5, 2.0, 10.0] {
            let cubic = CubicPolynomial::from_boundary(2.0, 3.0, -1.0, -0.5, duration);
            assert_eq!(cubic.at(0.0)[..2], [2.0, 3.0]);
            let end = cubic.at(duration);
            assert!((end[0] + 1.0).abs() < 1e-12);
            assert!((end[1] + 0.5).abs() < 1e-12);
        }
        let cubic = CubicPolynomial([-2.0, 3.0, -4.0, 5.0]);
        assert_eq!(cubic.at(0.5), [-0.875, 2.75, 7.0]);
        assert_eq!(cubic.at(-1.0), [-14.0, 26.0, -38.0]);
    }
}
