use crate::simulation::Position;

/// Estimate the path heading at `b` by bisecting the directions of `a → b` and `b → c`.
/// Panics if either segment has zero length; an exact reversal has no unique bisector.
pub(crate) fn vertex_heading(a: Position, b: Position, c: Position) -> f64 {
    let ab = (b - a).unit();
    let bc = (c - b).unit();
    let tangent = ab + bc;
    tangent.angle()
}

/// Wrap an angle to [-pi, pi).
pub(crate) fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Shortest signed rotation from `from` to `to`.
pub(crate) fn angle_delta(from: f64, to: f64) -> f64 {
    wrap_angle(to - from)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_angle_returns_principal_angle() {
        assert_eq!(wrap_angle(0.0), 0.0);
        assert!((wrap_angle(3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1e-12);
        assert!((wrap_angle(-3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn angle_delta_takes_the_short_arc() {
        let from = std::f64::consts::PI - 0.2;
        let to = -std::f64::consts::PI + 0.2;
        assert!((angle_delta(from, to) - 0.4).abs() < 1e-12);
    }

    #[test]
    fn tangent_heading_bisects_segment_directions() {
        let a = Position::new(0.0, -1.0);
        let b = Position::new(1.0, 0.0);
        let c = Position::new(0.0, 1.0);
        assert!((vertex_heading(a, b, c) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }
}
