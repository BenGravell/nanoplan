pub(crate) fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

/// Wrap an angle to (-pi, pi].
pub(crate) fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Shortest signed rotation from `from` to `to`.
pub(crate) fn angle_delta(from: f64, to: f64) -> f64 {
    wrap_angle(to - from)
}

/// Interpolate angles along their shortest arc.
pub(crate) fn lerp_angle(from: f64, to: f64, t: f64) -> f64 {
    from + angle_delta(from, to) * t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoothstep_clamps_and_eases() {
        assert_eq!(smoothstep(-1.0), 0.0);
        assert_eq!(smoothstep(0.0), 0.0);
        assert_eq!(smoothstep(0.5), 0.5);
        assert_eq!(smoothstep(1.0), 1.0);
        assert_eq!(smoothstep(2.0), 1.0);
    }

    #[test]
    fn wrap_angle_returns_principal_angle() {
        assert_eq!(wrap_angle(0.0), 0.0);
        assert!((wrap_angle(3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1e-12);
        assert!((wrap_angle(-3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn angle_interpolation_takes_the_short_arc() {
        let from = std::f64::consts::PI - 0.2;
        let to = -std::f64::consts::PI + 0.2;
        assert!((angle_delta(from, to) - 0.4).abs() < 1e-12);
        assert!((lerp_angle(from, to, 0.5) - std::f64::consts::PI).abs() < 1e-12);
    }
}
