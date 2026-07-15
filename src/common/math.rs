pub(crate) fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

pub(crate) fn signed_fraction(value: f32, positive_max: f32, negative_max: f32) -> f32 {
    if value >= 0.0 {
        (value / positive_max).clamp(0.0, 1.0)
    } else {
        (value / negative_max).clamp(-1.0, 0.0)
    }
}

/// Wrap an angle to (-pi, pi].
pub(crate) fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
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
    fn signed_fraction_uses_each_side_of_zero() {
        assert_eq!(signed_fraction(5.0, 10.0, 20.0), 0.5);
        assert_eq!(signed_fraction(-5.0, 10.0, 20.0), -0.25);
        assert_eq!(signed_fraction(30.0, 10.0, 20.0), 1.0);
    }
}
