pub(crate) fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

pub(crate) const fn multiples<const N: usize>(step: f32) -> [f32; N] {
    let mut values = [0.0; N];
    let mut i = 0;
    while i < N {
        values[i] = i as f32 * step;
        i += 1;
    }
    values
}

pub(crate) const fn inclusive_step_count(max: f32, step: f32) -> usize {
    (max / step) as usize + 1
}

/// Smooth rise from zero, asymptotic approach to one.
pub(crate) fn smooth_exp_step(t: f32, t_constant: f32) -> f32 {
    1.0 - (-t / t_constant).exp()
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
    fn multiples_are_evenly_spaced_from_zero() {
        assert_eq!(multiples::<4>(0.5), [0.0, 0.5, 1.0, 1.5]);
        assert_eq!(inclusive_step_count(1.5, 0.5), 4);
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
