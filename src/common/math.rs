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
}
