//! Interpolation helpers.

use super::types::State;

/// Linearly interpolate between a and b with ratio t.
/// t=0 -> a, t=1 -> b
pub(crate) fn lerp(a: f64, b: f64, t: f64) -> f64 {
    (1.0 - t) * a + t * b
}

/// Linearly interpolate `fp` at `x` using monotonically increasing sample points `xp`.
/// Values outside the sampled interval use the nearest endpoint.
pub(crate) fn interp1d(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    assert!(!xp.is_empty(), "interp1d requires at least one sample");
    assert_eq!(xp.len(), fp.len(), "xp and fp must have equal lengths");
    assert!(
        xp.windows(2).all(|w| w[0] < w[1]),
        "xp must be strictly increasing"
    );
    if x.is_nan() {
        return f64::NAN;
    }

    if x <= xp[0] {
        return fp[0];
    }
    if x >= xp[xp.len() - 1] {
        return fp[fp.len() - 1];
    }

    let right = xp.partition_point(|&point| point < x);
    let left = right - 1;
    let t = (x - xp[left]) / (xp[right] - xp[left]);
    lerp(fp[left], fp[right], t)
}

pub(crate) fn interpolate_state(previous: State, current: State, alpha: f64) -> State {
    let yaw_delta = (current.yaw - previous.yaw + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    State {
        x: previous.x + (current.x - previous.x) * alpha,
        y: previous.y + (current.y - previous.y) * alpha,
        yaw: previous.yaw + yaw_delta * alpha,
        speed: previous.speed + (current.speed - previous.speed) * alpha,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_between_samples() {
        assert_eq!(interp1d(1.5, &[0.0, 1.0, 3.0], &[0.0, 10.0, 20.0]), 12.5);
    }

    #[test]
    fn uses_endpoint_values_outside_the_interval() {
        let xp = [1.0, 2.0];
        let fp = [10.0, 20.0];
        assert_eq!(interp1d(-1.0, &xp, &fp), 10.0);
        assert_eq!(interp1d(4.0, &xp, &fp), 20.0);
    }

    #[test]
    fn one_sample_is_constant() {
        assert_eq!(interp1d(50.0, &[2.0], &[7.0]), 7.0);
    }
}
