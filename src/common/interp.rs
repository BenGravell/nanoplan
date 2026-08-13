//! Interpolation helpers.

use super::types::State;
use crate::common::math::lerp_angle;
use std::ops::{Add, Div, Mul, Sub};

/// Linearly interpolate between a and b with ratio t.
/// t=0 -> a, t=1 -> b
pub(crate) fn lerp<T, U>(a: T, b: T, t: U) -> T
where
    T: Copy + Add<Output = T> + Sub<Output = T> + Mul<U, Output = T>,
{
    a + (b - a) * t
}

/// Return the interpolation ratio of `value` between `a` and `b`.
pub(crate) fn inverse_lerp<T>(a: T, b: T, value: T) -> T
where
    T: Copy + Sub<Output = T> + Div<Output = T>,
{
    (value - a) / (b - a)
}

/// Linearly interpolate `fp` at `x` using monotonically increasing sample points `xp`.
/// Values outside the sampled interval use the nearest endpoint.
pub(crate) fn interp1d(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    assert!(!xp.is_empty(), "interp1d requires at least one sample");
    assert_eq!(xp.len(), fp.len(), "xp and fp must have equal lengths");
    assert!(xp.windows(2).all(|w| w[0] < w[1]), "xp must be strictly increasing");
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
    let t = inverse_lerp(xp[left], xp[right], x);
    lerp(fp[left], fp[right], t)
}

pub(crate) fn lerp_state(previous: State, current: State, alpha: f64) -> State {
    (
        lerp(previous.position(), current.position(), alpha),
        lerp_angle(previous.pose.yaw, current.pose.yaw, alpha),
        lerp(previous.speed, current.speed, alpha),
    )
        .into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::types::Position;

    #[test]
    fn interpolates_between_samples() {
        assert_eq!(interp1d(1.5, &[0.0, 1.0, 3.0], &[0.0, 10.0, 20.0]), 12.5);
    }

    #[test]
    fn lerp_supports_f32() {
        assert_eq!(lerp(1.0_f32, 5.0, 0.25), 2.0);
    }

    #[test]
    fn inverse_lerp_returns_ratio() {
        assert_eq!(inverse_lerp(1.0_f32, 5.0, 2.0), 0.25);
    }

    #[test]
    fn lerp_supports_position() {
        assert_eq!(
            lerp(Position::new(1.0, 2.0), Position::new(5.0, 10.0), 0.25),
            Position::new(2.0, 4.0)
        );
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
