//! Shared numerical differencing helpers.

/// First-order forward difference from `previous` to `next` over `dt`.
pub(crate) fn forward_difference(previous: f64, next: f64, dt: f64) -> f64 {
    (next - previous) / dt
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn computes_signed_rate_of_change() {
        assert_eq!(forward_difference(2.0, 5.0, 0.5), 6.0);
        assert_eq!(forward_difference(5.0, 2.0, 0.5), -6.0);
    }
}
