/// Planner-independent share of a calibrated planning allowance.
/// Each planner converts this into its own useful unit (usually samples).
#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ComputeBudget(u16);

pub(crate) const NOMINAL_COMPUTE_BUDGET_PERCENT: f32 = 100.0;

pub(crate) const COMPUTE_BUDGET_BREAKPOINTS: [f32; 7] =
    [5.0, 10.0, 20.0, 50.0, NOMINAL_COMPUTE_BUDGET_PERCENT, 200.0, 500.0];

const MIN_COMPUTE_BUDGET_PERCENT: f32 = COMPUTE_BUDGET_BREAKPOINTS[0];
const MAX_COMPUTE_BUDGET_PERCENT: f32 = COMPUTE_BUDGET_BREAKPOINTS[COMPUTE_BUDGET_BREAKPOINTS.len() - 1];

impl ComputeBudget {
    pub(crate) const NOMINAL: Self = Self(NOMINAL_COMPUTE_BUDGET_PERCENT as u16);

    pub(crate) fn from_percent(percent: f32) -> Self {
        Self(
            percent
                .round()
                .clamp(MIN_COMPUTE_BUDGET_PERCENT, MAX_COMPUTE_BUDGET_PERCENT) as u16,
        )
    }

    /// Scale a planner's offline-calibrated 100 ms workload.
    pub(crate) fn scale(self, at_100_ms: usize, minimum: usize) -> usize {
        (at_100_ms * self.0 as usize / NOMINAL_COMPUTE_BUDGET_PERCENT as usize).max(minimum)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentage_scales_calibrated_work_with_a_floor() {
        assert_eq!(ComputeBudget::NOMINAL.scale(1_000, 100), 1_000);
        assert_eq!(ComputeBudget::from_percent(25.0).scale(1_000, 100), 250);
        assert_eq!(ComputeBudget::from_percent(1.0).scale(32, 8), 8);
    }

    #[test]
    fn breakpoints_are_strictly_increasing() {
        assert!(COMPUTE_BUDGET_BREAKPOINTS.windows(2).all(|pair| pair[0] < pair[1]));
    }
}
