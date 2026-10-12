//! MPPI (judo `mppi.py`): fixed-sigma Gaussian noise around the nominal,
//! then a Boltzmann (softmax) reward-weighted average of all rollouts.

use super::{Knot, NU, Optimizer};

/// Model Predictive Path Integral control. Samples like predictive sampling
/// but, instead of taking the single best, blends every rollout by an
/// exponential reward weight (judo's `exp(-(costs - min)/temperature)`,
/// normalized).
pub(crate) struct Mppi {
    /// Softmax temperature (judo's `temperature`): lower sharpens the
    /// weighting toward the best rollout, higher softens it toward a plain
    /// average. Interpreted relative to the rollout cost *spread* (see
    /// `update_nominal_knots`), so it stays a dimensionless `[0, 1]`-ish
    /// knob independent of the rollout's absolute cost scale.
    temperature: f64,
}

impl Default for Mppi {
    fn default() -> Self {
        Mppi { temperature: 0.1 }
    }
}

impl Optimizer for Mppi {
    const NAME: &'static str = "MPPI (judo)";

    fn num_elites(&self) -> usize {
        usize::MAX
    }

    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot> {
        // Normalize temperature by the median cost spread to avoid outlier-driven weights.
        let costs: Vec<f64> = rewards.iter().map(|r| -r).collect();
        let lo = costs.iter().copied().fold(f64::INFINITY, f64::min);
        let mut sorted = costs.clone();
        sorted.sort_by(f64::total_cmp);
        let median = sorted[sorted.len() / 2];
        let scale = (median - lo).max(1e-9);
        let weights: Vec<f64> = costs
            .iter()
            .map(|c| (-(c - lo) / (self.temperature * scale)).exp())
            .collect();
        let wsum: f64 = weights.iter().sum();

        let num_nodes = sampled[0].len();
        let mut nominal = vec![[0.0; NU]; num_nodes];
        for (w, knots) in weights.iter().zip(sampled) {
            let wn = w / wsum;
            for n in 0..num_nodes {
                for c in 0..NU {
                    nominal[n][c] += wn * knots[n][c];
                }
            }
        }
        nominal
    }
}
