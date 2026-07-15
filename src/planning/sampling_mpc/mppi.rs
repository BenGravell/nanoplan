//! MPPI (judo `mppi.py`): fixed-sigma Gaussian noise around the nominal,
//! then a Boltzmann (softmax) reward-weighted average of all rollouts.

use super::{Knot, Optimizer, OptimizerConfig, noised_knots, ramp};

/// Model Predictive Path Integral control. Samples like predictive sampling
/// but, instead of taking the single best, blends every rollout by an
/// exponential reward weight (judo's `exp(-(costs - min)/temperature)`,
/// normalized).
pub(crate) struct Mppi {
    cfg: OptimizerConfig,
    /// Dimensionless sampling std (judo's `sigma`).
    sigma: f64,
    /// Softmax temperature (judo's `temperature`): lower sharpens the
    /// weighting toward the best rollout, higher softens it toward a plain
    /// average. Interpreted relative to the rollout cost *spread* (see
    /// `update_nominal_knots`), so it stays a dimensionless `[0, 1]`-ish
    /// knob independent of the rollout's absolute cost scale.
    temperature: f64,
}

impl Default for Mppi {
    fn default() -> Self {
        Mppi {
            cfg: OptimizerConfig::default(),
            sigma: 1.0,
            temperature: 0.1,
        }
    }
}

impl Optimizer for Mppi {
    const NAME: &'static str = "MPPI (judo)";

    fn config(&self) -> OptimizerConfig {
        self.cfg
    }

    fn sample_control_knots(&mut self, nominal: &[Knot], sample_base: usize) -> Vec<Vec<Knot>> {
        let sigma = self.sigma;
        let cfg = self.cfg;
        noised_knots(nominal, cfg.num_rollouts, sample_base, |n| {
            let r = ramp(&cfg, n) * sigma;
            [r, r]
        })
    }

    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot> {
        // judo: costs = -rewards; beta = min(costs);
        // weights ∝ exp(-(costs - beta)/temperature).
        //
        // nanoplan adaptation: divide the exponent by a cost *spread* so
        // `temperature` is a scale-free knob rather than tied to a rollout's
        // absolute cost magnitude — the same idea behind PI²-DDP's min/max
        // normalization of its eq.-12 rollout weighting. But the spread is
        // taken as `median − min`, not `max − min`: a couple of off-road
        // sampled rollouts cost orders of magnitude more than the rest, and
        // an outlier-dominated `max − min` flattens the weights toward a
        // uniform average — which, for a reward-weighted *mean*, just
        // regresses the nominal back to itself and stalls the optimization
        // (the acceleration deviation then drifts until it cancels the base
        // policy's speed hold). The median spread reflects the good cluster
        // and is immune to the outliers, so the weighting stays sharp.
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
        let mut nominal = vec![[0.0; super::NU]; num_nodes];
        for (w, knots) in weights.iter().zip(sampled) {
            let wn = w / wsum;
            for n in 0..num_nodes {
                for c in 0..super::NU {
                    nominal[n][c] += wn * knots[n][c];
                }
            }
        }
        nominal
    }
}
