//! Predictive sampling (judo `ps.py`): fixed-sigma Gaussian noise around
//! the nominal, take the single best rollout.

use super::{Knot, Optimizer, OptimizerConfig, noised_knots, ramp};

/// Predictive sampling. The simplest optimizer: perturb the nominal with a
/// fixed sampling std and keep whichever sample scored best (judo's
/// `sampled_knots[rewards.argmax()]`).
pub(crate) struct PredictiveSampling {
    cfg: OptimizerConfig,
    /// Dimensionless sampling std (judo's `sigma`), scaled per control
    /// dimension by [`super::SIGMA_SCALE`] at sampling time.
    sigma: f64,
}

impl Default for PredictiveSampling {
    fn default() -> Self {
        PredictiveSampling {
            cfg: OptimizerConfig::default(),
            sigma: 1.0,
        }
    }
}

impl Optimizer for PredictiveSampling {
    const NAME: &'static str = "predictive sampling (judo)";

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
        // take the best sampled trajectory (judo: rewards.argmax())
        let best = (0..rewards.len())
            .max_by(|&a, &b| rewards[a].total_cmp(&rewards[b]))
            .unwrap();
        sampled[best].clone()
    }
}
