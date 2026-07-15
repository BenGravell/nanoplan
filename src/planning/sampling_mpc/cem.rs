//! Cross-entropy method (judo `cem.py`): adaptive per-node Gaussian, refit
//! to the elite (best-scoring) rollouts each iteration.

use super::{Knot, NU, Optimizer, OptimizerConfig, SIGMA_SCALE, noised_knots, ramp};

/// The cross-entropy method. Unlike predictive sampling and MPPI, which
/// perturb with a fixed sigma, CEM maintains a per-node sampling std and
/// re-fits it to the elite rollouts every iteration: the nominal becomes
/// the elite mean and the std the elite std (clipped), so the distribution
/// contracts around whatever region keeps scoring well.
pub(crate) struct Cem {
    cfg: OptimizerConfig,
    /// Per-node dimensionless sampling std, `[acceleration, curvature]` each,
    /// scaled by [`SIGMA_SCALE`] at sampling time. Adapted in
    /// `update_nominal_knots`; there is one entry per knot.
    sigma: Vec<[f64; NU]>,
    /// Clip bounds on the adapted std (judo's `sigma_min`/`sigma_max`),
    /// dimensionless like [`Cem::sigma`].
    sigma_min: f64,
    sigma_max: f64,
    /// How many top rollouts define the elite set (judo's `num_elites`).
    num_elites: usize,
}

impl Default for Cem {
    fn default() -> Self {
        let cfg = OptimizerConfig::default();
        let sigma_min = 0.1;
        let sigma_max = 2.0;
        let mid = (sigma_min + sigma_max) / 2.0;
        Cem {
            sigma: vec![[mid; NU]; cfg.num_nodes],
            cfg,
            sigma_min,
            sigma_max,
            num_elites: 6,
        }
    }
}

impl Optimizer for Cem {
    const NAME: &'static str = "CEM (judo)";

    fn config(&self) -> OptimizerConfig {
        self.cfg
    }

    fn sample_control_knots(&mut self, nominal: &[Knot], sample_base: usize) -> Vec<Vec<Knot>> {
        // guard against a warm-started nominal whose node count no longer
        // matches this optimizer's adapted-sigma vector
        if self.sigma.len() != nominal.len() {
            let mid = (self.sigma_min + self.sigma_max) / 2.0;
            self.sigma = vec![[mid; NU]; nominal.len()];
        }
        let cfg = self.cfg;
        let sigma = self.sigma.clone();
        noised_knots(nominal, cfg.num_rollouts, sample_base, |n| {
            let r = ramp(&cfg, n);
            [sigma[n][0] * r, sigma[n][1] * r]
        })
    }

    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], rewards: &[f64]) -> Vec<Knot> {
        let num_nodes = sampled[0].len();
        // elite indices: the `num_elites` highest rewards (judo:
        // flip(argsort(rewards))[:num_elites])
        let mut order: Vec<usize> = (0..rewards.len()).collect();
        order.sort_by(|&a, &b| rewards[b].total_cmp(&rewards[a]));
        let elites = &order[..self.num_elites.min(order.len())];

        // nominal ← elite mean; sigma ← clip(elite std)
        let mut nominal = vec![[0.0; NU]; num_nodes];
        for n in 0..num_nodes {
            for c in 0..NU {
                let mean: f64 =
                    elites.iter().map(|&e| sampled[e][n][c]).sum::<f64>() / elites.len() as f64;
                let var: f64 = elites
                    .iter()
                    .map(|&e| (sampled[e][n][c] - mean).powi(2))
                    .sum::<f64>()
                    / elites.len() as f64;
                nominal[n][c] = mean;
                // the elite std is in physical control units; store it back
                // in the dimensionless units SIGMA_SCALE re-inflates at
                // sampling time, then clip to the configured bounds
                self.sigma[n][c] =
                    (var.sqrt() / SIGMA_SCALE[c]).clamp(self.sigma_min, self.sigma_max);
            }
        }
        nominal
    }
}
