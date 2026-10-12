//! Cross-entropy method (judo `cem.py`): adaptive per-node Gaussian, refit
//! to the elite (best-scoring) rollouts each iteration.

use super::{Knot, NU, Optimizer, SIGMA_SCALE};

/// The cross-entropy method. Unlike predictive sampling and MPPI, which
/// perturb with a fixed sigma, CEM maintains a per-node sampling std and
/// re-fits it to the elite rollouts every iteration: the nominal becomes
/// the elite mean and the std the elite std (clipped), so the distribution
/// contracts around whatever region keeps scoring well.
pub(crate) struct Cem {
    /// Per-node dimensionless sampling std, `[station, station speed, lateral offset, lateral speed]` each,
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
        let sigma_min = 0.1;
        let sigma_max = 2.0;
        Cem {
            sigma: Vec::new(),
            sigma_min,
            sigma_max,
            num_elites: 6,
        }
    }
}

impl Optimizer for Cem {
    const NAME: &'static str = "CEM (judo)";

    fn num_elites(&self) -> usize {
        self.num_elites
    }

    fn sigma(&self, node: usize) -> Knot {
        self.sigma
            .get(node)
            .copied()
            .unwrap_or([(self.sigma_min + self.sigma_max) / 2.0; NU])
    }

    fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], _rewards: &[f64]) -> Vec<Knot> {
        let num_nodes = sampled[0].len();
        self.sigma.resize(num_nodes, [0.0; NU]);
        // nominal ← elite mean; sigma ← clip(elite std)
        let mut nominal = vec![[0.0; NU]; num_nodes];
        for n in 0..num_nodes {
            for c in 0..NU {
                let mean: f64 = sampled.iter().map(|knots| knots[n][c]).sum::<f64>() / sampled.len() as f64;
                let var: f64 =
                    sampled.iter().map(|knots| (knots[n][c] - mean).powi(2)).sum::<f64>() / sampled.len() as f64;
                nominal[n][c] = mean;
                // the elite std is in physical endpoint units; store it back
                // in the dimensionless units SIGMA_SCALE re-inflates at
                // sampling time, then clip to the configured bounds
                self.sigma[n][c] = (var.sqrt() / SIGMA_SCALE[c]).clamp(self.sigma_min, self.sigma_max);
            }
        }
        nominal
    }
}
