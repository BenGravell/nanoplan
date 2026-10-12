//! Predictive sampling: fixed-sigma Gaussian perturbations, retaining the best sample.

use super::Optimizer;

#[derive(Default)]
pub(crate) struct PredictiveSampling;

impl Optimizer for PredictiveSampling {
    const NAME: &'static str = "predictive sampling (judo)";
}
