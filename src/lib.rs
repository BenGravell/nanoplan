//! Ultra minimalist motion planner for car-like vehicles.
//!
//! Core components:
//! - [`planning`]: the planner interface and one module per planner
//! - [`simulation`]: the kinematic model and closed-loop simulator
//! - [`metrics`]: nuPlan closed-loop quality metrics, one module per metric
//! - [`scenarios`]: scenario data, road geometry, loading, and generation

pub mod metrics;
pub mod planning;
pub mod scenarios;
pub mod simulation;

pub use metrics::Metrics;
pub use planning::{
    BezierIdmPlanner, Context, LatticePlanner, Pi2DdpPlanner, Planner, PlannerKind, StraightPlanner,
};
pub use scenarios::{Path, Scenario};
pub use simulation::{Control, Rollout, Simulator, State, simulate, step};

/// Deterministic xorshift* RNG with Box-Muller normals; avoids a rand
/// dependency and keeps batches and tests reproducible.
pub(crate) struct Rng(pub u64);

impl Rng {
    pub fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }

    /// Uniform sample in [lo, hi).
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }
}

/// Wrap an angle to (-pi, pi].
pub(crate) fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    // ponytail: smoke test that bevy links and boots headless; delete once a real app exists
    #[test]
    fn bevy_app_boots() {
        use bevy::prelude::*;
        App::new().add_plugins(MinimalPlugins).update();
    }
}
