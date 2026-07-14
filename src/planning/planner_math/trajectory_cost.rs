use crate::planning::{Context, cost};
use crate::simulation::{Control, State};
use crate::track::Path;

#[derive(Clone, Copy)]
pub(crate) struct TrajectoryCostWeights {
    pub center: f64,
    pub progress: f64,
    pub acceleration: f64,
    pub curvature: f64,
    pub scale: f64,
    pub timed_shared_cost: bool,
}

pub(crate) struct TrajectoryCost<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
    weights: TrajectoryCostWeights,
}

impl<'a, 'b> TrajectoryCost<'a, 'b> {
    pub(crate) fn new(
        path: &'a Path,
        ctx: &'a Context<'b>,
        weights: TrajectoryCostWeights,
    ) -> Self {
        TrajectoryCost { path, ctx, weights }
    }

    pub(crate) fn stage(&self, x: &State, u: Control, t: usize, s_hint: Option<f64>) -> f64 {
        let (s, sample) = super::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        self.stage_sample(s, sample, u, self.ctx.actors, Some(self.path))
    }

    pub(crate) fn stage_with_predicted_actors(
        &self,
        x: &State,
        u: Control,
        t: usize,
        s_hint: Option<f64>,
        predicted_actors: &[State],
    ) -> f64 {
        let (s, mut sample) =
            super::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        sample.t = 0.0;
        self.stage_sample(s, sample, u, predicted_actors, None)
    }

    fn stage_sample(
        &self,
        progress: f64,
        sample: cost::Sample,
        u: Control,
        actors: &[State],
        lane: Option<&Path>,
    ) -> f64 {
        let mut sample = sample;
        sample.accel = u.acceleration;
        sample.curvature = u.curvature;
        let constraints = cost::HardConstraints::new(self.ctx.road.half_width, actors, lane);
        let shared = if self.weights.timed_shared_cost {
            self.ctx
                .time("cost", || constraints.soft_point_cost(&sample))
        } else {
            constraints.soft_point_cost(&sample)
        };
        let structural = self.weights.center * sample.lateral * sample.lateral
            - self.weights.progress * progress
            + self.weights.acceleration * u.acceleration * u.acceleration
            + self.weights.curvature * u.curvature * u.curvature;
        (shared + structural) * self.weights.scale
    }
}
