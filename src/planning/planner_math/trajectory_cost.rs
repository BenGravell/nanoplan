use crate::planning::{Context, cost};
use crate::simulation::{Control, State};
use crate::track::Path;

#[derive(Clone, Copy)]
pub(crate) struct TrajectoryCostWeights {
    pub center: f64,
    pub speed: f64,
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
        let (_, sample) = super::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        self.stage_sample(sample, u, self.ctx.actors, Some(self.path))
    }

    pub(crate) fn stage_with_predicted_actors(
        &self,
        x: &State,
        u: Control,
        t: usize,
        s_hint: Option<f64>,
        predicted_actors: &[State],
    ) -> f64 {
        let (_, mut sample) =
            super::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        sample.t = 0.0;
        self.stage_sample(sample, u, predicted_actors, None)
    }

    fn stage_sample(
        &self,
        sample: cost::Sample,
        u: Control,
        actors: &[State],
        lane: Option<&Path>,
    ) -> f64 {
        let mut sample = sample;
        sample.accel = u.acceleration;
        sample.curvature = u.curvature;
        let target = self.ctx.road.target_speed;
        let constraints = cost::HardConstraints::new(self.ctx.road.half_width, actors, lane);
        let shared = if self.weights.timed_shared_cost {
            self.ctx
                .time("cost", || constraints.soft_point_cost(&sample, target))
        } else {
            constraints.soft_point_cost(&sample, target)
        };
        let dv = sample.speed - target;
        let structural = self.weights.center * sample.lateral * sample.lateral
            + self.weights.speed * dv * dv
            + self.weights.acceleration * u.acceleration * u.acceleration
            + self.weights.curvature * u.curvature * u.curvature;
        (shared + structural) * self.weights.scale
    }
}
