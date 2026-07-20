use crate::planning::{Context, cost};
use crate::simulation::{Control, State};
use crate::track::Path;

pub(crate) struct TrajectoryCost<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
}

impl<'a, 'b> TrajectoryCost<'a, 'b> {
    pub(crate) fn new(path: &'a Path, ctx: &'a Context<'b>) -> Self {
        TrajectoryCost { path, ctx }
    }

    pub(crate) fn stage(&self, x: &State, u: Control, t: usize, s_hint: Option<f64>) -> f64 {
        let (_, sample) = super::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        self.stage_sample(sample, u, self.ctx.actors)
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
        self.stage_sample(sample, u, predicted_actors)
    }

    fn stage_sample(&self, sample: cost::Sample, u: Control, actors: &[State]) -> f64 {
        let mut sample = sample;
        sample.accel = u.acceleration;
        sample.curvature = u.curvature;
        let constraints = cost::HardConstraints::new(self.ctx.road.half_width, actors, self.path);
        self.ctx
            .time("cost", || constraints.soft_point_cost(&sample))
    }
}
