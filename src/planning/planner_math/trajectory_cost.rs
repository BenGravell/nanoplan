use crate::planning::{Context, cost};
use crate::scenarios::Path;
use crate::simulation::{Control, State};

#[derive(Clone, Copy)]
pub(crate) struct TrajectoryCostWeights {
    pub center: f64,
    pub speed: f64,
    pub jerk: f64,
    pub curvature_rate: f64,
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
        let target = self.ctx.road.target_speed;
        let constraints =
            cost::HardConstraints::new(self.ctx.road.half_width, self.ctx.actors, Some(self.path));
        let shared = if self.weights.timed_shared_cost {
            self.ctx
                .time("cost", || constraints.soft_point_cost(&sample, target))
        } else {
            constraints.soft_point_cost(&sample, target)
        };
        let dv = x.speed - target;
        let structural = self.weights.center * sample.lateral * sample.lateral
            + self.weights.speed * dv * dv
            + self.weights.jerk * u.jerk * u.jerk
            + self.weights.curvature_rate * u.curvature_rate * u.curvature_rate;
        (shared + structural) * self.weights.scale
    }
}
