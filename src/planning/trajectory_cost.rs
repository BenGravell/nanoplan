use crate::constraints::{Constraints, Sample};
use crate::planning::{Context, planner_math};
use crate::simulation::{Control, State};
use crate::track::Path;

pub(crate) struct TrajectoryCost<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
    initial_station: f64,
    cost_samples: usize,
}

impl<'a, 'b> TrajectoryCost<'a, 'b> {
    pub(crate) fn new(path: &'a Path, ctx: &'a Context<'b>, ego: State) -> Self {
        TrajectoryCost {
            path,
            ctx,
            initial_station: ctx.project_ego(ego).s,
            // Standalone optimizers use PLANNING_TICKS even for a shorter output prefix.
            // Include the terminal cost; refiners may request a longer horizon.
            cost_samples: ctx.horizon.max(crate::planning::PLANNING_TICKS) + 1,
        }
    }

    pub(crate) fn stage(&self, x: &State, u: Control, t: usize, s_hint: Option<f64>) -> f64 {
        let (_, sample) = planner_math::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        self.stage_sample(sample.with_control(u, x.speed), self.ctx.actors, false)
    }

    pub(super) fn stage_sample(&self, sample: Sample, actors: &[State], actors_are_predicted: bool) -> f64 {
        let constraints = Constraints::new(self.ctx.road.half_width, actors, self.path, self.initial_station);
        self.ctx.time("cost", || {
            if actors_are_predicted {
                constraints.soft_point_cost_with_predicted_actors(&sample, self.cost_samples)
            } else {
                constraints.soft_point_cost(&sample, self.cost_samples)
            }
        })
    }
}
