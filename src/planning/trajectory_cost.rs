use crate::planning::constraints::{HardConstraints, Sample};
use crate::planning::{Context, planner_math};
use crate::simulation::{Control, State};
use crate::track::Path;

pub(crate) struct TrajectoryCost<'a, 'b> {
    path: &'a Path,
    ctx: &'a Context<'b>,
    initial_speed: f64,
}

impl<'a, 'b> TrajectoryCost<'a, 'b> {
    pub(crate) fn new(path: &'a Path, ctx: &'a Context<'b>, initial_speed: f64) -> Self {
        TrajectoryCost {
            path,
            ctx,
            initial_speed,
        }
    }

    pub(crate) fn stage(&self, x: &State, _u: Control, t: usize, s_hint: Option<f64>) -> f64 {
        let (_, sample) =
            planner_math::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        self.stage_sample(sample, self.ctx.actors, false)
    }

    pub(crate) fn stage_with_predicted_actors(
        &self,
        x: &State,
        _u: Control,
        t: usize,
        s_hint: Option<f64>,
        predicted_actors: &[State],
    ) -> f64 {
        let (_, sample) =
            planner_math::state_sample(self.path, x, t as f64 * self.ctx.road.dt, s_hint);
        self.stage_sample(sample, predicted_actors, true)
    }

    fn stage_sample(&self, sample: Sample, actors: &[State], actors_are_predicted: bool) -> f64 {
        let constraints = HardConstraints::new(
            self.ctx.road.half_width,
            actors,
            self.path,
            self.initial_speed,
            self.ctx.road.dt,
        );
        self.ctx.time("cost", || {
            if actors_are_predicted {
                constraints.soft_point_cost_with_predicted_actors(&sample)
            } else {
                constraints.soft_point_cost(&sample)
            }
        })
    }
}
