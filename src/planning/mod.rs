//! The planner interface, shared infrastructure, and concrete implementations in [`planners`].

use std::cell::OnceCell;

mod catalog;
mod compute_budget;
mod config;
pub(crate) mod controls;
pub(crate) mod diagnostics;
pub(crate) mod engine;
pub(crate) mod frenet;
pub(crate) mod latency;
pub(crate) mod planner_math;
pub(crate) mod planners;
pub(crate) mod policy;
pub(crate) mod sampling;
pub(crate) mod search_tree;
pub(crate) mod steering;
mod trajectory_cost;
mod warm_start;

pub(crate) use catalog::PlannerKind;
pub(crate) use compute_budget::{COMPUTE_BUDGET_BREAKPOINTS, ComputeBudget, NOMINAL_COMPUTE_BUDGET_PERCENT};
pub(crate) use config::{PLANNING_DT_S, PLANNING_HORIZON_S, PLANNING_TICKS};
pub(crate) use diagnostics::{Diagnostics, DiagnosticsData};
pub(crate) use latency::{Latency, LatencyStats, Span};
pub(crate) use planners::{
    BezierToppraPlanner, Cem, FrenetSamplingPlanner, IlqrPlanner, LatticePlanner, LeeroyJenkinsPlanner, Mppi,
    Pi2DdpPlanner, PredictiveSampling, RrtStarPlanner, SamplingPlanner, TreePlanner, TreetopPlanner,
};
pub(crate) use trajectory_cost::TrajectoryCost;
pub(crate) use warm_start::take_warm;

use crate::common::types::FrenetPosition;
#[cfg(test)]
use crate::simulation::Position;
use crate::simulation::{Control, State};
use crate::track::{Path, Road};

/// Everything a planner sees besides the ego state.
pub(crate) struct Context<'a> {
    path: OnceCell<Path>,
    /// The fixed setting of the run: centerline and tick length.
    pub(crate) road: &'a Road,
    /// Current states of the other actors.
    pub(crate) actors: &'a [State],
    /// Requested number of controls (planners may return fewer or more).
    pub(crate) horizon: usize,
    /// Abstract compute allowance.
    pub(crate) compute_budget: ComputeBudget,
    /// Latency recorder for this plan call, when diagnostics are collected.
    pub(crate) latency: Option<&'a Latency>,
    /// Introspection recorder for this plan call, when a caller (the
    /// viewer's diagnostic overlay) wants to see the planner's search
    /// geometry. See [`diagnostics`] for what each planner records.
    pub(crate) diagnostics: Option<&'a Diagnostics>,
}

impl<'a> Context<'a> {
    pub(crate) fn new(
        road: &'a Road,
        actors: &'a [State],
        horizon: usize,
        compute_budget: ComputeBudget,
        latency: Option<&'a Latency>,
        diagnostics: Option<&'a Diagnostics>,
    ) -> Self {
        Context {
            path: OnceCell::new(),
            road,
            actors,
            horizon,
            compute_budget,
            latency,
            diagnostics,
        }
    }

    pub(crate) fn path(&self) -> &Path {
        self.path.get_or_init(|| self.road.path())
    }

    pub(crate) fn project_ego(&self, ego: State) -> FrenetPosition {
        match self.road.ego_projection_window {
            Some((hint, radius)) => self.path().project_near(ego.position(), hint, radius),
            None => self.path().project(ego.position()),
        }
    }

    /// Time `f` under the seam `name` when diagnostics are on; otherwise
    /// just run it. See [`latency`] for the standardized seam names.
    pub(crate) fn time<T>(&self, name: &'static str, f: impl FnOnce() -> T) -> T {
        match self.latency {
            Some(l) => l.time(name, || {
                let output = f();
                // Every instrumented planner operation has a stable base cost;
                // planners can add data-dependent work with `Context::work`.
                l.work(1);
                output
            }),
            None => f(),
        }
    }

    /// Advance the hardware-independent profiling clock by `clocks` work units.
    pub(crate) fn work(&self, clocks: u64) {
        if let Some(latency) = self.latency {
            latency.work(clocks);
        }
    }
}

/// A planner turns the current 4D state into a direct acceleration/curvature
/// command trajectory. The simulator applies the first command after clamping
/// it to the vehicle's static limits.
pub(crate) trait Planner: Send {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}

#[cfg(test)]
pub(crate) const TEST_HALF_WIDTH_M: f64 = 5.5;

#[cfg(test)]
pub(crate) fn test_road<P: Copy + Into<crate::simulation::Position>>(centerline: &[P]) -> Road {
    Road::new(centerline.to_vec(), TEST_HALF_WIDTH_M, 0.1)
}

#[cfg(test)]
pub(crate) fn test_ctx<'a>(road: &'a Road, actors: &'a [State]) -> Context<'a> {
    Context::new(road, actors, 10, ComputeBudget::NOMINAL, None, None)
}

#[cfg(test)]
#[test]
fn context_reuses_its_path() {
    let road = test_road(&[[0.0, 0.0], [10.0, 0.0]]);
    let ctx = test_ctx(&road, &[]);

    assert!(std::ptr::eq(ctx.path(), ctx.path()));
}

#[cfg(test)]
pub(crate) fn test_run(planner: &mut dyn Planner, ego: State, actors: &[State], ticks: usize) -> Vec<State> {
    let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
    test_run_on(planner, &road, ego, actors, ticks)
}

/// [`test_run`] against a caller-supplied [`Road`], so a test can vary the
/// drivable half-width (or any other road property) the planner sees.
#[cfg(test)]
pub(crate) fn test_run_on(
    planner: &mut dyn Planner,
    road: &Road,
    ego: State,
    actors: &[State],
    ticks: usize,
) -> Vec<State> {
    test_steps_on(planner, road, ego, actors, ticks).collect()
}

/// Lazy closed-loop steps, allowing callers to stop at the first matching state.
#[cfg(test)]
pub(crate) fn test_steps_on<'a>(
    planner: &'a mut dyn Planner,
    road: &'a Road,
    ego: State,
    actors: &'a [State],
    ticks: usize,
) -> impl Iterator<Item = State> + 'a {
    let mut sim = crate::simulation::Simulator::new(ego, road.dt);
    (0..ticks).map(move |_| {
        let command = planner
            .plan(sim.state, &test_ctx(road, actors))
            .first()
            .copied()
            .unwrap_or_default();
        let previous = sim.state;
        sim.step(command);
        sim.state = crate::common::geometry::barrier::collide_with_road_barriers(
            previous,
            sim.state,
            crate::common::geometry::EGO_FOOTPRINT,
            road,
        );
        // Planner fixtures describe prescribed obstacle trajectories, not
        // live-world dynamic actors. Keep those fixtures fixed while the
        // production world resolves all vehicles symmetrically.
        sim.state = actors.iter().fold(sim.state, |state, actor| {
            let Some(hit) = crate::common::geometry::overlap_mtv(
                state.pose(),
                crate::common::geometry::EGO_FOOTPRINT,
                actor.pose(),
                crate::common::geometry::CAR_FOOTPRINT,
            ) else {
                return state;
            };
            let direction = Position::from_angle(state.pose.yaw);
            let mut velocity = [state.speed * direction.x, state.speed * direction.y];
            let normal_speed = velocity[0] * hit.normal[0] + velocity[1] * hit.normal[1];
            if normal_speed < 0.0 {
                velocity[0] -= 1.1 * normal_speed * hit.normal[0];
                velocity[1] -= 1.1 * normal_speed * hit.normal[1];
            }
            {
                let mut state = state;
                state.pose.position.x = state.position().x + hit.normal[0] * hit.depth;
                state.pose.position.y = state.position().y + hit.normal[1] * hit.depth;
                state.speed = velocity[0].hypot(velocity[1]);
                state
            }
        });
        sim.state
    })
}
