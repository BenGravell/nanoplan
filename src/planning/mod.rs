//! The planner interface and one module per planner.

pub(crate) mod basic;
pub(crate) mod bezier_toppra;
mod catalog;
mod compute_budget;
mod config;
pub(crate) mod constraints;
pub(crate) mod diagnostics;
pub(crate) mod latency;
pub(crate) mod lattice;
pub(crate) mod pi2ddp;
pub(crate) mod planner_math;
pub(crate) mod policy;
pub(crate) mod rrt_star;
pub(crate) mod sampling;
pub(crate) mod sampling_mpc;
pub(crate) mod search_tree;
pub(crate) mod steering;
pub(crate) mod straight;
mod trajectory_cost;
pub(crate) mod treetop;

pub(crate) use basic::BasicPlanner;
pub(crate) use bezier_toppra::BezierToppraPlanner;
pub(crate) use catalog::PlannerKind;
pub(crate) use compute_budget::{COMPUTE_BUDGET_BREAKPOINTS, ComputeBudget, NOMINAL_COMPUTE_BUDGET_PERCENT};
pub(crate) use config::{PLANNING_DT_S, PLANNING_HORIZON_S, PLANNING_TICKS, warm_start_matches};
pub(crate) use diagnostics::{Diagnostics, DiagnosticsData};
pub(crate) use latency::{Latency, LatencyStats};
pub(crate) use lattice::LatticePlanner;
pub(crate) use pi2ddp::Pi2DdpPlanner;
pub(crate) use rrt_star::RrtStarPlanner;
pub(crate) use sampling_mpc::{Cem, Mppi, PredictiveSampling, SamplingPlanner};
pub(crate) use straight::StraightPlanner;
pub(crate) use trajectory_cost::TrajectoryCost;
pub(crate) use treetop::{IlqrPlanner, RrtPlanner, TreetopPlanner};

use crate::simulation::{Control, State};
use crate::track::Road;

/// Everything a planner sees besides the ego state.
pub(crate) struct Context<'a> {
    /// The fixed setting of the run: centerline, target speed, tick length.
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

impl Context<'_> {
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
pub(crate) trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}

#[cfg(test)]
pub(crate) const TEST_HALF_WIDTH_M: f64 = 5.5;

#[cfg(test)]
pub(crate) fn test_road(centerline: &[[f64; 2]]) -> Road {
    Road::new(centerline.to_vec(), 10.0, TEST_HALF_WIDTH_M, 0.1)
}

#[cfg(test)]
pub(crate) fn test_ctx<'a>(road: &'a Road, actors: &'a [State]) -> Context<'a> {
    Context {
        road,
        actors,
        horizon: 10,
        compute_budget: ComputeBudget::NOMINAL,
        latency: None,
        diagnostics: None,
    }
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
    let mut sim = crate::simulation::Simulator::new(ego, road.dt);
    (0..ticks)
        .map(|_| {
            let command = planner
                .plan(sim.state, &test_ctx(road, actors))
                .first()
                .copied()
                .unwrap_or_default();
            let previous = sim.state;
            sim.step(command);
            sim.state = crate::geometry::barrier::collide_with_road_barriers(
                previous,
                sim.state,
                crate::geometry::EGO_FOOTPRINT,
                road,
            );
            // Planner fixtures describe prescribed obstacle trajectories, not
            // live-world dynamic actors. Keep those fixtures fixed while the
            // production world resolves all vehicles symmetrically.
            sim.state = actors.iter().fold(sim.state, |state, actor| {
                let Some(hit) = crate::geometry::overlap_mtv(
                    state.pose(),
                    crate::geometry::EGO_FOOTPRINT,
                    actor.pose(),
                    crate::geometry::CAR_FOOTPRINT,
                ) else {
                    return state;
                };
                let mut velocity = [state.speed * state.yaw.cos(), state.speed * state.yaw.sin()];
                let normal_speed = velocity[0] * hit.normal[0] + velocity[1] * hit.normal[1];
                if normal_speed < 0.0 {
                    velocity[0] -= 1.1 * normal_speed * hit.normal[0];
                    velocity[1] -= 1.1 * normal_speed * hit.normal[1];
                }
                State {
                    x: state.x + hit.normal[0] * hit.depth,
                    y: state.y + hit.normal[1] * hit.depth,
                    speed: velocity[0].hypot(velocity[1]),
                    ..state
                }
            });
            sim.state
        })
        .collect()
}
