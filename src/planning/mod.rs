//! The planner interface and one module per planner.

pub(crate) mod basic;
pub(crate) mod bezier_toppra;
pub(crate) mod constraints;
pub(crate) mod cost;
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
pub(crate) mod treetop;

pub(crate) use basic::BasicPlanner;
pub(crate) use bezier_toppra::BezierToppraPlanner;
pub(crate) use diagnostics::{Diagnostics, DiagnosticsData};
pub(crate) use latency::{Latency, LatencyStats};
pub(crate) use lattice::LatticePlanner;
pub(crate) use pi2ddp::Pi2DdpPlanner;
pub(crate) use rrt_star::RrtStarPlanner;
pub(crate) use sampling_mpc::{Cem, Mppi, PredictiveSampling, SamplingPlanner};
pub(crate) use straight::StraightPlanner;
pub(crate) use treetop::{IlqrPlanner, RrtPlanner, TreetopPlanner};

use crate::simulation::{Control, State};
use crate::track::Road;

/// How far ahead planners with a genuine receding-horizon cost model
/// (lattice, PI²-DDP, RRT*) look when predicting collisions and optimizing a
/// trajectory. Not `Context::horizon`, which is just the requested length
/// of the *returned* control trajectory — see its doc comment.
pub(crate) const PLANNING_HORIZON_S: f64 = 10.0;
pub(crate) const PLANNING_DT_S: f64 = 0.1;
pub(crate) const PLANNING_TICKS: usize = (PLANNING_HORIZON_S / PLANNING_DT_S) as usize;

pub(crate) const WARM_START_MAX_POSITION_ERROR_M: f64 = 1.0;

pub(crate) fn warm_start_matches(expected_next: State, ego: State) -> bool {
    (expected_next.x - ego.x).hypot(expected_next.y - ego.y) < WARM_START_MAX_POSITION_ERROR_M
}

/// Everything a planner sees besides the ego state.
pub(crate) struct Context<'a> {
    /// The fixed setting of the run: centerline, target speed, tick length.
    pub(crate) road: &'a Road,
    /// Current states of the other actors.
    pub(crate) actors: &'a [State],
    /// Requested number of controls (planners may return fewer or more).
    pub(crate) horizon: usize,
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
            Some(l) => l.time(name, f),
            None => f(),
        }
    }
}

/// A planner turns the current 4D state into a direct acceleration/curvature
/// command trajectory. The simulator applies the first command after clamping
/// it to the vehicle's static limits.
pub(crate) trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}

/// Configuration: which planner to run. Everything else about a planner —
/// display name, constructor, capabilities — lives in its [`PlannerSpec`]
/// row, so adding a planner means one enum variant plus one complete row
/// in [`SPECS`], not edits to scattered `match`es.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum PlannerKind {
    Straight,
    Basic,
    BezierToppra,
    Lattice,
    Pi2Ddp,
    RrtStar,
    PredictiveSampling,
    Cem,
    Mppi,
    Rrt,
    Ilqr,
    Treetop,
}

/// One planner, whole: the registry metadata behind a [`PlannerKind`]
/// (Factory Method as a table: the `build` slot constructs the strategy).
pub(crate) struct PlannerSpec {
    /// Which kind this row describes — must match its position in [`SPECS`]
    /// (enforced by the `specs_align_with_kinds` test).
    pub(crate) kind: PlannerKind,
    pub(crate) name: &'static str,
    pub(crate) build: fn() -> Box<dyn Planner>,
    /// Whether this planner records anything into a [`Diagnostics`]
    /// recorder. Planners without receding-horizon search geometry to show
    /// never call `ctx.diagnostics`.
    pub(crate) has_diagnostics: bool,
}

/// The planner registry, indexed by `PlannerKind as usize`.
const SPECS: [PlannerSpec; 12] = [
    PlannerSpec {
        kind: PlannerKind::Straight,
        name: "straight (strawman)",
        build: || Box::new(StraightPlanner),
        has_diagnostics: false,
    },
    PlannerSpec {
        kind: PlannerKind::Basic,
        name: "basic cubic",
        build: || Box::new(BasicPlanner),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::BezierToppra,
        name: "bezier + TOPP-RA",
        build: || Box::new(BezierToppraPlanner),
        has_diagnostics: false,
    },
    PlannerSpec {
        kind: PlannerKind::Lattice,
        name: "frenet lattice",
        build: || Box::new(LatticePlanner),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Pi2Ddp,
        name: "PI2-DDP",
        build: || Box::new(Pi2DdpPlanner::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::RrtStar,
        name: "RRT*",
        build: || Box::new(RrtStarPlanner::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::PredictiveSampling,
        name: SamplingPlanner::<PredictiveSampling>::NAME,
        build: || Box::new(SamplingPlanner::<PredictiveSampling>::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Cem,
        name: SamplingPlanner::<Cem>::NAME,
        build: || Box::new(SamplingPlanner::<Cem>::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Mppi,
        name: SamplingPlanner::<Mppi>::NAME,
        build: || Box::new(SamplingPlanner::<Mppi>::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Rrt,
        name: "RRT (treetop tree)",
        build: || Box::new(RrtPlanner::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Ilqr,
        name: "iLQR (finite diff)",
        build: || Box::new(IlqrPlanner::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Treetop,
        name: "treetop (RRT+iLQR)",
        build: || Box::new(TreetopPlanner::default()),
        has_diagnostics: true,
    },
];

impl PlannerKind {
    pub(crate) const ALL: [PlannerKind; 12] = [
        PlannerKind::Straight,
        PlannerKind::Basic,
        PlannerKind::BezierToppra,
        PlannerKind::Lattice,
        PlannerKind::Pi2Ddp,
        PlannerKind::RrtStar,
        PlannerKind::PredictiveSampling,
        PlannerKind::Cem,
        PlannerKind::Mppi,
        PlannerKind::Rrt,
        PlannerKind::Ilqr,
        PlannerKind::Treetop,
    ];

    /// This planner's registry row.
    pub(crate) fn spec(self) -> &'static PlannerSpec {
        let spec = &SPECS[self as usize];
        debug_assert_eq!(spec.kind, self);
        spec
    }

    pub(crate) fn name(self) -> &'static str {
        self.spec().name
    }

    pub(crate) fn build(self) -> Box<dyn Planner> {
        (self.spec().build)()
    }

    /// See [`PlannerSpec::has_diagnostics`].
    pub(crate) fn has_diagnostics(self) -> bool {
        self.spec().has_diagnostics
    }
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
        latency: None,
        diagnostics: None,
    }
}

#[cfg(test)]
pub(crate) fn test_run(
    planner: &mut dyn Planner,
    ego: State,
    actors: &[State],
    ticks: usize,
) -> Vec<State> {
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
            sim.step(
                command,
                road,
                actors
                    .iter()
                    .copied()
                    .map(|actor| (actor, crate::geometry::CAR_FOOTPRINT)),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `spec()` indexes SPECS by discriminant; a row out of order would
    /// silently hand every planner the wrong name/constructor/flags.
    #[test]
    fn specs_align_with_kinds() {
        assert_eq!(PlannerKind::ALL.len(), SPECS.len());
        for kind in PlannerKind::ALL {
            assert_eq!(kind.spec().kind, kind);
        }
    }
}
