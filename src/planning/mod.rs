//! The planner interface and one module per planner.

pub mod basic;
pub mod bezier_idm;
pub(crate) mod constraints;
pub(crate) mod cost;
pub mod diagnostics;
pub mod latency;
pub mod lattice;
pub mod pi2ddp;
pub(crate) mod planner_math;
pub mod rrt_star;
pub(crate) mod sampling;
pub mod sampling_mpc;
pub(crate) mod search_tree;
pub(crate) mod steering;
pub mod straight;
pub mod treetop;

pub use basic::BasicPlanner;
pub use bezier_idm::BezierIdmPlanner;
pub use diagnostics::{Diagnostics, DiagnosticsData};
pub use latency::{Latency, LatencyStats, SeamStats};
pub use lattice::LatticePlanner;
pub use pi2ddp::Pi2DdpPlanner;
pub use rrt_star::RrtStarPlanner;
pub use sampling_mpc::{Cem, Mppi, PredictiveSampling, SamplingPlanner};
pub use straight::StraightPlanner;
pub use treetop::{IlqrPlanner, RrtPlanner, TreetopPlanner};

use crate::scenarios::Road;
use crate::simulation::{Control, State};

/// How far ahead planners with a genuine receding-horizon cost model
/// (lattice, PI²-DDP, RRT*) look when predicting collisions and optimizing a
/// trajectory. Not `Context::horizon`, which is just the requested length
/// of the *returned* control trajectory — see its doc comment.
pub const PLANNING_HORIZON_S: f64 = 10.0;

/// Everything a planner sees besides the ego state.
pub struct Context<'a> {
    /// The fixed setting of the run: centerline, target speed, tick length.
    pub road: &'a Road,
    /// Current states of the other actors.
    pub actors: &'a [State],
    /// Requested number of controls (planners may return fewer or more).
    pub horizon: usize,
    /// Latency recorder for this plan call, when diagnostics are collected.
    pub latency: Option<&'a Latency>,
    /// Introspection recorder for this plan call, when a caller (the
    /// viewer's diagnostic overlay) wants to see the planner's search
    /// geometry. See [`diagnostics`] for what each planner records.
    pub diagnostics: Option<&'a Diagnostics>,
}

impl Context<'_> {
    /// Time `f` under the seam `name` when diagnostics are on; otherwise
    /// just run it. See [`latency`] for the standardized seam names.
    pub fn time<T>(&self, name: &'static str, f: impl FnOnce() -> T) -> T {
        match self.latency {
            Some(l) => l.time(name, f),
            None => f(),
        }
    }
}

/// A planner turns the current 4D state into a direct acceleration/curvature
/// command trajectory. It does not receive actuator state; the simulator may
/// slew-rate limit the first command before applying it.
pub trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}

/// Configuration: which planner to run. Lets the app and the batch runner
/// compare planners on the same scenario. Everything else about a planner —
/// display name, constructor, capabilities — lives in its [`PlannerSpec`]
/// row, so adding a planner means one enum variant plus one complete row
/// in [`SPECS`], not edits to scattered `match`es.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PlannerKind {
    Straight,
    Basic,
    BezierIdm,
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
pub struct PlannerSpec {
    /// Which kind this row describes — must match its position in [`SPECS`]
    /// (enforced by the `specs_align_with_kinds` test).
    pub kind: PlannerKind,
    pub name: &'static str,
    pub build: fn() -> Box<dyn Planner>,
    /// Whether this planner records anything into a [`Diagnostics`]
    /// recorder. Planners without receding-horizon search geometry to show
    /// never call `ctx.diagnostics`.
    pub has_diagnostics: bool,
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
        has_diagnostics: false,
    },
    PlannerSpec {
        kind: PlannerKind::BezierIdm,
        name: "bezier + IDM",
        build: || Box::new(BezierIdmPlanner),
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
    pub const ALL: [PlannerKind; 12] = [
        PlannerKind::Straight,
        PlannerKind::Basic,
        PlannerKind::BezierIdm,
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
    pub fn spec(self) -> &'static PlannerSpec {
        &SPECS[self as usize]
    }

    pub fn name(self) -> &'static str {
        self.spec().name
    }

    pub fn build(self) -> Box<dyn Planner> {
        (self.spec().build)()
    }

    /// See [`PlannerSpec::has_diagnostics`].
    pub fn has_diagnostics(self) -> bool {
        self.spec().has_diagnostics
    }
}

#[cfg(test)]
pub(crate) fn test_road(centerline: &[[f64; 2]]) -> Road {
    Road {
        centerline: centerline.to_vec(),
        target_speed: 10.0,
        half_width: crate::metrics::drivable_area::ROAD_HALF_WIDTH_M,
        dt: 0.1,
    }
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
    let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
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
        .map(|_| sim.tick(planner, &test_ctx(road, actors)))
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
