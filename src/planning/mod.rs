//! The planner interface and one module per planner.

pub mod bezier_idm;
pub mod diagnostics;
pub mod latency;
pub mod lattice;
pub mod pi2ddp;
pub mod rrt_star;
pub mod straight;

pub use bezier_idm::BezierIdmPlanner;
pub use diagnostics::{Diagnostics, DiagnosticsData};
pub use latency::{Latency, LatencyStats, SeamStats};
pub use lattice::LatticePlanner;
pub use pi2ddp::Pi2DdpPlanner;
pub use rrt_star::RrtStarPlanner;
pub use straight::StraightPlanner;

use crate::simulation::{Control, State};

/// How far ahead planners with a genuine receding-horizon cost model
/// (lattice, PI²-DDP, RRT*) look when predicting collisions and optimizing a
/// trajectory. Not `Context::horizon`, which is just the requested length
/// of the *returned* control trajectory — see its doc comment.
pub const PLANNING_HORIZON_S: f64 = 10.0;

/// Everything a planner sees besides the ego state.
pub struct Context<'a> {
    /// Lane centerline the ego should follow, as a polyline.
    pub centerline: &'a [[f64; 2]],
    /// Current states of the other actors.
    pub actors: &'a [State],
    /// Desired cruise speed.
    pub target_speed: f64,
    /// Tick length of the returned control trajectory.
    pub dt: f64,
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

/// A planner turns the current state into a control trajectory.
/// The simulator applies the first control each tick (receding horizon).
pub trait Planner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control>;
}

/// Configuration: which planner to run. Lets the app and the batch runner
/// compare planners on the same scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PlannerKind {
    Straight,
    BezierIdm,
    Lattice,
    Pi2Ddp,
    RrtStar,
}

impl PlannerKind {
    pub const ALL: [PlannerKind; 5] = [
        PlannerKind::Straight,
        PlannerKind::BezierIdm,
        PlannerKind::Lattice,
        PlannerKind::Pi2Ddp,
        PlannerKind::RrtStar,
    ];

    pub fn name(self) -> &'static str {
        match self {
            PlannerKind::Straight => "straight (strawman)",
            PlannerKind::BezierIdm => "bezier + IDM",
            PlannerKind::Lattice => "frenet lattice",
            PlannerKind::Pi2Ddp => "PI2-DDP",
            PlannerKind::RrtStar => "RRT*",
        }
    }

    pub fn build(self) -> Box<dyn Planner> {
        match self {
            PlannerKind::Straight => Box::new(StraightPlanner),
            PlannerKind::BezierIdm => Box::new(BezierIdmPlanner),
            PlannerKind::Lattice => Box::new(LatticePlanner),
            PlannerKind::Pi2Ddp => Box::new(Pi2DdpPlanner::default()),
            PlannerKind::RrtStar => Box::new(RrtStarPlanner::default()),
        }
    }

    /// Whether this planner records anything into a [`Diagnostics`]
    /// recorder. `Straight` and `BezierIdm` have no receding-horizon search
    /// geometry to show — they never call `ctx.diagnostics`.
    pub fn has_diagnostics(self) -> bool {
        matches!(
            self,
            PlannerKind::Lattice | PlannerKind::Pi2Ddp | PlannerKind::RrtStar
        )
    }
}

#[cfg(test)]
pub(crate) fn test_ctx<'a>(centerline: &'a [[f64; 2]], actors: &'a [State]) -> Context<'a> {
    Context {
        centerline,
        actors,
        target_speed: 10.0,
        dt: 0.1,
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
    const CENTERLINE: [[f64; 2]; 2] = [[-20.0, 0.0], [400.0, 0.0]];
    let mut sim = crate::simulation::Simulator {
        state: ego,
        dt: 0.1,
    };
    (0..ticks)
        .map(|_| sim.tick(planner, &test_ctx(&CENTERLINE, actors)))
        .collect()
}
