use super::{
    BasicPlanner, BezierToppraPlanner, Cem, IlqrPlanner, LatticePlanner, Mppi, Pi2DdpPlanner,
    Planner, PredictiveSampling, RrtPlanner, RrtStarPlanner, SamplingPlanner, StraightPlanner,
    TreetopPlanner,
};

/// PlannerKind: selects which planner to run.
/// Everything else about a planner (display name, constructor, capabilities) lives in its PlannerSpec row,
/// so adding a planner means one enum variant plus one complete row.
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

struct PlannerSpec {
    kind: PlannerKind,
    name: &'static str,
    build: fn() -> Box<dyn Planner>,
    has_diagnostics: bool,
}

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

    fn spec(self) -> &'static PlannerSpec {
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

    pub(crate) fn has_diagnostics(self) -> bool {
        self.spec().has_diagnostics
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn specs_align_with_kinds() {
        assert_eq!(PlannerKind::ALL.len(), SPECS.len());
        for kind in PlannerKind::ALL {
            assert_eq!(kind.spec().kind, kind);
        }
    }
}
