use super::{
    BezierToppraPlanner, Cem, Connections, FrenetSamplingPlanner, GraphPlanner, IlqrPlanner, LeeroyJenkinsPlanner,
    Mppi, Pi2DdpPlanner, Planner, PredictiveSampling, SamplingPlanner, TreetopPlanner,
};

/// PlannerKind: selects which planner to run.
/// Everything else about a planner (display name, constructor, capabilities) lives in its PlannerSpec row,
/// so adding a planner means one enum variant plus one complete row.
#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum PlannerKind {
    LeeroyJenkins,
    BezierToppra,
    Lattice,
    FrenetSampling,
    Pi2Ddp,
    PredictiveSampling,
    Cem,
    Mppi,
    Tree,
    Ilqr,
    Treetop,
}

struct PlannerSpec {
    kind: PlannerKind,
    name: &'static str,
    build: fn() -> Box<dyn Planner>,
    has_diagnostics: bool,
}

const SPECS: [PlannerSpec; 11] = [
    PlannerSpec {
        kind: PlannerKind::LeeroyJenkins,
        name: "Leeroy Jenkins",
        build: || Box::new(LeeroyJenkinsPlanner),
        has_diagnostics: false,
    },
    PlannerSpec {
        kind: PlannerKind::BezierToppra,
        name: "bezier + TOPP-RA",
        build: || Box::new(BezierToppraPlanner::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Lattice,
        name: "frenet lattice",
        build: || Box::new(GraphPlanner::new(Connections::Lattice)),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::FrenetSampling,
        name: "Frenet sampling",
        build: || Box::new(FrenetSamplingPlanner::default()),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Pi2Ddp,
        name: "PI2-DDP",
        build: || Box::new(Pi2DdpPlanner::default()),
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
        kind: PlannerKind::Tree,
        name: "Tree",
        build: || Box::new(GraphPlanner::new(Connections::NearestZap)),
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
        name: "treetop (Tree+iLQR)",
        build: || Box::new(TreetopPlanner::default()),
        has_diagnostics: true,
    },
];

impl PlannerKind {
    pub(crate) const ALL: [PlannerKind; 11] = [
        PlannerKind::LeeroyJenkins,
        PlannerKind::BezierToppra,
        PlannerKind::Lattice,
        PlannerKind::FrenetSampling,
        PlannerKind::Pi2Ddp,
        PlannerKind::PredictiveSampling,
        PlannerKind::Cem,
        PlannerKind::Mppi,
        PlannerKind::Tree,
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
