use super::{
    Cem, CenterlineFollower, Connections, FrenetSamplingPlanner, GraphPlanner, IlqrPlanner, LeeroyJenkinsPlanner, Mppi,
    Pi2DdpPlanner, Planner, PredictiveSampling, SamplingPlanner,
};

/// PlannerKind: selects which planner to run.
/// Everything else about a planner (display name, constructor, capabilities) lives in its PlannerSpec row,
/// so adding a planner means one enum variant plus one complete row.
#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum PlannerKind {
    LeeroyJenkins,
    CenterlineFollower,
    Lattice,
    FrenetSampling,
    Pi2Ddp,
    PredictiveSampling,
    Cem,
    Mppi,
    Tree,
    Ilqr,
}

struct PlannerSpec {
    kind: PlannerKind,
    name: &'static str,
    build: fn(bool) -> Box<dyn Planner>,
    has_diagnostics: bool,
}

const SPECS: [PlannerSpec; 10] = [
    PlannerSpec {
        kind: PlannerKind::LeeroyJenkins,
        name: "Leeroy Jenkins",
        build: |optimize| compose(LeeroyJenkinsPlanner, optimize),
        has_diagnostics: false,
    },
    PlannerSpec {
        kind: PlannerKind::CenterlineFollower,
        name: "CenterlineFollower",
        build: |optimize| compose(CenterlineFollower, optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Lattice,
        name: "frenet lattice",
        build: |optimize| compose(GraphPlanner::new(Connections::Lattice), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::FrenetSampling,
        name: "Frenet sampling",
        build: |optimize| compose(FrenetSamplingPlanner::default(), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Pi2Ddp,
        name: "PI2-DDP",
        build: |optimize| compose(Pi2DdpPlanner::default(), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::PredictiveSampling,
        name: SamplingPlanner::<PredictiveSampling>::NAME,
        build: |optimize| compose(SamplingPlanner::<PredictiveSampling>::default(), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Cem,
        name: SamplingPlanner::<Cem>::NAME,
        build: |optimize| compose(SamplingPlanner::<Cem>::default(), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Mppi,
        name: SamplingPlanner::<Mppi>::NAME,
        build: |optimize| compose(SamplingPlanner::<Mppi>::default(), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Tree,
        name: "Tree",
        build: |optimize| compose(GraphPlanner::new(Connections::NearestZap), optimize),
        has_diagnostics: true,
    },
    PlannerSpec {
        kind: PlannerKind::Ilqr,
        name: "iLQR (finite diff)",
        build: |_| Box::new(IlqrPlanner::default()),
        has_diagnostics: true,
    },
];

fn compose<P: Planner + 'static>(planner: P, optimize: bool) -> Box<dyn Planner> {
    if optimize {
        Box::new(super::planners::trajectory_optimization::Refined(planner))
    } else {
        Box::new(planner)
    }
}

impl PlannerKind {
    pub(crate) const ALL: [PlannerKind; 10] = [
        PlannerKind::LeeroyJenkins,
        PlannerKind::CenterlineFollower,
        PlannerKind::Lattice,
        PlannerKind::FrenetSampling,
        PlannerKind::Pi2Ddp,
        PlannerKind::PredictiveSampling,
        PlannerKind::Cem,
        PlannerKind::Mppi,
        PlannerKind::Tree,
        PlannerKind::Ilqr,
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
        self.build_with_optimization(false)
    }

    pub(crate) fn build_with_optimization(self, enabled: bool) -> Box<dyn Planner> {
        (self.spec().build)(enabled)
    }

    pub(crate) fn supports_post_optimization(self) -> bool {
        !matches!(self, Self::Ilqr)
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
