//! Statically composed, optional iLQR post-processing for trajectory generators.

mod ilqr;

use crate::common::types::Trajectory;
use crate::planning::feasibility::trajectory_is_feasible;
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::simulation::{Control, State, rollout};
pub(crate) use ilqr::IlqrPlanner;
use ilqr::Ocp;

/// Selected once when constructing the planner. Disabled mode uses P directly.
pub(crate) struct Refined<P>(pub(crate) P);

impl<P: Planner> Planner for Refined<P> {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        // Ask the generator for the complete seed, not the viewer's output prefix.
        // Road geometry stays prepared; both stages reuse this context's path cache.
        let full = Context::new(
            ctx.road,
            ctx.actors,
            ctx.horizon.max((PLANNING_HORIZON_S / ctx.road.dt).ceil() as usize),
            ctx.compute_budget,
            ctx.latency,
            ctx.diagnostics,
        );
        let mut controls = self.0.plan(ego, &full);
        if !controls.is_empty()
            && let Some(optimized) = full.time("traj_opt", || refine(ego, &controls, &full))
        {
            controls = optimized.controls[..ctx.horizon.min(optimized.controls.len())].to_vec();
            self.0.accept_optimized(optimized, &full);
        }
        controls.truncate(ctx.horizon);
        controls
    }
}

fn refine(ego: State, controls: &[Control], ctx: &Context) -> Option<Trajectory> {
    let seed = rollout(ego, ctx.road.dt, controls.len(), |tick, _| Some(controls[tick]))?;
    let ocp = Ocp::new(ctx.path(), ego, ctx);
    let initial_cost = ocp.traj_cost(&seed.states, &seed.controls);
    let initial_progress = progress(&seed.states, ctx);
    // One candidate, 1–4 iterations (2 at nominal budget), independent of generator sample count.
    let iterations = ctx.compute_budget.scale(2, 1).min(4);
    let solution = ilqr::solve_from_rollout(&ocp, seed, iterations);
    let trajectory = Trajectory {
        states: solution.states,
        controls: solution.controls,
    };
    if !solution.cost.is_finite()
        || solution.cost >= initial_cost
        || progress(&trajectory.states, ctx) < initial_progress
        || !ctx.time("collision", || {
            trajectory_is_feasible(&trajectory.states, &trajectory.controls, ctx)
        })
    {
        return None;
    }
    if let Some(diag) = ctx.diagnostics {
        diag.record_trajectory(trajectory.states.iter().map(Into::into).collect());
    }
    Some(trajectory)
}

fn progress(states: &[State], ctx: &Context) -> f64 {
    // Sequential projections preserve the road branch on overlapping sections.
    let mut station = ctx.project_ego(states[0]).s;
    let start = station;
    for pair in states.windows(2) {
        station = ctx
            .path()
            .project_near(
                pair[1].position(),
                station + pair[0].speed * ctx.road.dt,
                crate::planning::planner_math::STATE_SAMPLE_PROJECTION_RADIUS_M,
            )
            .s;
    }
    station - start
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{ComputeBudget, Latency, PlannerKind, test_ctx, test_road};

    #[test]
    fn rollout_constrained_uses_the_shared_step() {
        let x0 = State {
            speed: 5.0,
            ..Default::default()
        };
        let actions = [Control {
            acceleration: 2.0,
            curvature: 0.1,
        }];
        let (xs, us) = crate::planning::controls::rollout_constrained(x0, &actions, 0.1);
        assert_eq!(us, actions);
        assert_eq!(xs[1], crate::simulation::world_step(x0, actions[0], 0.1));
    }

    #[test]
    fn stays_on_road_and_accelerates() {
        let ego = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        );
        let trace = crate::planning::test_run(&mut *PlannerKind::Tree.build_with_optimization(true), ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.position().y.abs() < 5.5, "offset {}", end.position().y);
        assert!(end.speed > ego.speed, "speed {}", end.speed);
    }

    #[test]
    fn avoids_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let trace = crate::planning::test_run(
            &mut *PlannerKind::Tree.build_with_optimization(true),
            ego,
            &[obstacle],
            150,
        );
        let min_gap = trace
            .iter()
            .map(|s| (s.position().x - 40.0).hypot(s.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(
            trace.last().unwrap().position().x > 50.0,
            "did not pass, x {}",
            trace.last().unwrap().position().x
        );
    }

    #[test]
    fn plan_is_a_pure_function_of_state() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let actors = [obstacle];
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ctx = crate::planning::test_ctx(&road, &actors);
        let a = PlannerKind::Tree.build_with_optimization(true).plan(ego, &ctx);
        let b = PlannerKind::Tree.build_with_optimization(true).plan(ego, &ctx);
        assert_eq!(a, b);
    }

    #[test]
    fn records_diagnostics_when_requested() {
        use crate::planning::Diagnostics;
        let diag = Diagnostics::default();
        let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let mut ctx = crate::planning::test_ctx(&road, &[]);
        ctx.diagnostics = Some(&diag);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        PlannerKind::Tree.build_with_optimization(true).plan(ego, &ctx);
        let data = diag.take();
        // Tree search geometry remains available through the shared composition.
        assert!(!data.points.is_empty());
        assert!(data.trajectories.len() >= 2);
    }

    #[derive(Default)]
    struct Coast {
        accepted: Option<Trajectory>,
    }

    impl Planner for Coast {
        fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
            vec![Control::default(); ctx.horizon]
        }
        fn accept_optimized(&mut self, trajectory: Trajectory, _ctx: &Context) {
            self.accepted = Some(trajectory);
        }
    }

    #[test]
    fn refines_the_full_horizon_and_feeds_back_only_safe_improvements() {
        let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        for horizon in [1, 125] {
            let mut ctx = test_ctx(&road, &[]);
            ctx.horizon = horizon;
            let mut planner = Refined(Coast::default());
            let output = planner.plan(ego, &ctx);
            assert_eq!(output.len(), horizon);
            let trajectory = planner.0.accepted.as_ref().expect("coasting should improve");
            assert_eq!(trajectory.controls.len(), horizon.max(100));
            assert_eq!(output, trajectory.controls[..horizon]);
            assert!(trajectory_is_feasible(&trajectory.states, &trajectory.controls, &ctx));
            let coast = rollout(ego, road.dt, trajectory.controls.len(), |_, _| Some(Control::default())).unwrap();
            assert!(progress(&trajectory.states, &ctx) > progress(&coast.states, &ctx));
        }
        let mut planner = Refined(Coast::default());
        let actors = [ego]; // No feasible improvement is possible from overlapping vehicles.
        let ctx = test_ctx(&road, &actors);
        assert_eq!(planner.plan(ego, &ctx), vec![Control::default(); ctx.horizon]);
        assert!(planner.0.accepted.is_none());
        let mut ctx = test_ctx(&road, &[]);
        ctx.horizon = 0;
        assert!(planner.plan(ego, &ctx).is_empty());
        assert!(planner.0.accepted.is_none());
    }

    #[test]
    fn every_generator_supports_optional_refinement_without_extra_work_when_disabled() {
        let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        road.prepare();
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        for kind in PlannerKind::ALL
            .into_iter()
            .filter(|kind| kind.supports_post_optimization())
        {
            let latency = Latency::default();
            let mut ctx = test_ctx(&road, &[]);
            ctx.horizon = 100;
            ctx.compute_budget = ComputeBudget::from_percent(5.0);
            ctx.latency = Some(&latency);
            let baseline = kind.build().plan(ego, &ctx);
            let baseline_work: u64 = latency.take().iter().map(|span| span.clocks).sum();
            assert_eq!(kind.build_with_optimization(false).plan(ego, &ctx), baseline);
            let spans = latency.take();
            assert!(spans.iter().all(|span| span.name != "traj_opt"));
            assert_eq!(spans.iter().map(|span| span.clocks).sum::<u64>(), baseline_work);
            let optimized = kind.build_with_optimization(true).plan(ego, &ctx);
            assert_eq!(optimized.len(), ctx.horizon, "{kind:?}");
            assert!(latency.take().iter().any(|span| span.name == "traj_opt"), "{kind:?}");
            if optimized != baseline {
                let trajectory = rollout(ego, road.dt, ctx.horizon, |tick, _| Some(optimized[tick])).unwrap();
                assert!(
                    trajectory_is_feasible(&trajectory.states, &trajectory.controls, &ctx),
                    "{kind:?}"
                );
                let baseline = rollout(ego, road.dt, ctx.horizon, |tick, _| Some(baseline[tick])).unwrap();
                assert!(
                    progress(&trajectory.states, &ctx) >= progress(&baseline.states, &ctx),
                    "{kind:?}"
                );
            }
        }
    }

    #[test]
    #[ignore = "manual timing benchmark"]
    fn benchmark_optional_refinement() {
        let straight = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        let points: Vec<_> = (0..1800)
            .map(|i| {
                let angle = -0.25 + i as f64 * 0.002;
                [80.0 * angle.sin(), 80.0 * (1.0 - angle.cos())]
            })
            .collect();
        let bend = test_road(&points);
        let actors: Vec<_> = [35.0_f64, 65.0, 95.0, 125.0, 155.0]
            .into_iter()
            .map(|s| {
                let angle = s / 80.0;
                State::from((
                    crate::simulation::Position::new(80.0 * angle.sin(), 80.0 * (1.0 - angle.cos())),
                    angle,
                    6.0,
                ))
            })
            .collect();
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        for (name, road, actors) in [
            ("empty straight", &straight, &[][..]),
            ("bend, five actors", &bend, actors.as_slice()),
        ] {
            road.prepare();
            let mut ctx = test_ctx(road, actors);
            ctx.horizon = 100;
            for kind in PlannerKind::ALL
                .into_iter()
                .filter(|kind| kind.supports_post_optimization())
            {
                let mut medians = [0.0; 2];
                for (optimize, median) in [false, true].into_iter().zip(&mut medians) {
                    let mut times = Vec::new();
                    for sample in 0..10 {
                        let mut planner = kind.build_with_optimization(optimize);
                        let start = std::time::Instant::now();
                        std::hint::black_box(planner.plan(ego, &ctx));
                        if sample > 0 {
                            times.push(start.elapsed().as_secs_f64() * 1000.0);
                        }
                    }
                    times.sort_by(f64::total_cmp);
                    *median = times[times.len() / 2];
                }
                eprintln!(
                    "{name}, {kind:?}: generator {:.3} ms, +iLQR {:.3} ms, added {:.3} ms",
                    medians[0],
                    medians[1],
                    medians[1] - medians[0]
                );
            }
        }
    }
}
