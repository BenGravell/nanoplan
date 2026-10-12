use super::*;

fn run_planner<O: Optimizer>(ego: State, actors: &[State], ticks: usize) -> Vec<State> {
    use crate::common::geometry::barrier::collide_with_road_barriers;
    use crate::common::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};
    let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let ctx = crate::planning::test_ctx(&road, actors);
    let mut planner = SamplingPlanner::<O>::default();
    let mut state = ego;
    (0..ticks)
        .map(|tick| {
            let control = planner.plan(state, &ctx)[0];
            let next = world_step(state, control, road.dt);
            assert_eq!(
                collide_with_road_barriers(state, next, EGO_FOOTPRINT, &road),
                next,
                "{} road contact at tick {tick}",
                O::NAME
            );
            assert!(
                actors
                    .iter()
                    .all(|actor| !footprints_overlap(next.pose(), EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT)),
                "{} actor contact at tick {tick}",
                O::NAME
            );
            state = next;
            state
        })
        .collect()
}

#[test]
fn cubic_segments_match_endpoints_and_share_velocity() {
    let initial = [20.0, 8.0, 2.0, 0.3];
    let knots = [[40.0, 9.0, -1.0, -0.2], [90.0, 11.0, 1.0, 0.5]];
    let motions = segments(initial, &knots, PLANNING_HORIZON_S);
    let duration = PLANNING_HORIZON_S / knots.len() as f64;
    let mut start = initial;
    for (motion, end) in motions.iter().zip(knots) {
        let lon = motion.longitudinal.at(0.0);
        let lat = motion.lateral.at(0.0);
        assert_eq!([lon[0], lon[1], lat[0], lat[1]], start);
        let lon = motion.longitudinal.at(duration);
        let lat = motion.lateral.at(duration);
        start = [initial[0] + end[0], end[1], end[2], end[3]];
        for (actual, expected) in [lon[0], lon[1], lat[0], lat[1]].into_iter().zip(start) {
            assert!((actual - expected).abs() < 1e-10);
        }
    }
}

#[test]
fn cubic_controls_follow_curved_frenet_reference() {
    let radius = 80.0;
    let points: Vec<_> = (0..2000)
        .map(|i| {
            let angle = i as f64 * 0.001;
            crate::simulation::Position::new(radius * angle.sin(), radius * (1.0 - angle.cos()))
        })
        .collect();
    let path = Path::new(&points);
    let knots = [[40.0, 8.0, 2.0, 0.0], [80.0, 8.0, 2.0, 0.0]];
    let motions = segments([10.0, 8.0, 2.0, 0.0], &knots, PLANNING_HORIZON_S);
    for t in [0.0, 1.25, 5.0, 7.5, 10.0] {
        let (state, control) = motion_at(&motions, &path, t, PLANNING_HORIZON_S).unwrap();
        assert!((state.position().x.hypot(state.position().y - radius) - 78.0).abs() < 1e-4);
        assert!((state.speed - 7.8).abs() < 1e-8);
        assert!((control.curvature - 1.0 / 78.0).abs() < 1e-8);
    }
}

#[test]
fn invalid_frenet_segments_are_rejected() {
    let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let ctx = crate::planning::test_ctx(&road, &[]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let start = Start::new(ego, &ctx).unwrap();
    for knot in [
        [40.0, -8.0, 0.0, 0.0],
        [500.0, 8.0, 0.0, 0.0],
        [f64::NAN, 8.0, 0.0, 0.0],
    ] {
        assert!(rollout(&[knot], &start, &ctx).is_none());
    }
}

#[test]
fn output_length_does_not_change_optimization_horizon() {
    let mut road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    for dt in [0.1, 0.2, 0.3] {
        road.dt = dt;
        let mut ctx = crate::planning::test_ctx(&road, &[]);
        let short = SamplingPlanner::<PredictiveSampling>::default().plan(State::default(), &ctx);
        assert!(short[0].acceleration > 0.0);
        ctx.horizon = (PLANNING_HORIZON_S / dt).ceil() as usize;
        let long = SamplingPlanner::<PredictiveSampling>::default().plan(State::default(), &ctx);
        assert_eq!(short, long[..short.len()]);
        assert_eq!(long.len(), ctx.horizon);
        ctx.horizon = 0;
        assert!(
            SamplingPlanner::<PredictiveSampling>::default()
                .plan(State::default(), &ctx)
                .is_empty()
        );
    }
}

#[test]
fn warm_start_survives_a_new_road_origin_and_rejects_divergence() {
    let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let ctx = crate::planning::test_ctx(&road, &[]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let mut planner = SamplingPlanner::<PredictiveSampling>::default();
    let controls = planner.plan(ego, &ctx);
    let next = world_step(ego, controls[0], road.dt);
    assert_eq!(next, planner.expected_next);
    let shifted_targets = planner.nominal.clone().unwrap();
    let mut other = SamplingPlanner::<PredictiveSampling> {
        nominal: Some(shifted_targets),
        expected_next: next,
        previous: planner.previous.clone(),
        ..Default::default()
    };
    let shifted_road = crate::planning::test_road(&[[-10.0, 0.0], [2000.0, 0.0]]);
    let a = planner.plan(next, &ctx);
    let b = other.plan(next, &crate::planning::test_ctx(&shifted_road, &[]));
    for (a, b) in a.iter().zip(b) {
        assert!((a.acceleration - b.acceleration).abs() < 1e-8);
        assert!((a.curvature - b.curvature).abs() < 1e-8);
    }

    // Replanning from an unrelated position must discard the old endpoints.
    let teleported = State::from((crate::simulation::Position::new(100.0, 0.0), 0.0, 8.0));
    assert_eq!(
        planner.plan(teleported, &ctx),
        SamplingPlanner::<PredictiveSampling>::default().plan(teleported, &ctx)
    );
}

#[test]
fn seed_fits_a_finite_road_window_at_cruising_speed() {
    let road = crate::planning::test_road(&[[-20.0, 0.0], [250.0, 0.0]]);
    let ctx = crate::planning::test_ctx(&road, &[]);
    let ego = State {
        speed: 20.0,
        ..Default::default()
    };
    let start = Start::new(ego, &ctx).unwrap();
    let knots = initial_knots(&start, &ctx, 4);
    assert!(rollout(&knots, &start, &ctx).is_some());
    let controls = SamplingPlanner::<PredictiveSampling>::default().plan(ego, &ctx);
    assert_eq!(controls.len(), ctx.horizon);
    assert!(controls[0].acceleration > crate::vehicle::MIN_LON_ACCEL);
}

// --- closed-loop tests, one battery per optimizer -----------------
//
// Same style as every other planner's tests (see the "Test harness"
// section of the README): a single `plan()` call proves little, so each
// optimizer is driven closed-loop and its realized trajectory checked.

/// From an initial lateral offset, stay on-road and accelerate without
/// exceeding the vehicle's physical terminal envelope.
fn stays_on_road_and_accelerates<O: Optimizer>() {
    let ego = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
        6.0,
    );
    let trace = run_planner::<O>(ego, &[], 150);
    let end = trace.last().unwrap();
    assert!(end.position().y.abs() < 5.5, "{} offset {}", O::NAME, end.position().y);
    assert!(end.speed > ego.speed, "{} speed {}", O::NAME, end.speed);
    assert!(
        end.speed <= *crate::simulation::MAX_TERMINAL_SPEED_MPS + 1e-9,
        "{} speed {}",
        O::NAME,
        end.speed
    );
}

/// Swerve around a stationary obstacle straddling the centerline, keep
/// real clearance, and still make it past — the point of the whole
/// exercise.
fn avoids_stopped_obstacle<O: Optimizer>() {
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let obstacle = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
        0.0,
    );
    let trace = run_planner::<O>(ego, &[obstacle], 150);
    let min_gap = trace
        .iter()
        .map(|s| (s.position().x - 40.0).hypot(s.position().y))
        .fold(f64::INFINITY, f64::min);
    assert!(min_gap > 2.0, "{} min gap {min_gap}", O::NAME);
    assert!(
        trace.last().unwrap().position().x > 50.0,
        "{} did not pass, x {}",
        O::NAME,
        trace.last().unwrap().position().x
    );
}

/// The knot noise is QMC (a pure function of the sample index), the
/// nominal is a deterministic Frenet cubic, and there is no `Rng`: two
/// fresh planners replanning from the identical state must produce the
/// identical plan, like the tree planner and unlike PI²-DDP.
fn is_a_pure_function_of_state<O: Optimizer>() {
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
    let a = SamplingPlanner::<O>::default().plan(ego, &ctx);
    let b = SamplingPlanner::<O>::default().plan(ego, &ctx);
    assert_eq!(a, b);
}

#[test]
fn ps_stays_on_road_and_accelerates() {
    stays_on_road_and_accelerates::<PredictiveSampling>();
}
#[test]
fn ps_avoids_stopped_obstacle() {
    avoids_stopped_obstacle::<PredictiveSampling>();
}
#[test]
fn ps_is_a_pure_function_of_state() {
    is_a_pure_function_of_state::<PredictiveSampling>();
}

#[test]
fn cem_stays_on_road_and_accelerates() {
    stays_on_road_and_accelerates::<Cem>();
}
#[test]
fn cem_avoids_stopped_obstacle() {
    avoids_stopped_obstacle::<Cem>();
}
#[test]
fn cem_is_a_pure_function_of_state() {
    is_a_pure_function_of_state::<Cem>();
}

#[test]
fn mppi_stays_on_road_and_accelerates() {
    stays_on_road_and_accelerates::<Mppi>();
}
#[test]
fn mppi_avoids_stopped_obstacle() {
    avoids_stopped_obstacle::<Mppi>();
}
#[test]
fn mppi_is_a_pure_function_of_state() {
    is_a_pure_function_of_state::<Mppi>();
}

#[test]
fn rejects_raw_commands_before_scoring_or_collision_work() {
    let road = crate::planning::test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let latency = crate::planning::Latency::default();
    let ctx = Context {
        latency: Some(&latency),
        ..crate::planning::test_ctx(&road, &[])
    };
    let mut calls = 0;
    assert!(
        rollout_controls(
            State {
                speed: 8.0,
                ..Default::default()
            },
            &ctx,
            |_, _| {
                calls += 1;
                Some(Control {
                    acceleration: MAX_LON_ACCEL + 1.0,
                    curvature: 0.0,
                })
            }
        )
        .is_none()
    );
    assert_eq!(calls, 1);
    assert!(latency.take().iter().all(|span| span.name == "kinodynamic"));
}

#[test]
fn filters_unsafe_samples_and_rejects_an_unsafe_mean() {
    #[derive(Default)]
    struct UnsafeMean {
        iteration: usize,
    }
    impl Optimizer for UnsafeMean {
        const NAME: &'static str = "unsafe mean regression";
        fn config(&self) -> OptimizerConfig {
            OptimizerConfig {
                iterations: 2,
                ..Default::default()
            }
        }
        fn num_elites(&self) -> usize {
            usize::MAX
        }
        fn sample_knots(&mut self, _: &[Knot], _: usize, _: usize) -> Vec<Vec<Knot>> {
            // On the second pass only colliding samples remain. The feasible
            // incumbent from the first pass must survive both this and the mean.
            self.iteration += 1;
            let offsets: &[f64] = if self.iteration == 1 { &[-3.4, 3.4, 0.0] } else { &[0.0] };
            offsets
                .iter()
                .map(|&d| (1..=4).map(|n| [20.0 * n as f64, 8.0, d, 0.0]).collect())
                .collect()
        }
        fn update_nominal_knots(&mut self, sampled: &[Vec<Knot>], _: &[f64]) -> Vec<Knot> {
            if self.iteration == 1 {
                assert_eq!(sampled.len(), 2);
                assert!(sampled.iter().all(|knots| knots[0][2] != 0.0));
            }
            (1..=4).map(|n| [20.0 * n as f64, 8.0, 0.0, 0.0]).collect()
        }
    }
    let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let actors = [State::from((crate::simulation::Position::new(40.0, 0.0), 0.0, 0.0))];
    let ctx = crate::planning::test_ctx(&road, &actors);
    let mut planner = SamplingPlanner::<UnsafeMean>::default();
    planner.plan(ego, &ctx);
    let winner = planner.previous.unwrap();
    assert!(trajectory_is_feasible(&winner.states, &winner.controls, &ctx));
    assert!(winner.states.last().unwrap().position().x > 45.0);
}

#[test]
fn previous_plan_is_revalidated_when_an_obstacle_appears() {
    let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let mut planner = SamplingPlanner::<PredictiveSampling>::default();
    planner.plan(ego, &crate::planning::test_ctx(&road, &[]));
    let previous = planner.previous.clone().unwrap();
    let ego = planner.expected_next;
    let actors = [State::from((crate::simulation::Position::new(20.0, 0.0), 0.0, 0.0))];
    let ctx = crate::planning::test_ctx(&road, &actors);
    assert!(!trajectory_is_feasible(
        &previous.states[1..],
        &previous.controls[1..],
        &ctx
    ));
    planner.plan(ego, &ctx);
    let winner = planner.previous.unwrap();
    assert!(trajectory_is_feasible(&winner.states, &winner.controls, &ctx));
}

#[test]
fn unvalidated_emergency_braking_is_never_cached_as_safe() {
    let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let actors = [ego];
    let ctx = crate::planning::test_ctx(&road, &actors);
    let mut planner = SamplingPlanner::<PredictiveSampling>::default();
    let controls = planner.plan(ego, &ctx);
    assert_eq!(controls.len(), ctx.horizon);
    assert!(controls[0].acceleration < 0.0);
    assert!(planner.previous.is_none());
    assert!(planner.nominal.is_none());
}

#[test]
fn live_small_track_has_no_contacts() {
    use crate::planning::{Latency, PlannerKind};
    for kind in [PlannerKind::PredictiveSampling, PlannerKind::Cem, PlannerKind::Mppi] {
        let mut world = crate::world::LiveWorld::with_track(1, 1, kind, 0, 0.1);
        let latency = Latency::default();
        for tick in 0..400 {
            world.tick_recording_latency_blocking(&latency);
            latency.take();
            assert_eq!(
                world.ego_collision_count,
                0,
                "{kind:?} tick {tick} progress {} state {:?}",
                world.track_progress,
                world.ego()
            );
        }
        assert!(
            world.track_progress > 100.0,
            "{kind:?} stalled: {}",
            world.track_progress
        );
    }
}

#[test]
fn full_horizon_predictions_clear_baked_track_barriers() {
    use crate::common::geometry::{EGO_FOOTPRINT, barrier::collide_with_road_barriers};
    use crate::track::{Road, Track};
    fn check<O: Optimizer>() {
        for index in 0..2 {
            let track = Track::from_catalog(index);
            for n in 0..20 {
                let progress = track.lap_length().unwrap() * n as f64 / 20.0;
                let road = Road::new(
                    track.centerline(progress - 50.0, progress + 250.0, 15.0),
                    track.half_width(progress),
                    0.1,
                );
                let (p, yaw) = track.pose(progress);
                for speed in [8.0, 20.0] {
                    let ego = State::from((p, yaw, speed));
                    let mut ctx = crate::planning::test_ctx(&road, &[]);
                    ctx.horizon = 100;
                    let mut planner = SamplingPlanner::<O>::default();
                    let controls = planner.plan(ego, &ctx);
                    assert!(
                        planner.previous.is_some(),
                        "{} no feasible plan: track {index} n {n} speed {speed}",
                        O::NAME
                    );
                    let mut state = ego;
                    for (tick, control) in controls.into_iter().enumerate() {
                        let next = world_step(state, control, road.dt);
                        assert_eq!(
                            collide_with_road_barriers(state, next, EGO_FOOTPRINT, &road),
                            next,
                            "{} track {index} n {n} speed {speed} tick {tick}",
                            O::NAME
                        );
                        state = next;
                    }
                }
            }
        }
    }
    check::<PredictiveSampling>();
    check::<Cem>();
    check::<Mppi>();
}

#[test]
fn records_diagnostics_when_requested() {
    use crate::planning::Diagnostics;
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let diag = Diagnostics::default();
    let road = crate::planning::test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let mut ctx = crate::planning::test_ctx(&road, &[]);
    ctx.diagnostics = Some(&diag);
    SamplingPlanner::<Mppi>::default().plan(ego, &ctx);
    let data = diag.take();
    // All scored candidates passed the dynamics filter and span the full horizon.
    assert!(!data.trajectories.is_empty());
    let horizon = (PLANNING_HORIZON_S / road.dt).ceil() as usize;
    assert!(data.trajectories.iter().all(|t| t.len() == horizon + 1));
    assert_eq!(data.points.len(), data.trajectories.len() * (horizon + 1));
}

#[test]
fn strategies_keep_distinct_search_spaces_and_share_the_launch() {
    use crate::planning::{PlannerKind, test_ctx, test_road};
    let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let ctx = test_ctx(&road, &[]);
    let ego = State::default();
    let start = Start::new(ego, &ctx).unwrap();
    assert!(!start.launch.is_empty());
    assert!(start.state.speed >= CUBIC_LAUNCH_SPEED_MPS);
    assert_eq!(start.duration + start.launch.len() as f64 * road.dt, PLANNING_HORIZON_S);
    let grid = frenet::Frenet.seeds(&start, &ctx);
    let ps = PredictiveSampling.seeds(&start, &ctx);
    assert!(grid.len() > ps.len());
    assert!(grid.iter().all(|candidate| candidate.knots.len() == 1));
    assert!(ps.iter().all(|candidate| candidate.knots.len() == 4));
    for kind in [
        PlannerKind::FrenetSampling,
        PlannerKind::PredictiveSampling,
        PlannerKind::Cem,
        PlannerKind::Mppi,
    ] {
        let controls = kind.build().plan(ego, &ctx);
        assert_eq!(&controls[..start.launch.len()], start.launch.as_slice(), "{kind:?}");
    }
}

#[test]
fn terminal_progress_wins_over_early_progress() {
    use crate::planning::{test_ctx, test_road};
    let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let ctx = test_ctx(&road, &[]);
    let ego = State {
        speed: 10.0,
        ..Default::default()
    };
    let early = rollout_controls(ego, &ctx, |tick, _| {
        Some(Control {
            acceleration: if tick < 20 { 2.0 } else { -0.5 },
            curvature: 0.0,
        })
    })
    .unwrap();
    let late = rollout_controls(ego, &ctx, |tick, _| {
        Some(Control {
            acceleration: if tick < 50 { 0.0 } else { 2.0 },
            curvature: 0.0,
        })
    })
    .unwrap();
    let accumulated = |candidate: &Candidate| {
        candidate
            .states
            .iter()
            .enumerate()
            .skip(1)
            .map(|(tick, state)| metrics::progress_score(state.position().x, ego.speed, tick as f64 * road.dt))
            .sum::<f64>()
    };
    assert!(accumulated(&early) > accumulated(&late));
    assert!(late.states.last().unwrap().position().x > early.states.last().unwrap().position().x);
    let expected = late.controls.clone();
    let candidates = score_candidates(vec![early, late], &ctx);
    assert_eq!(select_candidates(candidates, 1, &ctx)[0].controls, expected);
}

#[test]
fn collision_checks_stop_after_enough_safe_elites_and_cover_the_full_horizon() {
    use crate::planning::{test_ctx, test_road};
    let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let actors = [State::from((crate::simulation::Position::new(40.0, 0.0), 0.0, 0.0))];
    let latency = crate::planning::Latency::default();
    let ctx = Context {
        horizon: 1,
        latency: Some(&latency),
        ..test_ctx(&road, &actors)
    };
    let accelerating = rollout_controls(ego, &ctx, |_, _| {
        Some(Control {
            acceleration: MAX_LON_ACCEL,
            curvature: 0.0,
        })
    })
    .unwrap();
    let braking = rollout_controls(ego, &ctx, |_, state| Some(road_control(state, &ctx, MIN_LON_ACCEL))).unwrap();
    for count in [1, 2] {
        let candidates = score_candidates(vec![braking.clone(), accelerating.clone(), braking.clone()], &ctx);
        latency.take();
        let selected = select_candidates(candidates, count, &ctx);
        assert_eq!(selected.len(), count);
        assert!(selected.iter().all(|candidate| candidate.controls == braking.controls));
        assert_eq!(
            latency.take().iter().filter(|span| span.name == "collision").count(),
            count + 1
        );
    }
}

#[test]
fn grid_filters_dynamics_then_scores_then_checks_collisions() {
    use crate::planning::{test_ctx, test_road};
    let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    let latency = crate::planning::Latency::default();
    let ctx = Context {
        latency: Some(&latency),
        ..test_ctx(&road, &[])
    };
    let start = Start::new(
        State {
            speed: 20.0,
            ..Default::default()
        },
        &ctx,
    )
    .unwrap();
    let candidates = score_candidates(frenet::Frenet.seeds(&start, &ctx), &ctx);
    assert!(!select_candidates(candidates, 1, &ctx).is_empty());
    let spans = latency.take();
    let count = |name: &str| spans.iter().filter(|span| span.name == name).count();
    assert!(count("kinodynamic") > count("cost"));
    assert!(count("cost") > count("collision"));
    assert!(count("collision") > 0);
    let first_cost = spans.iter().position(|span| span.name == "cost").unwrap();
    assert!(spans[..first_cost].iter().all(|span| span.name == "kinodynamic"));
    let first_collision = spans.iter().position(|span| span.name == "collision").unwrap();
    assert!(
        spans[first_cost..first_collision]
            .iter()
            .all(|span| span.name == "cost")
    );
    assert!(spans[first_collision..].iter().all(|span| span.name == "collision"));
}
