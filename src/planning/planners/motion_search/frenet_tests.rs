use super::*;
use crate::common::geometry::barrier::collides_with_road_barrier;
use crate::planning::{
    ComputeBudget, Context, Diagnostics, PLANNING_HORIZON_S, Planner, test_ctx, test_road, test_run, test_run_on,
};
use crate::simulation::{Position, State, rollout, world_step};
use crate::track::{Path, Track};
use crate::vehicle::MAX_LON_ACCEL;

#[test]
#[ignore = "manual fixed-workload timing; no hardware-dependent assertion"]
fn benchmark_fixed_workloads() {
    let straight = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
    let track = Track::from_catalog(1);
    let bend = track.prepared().window(250.0, 8.0, 0.1);
    let (point, yaw) = track.pose(250.0);
    for (name, road, ego) in [
        (
            "straight",
            &straight,
            State {
                speed: 20.0,
                ..Default::default()
            },
        ),
        ("bend", &bend, State::from((point, yaw, 8.0))),
    ] {
        let mut timings = Vec::new();
        for i in 0..8 {
            let ctx = test_ctx(road, &[]);
            let start = std::time::Instant::now();
            let controls = FrenetSamplingPlanner::default().plan(ego, &ctx);
            std::hint::black_box(controls);
            if i > 0 {
                timings.push(start.elapsed().as_secs_f64() * 1e3);
            }
        }
        timings.sort_by(f64::total_cmp);
        eprintln!("{name}: median {:.3} ms", timings[timings.len() / 2]);
    }
}

#[test]
fn completes_small_track_with_five_opponents() {
    use crate::planning::{Latency, PlannerKind};
    use crate::world::LiveWorld;

    let mut world = LiveWorld::with_track(1, 1, PlannerKind::FrenetSampling, 5, 0.1);
    let lap = world.track.lap_length().unwrap();
    let latency = Latency::default();
    for _ in 0..800 {
        let previous = world.ego();
        world.tick_recording_latency_blocking(&latency);
        latency.take();
        assert_eq!(
            world.ego_collision_count,
            0,
            "progress {}, previous {:?}, ego {:?}, control {:?}",
            world.track_progress,
            previous,
            world.ego(),
            world.actuation()
        );
        if world.track_progress >= lap {
            break;
        }
    }
    assert!(
        world.track_progress >= lap,
        "progress {} / {lap}, speed {}, contacts {}",
        world.track_progress,
        world.ego().speed,
        world.ego_collision_count
    );
    assert_eq!(world.ego_collision_count, 0);
}

#[test]
fn restarts_from_the_stopped_small_track_snapshot() {
    use crate::planning::{ComputeBudget, PlannerKind};
    use crate::world::{EgoStart, LiveWorld};

    let world = LiveWorld::with_track_at(
        1,
        1,
        PlannerKind::FrenetSampling,
        0,
        0.1,
        EgoStart {
            progress: 294.902,
            ..Default::default()
        },
    );
    let ego = State::from((
        Position::new(153.13624873412616, -21.39139512061829),
        -1.938120058194263,
        0.0,
    ));
    let mut ctx = test_ctx(&world.road, &[]);
    for percent in [5.0, 100.0] {
        ctx.compute_budget = ComputeBudget::from_percent(percent);
        let controls = FrenetSamplingPlanner::default().plan(ego, &ctx);
        assert!(controls[0].acceleration > 0.0, "budget {percent}: {:?}", controls[0]);
        let mut state = ego;
        for _ in 0..100 {
            let mut ctx = test_ctx(&world.road, &[]);
            ctx.compute_budget = ComputeBudget::from_percent(percent);
            state = world_step(
                state,
                FrenetSamplingPlanner::default().plan(state, &ctx)[0],
                world.road.dt,
            );
        }
        assert!(
            state.position().distance(ego.position()) > 10.0,
            "budget {percent}: {state:?}"
        );
    }
}

#[test]
fn stops_for_a_blocking_opponent_and_restarts_when_clear() {
    use crate::common::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};

    // Leave no room to pass: lateral sampling should still stop when blocked.
    let road = crate::track::Road::new(vec![[-20.0, 0.0], [1_000.0, 0.0]], 2.0, 0.1);
    let ego = State::from((Position::new(0.0, 0.0), 0.0, 8.0));
    let actor = State::from((Position::new(30.0, 0.0), 0.0, 0.0));
    let trace = test_run_on(&mut FrenetSamplingPlanner::default(), &road, ego, &[actor], 150);
    let stopped = *trace.last().unwrap();
    assert!(stopped.position().x < actor.position().x);
    assert!(stopped.speed.abs() < 0.1, "speed {}", stopped.speed);
    let controls = FrenetSamplingPlanner::default().plan(stopped, &test_ctx(&road, &[]));
    assert!(controls[0].acceleration > 0.0);
    assert!(controls[0].curvature.abs() < 1e-9);
    assert!(
        trace
            .iter()
            .all(|state| !footprints_overlap(state.pose(), EGO_FOOTPRINT, actor.pose(), CAR_FOOTPRINT))
    );
}

#[test]
fn converges_to_centerline() {
    let ego = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 3.0), 0.0),
        8.0,
    );
    let trace = test_run(&mut FrenetSamplingPlanner::default(), ego, &[], 200);
    let end = trace.last().unwrap();
    assert!(end.position().y.abs() < 0.4, "offset {}", end.position().y);
}

#[test]
fn rolling_road_window_recovers_from_a_lateral_offset() {
    let track = Track::from_catalog(1);
    let road = test_road(&track.centerline(-50.0, 250.0, 15.0));
    let path = Path::new(road.centerline());
    let (p, yaw) = path.pose_at(50.0);
    let left = Position::from_angle(yaw + std::f64::consts::FRAC_PI_2);
    let ego = State::from((Position::new(p.x + 3.0 * left.x, p.y + 3.0 * left.y), yaw, 8.0));

    // Free lateral sampling may favor an offset on a bend, but must recover safely.
    let trace = test_run_on(&mut FrenetSamplingPlanner::default(), &road, ego, &[], 200);
    let d = path.project(trace.last().unwrap().position()).d;
    assert!(d.abs() < 3.0, "offset {d}");
    assert!(
        trace
            .iter()
            .all(|state| !crate::common::geometry::barrier::collides_with_road_barrier(*state, &road))
    );
}

#[test]
fn accelerates_on_a_clear_straight() {
    let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
    let controls = FrenetSamplingPlanner::default().plan(State::default(), &test_ctx(&road, &[]));
    assert!(controls[0].acceleration > 0.0);
    assert!(controls.iter().all(|u| u.acceleration <= MAX_LON_ACCEL));
    assert!(controls.iter().all(|u| u.curvature.is_finite()));
}

#[test]
fn sustained_acceleration_targets_drive_strongly() {
    let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
    for speed in [2.0, 8.0, 20.0] {
        let ego = State {
            speed,
            ..Default::default()
        };
        let mut ctx = test_ctx(&road, &[]);
        ctx.horizon = 100;
        let controls = FrenetSamplingPlanner::default().plan(ego, &ctx);
        assert!(
            controls[0].acceleration > 0.6 * MAX_LON_ACCEL,
            "speed {speed}: {:?}",
            controls[0]
        );
        assert!(rollout(ego, ctx.road.dt, controls.len(), |tick, _| Some(controls[tick])).is_some());
    }
}

#[test]
fn stops_inside_a_finite_road_window() {
    let road = test_road(&[[-20.0, 0.0], [80.0, 0.0]]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let trace = test_run_on(&mut FrenetSamplingPlanner::default(), &road, ego, &[], 200);
    assert!(trace.iter().all(|state| state.position().x <= 80.0));
    assert!(trace.last().unwrap().speed.abs() < 0.1, "end {:?}", trace.last());
}

#[test]
fn returns_a_full_horizon() {
    let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let ctx = test_ctx(&road, &[]);
    let plan = FrenetSamplingPlanner::default().plan(
        State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 2.0), 0.0),
            6.0,
        ),
        &ctx,
    );
    assert_eq!(plan.len(), ctx.horizon);
}

#[test]
fn fixed_horizon_and_relative_origin_do_not_depend_on_output_length_or_road_origin() {
    let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let ego = State {
        speed: 8.0,
        ..Default::default()
    };
    let mut ctx = test_ctx(&road, &[]);
    let short = FrenetSamplingPlanner::default().plan(ego, &ctx);
    ctx.horizon = (PLANNING_HORIZON_S / road.dt).ceil() as usize;
    let full = FrenetSamplingPlanner::default().plan(ego, &ctx);
    assert_eq!(short, full[..short.len()]);
    ctx.horizon = 0;
    assert!(FrenetSamplingPlanner::default().plan(ego, &ctx).is_empty());

    // Rotating and shifting the road changes absolute station and yaw, not relative targets.
    let shifted = test_road(&[[30.0, -100.0], [30.0, 400.0]]);
    let shifted_ego = State::from((Position::new(30.0, 0.0), std::f64::consts::FRAC_PI_2, ego.speed));
    let shifted_plan = FrenetSamplingPlanner::default().plan(shifted_ego, &test_ctx(&shifted, &[]));
    for (a, b) in short.iter().zip(&shifted_plan) {
        assert!((a.acceleration - b.acceleration).abs() < 1e-9);
        assert!((a.curvature - b.curvature).abs() < 1e-9);
    }
}

#[test]
fn diagnostics_show_candidates_without_changing_selection_or_collision_work() {
    use crate::planning::Diagnostics;

    let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
    let diagnostics = Diagnostics::default();
    let latency = crate::planning::Latency::default();
    let mut ctx = Context {
        latency: Some(&latency),
        ..test_ctx(&road, &[])
    };
    let plain = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
    let collision_checks =
        |spans: Vec<crate::planning::Span>| spans.iter().filter(|span| span.name == "collision").count();
    let plain_checks = collision_checks(latency.take());
    ctx.diagnostics = Some(&diagnostics);
    let displayed = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
    let spans = latency.take();
    let scored = spans.iter().filter(|span| span.name == "cost").count();
    assert_eq!(displayed, plain);
    assert_eq!(collision_checks(spans), plain_checks);
    let data = diagnostics.take();

    assert!(data.trajectories.len() > 1);
    assert_eq!(data.trajectories.len(), scored);
    assert!(
        data.trajectories
            .iter()
            .all(|trajectory| trajectory.len() == (PLANNING_HORIZON_S / road.dt).ceil() as usize + 1)
    );
}

#[test]
fn baked_track_predictions_stay_inside_road_for_full_horizon() {
    use crate::track::{Road, TRACK_PRESETS};

    for track_index in 0..TRACK_PRESETS.len() {
        let track = Track::from_catalog(track_index);
        let lap = track.lap_length().unwrap();
        for n in 0..20 {
            let progress = lap * n as f64 / 20.0;
            let centerline = track.centerline(progress - 50.0, progress + 250.0, 15.0);
            let road = Road::new(centerline, track.half_width(progress), 0.1);
            let (p, yaw) = track.pose(progress);
            let ego = State::from((p, yaw, 20.0));
            let ctx = Context::new(&road, &[], 100, crate::planning::ComputeBudget::NOMINAL, None, None);
            let mut state = ego;
            for (tick, control) in FrenetSamplingPlanner::default().plan(ego, &ctx).into_iter().enumerate() {
                state = world_step(state, control, road.dt);
                let d = Path::new(road.centerline()).project(state.position()).d;
                assert!(
                    !collides_with_road_barrier(state, &road),
                    "track {track_index} progress {progress} width {} tick {tick} d {d} state {state:?}",
                    road.half_width
                );
            }
        }
    }
}
#[test]
fn accelerates_on_an_empty_straight_at_cruising_speeds() {
    let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
    let controls: Vec<_> = [0.0, 10.0, 20.0, 40.0]
        .into_iter()
        .map(|speed| {
            let ego = State {
                speed,
                ..Default::default()
            };
            (
                speed,
                FrenetSamplingPlanner::default().plan(ego, &test_ctx(&road, &[]))[0],
            )
        })
        .collect();
    assert!(
        controls.iter().all(|(_, u)| u.acceleration > 0.0),
        "empty-road first controls: {controls:?}"
    );
}

#[test]
fn evaluates_shared_horizon_independently_of_requested_controls() {
    for dt in [0.1, 0.2, 0.3] {
        let mut road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        road.dt = dt;
        let diagnostics = Diagnostics::default();
        let mut ctx = Context::new(&road, &[], 1, ComputeBudget::NOMINAL, None, Some(&diagnostics));
        let short = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
        assert_eq!(short.len(), 1);
        assert!(short[0].acceleration > 0.0);
        let trajectories = diagnostics.take().trajectories;
        assert!(!trajectories.is_empty());
        let ticks = (PLANNING_HORIZON_S / dt).ceil() as usize;
        assert!(trajectories.iter().all(|points| points.len() == ticks + 1));
        ctx.horizon = ticks;
        let long = FrenetSamplingPlanner::default().plan(State::default(), &ctx);
        assert_eq!(long.len(), ticks);
        assert_eq!(short[0], long[0]);
        assert!(
            long.iter()
                .all(|u| u.acceleration.is_finite() && u.curvature.is_finite())
        );
        ctx.horizon = 0;
        assert!(FrenetSamplingPlanner::default().plan(State::default(), &ctx).is_empty());
    }
}
