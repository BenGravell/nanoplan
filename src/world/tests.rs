use super::road::ROAD_AHEAD_M;
use super::traffic::lateral_target;
use super::*;
use crate::geometry::barrier::collides_with_road_barrier;
use crate::planning::LatencyStats;
use crate::simulation::Position;
use crate::track::ROAD_SAMPLE_STEP_M;

#[test]
fn ego_can_start_from_a_frenet_state() {
    let start = EgoStart {
        progress: 123.0,
        transverse: 1.25,
        yaw_offset: -0.2,
        speed: 17.0,
    };
    let world = LiveWorld::with_track_at(0, 1, PlannerKind::Straight, 0, 0.1, start);
    let (center, centerline_yaw) = world.track.pose(start.progress);
    let left = Position::from_angle(centerline_yaw + std::f64::consts::FRAC_PI_2);
    let ego = world.ego();

    assert_eq!(world.track_progress, start.progress);
    assert_eq!(world.road_anchor_x, start.progress);
    assert!((ego.position().x - (center.x + start.transverse * left.x)).abs() < 1e-12);
    assert!((ego.position().y - (center.y + start.transverse * left.y)).abs() < 1e-12);
    assert_eq!(ego.pose.yaw, centerline_yaw + start.yaw_offset);
    assert_eq!(ego.speed, start.speed);
}

#[test]
fn lattice_small_track_accelerates_and_previews_stay_on_road() {
    let small_track = crate::track::TRACK_PRESETS.len();
    let mut world = LiveWorld::with_track(small_track, 1, PlannerKind::Lattice, 0, 0.1);
    world.tick_with_latency(None);
    assert!(
        world.actuation().acceleration > MAX_LON_ACCEL - 0.1,
        "initial acceleration was {}",
        world.actuation().acceleration
    );

    let approach_progress = 100.0;
    let (position, yaw) = world.track.pose(approach_progress);
    world.simulator.state = State::from((position, yaw, 34.0));
    world.track_progress = approach_progress;
    world.road_anchor_x = approach_progress;
    world.road = road_window(&world.track, approach_progress, world.ego().speed, world.dt(), true);
    world.tick_with_latency(None);

    assert!(
        world
            .trajectory
            .states
            .iter()
            .all(|state| !collides_with_road_barrier(*state, &world.road)),
        "corner preview left the road: {:?}",
        world.trajectory.states
    );
}

#[test]
fn bezier_toppra_one_lap_logical_clocks_are_stable() {
    let small_track = crate::track::TRACK_PRESETS.len();
    let mut world = LiveWorld::with_track(small_track, 1, PlannerKind::BezierToppra, 5, 0.1);
    let lap_length = world.track.lap_length().unwrap();
    let recorder = Latency::default();
    let mut latency = LatencyStats::default();
    let mut ticks = 0;

    while world.track_progress < lap_length && ticks < 2_000 {
        world.tick_recording_latency(&recorder);
        latency.absorb(recorder.take());
        ticks += 1;
    }

    assert_eq!(ticks, 297);
    for (name, calls, total_clocks, max_clocks) in [
        ("simulation.progress", 297, 297, 1),
        ("simulation.actors", 297, 1_485, 5),
        ("simulation.actor_culling", 297, 1_485, 5),
        ("route", 297, 89_694, 302),
        ("bezier_fit", 297, 594, 2),
        ("optimize", 297, 635_683, 16_329),
        ("extract", 297, 9_207, 31),
        ("planner.total", 297, 735_178, 16_664),
        ("simulation.preview", 297, 8_910, 30),
        ("simulation.ego", 297, 297, 1),
        ("simulation.collisions", 297, 1_782, 6),
        ("simulation.total", 297, 760_270, 16_712),
        ("simulation.roads", 36, 10_836, 301),
    ] {
        let seam = latency
            .seams
            .iter()
            .find(|seam| seam.name == name)
            .unwrap_or_else(|| panic!("missing logical clock seam {name}"));
        assert_eq!(
            (seam.calls, seam.total_clocks, seam.max_clocks),
            (calls, total_clocks, max_clocks),
            "{name}"
        );
    }
}

#[test]
fn world_keeps_driving_without_a_route_or_goal() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::BezierToppra, 0, 0.1);
    for _ in 0..100 {
        world.tick_with_latency(None);
    }
    assert!(world.track_progress > 5.0);
    assert!(world.road.centerline().len() > 10);
}

#[test]
fn grid_position_ranks_ego_against_every_racer() {
    let world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 2, 0.1);

    assert_eq!(world.grid_position(), (2, 3));
}

#[test]
fn racer_progress_uses_the_farthest_corner() {
    let world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 0, 0.1);
    let state = world.ego();
    let corner_progress = CAR_FOOTPRINT
        .corners(state.pose())
        .map(|corner| world.track.project_progress(corner, 0.0));

    assert_eq!(
        racer_progress(&world.track, state, EGO_FOOTPRINT, 0.0),
        corner_progress.into_iter().max_by(f64::total_cmp).unwrap()
    );
    assert!(racer_progress(&world.track, state, EGO_FOOTPRINT, 0.0) > world.track_progress);
}

#[test]
fn resizing_traffic_removes_the_farthest_behind_and_adds_only_behind() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 5, 0.1);
    let ego_position = world.grid_position().0;
    let least_progress_id = world
        .actors
        .iter()
        .min_by(|a, b| a.track_x.total_cmp(&b.track_x))
        .unwrap()
        .id;

    world.set_actor_count(1, 4);

    assert_eq!(world.grid_position(), (ego_position, 5));
    assert!(world.actors.iter().all(|actor| actor.id != least_progress_id));
    let retained_ids: Vec<_> = world.actors.iter().map(|actor| actor.id).collect();

    world.set_actor_count(1, 7);

    assert_eq!(world.grid_position(), (ego_position, 8));
    assert!(
        world
            .actors
            .iter()
            .filter(|actor| !retained_ids.contains(&actor.id))
            .all(|actor| actor.track_x < world.track_progress)
    );
}

#[test]
fn app_ticks_keep_traffic_motion_continuous_and_forward() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Basic, 12, crate::viewer::DT);

    for tick in 0..1_500 {
        let previous: Vec<_> = world
            .actors
            .iter()
            .map(|actor| (actor.id, actor.state, actor.track_x, actor.lateral))
            .collect();
        world.tick_with_latency(None);

        for actor in &world.actors {
            let (_, before, before_track_x, before_lateral) =
                previous.iter().find(|(id, _, _, _)| *id == actor.id).copied().unwrap();
            let displacement = before.position().distance(actor.state.position());
            assert!(
                displacement < 20.0,
                "actor {} teleported {displacement:.1} m on app tick {tick}, progress {before_track_x:.1} -> {:.1} of {:?}, lateral {before_lateral:.1} -> {:.1}, track {:?} -> {:?}: {before:?} -> {:?}",
                actor.id,
                actor.track_x,
                world.track.lap_length(),
                actor.lateral,
                world.track.point(before_track_x),
                world.track.point(actor.track_x),
                actor.state
            );

            let (_, lane_yaw) = world.track.pose(actor.track_x);
            let forward_speed = actor.state.speed * (actor.state.pose.yaw - lane_yaw).cos();
            assert!(
                forward_speed >= -1e-6,
                "actor {} reversed at {forward_speed:.1} m/s on app tick {tick}: {:?}",
                actor.id,
                actor.state
            );
        }
    }
}

#[test]
fn planner_only_sees_reachable_traffic() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 12, 0.1);
    world.tick_with_latency(None);
    assert!(world.last_planner_actors > 0);
    assert!(world.last_planner_actors < world.actors.len());
}

#[test]
fn ego_bounces_off_road_barriers() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 0, 0.1);
    world.road = Road::new(vec![[-100.0, 0.0], [100.0, 0.0]], 10.0, 3.5, 0.1);
    world.collision_road = world.road.clone();
    world.simulator.state = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 0.0), std::f64::consts::FRAC_PI_2),
        20.0,
    );
    world.track_progress = world.track.project_progress(Position::default(), 0.0);
    world.road_anchor_x = world.track_progress;

    world.tick_with_latency(None);

    let support = EGO_FOOTPRINT.support(world.ego().pose.yaw, [0.0, 1.0]);
    assert!(
        world.ego().position().y <= world.road.half_width - support + 1e-9,
        "ego {:?}, support {support}",
        world.ego()
    );
    assert!(world.ego().pose.yaw < 0.0, "ego {:?}", world.ego());
}

#[test]
fn traffic_starts_on_both_sides_and_personality_moves_it_laterally() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 12, 0.1);
    assert!(world.actors.iter().any(|a| a.track_x < world.track_progress));
    assert!(world.actors.iter().any(|a| a.track_x > world.track_progress));

    let timid = Personality {
        aggressiveness: 0.0,
        sloppiness: 0.0,
    };
    assert!(lateral_target(timid, 4.0, 0.5) < 0.0);

    let before: Vec<f64> = world.actors.iter().map(|a| a.lateral).collect();
    for _ in 0..500 {
        world.step_traffic();
    }
    assert!(
        world
            .actors
            .iter()
            .zip(before)
            .any(|(actor, start)| (actor.lateral - start).abs() > 0.1)
    );
}

#[test]
fn unblocked_traffic_accelerates() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
    let before = world.actors[0].state.speed;
    world.step_traffic();
    assert!(world.actors[0].state.speed > before);
}

#[test]
fn traffic_keeps_rebound_velocity_on_the_next_tick() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
    let (p, lane_yaw) = world.track.pose(0.0);
    world.actors[0].track_x = 0.0;
    world.actors[0].lateral = 0.0;
    world.actors[0].lateral_target = 0.0;
    world.actors[0].state = State::new(
        crate::simulation::Pose::new(
            crate::simulation::Position::new(p.x, p.y),
            lane_yaw + std::f64::consts::PI,
        ),
        10.0,
    );

    world.step_traffic();

    assert!(world.actors[0].track_x < 0.0);
    let (_, next_lane_yaw) = world.track.pose(world.actors[0].track_x);
    assert!(world.actors[0].state.speed * (world.actors[0].state.pose.yaw - next_lane_yaw).cos() < 0.0);
}

#[test]
fn ego_and_actor_both_receive_collision_response() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
    world.road = Road::new(vec![[-100.0, 0.0], [100.0, 0.0]], 10.0, 50.0, 0.1);
    world.collision_road = world.road.clone();
    world.simulator.state = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 0.0), 0.0),
        10.0,
    );
    world.actors[0].state = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(4.0, 0.0), 0.0),
        0.0,
    );
    let previous_ego = world.ego();
    let previous_actors = [(world.actors[0].id, world.actors[0].state)];

    world.resolve_collisions(previous_ego, &previous_actors);

    assert!(world.ego().speed < 10.0);
    assert!(world.actors[0].state.speed > 0.0);
    assert!(world.ego().position().x < 0.0);
    assert!(world.actors[0].state.position().x > 4.0);
}

#[test]
fn traffic_bounces_off_static_road_barriers() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
    world.road = Road::new(vec![[-100.0, 0.0], [100.0, 0.0]], 10.0, 3.5, 0.1);
    world.collision_road = world.road.clone();
    world.simulator.state = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(-50.0, 0.0), 0.0),
        0.0,
    );
    let before = State::new(
        crate::simulation::Pose::new(crate::simulation::Position::new(12.0, 0.0), std::f64::consts::FRAC_PI_2),
        10.0,
    );
    world.actors[0].state = {
        let mut state = before;
        state.pose.position.y = 4.5;
        state
    };
    let previous_ego = world.ego();
    let previous_actors = [(world.actors[0].id, before)];

    world.resolve_collisions(previous_ego, &previous_actors);

    let actor = world.actors[0].state;
    assert!(actor.pose.yaw < 0.0, "actor did not rebound: {actor:?}");
    assert!(actor.position().y + CAR_FOOTPRINT.support(actor.pose.yaw, [0.0, 1.0]) <= world.road.half_width + 1e-9);
}

#[test]
fn traffic_continues_past_the_rolling_road_window_end() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
    let progress = world.road_anchor_x + ROAD_AHEAD_M + 2.0 * ROAD_SAMPLE_STEP_M;
    let (p, yaw) = world.track.pose(progress);
    let actor = State::from((p, yaw, 10.0));
    world.actors[0].track_x = progress;
    world.actors[0].state = actor;
    let previous_ego = world.ego();
    let previous_actors = [(world.actors[0].id, actor)];

    world.resolve_collisions(previous_ego, &previous_actors);

    assert_eq!(world.actors[0].state, actor);
}

#[test]
fn preview_horizon_and_diagnostics_are_live_configurable() {
    let mut world = LiveWorld::with_track(0, 1, PlannerKind::Lattice, 0, 0.1);
    world.preview_ticks = 5;
    world.diagnostics_enabled = true;
    world.tick_with_latency(None);
    assert_eq!(world.trajectory.len(), 5);
    assert!(!world.diagnostics.points.is_empty());

    world.preview_ticks = 0;
    world.diagnostics_enabled = false;
    world.tick_with_latency(None);
    assert_eq!(world.trajectory.len(), 1);
    assert!(world.diagnostics.points.is_empty());
}
