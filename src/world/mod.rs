//! Realtime driving on a generated or downloaded closed race track.

use web_time::Instant;

use crate::common::rng::Rng;
use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, Footprint};
use crate::planning::{
    Context, Diagnostics, DiagnosticsData, Latency, PLANNING_HORIZON_S, Planner, PlannerKind,
};
use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::simulation::{Control, DynamicBody, Simulator, State, collide_dynamic_bodies};
use crate::track::{ROAD_SAMPLE_STEP_M, Road, Track};
use crate::vehicle::{MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL};

const DEFAULT_PREVIEW_TICKS: usize = 30;
const ROAD_BEHIND_M: f64 = 50.0;
const ROAD_AHEAD_M: f64 = 250.0;
const ACTOR_MARGIN_M: f64 = 25.0;

/// A car following the same single track as the ego.
pub(crate) struct SmartActor {
    pub(crate) id: usize,
    pub(crate) state: State,
    pub(crate) personality: Personality,
    track_x: f64,
    lateral: f64,
    lateral_target: f64,
    next_wander_x: f64,
    rng: Rng,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Personality {
    pub(crate) aggressiveness: f64,
    pub(crate) sloppiness: f64,
}

/// The complete demo world: one track, traffic, ego, and planner.
pub(crate) struct LiveWorld {
    pub(crate) track: Track,
    pub(crate) track_progress: f64,
    pub(crate) road: Road,
    pub(crate) actors: Vec<SmartActor>,
    pub(crate) plan: Vec<State>,
    pub(crate) plan_controls: Vec<Control>,
    pub(crate) diagnostics: DiagnosticsData,
    pub(crate) last_plan_ms: f64,
    pub(crate) last_planner_actors: usize,
    pub(crate) preview_ticks: usize,
    pub(crate) diagnostics_enabled: bool,
    planner_kind: PlannerKind,
    planner: Box<dyn Planner>,
    simulator: Simulator,
    collision_road: Road,
    road_anchor_x: f64,
}

impl LiveWorld {
    pub(crate) fn with_track(
        track_index: usize,
        seed: u64,
        planner: PlannerKind,
        max_actors: usize,
        dt: f64,
    ) -> Self {
        let track = Track::from_catalog(track_index, seed);
        let (p, yaw) = track.pose(0.0);
        let ego = State {
            x: p[0],
            y: p[1],
            yaw,
            ..Default::default()
        };
        let road = road_window(&track, 0.0, dt);
        let collision_road = full_circuit_road(&track, dt);
        let actor_count = max_actors.min(15);
        let behind = if actor_count > 1 {
            (actor_count / 3).max(1)
        } else {
            0
        };
        let mut rng = Rng(seed.max(1));
        let actors = (0..actor_count)
            .map(|i| {
                let x = if i < behind {
                    -45.0 * (i + 1) as f64
                } else {
                    55.0 * (i - behind + 1) as f64
                };
                let personality = Personality {
                    aggressiveness: rng.uniform(),
                    sloppiness: rng.uniform(),
                };
                let mut actor_rng = Rng(rng.0.max(1));
                let lateral = lateral_target(personality, track.half_width(x), actor_rng.uniform());
                let (p, yaw) = track.pose(x);
                SmartActor {
                    id: i,
                    state: State {
                        x: p[0] - lateral * yaw.sin(),
                        y: p[1] + lateral * yaw.cos(),
                        yaw,
                        speed: 5.0 + 4.0 * rng.uniform(),
                    },
                    personality,
                    track_x: x,
                    lateral,
                    lateral_target: lateral,
                    next_wander_x: x + 15.0 + 25.0 * actor_rng.uniform(),
                    rng: actor_rng,
                }
            })
            .collect();
        Self {
            track,
            track_progress: 0.0,
            road,
            actors,
            plan: vec![],
            plan_controls: vec![],
            diagnostics: DiagnosticsData::default(),
            last_plan_ms: 0.0,
            last_planner_actors: 0,
            preview_ticks: DEFAULT_PREVIEW_TICKS,
            diagnostics_enabled: false,
            planner_kind: planner,
            planner: planner.build(),
            simulator: Simulator::new(ego, dt),
            collision_road,
            road_anchor_x: 0.0,
        }
    }

    pub(crate) fn set_planner(&mut self, kind: PlannerKind) {
        if kind != self.planner_kind {
            self.planner_kind = kind;
            self.planner = kind.build();
        }
    }

    pub(crate) fn set_actor_count(&mut self, seed: u64, actor_count: usize) {
        let actor_count = actor_count.min(15);
        while self.actors.len() > actor_count {
            let least_progress = self
                .actors
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.track_x.total_cmp(&b.track_x))
                .map(|(index, _)| index)
                .expect("non-empty traffic has a least-progress racer");
            self.actors.remove(least_progress);
        }

        let mut slot = 0;
        while self.actors.len() < actor_count {
            let next_id = (0..)
                .find(|id| self.actors.iter().all(|actor| actor.id != *id))
                .expect("there is always another actor id");
            let x = loop {
                let offset = -45.0 * (slot + 1) as f64;
                slot += 1;
                let candidate = self.track_progress + offset;
                if self
                    .actors
                    .iter()
                    .all(|actor| (actor.track_x - candidate).abs() > ACTOR_MARGIN_M)
                {
                    break candidate;
                }
            };
            let mut rng = Rng(seed.max(1));
            for _ in 0..next_id * 4 {
                rng.uniform();
            }
            let personality = Personality {
                aggressiveness: rng.uniform(),
                sloppiness: rng.uniform(),
            };
            let mut actor_rng = Rng(rng.0.max(1));
            let lateral =
                lateral_target(personality, self.track.half_width(x), actor_rng.uniform());
            let (p, yaw) = self.track.pose(x);
            self.actors.push(SmartActor {
                id: next_id,
                state: State {
                    x: p[0] - lateral * yaw.sin(),
                    y: p[1] + lateral * yaw.cos(),
                    yaw,
                    speed: 5.0 + 4.0 * rng.uniform(),
                },
                personality,
                track_x: x,
                lateral,
                lateral_target: lateral,
                next_wander_x: x + 15.0 + 25.0 * actor_rng.uniform(),
                rng: actor_rng,
            });
        }
    }

    pub(crate) fn actuation(&self) -> Control {
        self.simulator.actuation()
    }

    pub(crate) fn ego(&self) -> State {
        self.simulator.state
    }

    pub(crate) fn dt(&self) -> f64 {
        self.simulator.dt
    }

    /// Ego's race position and the total number of racers.
    pub(crate) fn grid_position(&self) -> (usize, usize) {
        let ego_progress =
            racer_progress(&self.track, self.ego(), EGO_FOOTPRINT, self.track_progress);
        let ahead = self
            .actors
            .iter()
            .filter(|actor| {
                racer_progress(&self.track, actor.state, CAR_FOOTPRINT, actor.track_x)
                    > ego_progress
            })
            .count();
        (ahead + 1, self.actors.len() + 1)
    }

    pub(crate) fn tick_recording_latency(&mut self, latency: &Latency) {
        latency.time("simulation.total", || self.tick_with_latency(Some(latency)));
    }

    fn tick_with_latency(&mut self, latency: Option<&Latency>) {
        self.track_progress = timed(latency, "simulation.progress", || {
            self.track
                .project_progress([self.ego().x, self.ego().y], self.track_progress)
        });
        if (self.track_progress - self.road_anchor_x).abs() >= 20.0 {
            self.road_anchor_x = (self.track_progress / 20.0).floor() * 20.0;
            self.road = timed(latency, "simulation.roads", || {
                road_window(&self.track, self.road_anchor_x, self.dt())
            });
        }

        let previous_actors: Vec<_> = self.actors.iter().map(|a| (a.id, a.state)).collect();
        timed(latency, "simulation.actors", || self.step_traffic());
        let ego_reach = self.ego().speed.max(0.0) * PLANNING_HORIZON_S
            + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2);
        let actor_states: Vec<State> = timed(latency, "simulation.actor_culling", || {
            self.actors
                .iter()
                .filter(|a| {
                    a.track_x <= self.track_progress + ego_reach + ACTOR_MARGIN_M
                        && a.track_x + a.state.speed * PLANNING_HORIZON_S
                            >= self.track_progress - ACTOR_MARGIN_M
                })
                .map(|a| a.state)
                .collect()
        });
        self.last_planner_actors = actor_states.len();

        let diagnostics = Diagnostics::default();
        let ego = self.ego();
        let controls = {
            let ctx = Context {
                road: &self.road,
                actors: &actor_states,
                horizon: self.preview_ticks.max(1),
                latency,
                diagnostics: self.diagnostics_enabled.then_some(&diagnostics),
            };
            let start = Instant::now();
            let controls = match latency {
                Some(l) => l.time("planner.total", || self.planner.plan(ego, &ctx)),
                None => self.planner.plan(ego, &ctx),
            };
            self.last_plan_ms = start.elapsed().as_secs_f64() * 1e3;
            controls
        };
        self.diagnostics = diagnostics.take();

        self.plan = timed(latency, "simulation.preview", || {
            self.simulator.preview(&controls, self.preview_ticks)
        });
        self.plan_controls = controls.into_iter().take(self.plan.len()).collect();
        let previous_ego = self.ego();
        timed(latency, "simulation.ego", || {
            self.simulator
                .step(self.plan_controls.first().copied().unwrap_or_default());
        });
        timed(latency, "simulation.collisions", || {
            self.resolve_collisions(previous_ego, &previous_actors);
        });
    }

    fn step_traffic(&mut self) {
        let dt = self.dt();
        for actor in &mut self.actors {
            actor.track_x = self
                .track
                .project_progress([actor.state.x, actor.state.y], actor.track_x);
            let (p, lane_yaw) = self.track.pose(actor.track_x);
            let left = [-lane_yaw.sin(), lane_yaw.cos()];
            actor.lateral = (actor.state.x - p[0]) * left[0] + (actor.state.y - p[1]) * left[1];
        }
        self.actors.sort_by(|a, b| a.track_x.total_cmp(&b.track_x));
        let snapshot: Vec<(f64, f64)> = self
            .actors
            .iter()
            .map(|a| {
                let (_, lane_yaw) = self.track.pose(a.track_x);
                let forward_speed = a.state.speed * (a.state.yaw - lane_yaw).cos();
                (a.track_x, forward_speed)
            })
            .collect();
        for (i, actor) in self.actors.iter_mut().enumerate() {
            let (_, lane_yaw) = self.track.pose(actor.track_x);
            let mut forward_speed = actor.state.speed * (actor.state.yaw - lane_yaw).cos();
            let mut lateral_speed = actor.state.speed * (actor.state.yaw - lane_yaw).sin();
            let lead = snapshot
                .get(i + 1)
                .map(|next| (next.0 - actor.track_x - CAR_FOOTPRINT.length, next.1));
            let accel = lead.map_or(MAX_LON_ACCEL, |(gap, lead_speed)| {
                ((lead_speed * lead_speed - forward_speed * forward_speed) / (2.0 * gap.max(1.0)))
                    .clamp(crate::vehicle::MIN_LON_ACCEL, MAX_LON_ACCEL)
            });
            forward_speed = (forward_speed + accel * dt)
                .clamp(-*MAX_TERMINAL_SPEED_MPS, *MAX_TERMINAL_SPEED_MPS);
            actor.track_x += forward_speed * dt;
            if actor.track_x >= actor.next_wander_x {
                actor.lateral_target = lateral_target(
                    actor.personality,
                    self.track.half_width(actor.track_x),
                    actor.rng.uniform(),
                );
                actor.next_wander_x = actor.track_x + 15.0 + 25.0 * actor.rng.uniform();
            }
            let desired_lateral_speed = (actor.lateral_target - actor.lateral).clamp(-0.35, 0.35);
            lateral_speed += (desired_lateral_speed - lateral_speed)
                .clamp(-MAX_ABS_LAT_ACCEL * dt, MAX_ABS_LAT_ACCEL * dt);
            actor.lateral += lateral_speed * dt;
            let (p, lane_yaw) = self.track.pose(actor.track_x);
            let velocity = [
                forward_speed * lane_yaw.cos() - lateral_speed * lane_yaw.sin(),
                forward_speed * lane_yaw.sin() + lateral_speed * lane_yaw.cos(),
            ];
            let speed = velocity[0].hypot(velocity[1]);
            let yaw = if speed > 1e-9 {
                velocity[1].atan2(velocity[0])
            } else {
                lane_yaw
            };
            actor.state = State {
                x: p[0] - actor.lateral * lane_yaw.sin(),
                y: p[1] + actor.lateral * lane_yaw.cos(),
                yaw,
                speed,
            };
        }
    }

    fn resolve_collisions(&mut self, previous_ego: State, previous_actors: &[(usize, State)]) {
        let mut previous = Vec::with_capacity(self.actors.len() + 1);
        previous.push(DynamicBody::new(
            previous_ego,
            crate::geometry::EGO_FOOTPRINT,
        ));
        previous.extend(self.actors.iter().map(|actor| {
            let state = previous_actors
                .iter()
                .find(|(id, _)| *id == actor.id)
                .map_or(actor.state, |(_, state)| *state);
            DynamicBody::new(state, CAR_FOOTPRINT)
        }));

        let mut bodies = Vec::with_capacity(self.actors.len() + 1);
        bodies.push(DynamicBody::new(
            self.simulator.state,
            crate::geometry::EGO_FOOTPRINT,
        ));
        bodies.extend(
            self.actors
                .iter()
                .map(|actor| DynamicBody::new(actor.state, CAR_FOOTPRINT)),
        );

        // Every moving body meets the same immovable road boundary before it
        // participates in symmetric vehicle-to-vehicle contacts.
        for (before, body) in previous.iter().zip(&mut bodies) {
            body.state = crate::geometry::barrier::collide_with_road_barriers(
                before.state,
                body.state,
                body.footprint,
                &self.collision_road,
            );
        }
        let before_dynamic = bodies.clone();
        collide_dynamic_bodies(&mut bodies);
        for (before, body) in before_dynamic.iter().zip(&mut bodies) {
            body.state = crate::geometry::barrier::collide_with_road_barriers(
                before.state,
                body.state,
                body.footprint,
                &self.collision_road,
            );
        }

        self.simulator.state = bodies[0].state;
        for (actor, body) in self.actors.iter_mut().zip(&bodies[1..]) {
            actor.state = body.state;
        }
    }
}

fn racer_progress(track: &Track, state: State, footprint: Footprint, hint: f64) -> f64 {
    footprint
        .corners(state.pose())
        .into_iter()
        .map(|corner| track.project_progress(corner, hint))
        .max_by(f64::total_cmp)
        .expect("a car footprint always has corners")
}

fn lateral_target(personality: Personality, half_width: f64, random: f64) -> f64 {
    let room = (half_width - CAR_FOOTPRINT.width / 2.0 - 0.3).max(0.0);
    let timid_bias = -0.65 * (1.0 - personality.aggressiveness).powi(2) * room;
    (timid_bias + (2.0 * random - 1.0) * 0.55 * personality.sloppiness * room).clamp(-room, room)
}

fn road_window(track: &Track, x: f64, dt: f64) -> Road {
    let polygon = track
        .road_polygon(
            x - ROAD_BEHIND_M,
            x + ROAD_AHEAD_M,
            ROAD_SAMPLE_STEP_M,
            false,
        )
        .expect("track road window must form a valid polygon");
    Road::from_polygon(polygon, *MAX_TERMINAL_SPEED_MPS, dt)
}

fn full_circuit_road(track: &Track, dt: f64) -> Road {
    let length = track
        .lap_length()
        .expect("the live driving world requires a closed circuit");
    let polygon = track
        .road_polygon(0.0, length, ROAD_SAMPLE_STEP_M, true)
        .expect("track road must form a valid closed polygon");
    Road::from_polygon(polygon, *MAX_TERMINAL_SPEED_MPS, dt)
}

fn timed<T>(latency: Option<&Latency>, name: &'static str, f: impl FnOnce() -> T) -> T {
    match latency {
        Some(l) => l.time(name, f),
        None => f(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::LatencyStats;

    /// Manual end-to-end profiling run. Kept ignored because wall-clock
    /// measurements are meaningful in an optimized build on an otherwise
    /// idle machine, not as part of the correctness suite.
    ///
    /// Run with:
    /// `cargo test --release bezier_toppra_profiles_one_small_track_lap -- --ignored --nocapture`
    #[test]
    #[ignore = "end-to-end latency profile"]
    fn bezier_toppra_profiles_one_small_track_lap() {
        let small_track = crate::track::TRACK_PRESETS.len();
        let mut world = LiveWorld::with_track(small_track, 1, PlannerKind::BezierToppra, 5, 0.1);
        let lap_length = world.track.lap_length().unwrap();
        let recorder = Latency::default();
        let mut latency = LatencyStats::default();
        let started = Instant::now();
        let mut ticks = 0;

        while world.track_progress < lap_length && ticks < 2_000 {
            world.tick_recording_latency(&recorder);
            latency.absorb(recorder.take());
            ticks += 1;
        }

        assert!(
            world.track_progress >= lap_length,
            "planner only reached {:.1}/{lap_length:.1} m in {ticks} ticks",
            world.track_progress
        );
        for seam in &latency.seams {
            eprintln!(
                "{:<28} calls {:>4}  mean {:>8.3} ms  max {:>8.3} ms",
                seam.name,
                seam.calls,
                seam.mean_ms(),
                seam.max_ms
            );
            assert!(seam.total_ms.is_finite() && seam.total_ms >= 0.0);
        }
        eprintln!(
            "{:<28} ticks {:>4}  simulated {:>7.2} s  wall {:>8.3} ms",
            "one_lap",
            ticks,
            ticks as f64 * world.dt(),
            started.elapsed().as_secs_f64() * 1e3
        );

        for expected in [
            "simulation.total",
            "simulation.actors",
            "planner.total",
            "route",
            "optimize",
            "extract",
            "simulation.preview",
            "simulation.ego",
            "simulation.collisions",
        ] {
            assert!(
                latency.seams.iter().any(|seam| seam.name == expected),
                "missing latency seam {expected}"
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
        assert!(
            world
                .actors
                .iter()
                .all(|actor| actor.id != least_progress_id)
        );
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
                let (_, before, before_track_x, before_lateral) = previous
                    .iter()
                    .find(|(id, _, _, _)| *id == actor.id)
                    .copied()
                    .unwrap();
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
                let forward_speed = actor.state.speed * (actor.state.yaw - lane_yaw).cos();
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
        world.simulator.state = State {
            x: 0.0,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 20.0,
        };
        world.track_progress = world.track.project_progress([0.0, 0.0], 0.0);
        world.road_anchor_x = world.track_progress;

        world.tick_with_latency(None);

        let support = EGO_FOOTPRINT.support(world.ego().yaw, [0.0, 1.0]);
        assert!(
            world.ego().y <= world.road.half_width - support + 1e-9,
            "ego {:?}, support {support}",
            world.ego()
        );
        assert!(world.ego().yaw < 0.0, "ego {:?}", world.ego());
    }

    #[test]
    fn traffic_starts_on_both_sides_and_personality_moves_it_laterally() {
        let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 12, 0.1);
        assert!(
            world
                .actors
                .iter()
                .any(|a| a.track_x < world.track_progress)
        );
        assert!(
            world
                .actors
                .iter()
                .any(|a| a.track_x > world.track_progress)
        );

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
        world.actors[0].state = State {
            x: p[0],
            y: p[1],
            yaw: lane_yaw + std::f64::consts::PI,
            speed: 10.0,
        };

        world.step_traffic();

        assert!(world.actors[0].track_x < 0.0);
        let (_, next_lane_yaw) = world.track.pose(world.actors[0].track_x);
        assert!(
            world.actors[0].state.speed * (world.actors[0].state.yaw - next_lane_yaw).cos() < 0.0
        );
    }

    #[test]
    fn ego_and_actor_both_receive_collision_response() {
        let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
        world.road = Road::new(vec![[-100.0, 0.0], [100.0, 0.0]], 10.0, 50.0, 0.1);
        world.collision_road = world.road.clone();
        world.simulator.state = State {
            x: 0.0,
            speed: 10.0,
            ..Default::default()
        };
        world.actors[0].state = State {
            x: 4.0,
            ..Default::default()
        };
        let previous_ego = world.ego();
        let previous_actors = [(world.actors[0].id, world.actors[0].state)];

        world.resolve_collisions(previous_ego, &previous_actors);

        assert!(world.ego().speed < 10.0);
        assert!(world.actors[0].state.speed > 0.0);
        assert!(world.ego().x < 0.0);
        assert!(world.actors[0].state.x > 4.0);
    }

    #[test]
    fn traffic_bounces_off_static_road_barriers() {
        let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
        world.road = Road::new(vec![[-100.0, 0.0], [100.0, 0.0]], 10.0, 3.5, 0.1);
        world.collision_road = world.road.clone();
        world.simulator.state = State {
            x: -50.0,
            ..Default::default()
        };
        let before = State {
            x: 12.0,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 10.0,
        };
        world.actors[0].state = State { y: 4.5, ..before };
        let previous_ego = world.ego();
        let previous_actors = [(world.actors[0].id, before)];

        world.resolve_collisions(previous_ego, &previous_actors);

        let actor = world.actors[0].state;
        assert!(actor.yaw < 0.0, "actor did not rebound: {actor:?}");
        assert!(
            actor.y + CAR_FOOTPRINT.support(actor.yaw, [0.0, 1.0]) <= world.road.half_width + 1e-9
        );
    }

    #[test]
    fn traffic_continues_past_the_rolling_road_window_end() {
        let mut world = LiveWorld::with_track(0, 1, PlannerKind::Straight, 1, 0.1);
        let progress = world.road_anchor_x + ROAD_AHEAD_M + 2.0 * ROAD_SAMPLE_STEP_M;
        let (p, yaw) = world.track.pose(progress);
        let actor = State {
            x: p[0],
            y: p[1],
            yaw,
            speed: 10.0,
        };
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
        assert_eq!(world.plan.len(), 5);
        assert!(!world.diagnostics.points.is_empty());

        world.preview_ticks = 0;
        world.diagnostics_enabled = false;
        world.tick_with_latency(None);
        assert!(world.plan.is_empty());
        assert!(world.diagnostics.points.is_empty());
    }
}
