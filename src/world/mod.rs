//! Realtime driving on a generated or downloaded closed race track.

use web_time::Instant;

use crate::common::rng::Rng;
use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT};
use crate::planning::{
    Context, Diagnostics, DiagnosticsData, Latency, PLANNING_HORIZON_S, Planner, PlannerKind,
};
use crate::simulation::physics::MAX_TERMINAL_SPEED_MPS;
use crate::simulation::{Control, Simulator, State};
use crate::track::{Road, Track};
use crate::vehicle::MAX_LON_ACCEL;

const DEFAULT_PREVIEW_TICKS: usize = 30;
const ROAD_BEHIND_M: f64 = 50.0;
const ROAD_AHEAD_M: f64 = 250.0;
const ROAD_SAMPLE_M: f64 = 15.0;
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
    pub(crate) diagnostics: DiagnosticsData,
    pub(crate) last_plan_ms: f64,
    pub(crate) last_planner_actors: usize,
    pub(crate) preview_ticks: usize,
    pub(crate) diagnostics_enabled: bool,
    planner_kind: PlannerKind,
    planner: Box<dyn Planner>,
    simulator: Simulator,
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
        let actor_count = max_actors.min(16);
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
            diagnostics: DiagnosticsData::default(),
            last_plan_ms: 0.0,
            last_planner_actors: 0,
            preview_ticks: DEFAULT_PREVIEW_TICKS,
            diagnostics_enabled: false,
            planner_kind: planner,
            planner: planner.build(),
            simulator: Simulator::new(ego, dt),
            road_anchor_x: 0.0,
        }
    }

    pub(crate) fn set_planner(&mut self, kind: PlannerKind) {
        if kind != self.planner_kind {
            self.planner_kind = kind;
            self.planner = kind.build();
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

    pub(crate) fn tick_recording_latency(&mut self, latency: &Latency) {
        self.tick_with_latency(Some(latency));
    }

    fn tick_with_latency(&mut self, latency: Option<&Latency>) {
        self.track_progress = self
            .track
            .project_progress([self.ego().x, self.ego().y], self.track_progress);
        if (self.track_progress - self.road_anchor_x).abs() >= 20.0 {
            self.road_anchor_x = (self.track_progress / 20.0).floor() * 20.0;
            self.road = timed(latency, "world_track", || {
                road_window(&self.track, self.road_anchor_x, self.dt())
            });
        }

        timed(latency, "world_traffic", || self.step_traffic());
        let ego_reach = self.ego().speed.max(0.0) * PLANNING_HORIZON_S
            + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2);
        let actor_states: Vec<State> = self
            .actors
            .iter()
            .filter(|a| {
                a.track_x <= self.track_progress + ego_reach + ACTOR_MARGIN_M
                    && a.track_x + a.state.speed * PLANNING_HORIZON_S
                        >= self.track_progress - ACTOR_MARGIN_M
            })
            .map(|a| a.state)
            .collect();
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
                Some(l) => l.time("total", || self.planner.plan(ego, &ctx)),
                None => self.planner.plan(ego, &ctx),
            };
            self.last_plan_ms = start.elapsed().as_secs_f64() * 1e3;
            controls
        };
        self.diagnostics = diagnostics.take();

        self.plan = self.simulator.preview(&controls, self.preview_ticks);
        self.simulator.step(
            controls.first().copied().unwrap_or_default(),
            &self.road,
            self.actors.iter().map(|a| (a.state, CAR_FOOTPRINT)),
        );
    }

    fn step_traffic(&mut self) {
        let dt = self.dt();
        self.actors.sort_by(|a, b| a.track_x.total_cmp(&b.track_x));
        let snapshot: Vec<(f64, f64)> = self
            .actors
            .iter()
            .map(|a| (a.track_x, a.state.speed))
            .collect();
        for (i, actor) in self.actors.iter_mut().enumerate() {
            let lead = snapshot
                .get(i + 1)
                .map(|next| (next.0 - actor.track_x - CAR_FOOTPRINT.length, next.1));
            let accel = lead.map_or(MAX_LON_ACCEL, |(gap, lead_speed)| {
                ((lead_speed * lead_speed - actor.state.speed * actor.state.speed)
                    / (2.0 * gap.max(1.0)))
                .clamp(crate::vehicle::MIN_LON_ACCEL, MAX_LON_ACCEL)
            });
            let speed = (actor.state.speed + accel * dt).clamp(0.0, *MAX_TERMINAL_SPEED_MPS);
            actor.track_x += speed * dt;
            if actor.track_x >= actor.next_wander_x {
                actor.lateral_target = lateral_target(
                    actor.personality,
                    self.track.half_width(actor.track_x),
                    actor.rng.uniform(),
                );
                actor.next_wander_x = actor.track_x + 15.0 + 25.0 * actor.rng.uniform();
            }
            let lateral_step = (actor.lateral_target - actor.lateral).clamp(-0.35 * dt, 0.35 * dt);
            actor.lateral += lateral_step;
            let (p, lane_yaw) = self.track.pose(actor.track_x);
            let yaw = lane_yaw + (lateral_step / (speed * dt).max(0.1)).atan();
            actor.state = State {
                x: p[0] - actor.lateral * lane_yaw.sin(),
                y: p[1] + actor.lateral * lane_yaw.cos(),
                yaw,
                speed,
            };
        }
        let mut front = self
            .actors
            .iter()
            .map(|a| a.track_x)
            .fold(self.track_progress, f64::max);
        for actor in &mut self.actors {
            if actor.track_x < self.track_progress - 120.0 {
                front += 80.0;
                actor.track_x = front;
                actor.lateral_target = lateral_target(
                    actor.personality,
                    self.track.half_width(front),
                    actor.rng.uniform(),
                );
                actor.lateral = actor.lateral_target;
                actor.next_wander_x = front + 15.0 + 25.0 * actor.rng.uniform();
                let (p, yaw) = self.track.pose(front);
                actor.state = State {
                    x: p[0] - actor.lateral * yaw.sin(),
                    y: p[1] + actor.lateral * yaw.cos(),
                    yaw,
                    speed: actor.state.speed,
                };
            }
        }
    }
}

fn lateral_target(personality: Personality, half_width: f64, random: f64) -> f64 {
    let room = (half_width - CAR_FOOTPRINT.width / 2.0 - 0.3).max(0.0);
    let timid_bias = -0.65 * (1.0 - personality.aggressiveness).powi(2) * room;
    (timid_bias + (2.0 * random - 1.0) * 0.55 * personality.sloppiness * room).clamp(-room, room)
}

fn road_window(track: &Track, x: f64, dt: f64) -> Road {
    let centerline = track.centerline(x - ROAD_BEHIND_M, x + ROAD_AHEAD_M, ROAD_SAMPLE_M);
    // The planners currently accept one width per horizon; curvature and the
    // rendered track remain continuously varying.
    let half_width = track.half_width(x).max(EGO_FOOTPRINT.width / 2.0 + 0.5);
    Road::new(centerline, *MAX_TERMINAL_SPEED_MPS, half_width, dt)
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

    #[test]
    fn world_keeps_driving_without_a_route_or_goal() {
        let mut world = LiveWorld::with_track(0, 1, PlannerKind::BezierToppra, 0, 0.1);
        for _ in 0..100 {
            world.tick_with_latency(None);
        }
        assert!(world.track_progress > 5.0);
        assert!(world.road.centerline.len() > 10);
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
