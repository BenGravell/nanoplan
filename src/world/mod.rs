//! Realtime driving on one endless procedural track.

use web_time::Instant;

use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT};
use crate::planning::{
    Context, Diagnostics, DiagnosticsData, Latency, PLANNING_HORIZON_S, Planner, PlannerKind,
    bezier_idm::idm_accel,
};
use crate::simulation::{CommandLimiter, State, collide_with_actors};
use crate::track::{Road, Track};

const DEFAULT_PREVIEW_TICKS: usize = 30;
const ROAD_BEHIND_M: f64 = 50.0;
const ROAD_AHEAD_M: f64 = 250.0;
const ROAD_SAMPLE_M: f64 = 15.0;
const ACTOR_MARGIN_M: f64 = 25.0;

/// A car following the same single track as the ego.
pub struct SmartActor {
    pub state: State,
    target_speed: f64,
}

/// The complete demo world: one procedural track, traffic, ego, and planner.
pub struct LiveWorld {
    pub track: Track,
    pub road: Road,
    pub ego: State,
    pub actors: Vec<SmartActor>,
    pub plan: Vec<State>,
    pub diagnostics: DiagnosticsData,
    pub last_plan_ms: f64,
    pub last_planner_actors: usize,
    pub target_speed: f64,
    pub dt: f64,
    pub preview_ticks: usize,
    pub diagnostics_enabled: bool,
    planner_kind: PlannerKind,
    planner: Box<dyn Planner>,
    limiter: CommandLimiter,
    road_anchor_x: f64,
}

impl LiveWorld {
    pub fn new(seed: u64, planner: PlannerKind, max_actors: usize, dt: f64) -> Self {
        let track = Track::new(seed);
        let (p, yaw) = track.pose(0.0);
        let ego = State {
            x: p[0],
            y: p[1],
            yaw,
            ..Default::default()
        };
        let road = road_window(track, 0.0, 8.0, dt);
        let actors = (0..max_actors.min(16))
            .map(|i| {
                let x = 55.0 + i as f64 * 55.0;
                let (p, yaw) = track.pose(x);
                SmartActor {
                    state: State {
                        x: p[0],
                        y: p[1],
                        yaw,
                        speed: 5.0 + ((seed.wrapping_add(i as u64 * 17) % 40) as f64 / 10.0),
                    },
                    target_speed: 6.0 + ((seed.wrapping_add(i as u64 * 29) % 35) as f64 / 10.0),
                }
            })
            .collect();
        Self {
            track,
            road,
            ego,
            actors,
            plan: vec![],
            diagnostics: DiagnosticsData::default(),
            last_plan_ms: 0.0,
            last_planner_actors: 0,
            target_speed: 8.0,
            dt,
            preview_ticks: DEFAULT_PREVIEW_TICKS,
            diagnostics_enabled: false,
            planner_kind: planner,
            planner: planner.build(),
            limiter: CommandLimiter::new(),
            road_anchor_x: 0.0,
        }
    }

    pub fn set_planner(&mut self, kind: PlannerKind) {
        if kind != self.planner_kind {
            self.planner_kind = kind;
            self.planner = kind.build();
        }
    }

    pub fn tick(&mut self) {
        self.tick_with_latency(None);
    }

    pub fn tick_recording_latency(&mut self, latency: &Latency) {
        self.tick_with_latency(Some(latency));
    }

    fn tick_with_latency(&mut self, latency: Option<&Latency>) {
        if (self.ego.x - self.road_anchor_x).abs() >= 20.0 {
            self.road_anchor_x = (self.ego.x / 20.0).floor() * 20.0;
            self.road = timed(latency, "world_track", || {
                road_window(self.track, self.road_anchor_x, self.target_speed, self.dt)
            });
        } else {
            self.road.target_speed = self.target_speed;
        }

        timed(latency, "world_traffic", || self.step_traffic());
        let ego_reach = self.ego.speed.max(self.target_speed) * PLANNING_HORIZON_S;
        let actor_states: Vec<State> = self
            .actors
            .iter()
            .filter(|a| {
                a.state.x <= self.ego.x + ego_reach + ACTOR_MARGIN_M
                    && a.state.x + a.state.speed * PLANNING_HORIZON_S >= self.ego.x - ACTOR_MARGIN_M
            })
            .map(|a| a.state)
            .collect();
        self.last_planner_actors = actor_states.len();

        let diagnostics = Diagnostics::default();
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
                Some(l) => l.time("total", || self.planner.plan(self.ego, &ctx)),
                None => self.planner.plan(self.ego, &ctx),
            };
            self.last_plan_ms = start.elapsed().as_secs_f64() * 1e3;
            controls
        };
        self.diagnostics = diagnostics.take();

        let mut state = self.ego;
        let mut preview_limiter = self.limiter;
        self.plan = controls
            .iter()
            .take(self.preview_ticks)
            .map(|&u| {
                state = preview_limiter.step(state, u, self.dt);
                state
            })
            .collect();
        let next = self.limiter.step(
            self.ego,
            controls.first().copied().unwrap_or_default(),
            self.dt,
        );
        self.ego = collide_with_actors(next, self.actors.iter().map(|a| (a.state, CAR_FOOTPRINT)));
    }

    fn step_traffic(&mut self) {
        self.actors.sort_by(|a, b| a.state.x.total_cmp(&b.state.x));
        let snapshot: Vec<State> = self.actors.iter().map(|a| a.state).collect();
        for (i, actor) in self.actors.iter_mut().enumerate() {
            let lead = snapshot
                .get(i + 1)
                .map(|next| (next.x - actor.state.x - CAR_FOOTPRINT.length, next.speed));
            let accel = idm_accel(actor.state.speed, actor.target_speed, lead);
            let x = actor.state.x + (actor.state.speed + accel * self.dt).max(0.0) * self.dt;
            let (p, yaw) = self.track.pose(x);
            actor.state = State {
                x: p[0],
                y: p[1],
                yaw,
                speed: (actor.state.speed + accel * self.dt).max(0.0),
            };
        }
        let mut front = self
            .actors
            .iter()
            .map(|a| a.state.x)
            .fold(self.ego.x, f64::max);
        for actor in &mut self.actors {
            if actor.state.x < self.ego.x - 120.0 {
                front += 80.0;
                let x = front;
                let (p, yaw) = self.track.pose(x);
                actor.state = State {
                    x: p[0],
                    y: p[1],
                    yaw,
                    speed: actor.target_speed,
                };
            }
        }
    }
}

fn road_window(track: Track, x: f64, target_speed: f64, dt: f64) -> Road {
    let centerline = track.centerline(x - ROAD_BEHIND_M, x + ROAD_AHEAD_M, ROAD_SAMPLE_M);
    // The planners currently accept one width per horizon; curvature and the
    // rendered track remain continuously varying.
    let half_width = track.half_width(x).max(EGO_FOOTPRINT.width / 2.0 + 0.5);
    Road::new(centerline, target_speed, half_width, dt)
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
        let mut world = LiveWorld::new(1, PlannerKind::BezierIdm, 0, 0.1);
        for _ in 0..100 {
            world.tick();
        }
        assert!(world.ego.x > 5.0);
        assert!(world.road.centerline.last().unwrap()[0] > world.ego.x + 200.0);
    }

    #[test]
    fn planner_only_sees_reachable_traffic() {
        let mut world = LiveWorld::new(1, PlannerKind::Straight, 12, 0.1);
        world.tick();
        assert!(world.last_planner_actors > 0);
        assert!(world.last_planner_actors < world.actors.len());
    }

    #[test]
    fn preview_horizon_and_diagnostics_are_live_configurable() {
        let mut world = LiveWorld::new(1, PlannerKind::Lattice, 0, 0.1);
        world.preview_ticks = 5;
        world.diagnostics_enabled = true;
        world.tick();
        assert_eq!(world.plan.len(), 5);
        assert!(!world.diagnostics.points.is_empty());

        world.preview_ticks = 0;
        world.diagnostics_enabled = false;
        world.tick();
        assert!(world.plan.is_empty());
        assert!(world.diagnostics.points.is_empty());
    }
}
