//! Realtime driving on a generated or downloaded closed race track.

mod road;
mod traffic;

use web_time::Instant;

use crate::common::kinematics::TrajectoryKinematics;
use crate::common::rng::Rng;
use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, Footprint};
use crate::planning::{
    ComputeBudget, Context, Diagnostics, DiagnosticsData, Latency, PLANNING_HORIZON_S, Planner, PlannerKind,
};
use crate::simulation::{Control, DynamicBody, Position, Simulator, State, collide_dynamic_bodies};
use crate::track::{Road, Track};
use crate::vehicle::MAX_LON_ACCEL;

use road::{full_circuit_road, road_window};
pub(crate) use traffic::SmartActor;
use traffic::{ACTOR_MARGIN_M, ACTOR_SPACING_AHEAD_M, ACTOR_SPACING_BEHIND_M, MAX_ACTORS, Personality};

const DEFAULT_PREVIEW_TICKS: usize = 30;

/// The complete demo world: one track, traffic, ego, and planner.
pub(crate) struct LiveWorld {
    pub(crate) track: Track,
    pub(crate) track_progress: f64,
    pub(crate) road: Road,
    pub(crate) actors: Vec<SmartActor>,
    pub(crate) trajectory: TrajectoryKinematics,
    pub(crate) diagnostics: DiagnosticsData,
    pub(crate) last_plan_ms: f64,
    pub(crate) last_planner_actors: usize,
    pub(crate) ego_collision_count: usize,
    pub(crate) preview_ticks: usize,
    pub(crate) diagnostics_enabled: bool,
    pub(crate) compute_budget: ComputeBudget,
    planner_kind: PlannerKind,
    planner: Box<dyn Planner>,
    simulator: Simulator,
    collision_road: Road,
    road_anchor_x: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct EgoStart {
    pub(crate) progress: f64,
    pub(crate) transverse: f64,
    pub(crate) yaw_offset: f64,
    pub(crate) speed: f64,
}

impl LiveWorld {
    pub(crate) fn with_track(track_index: usize, seed: u64, planner: PlannerKind, max_actors: usize, dt: f64) -> Self {
        Self::with_track_at(track_index, seed, planner, max_actors, dt, EgoStart::default())
    }

    pub(crate) fn with_track_at(
        track_index: usize,
        seed: u64,
        planner: PlannerKind,
        max_actors: usize,
        dt: f64,
        start: EgoStart,
    ) -> Self {
        let track = Track::from_catalog(track_index, seed);
        let (p, centerline_yaw) = track.pose(start.progress);
        let left = Position::from_angle(centerline_yaw + std::f64::consts::FRAC_PI_2);
        let ego = State::from((
            Position::new(p.x + start.transverse * left.x, p.y + start.transverse * left.y),
            centerline_yaw + start.yaw_offset,
            start.speed,
        ));
        let road = road_window(&track, start.progress, ego.speed, dt, planner == PlannerKind::Lattice);
        let collision_road = full_circuit_road(&track, dt);
        let actor_count = max_actors.min(MAX_ACTORS);
        let behind = if actor_count > 1 { (actor_count / 3).max(1) } else { 0 };
        let mut rng = Rng(seed.max(1));
        let actors = (0..actor_count)
            .map(|i| {
                let offset = if i < behind {
                    -ACTOR_SPACING_BEHIND_M * (i + 1) as f64
                } else {
                    ACTOR_SPACING_AHEAD_M * (i - behind + 1) as f64
                };
                let x = start.progress + offset;
                let personality = Personality {
                    aggressiveness: rng.uniform(),
                    sloppiness: rng.uniform(),
                };
                let actor_rng = Rng(rng.0.max(1));
                SmartActor::new(i, x, personality, rng.uniform(), actor_rng, &track)
            })
            .collect();
        Self {
            track,
            track_progress: start.progress,
            road,
            actors,
            trajectory: TrajectoryKinematics::new(vec![ego], vec![Control::default()], dt),
            diagnostics: DiagnosticsData::default(),
            last_plan_ms: 0.0,
            last_planner_actors: 0,
            ego_collision_count: 0,
            preview_ticks: DEFAULT_PREVIEW_TICKS,
            diagnostics_enabled: false,
            compute_budget: ComputeBudget::NOMINAL,
            planner_kind: planner,
            planner: planner.build(),
            simulator: Simulator::new(ego, dt),
            collision_road,
            road_anchor_x: start.progress,
        }
    }

    pub(crate) fn set_planner(&mut self, kind: PlannerKind) {
        if kind != self.planner_kind {
            self.planner_kind = kind;
            self.planner = kind.build();
            self.road = road_window(
                &self.track,
                self.road_anchor_x,
                self.ego().speed,
                self.dt(),
                kind == PlannerKind::Lattice,
            );
        }
    }

    pub(crate) fn set_actor_count(&mut self, seed: u64, actor_count: usize) {
        let actor_count = actor_count.min(MAX_ACTORS);
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
                let offset = -ACTOR_SPACING_BEHIND_M * (slot + 1) as f64;
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
            let actor_rng = Rng(rng.0.max(1));
            self.actors.push(SmartActor::new(
                next_id,
                x,
                personality,
                rng.uniform(),
                actor_rng,
                &self.track,
            ));
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
        let ego_progress = racer_progress(&self.track, self.ego(), EGO_FOOTPRINT, self.track_progress);
        let ahead = self
            .actors
            .iter()
            .filter(|actor| racer_progress(&self.track, actor.state, CAR_FOOTPRINT, actor.track_x) > ego_progress)
            .count();
        (ahead + 1, self.actors.len() + 1)
    }

    pub(crate) fn tick_recording_latency(&mut self, latency: &Latency) {
        latency.time("simulation.total", || self.tick_with_latency(Some(latency)));
    }

    fn tick_with_latency(&mut self, latency: Option<&Latency>) {
        self.track_progress = timed(latency, "simulation.progress", || {
            let progress = self.track.project_progress(self.ego().position(), self.track_progress);
            work(latency, 1);
            progress
        });
        if (self.track_progress - self.road_anchor_x).abs() >= 20.0 {
            self.road_anchor_x = (self.track_progress / 20.0).floor() * 20.0;
            self.road = timed(latency, "simulation.roads", || {
                let road = road_window(
                    &self.track,
                    self.road_anchor_x,
                    self.ego().speed,
                    self.dt(),
                    self.planner_kind == PlannerKind::Lattice,
                );
                work(latency, road.centerline().len() as u64);
                road
            });
        }

        let previous_actors: Vec<_> = self.actors.iter().map(|a| (a.id, a.state)).collect();
        let actor_count = self.actors.len() as u64;
        timed(latency, "simulation.actors", || {
            self.step_traffic();
            work(latency, actor_count);
        });
        let ego_reach =
            self.ego().speed.max(0.0) * PLANNING_HORIZON_S + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2);
        let actor_states: Vec<State> = timed(latency, "simulation.actor_culling", || {
            let states = self
                .actors
                .iter()
                .filter(|a| {
                    a.track_x <= self.track_progress + ego_reach + ACTOR_MARGIN_M
                        && a.track_x + a.state.speed * PLANNING_HORIZON_S >= self.track_progress - ACTOR_MARGIN_M
                })
                .map(|a| a.state)
                .collect();
            work(latency, actor_count);
            states
        });
        self.last_planner_actors = actor_states.len();

        let diagnostics = Diagnostics::default();
        let ego = self.ego();
        let controls = {
            let ctx = Context::new(
                &self.road,
                &actor_states,
                self.preview_ticks.max(1),
                self.compute_budget,
                latency,
                self.diagnostics_enabled.then_some(&diagnostics),
            );
            let start = Instant::now();
            let controls = match latency {
                Some(l) => l.time("planner.total", || self.planner.plan(ego, &ctx)),
                None => self.planner.plan(ego, &ctx),
            };
            self.last_plan_ms = start.elapsed().as_secs_f64() * 1e3;
            controls
        };
        self.diagnostics = diagnostics.take();

        let plan = timed(latency, "simulation.preview", || {
            let plan = self.simulator.preview(&controls, self.preview_ticks);
            work(latency, plan.len() as u64);
            plan
        });
        let plan_controls: Vec<_> = controls.into_iter().take(plan.len()).collect();
        let previous_ego = self.ego();
        timed(latency, "simulation.ego", || {
            self.simulator.step(plan_controls.first().copied().unwrap_or_default());
            work(latency, 1);
        });
        timed(latency, "simulation.collisions", || {
            self.resolve_collisions(previous_ego, &previous_actors);
            work(latency, actor_count + 1);
        });
        let states: Vec<_> = std::iter::once(self.ego()).chain(plan.into_iter().skip(1)).collect();
        let controls = if plan_controls.is_empty() {
            vec![self.actuation()]
        } else {
            plan_controls
        };
        self.trajectory = TrajectoryKinematics::new(states, controls, self.dt());
    }

    fn resolve_collisions(&mut self, previous_ego: State, previous_actors: &[(usize, State)]) {
        let mut previous = Vec::with_capacity(self.actors.len() + 1);
        previous.push(DynamicBody::new(previous_ego, crate::geometry::EGO_FOOTPRINT));
        previous.extend(self.actors.iter().map(|actor| {
            let state = previous_actors
                .iter()
                .find(|(id, _)| *id == actor.id)
                .map_or(actor.state, |(_, state)| *state);
            DynamicBody::new(state, CAR_FOOTPRINT)
        }));

        let mut bodies = Vec::with_capacity(self.actors.len() + 1);
        bodies.push(DynamicBody::new(self.simulator.state, crate::geometry::EGO_FOOTPRINT));
        bodies.extend(
            self.actors
                .iter()
                .map(|actor| DynamicBody::new(actor.state, CAR_FOOTPRINT)),
        );

        // Every moving body meets the same immovable road boundary before it
        // participates in symmetric vehicle-to-vehicle contacts.
        for (i, (before, body)) in previous.iter().zip(&mut bodies).enumerate() {
            let unconstrained = body.state;
            body.state = crate::geometry::barrier::collide_with_road_barriers(
                before.state,
                body.state,
                body.footprint,
                &self.collision_road,
            );
            if i == 0 && body.state != unconstrained {
                self.ego_collision_count += 1;
            }
        }
        let before_dynamic = bodies.clone();
        collide_dynamic_bodies(&mut bodies);
        if bodies[0].state != before_dynamic[0].state {
            self.ego_collision_count += 1;
        }
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

fn timed<T>(latency: Option<&Latency>, name: &'static str, f: impl FnOnce() -> T) -> T {
    match latency {
        Some(l) => l.time(name, f),
        None => f(),
    }
}

fn work(latency: Option<&Latency>, clocks: u64) {
    if let Some(latency) = latency {
        latency.work(clocks);
    }
}

#[cfg(test)]
mod tests;
