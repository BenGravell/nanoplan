use crate::common::rng::Rng;
use crate::geometry::CAR_FOOTPRINT;
use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::simulation::{Position, State};
use crate::track::Track;
use crate::vehicle::{MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL};

pub(super) const MAX_ACTORS: usize = 15;
pub(super) const ACTOR_MARGIN_M: f64 = 25.0;
pub(super) const ACTOR_SPACING_BEHIND_M: f64 = 45.0;
pub(super) const ACTOR_SPACING_AHEAD_M: f64 = 55.0;

const MIN_INITIAL_SPEED_MPS: f64 = 5.0;
const INITIAL_SPEED_RANGE_MPS: f64 = 4.0;
const MIN_WANDER_DISTANCE_M: f64 = 15.0;
const WANDER_DISTANCE_RANGE_M: f64 = 25.0;
const MAX_LATERAL_SPEED_MPS: f64 = 0.35;
const ROAD_EDGE_CLEARANCE_M: f64 = 0.3;
const TIMID_LATERAL_BIAS: f64 = 0.65;
const SLOPPY_LATERAL_RANGE: f64 = 0.55;

/// A car following the same single track as the ego.
pub(crate) struct SmartActor {
    pub(crate) id: usize,
    pub(crate) state: State,
    pub(crate) personality: Personality,
    pub(super) track_x: f64,
    pub(super) lateral: f64,
    pub(super) lateral_target: f64,
    pub(super) next_wander_x: f64,
    pub(super) rng: Rng,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Personality {
    pub(crate) aggressiveness: f64,
    pub(crate) sloppiness: f64,
}

impl SmartActor {
    pub(super) fn new(
        id: usize,
        track_x: f64,
        personality: Personality,
        initial_speed_random: f64,
        mut rng: Rng,
        track: &Track,
    ) -> Self {
        let lateral = lateral_target(personality, track.half_width(track_x), rng.uniform());
        let (position, yaw) = track.pose(track_x);
        let left = Position::from_angle(yaw + std::f64::consts::FRAC_PI_2);
        let state = State::from((
            Position::new(position.x + lateral * left.x, position.y + lateral * left.y),
            yaw,
            MIN_INITIAL_SPEED_MPS + INITIAL_SPEED_RANGE_MPS * initial_speed_random,
        ));
        let next_wander_x = next_wander_x(track_x, rng.uniform());

        Self {
            id,
            state,
            personality,
            track_x,
            lateral,
            lateral_target: lateral,
            next_wander_x,
            rng,
        }
    }
}

impl super::LiveWorld {
    pub(super) fn step_traffic(&mut self) {
        let dt = self.dt();
        for actor in &mut self.actors {
            actor.track_x = self.track.project_progress(actor.state.position(), actor.track_x);
            let (p, lane_yaw) = self.track.pose(actor.track_x);
            let left = Position::from_angle(lane_yaw + std::f64::consts::FRAC_PI_2);
            actor.lateral = (actor.state.position().x - p.x) * left.x + (actor.state.position().y - p.y) * left.y;
        }
        self.actors.sort_by(|a, b| a.track_x.total_cmp(&b.track_x));
        let snapshot: Vec<(f64, f64)> = self
            .actors
            .iter()
            .map(|a| {
                let (_, lane_yaw) = self.track.pose(a.track_x);
                let forward_speed = a.state.speed * (a.state.pose.yaw - lane_yaw).cos();
                (a.track_x, forward_speed)
            })
            .collect();
        for (i, actor) in self.actors.iter_mut().enumerate() {
            let (_, lane_yaw) = self.track.pose(actor.track_x);
            let mut forward_speed = actor.state.speed * (actor.state.pose.yaw - lane_yaw).cos();
            let mut lateral_speed = actor.state.speed * (actor.state.pose.yaw - lane_yaw).sin();
            let lead = snapshot
                .get(i + 1)
                .map(|next| (next.0 - actor.track_x - CAR_FOOTPRINT.length, next.1));
            let accel = lead.map_or(MAX_LON_ACCEL, |(gap, lead_speed)| {
                ((lead_speed * lead_speed - forward_speed * forward_speed) / (2.0 * gap.max(1.0)))
                    .clamp(crate::vehicle::MIN_LON_ACCEL, MAX_LON_ACCEL)
            });
            forward_speed = (forward_speed + accel * dt).clamp(-*MAX_TERMINAL_SPEED_MPS, *MAX_TERMINAL_SPEED_MPS);
            actor.track_x += forward_speed * dt;
            if actor.track_x >= actor.next_wander_x {
                actor.lateral_target = lateral_target(
                    actor.personality,
                    self.track.half_width(actor.track_x),
                    actor.rng.uniform(),
                );
                actor.next_wander_x = next_wander_x(actor.track_x, actor.rng.uniform());
            }
            let desired_lateral_speed =
                (actor.lateral_target - actor.lateral).clamp(-MAX_LATERAL_SPEED_MPS, MAX_LATERAL_SPEED_MPS);
            lateral_speed +=
                (desired_lateral_speed - lateral_speed).clamp(-MAX_ABS_LAT_ACCEL * dt, MAX_ABS_LAT_ACCEL * dt);
            actor.lateral += lateral_speed * dt;
            let (p, lane_yaw) = self.track.pose(actor.track_x);
            let forward = Position::from_angle(lane_yaw);
            let left = Position::new(-forward.y, forward.x);
            let velocity = [
                forward_speed * forward.x + lateral_speed * left.x,
                forward_speed * forward.y + lateral_speed * left.y,
            ];
            let speed = velocity[0].hypot(velocity[1]);
            let yaw = if speed > 1e-9 {
                velocity[1].atan2(velocity[0])
            } else {
                lane_yaw
            };
            actor.state = State::from((
                Position::new(p.x + actor.lateral * left.x, p.y + actor.lateral * left.y),
                yaw,
                speed,
            ));
        }
    }
}

pub(super) fn lateral_target(personality: Personality, half_width: f64, random: f64) -> f64 {
    let room = (half_width - CAR_FOOTPRINT.width / 2.0 - ROAD_EDGE_CLEARANCE_M).max(0.0);
    let timid_bias = -TIMID_LATERAL_BIAS * (1.0 - personality.aggressiveness).powi(2) * room;
    (timid_bias + (2.0 * random - 1.0) * SLOPPY_LATERAL_RANGE * personality.sloppiness * room).clamp(-room, room)
}

fn next_wander_x(track_x: f64, random: f64) -> f64 {
    track_x + MIN_WANDER_DISTANCE_M + WANDER_DISTANCE_RANGE_M * random
}
