//! Bevy plumbing for the endless-track demo.

use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;

use nanoplan::planning::{Latency, LatencyStats};
use nanoplan::world::LiveWorld;
use nanoplan::{CAR_FOOTPRINT, PlannerKind};

use super::draw::{ACCENT, draw_agent, draw_car, ppx, px};
use super::{DT, UiState};

const MAX_ACTORS: usize = 12;
const MAX_TICKS_PER_FRAME: usize = 3;

pub(crate) struct Live {
    pub world: LiveWorld,
    pub seed: u64,
    pub paused: bool,
    pub zoom: f32,
    pub latency: LatencyStats,
    planner: PlannerKind,
    recorder: Latency,
    acc: f32,
}

impl Live {
    pub fn regenerate(&mut self, seed: u64, planner: PlannerKind) {
        self.seed = seed;
        self.world = LiveWorld::new(seed, planner, MAX_ACTORS, DT);
        self.planner = planner;
        self.latency = LatencyStats::default();
        self.recorder.take();
    }

    fn set_planner(&mut self, planner: PlannerKind) {
        if planner != self.planner {
            self.planner = planner;
            self.world.set_planner(planner);
            self.latency = LatencyStats::default();
            self.recorder.take();
        }
    }

    fn tick(&mut self) {
        self.world.tick_recording_latency(&self.recorder);
        self.latency.absorb(self.recorder.take());
    }
}

impl Default for Live {
    fn default() -> Self {
        Self {
            world: LiveWorld::new(1, PlannerKind::BezierIdm, MAX_ACTORS, DT),
            seed: 1,
            paused: false,
            zoom: 2.0,
            latency: LatencyStats::default(),
            planner: PlannerKind::BezierIdm,
            recorder: Latency::default(),
            acc: 0.0,
        }
    }
}

pub(crate) fn update(
    mut live: NonSendMut<Live>,
    state: Res<UiState>,
    time: Res<Time>,
    mut wheel: MessageReader<MouseWheel>,
) {
    for ev in wheel.read() {
        let steps = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y / 50.0,
        };
        live.zoom = (live.zoom * 0.9f32.powf(steps)).clamp(0.4, 8.0);
    }
    live.set_planner(state.planner);
    live.world.preview_ticks = (state.preview_s as f64 / DT).round() as usize;
    live.world.diagnostics_enabled = state.preview_s > 0.0
        && state.planner.has_diagnostics()
        && (state.show_diag_points || state.show_diag_trajectories);
    if live.paused {
        live.acc = 0.0;
        return;
    }
    live.acc = (live.acc + time.delta_secs()).min(0.3);
    let mut ticks = 0;
    while live.acc >= DT as f32 && ticks < MAX_TICKS_PER_FRAME {
        live.tick();
        live.acc -= DT as f32;
        ticks += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planner_change_resets_latency_stats() {
        let mut live = Live::default();
        live.latency.absorb(vec![("total", 1.0)]);
        live.set_planner(PlannerKind::Lattice);
        assert!(live.latency.seams.is_empty());
    }
}

pub(crate) fn draw(
    mut gizmos: Gizmos,
    state: Res<UiState>,
    live: NonSend<Live>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
    let w = &live.world;
    camera.translation = px(&w.ego).extend(camera.translation.z);
    camera.scale = Vec3::splat(live.zoom);

    let xs = ((w.ego.x - 250.0) / 5.0).floor() as i64..=((w.ego.x + 750.0) / 5.0).ceil() as i64;
    let samples: Vec<_> = xs
        .map(|i| {
            let x = i as f64 * 5.0;
            let (p, yaw) = w.track.pose(x);
            let width = w.track.half_width(x);
            (p, yaw, width)
        })
        .collect();
    for sign in [-1.0, 1.0] {
        gizmos.linestrip_2d(
            samples.iter().map(|&(p, yaw, width)| {
                ppx([
                    p[0] - sign * width * yaw.sin(),
                    p[1] + sign * width * yaw.cos(),
                ])
            }),
            Color::srgb(0.6, 0.6, 0.6),
        );
    }
    gizmos.linestrip_2d(
        samples.iter().map(|&(p, _, _)| ppx(p)),
        Color::srgb(0.25, 0.5, 0.35),
    );

    if state.show_diag_trajectories && state.planner.has_diagnostics() {
        for trajectory in &w.diagnostics.trajectories {
            gizmos.linestrip_2d(
                trajectory.iter().copied().map(ppx),
                Color::srgb(0.2, 0.85, 0.95),
            );
        }
    }
    if state.show_diag_points && state.planner.has_diagnostics() {
        for &point in &w.diagnostics.points {
            gizmos.circle_2d(
                ppx(point),
                0.3 * super::draw::PX_PER_M,
                Color::srgb(0.95, 0.85, 0.2),
            );
        }
    }

    if !w.plan.is_empty() {
        gizmos.linestrip_2d(std::iter::once(&w.ego).chain(&w.plan).map(px), ACCENT);
    }
    draw_car(&mut gizmos, &w.ego, Color::WHITE);
    for actor in &w.actors {
        draw_agent(
            &mut gizmos,
            &actor.state,
            CAR_FOOTPRINT,
            Color::srgb(0.6, 0.6, 0.6),
        );
    }
}
