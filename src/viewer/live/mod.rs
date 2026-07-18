//! Bevy plumbing for the live driving demo.

use crate::planning::{Latency, LatencyStats, PlannerKind};
use crate::world::LiveWorld;
use bevy::prelude::*;

use super::{DT, UiState};
use crate::viewer::ui::FrictionBox;

mod camera;
mod drawing;
mod rendering;
mod screen;

pub(crate) use camera::{CameraState, CameraTarget, MAX_ZOOM, MIN_ZOOM, camera_input};
pub(crate) use drawing::{
    DiagnosticPointGizmos, DiagnosticTrajectoryGizmos, EgoCarpetGizmos, PlannedTrajectoryGizmos,
    configure_carpet, configure_diagnostics, configure_plan,
};
use rendering::RenderSnapshot;
pub(crate) use rendering::draw;
use screen::px;

const MAX_ACTORS: usize = 12;
const MAX_TICKS_PER_FRAME: usize = 3;
const FRICTION_TRAIL_HORIZON_S: f64 = 4.0;

pub(crate) struct Live {
    pub(crate) world: LiveWorld,
    pub(crate) seed: u64,
    pub(crate) paused: bool,
    pub(crate) camera: CameraState,
    pub(crate) latency: LatencyStats,
    pub(crate) friction_box: FrictionBox,
    previous: RenderSnapshot,
    planner: PlannerKind,
    recorder: Latency,
    acc: f32,
}

impl Live {
    pub(crate) fn regenerate(&mut self, seed: u64, planner: PlannerKind, track: usize) {
        self.seed = seed;
        self.world = LiveWorld::with_track(track, seed, planner, MAX_ACTORS, DT);
        self.planner = planner;
        self.latency = LatencyStats::default();
        self.recorder.take();
        self.acc = 0.0;
        self.friction_box.clear();
        self.reset_render_history();
        self.reset_camera();
    }

    pub(crate) fn reset_camera(&mut self) {
        self.camera.reset(px(&self.world.ego()));
    }

    pub(crate) fn toggle_pause(&mut self) {
        self.paused = !self.paused;
        self.reset_render_history();
    }

    fn reset_render_history(&mut self) {
        self.previous = RenderSnapshot::capture(&self.world);
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
        self.previous = RenderSnapshot::capture(&self.world);
        self.world.tick_recording_latency(&self.recorder);
        self.friction_box
            .record(self.previous.ego, self.world.ego(), self.world.dt());
        self.latency.absorb(self.recorder.take());
    }
}

impl Default for Live {
    fn default() -> Self {
        #[cfg(test)]
        crate::track::loader::install_test_catalog();
        let world = LiveWorld::with_track(0, 1, PlannerKind::Basic, MAX_ACTORS, DT);
        let previous = RenderSnapshot::capture(&world);
        Self {
            camera: CameraState {
                center: px(&world.ego()),
                ..Default::default()
            },
            world,
            seed: 1,
            paused: false,
            latency: LatencyStats::default(),
            friction_box: FrictionBox::new(FRICTION_TRAIL_HORIZON_S),
            previous,
            planner: PlannerKind::Basic,
            recorder: Latency::default(),
            acc: 0.0,
        }
    }
}

pub(crate) fn update(mut live: NonSendMut<Live>, state: Res<UiState>, time: Res<Time>) {
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
mod tests;
