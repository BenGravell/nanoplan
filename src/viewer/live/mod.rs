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

pub(crate) use camera::{CameraState, MAX_ZOOM, MIN_ZOOM, camera_input};
pub(crate) use drawing::{
    DiagnosticPointGizmos, DiagnosticTrajectoryGizmos, PlannedTrajectoryGizmos,
    configure_diagnostics, configure_plan, setup_carpet, setup_grid, setup_road_surface,
};
use rendering::RenderSnapshot;
pub(crate) use rendering::draw;

const DEFAULT_ACTORS: usize = 5;
const MAX_TICKS_PER_FRAME: usize = 3;
const FRICTION_TRAIL_HORIZON_S: f64 = 4.0;
const FRAME_TIME_SMOOTHING: f64 = 0.1;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FrameRate {
    mean_seconds: Option<f64>,
}

impl FrameRate {
    fn observe(&mut self, seconds: f64) {
        if !seconds.is_finite() || seconds <= 0.0 {
            return;
        }
        self.mean_seconds = Some(self.mean_seconds.map_or(seconds, |mean| {
            mean + FRAME_TIME_SMOOTHING * (seconds - mean)
        }));
    }

    pub(crate) fn fps(self) -> f64 {
        self.mean_seconds.map_or(0.0, |seconds| 1.0 / seconds)
    }

    pub(crate) fn milliseconds(self) -> f64 {
        self.mean_seconds.unwrap_or(0.0) * 1e3
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct LapStats {
    pub(crate) current_s: f64,
    pub(crate) previous_s: Option<f64>,
    pub(crate) best_s: Option<f64>,
    pub(crate) completed: u64,
    next_finish_m: f64,
}

impl LapStats {
    fn new(lap_length: Option<f64>) -> Self {
        Self {
            next_finish_m: lap_length.unwrap_or(f64::INFINITY),
            ..Default::default()
        }
    }

    fn tick(&mut self, dt: f64, progress: f64, lap_length: Option<f64>) {
        self.current_s += dt;
        let Some(lap_length) = lap_length.filter(|length| *length > 0.0) else {
            return;
        };
        if !self.next_finish_m.is_finite() {
            self.next_finish_m = lap_length;
        }
        while progress >= self.next_finish_m {
            let lap_time = self.current_s;
            self.previous_s = Some(lap_time);
            self.best_s = Some(self.best_s.map_or(lap_time, |best| best.min(lap_time)));
            self.completed += 1;
            self.current_s = 0.0;
            self.next_finish_m += lap_length;
        }
    }
}

pub(crate) struct Live {
    pub(crate) world: LiveWorld,
    pub(crate) seed: u64,
    pub(crate) paused: bool,
    pub(crate) camera: CameraState,
    pub(crate) latency: LatencyStats,
    pub(crate) frame_rate: FrameRate,
    pub(crate) friction_box: FrictionBox,
    pub(crate) lap_stats: LapStats,
    previous: RenderSnapshot,
    planner: PlannerKind,
    recorder: Latency,
    acc: f32,
}

impl Live {
    pub(crate) fn regenerate_with_actor_count(
        &mut self,
        seed: u64,
        planner: PlannerKind,
        track: usize,
        actor_count: usize,
    ) {
        self.seed = seed;
        self.world = LiveWorld::with_track(track, seed, planner, actor_count, DT);
        self.planner = planner;
        self.latency = LatencyStats::default();
        self.recorder.take();
        self.acc = 0.0;
        self.friction_box.clear();
        self.lap_stats = LapStats::new(self.world.track.lap_length());
        self.reset_render_history();
        self.reset_camera();
    }

    pub(crate) fn reset_camera(&mut self) {
        self.camera.reset(self.world.ego());
    }

    pub(crate) fn set_actor_count(&mut self, actor_count: usize) {
        self.world.set_actor_count(self.seed, actor_count);
        self.reset_render_history();
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
        let progress = self.world.track.project_progress(
            [self.world.ego().x, self.world.ego().y],
            self.world.track_progress,
        );
        self.lap_stats
            .tick(self.world.dt(), progress, self.world.track.lap_length());
    }

    fn finish_frame(&mut self) {
        self.latency.absorb(self.recorder.take());
    }
}

impl Default for Live {
    fn default() -> Self {
        #[cfg(test)]
        crate::track::loader::install_test_catalog();
        let world = LiveWorld::with_track(0, 1, PlannerKind::Basic, DEFAULT_ACTORS, DT);
        let previous = RenderSnapshot::capture(&world);
        let lap_stats = LapStats::new(world.track.lap_length());
        let mut camera = CameraState::default();
        camera.reset(world.ego());
        Self {
            camera,
            world,
            seed: 1,
            paused: false,
            latency: LatencyStats::default(),
            frame_rate: FrameRate::default(),
            friction_box: FrictionBox::new(FRICTION_TRAIL_HORIZON_S),
            lap_stats,
            previous,
            planner: PlannerKind::Basic,
            recorder: Latency::default(),
            acc: 0.0,
        }
    }
}

pub(crate) fn update(mut live: NonSendMut<Live>, state: Res<UiState>, time: Res<Time>) {
    live.frame_rate.observe(time.delta_secs_f64());
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
