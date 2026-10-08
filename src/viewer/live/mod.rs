//! Bevy plumbing for the live driving demo.

use crate::planning::{ComputeBudget, Latency, LatencyStats, PlannerKind};
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
    DiagnosticPointGizmos, DiagnosticTrajectoryGizmos, PlannedTrajectoryGizmos, configure_diagnostics, configure_plan,
    prepare_road_surface, setup_carpet, setup_grid, setup_road_surface,
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
            crate::common::interp::lerp(mean, seconds, FRAME_TIME_SMOOTHING)
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

struct TrackPreparation {
    selection: (u64, PlannerKind, usize, usize),
    frame: u64,
    built: bool,
}

pub(crate) struct Live {
    pub(crate) world: LiveWorld,
    road_surface: Mesh,
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
    preparation: Option<TrackPreparation>,
}

impl Live {
    pub(crate) fn prepare_selection(&mut self, planner: PlannerKind, track: usize, actors: usize, frame: u64) -> bool {
        let selection = (self.seed, planner, track, actors);
        if self.preparation.as_ref().is_none_or(|p| p.selection != selection) {
            self.preparation = Some(TrackPreparation {
                selection,
                frame,
                built: false,
            });
            return false;
        }
        let preparation = self.preparation.as_ref().unwrap();
        // Egui may run several layout passes per frame. Show preparation
        // first, then give the surface upload an Update before Driving.
        if frame == preparation.frame {
            return false;
        }
        if !preparation.built {
            self.regenerate_with_actor_count(self.seed, planner, track, actors);
            self.preparation = Some(TrackPreparation {
                selection,
                frame,
                built: true,
            });
            return false;
        }
        self.world.is_prepared()
    }

    pub(crate) fn regenerate_with_actor_count(
        &mut self,
        seed: u64,
        planner: PlannerKind,
        track: usize,
        actor_count: usize,
    ) {
        self.preparation = None;
        self.seed = seed;
        self.world = LiveWorld::with_track(track, seed, planner, actor_count, DT);
        self.road_surface = drawing::track::surface_mesh(self.world.track.prepared().collision.polygon());
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

    fn tick(&mut self) -> bool {
        let previous = RenderSnapshot::capture(&self.world);
        if !self.world.tick_recording_latency(&self.recorder) {
            return false;
        }
        self.previous = previous;
        self.friction_box
            .record(self.previous.ego, self.world.ego(), self.world.dt());
        let progress = self
            .world
            .track
            .project_progress(self.world.ego().position(), self.world.track_progress);
        self.lap_stats
            .tick(self.world.dt(), progress, self.world.track.lap_length());
        true
    }

    fn finish_frame(&mut self) {
        self.latency.absorb(self.recorder.take());
    }
}

impl Default for Live {
    fn default() -> Self {
        let world = LiveWorld::with_track(0, 1, PlannerKind::FrenetSampling, DEFAULT_ACTORS, DT);
        let road_surface = drawing::track::surface_mesh(world.track.prepared().collision.polygon());
        let previous = RenderSnapshot::capture(&world);
        let lap_stats = LapStats::new(world.track.lap_length());
        let mut camera = CameraState::default();
        camera.reset(world.ego());
        Self {
            camera,
            world,
            road_surface,
            seed: 1,
            paused: false,
            latency: LatencyStats::default(),
            frame_rate: FrameRate::default(),
            friction_box: FrictionBox::new(FRICTION_TRAIL_HORIZON_S),
            lap_stats,
            previous,
            planner: PlannerKind::FrenetSampling,
            recorder: Latency::default(),
            acc: 0.0,
            preparation: None,
        }
    }
}

pub(crate) fn update(mut live: NonSendMut<Live>, state: Res<UiState>, time: Res<Time>) {
    live.frame_rate.observe(time.delta_secs_f64());
    live.set_planner(state.planner);
    live.world.compute_budget = ComputeBudget::from_percent(state.compute_budget_percent);
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
        if !live.tick() {
            break;
        }
        live.acc -= DT as f32;
        ticks += 1;
    }
    // Give the worker the interval between ticks, rather than starting its
    // computation only when the next step is already due.
    let Live { world, recorder, .. } = &mut *live;
    world.prepare_tick(Some(recorder));
}

#[cfg(test)]
mod tests;
