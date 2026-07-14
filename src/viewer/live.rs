//! Bevy plumbing for the endless-track demo.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;

use nanoplan::planning::{Latency, LatencyStats};
use nanoplan::world::LiveWorld;
use nanoplan::{CAR_FOOTPRINT, Path, PlannerKind, State};

use super::draw::{ACCENT, draw_agent, draw_car, ppx, px};
use super::{DT, UiState};

const MAX_ACTORS: usize = 12;
const MAX_TICKS_PER_FRAME: usize = 3;
const DEFAULT_ZOOM: f32 = 0.5;
pub(crate) const MIN_ZOOM: f32 = 0.125;
pub(crate) const MAX_ZOOM: f32 = 4.0;
const CAMERA_SMOOTH_RATE: f32 = 8.0;
const MAX_ACTOR_INTERPOLATION_M: f64 = 20.0;

#[derive(Clone, Copy)]
pub(crate) struct CameraState {
    pub center: Vec2,
    pub zoom: f32,
    pub rotation: f32,
    pub follow_position: bool,
    pub follow_heading: bool,
    pub align_track: bool,
    pub smooth: bool,
}

impl CameraState {
    fn reset(&mut self, ego: Vec2) {
        *self = Self {
            center: ego,
            zoom: DEFAULT_ZOOM,
            rotation: 0.0,
            follow_position: true,
            follow_heading: false,
            align_track: false,
            smooth: true,
        };
    }
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            center: Vec2::ZERO,
            zoom: DEFAULT_ZOOM,
            rotation: 0.0,
            follow_position: true,
            follow_heading: false,
            align_track: false,
            smooth: true,
        }
    }
}

struct RenderSnapshot {
    ego: State,
    actors: Vec<(usize, State)>,
}

impl RenderSnapshot {
    fn capture(world: &LiveWorld) -> Self {
        Self {
            ego: world.ego,
            actors: world.actors.iter().map(|a| (a.id, a.state)).collect(),
        }
    }
}

pub(crate) struct Live {
    pub world: LiveWorld,
    pub seed: u64,
    pub paused: bool,
    pub camera: CameraState,
    pub latency: LatencyStats,
    previous: RenderSnapshot,
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
        self.acc = 0.0;
        self.reset_render_history();
        self.reset_camera();
    }

    pub fn reset_camera(&mut self) {
        self.camera.reset(px(&self.world.ego));
    }

    pub fn toggle_pause(&mut self) {
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
        self.latency.absorb(self.recorder.take());
    }
}

impl Default for Live {
    fn default() -> Self {
        let world = LiveWorld::new(1, PlannerKind::BezierIdm, MAX_ACTORS, DT);
        let previous = RenderSnapshot::capture(&world);
        Self {
            camera: CameraState {
                center: px(&world.ego),
                ..Default::default()
            },
            world,
            seed: 1,
            paused: false,
            latency: LatencyStats::default(),
            previous,
            planner: PlannerKind::BezierIdm,
            recorder: Latency::default(),
            acc: 0.0,
        }
    }
}

pub(crate) fn camera_input(
    mut live: NonSendMut<Live>,
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    egui_input: Res<EguiWantsInput>,
    time: Res<Time>,
) {
    if !egui_input.wants_any_pointer_input() {
        let scroll_steps = match scroll.unit {
            MouseScrollUnit::Line => scroll.delta.y,
            MouseScrollUnit::Pixel => scroll.delta.y / 50.0,
        };
        live.camera.zoom = (live.camera.zoom * 1.1f32.powf(scroll_steps)).clamp(MIN_ZOOM, MAX_ZOOM);

        if mouse.pressed(MouseButton::Middle) && motion.delta != Vec2::ZERO {
            let drag = Rot2::radians(live.camera.rotation)
                * Vec2::new(motion.delta.x, -motion.delta.y)
                / live.camera.zoom;
            live.camera.center -= drag;
            live.camera.follow_position = false;
            live.camera.align_track = false;
        }
        if mouse.pressed(MouseButton::Right) && motion.delta.x != 0.0 {
            live.camera.rotation += motion.delta.x * 0.005;
            live.camera.follow_heading = false;
            live.camera.align_track = false;
        }
    }

    if egui_input.wants_any_keyboard_input() {
        return;
    }
    if keys.just_pressed(KeyCode::KeyF) {
        live.camera.follow_position = !live.camera.follow_position;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        live.reset_camera();
    }

    let mut pan = Vec2::ZERO;
    for (key, direction) in [
        (KeyCode::KeyA, -Vec2::X),
        (KeyCode::ArrowLeft, -Vec2::X),
        (KeyCode::KeyD, Vec2::X),
        (KeyCode::ArrowRight, Vec2::X),
        (KeyCode::KeyW, Vec2::Y),
        (KeyCode::ArrowUp, Vec2::Y),
        (KeyCode::KeyS, -Vec2::Y),
        (KeyCode::ArrowDown, -Vec2::Y),
    ] {
        if keys.pressed(key) {
            pan += direction;
        }
    }
    if pan != Vec2::ZERO {
        let camera = live.camera;
        live.camera.center +=
            Rot2::radians(camera.rotation) * pan.normalize() * 500.0 * time.delta_secs()
                / camera.zoom;
        live.camera.follow_position = false;
        live.camera.align_track = false;
    }
    let rotation_input = keys.pressed(KeyCode::KeyE) as i8 - keys.pressed(KeyCode::KeyQ) as i8;
    if rotation_input != 0 {
        live.camera.rotation += rotation_input as f32 * time.delta_secs();
        live.camera.follow_heading = false;
        live.camera.align_track = false;
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
mod tests {
    use super::*;

    #[test]
    fn planner_change_resets_latency_stats() {
        let mut live = Live::default();
        live.latency.absorb(vec![("total", 1.0)]);
        live.set_planner(PlannerKind::Lattice);
        assert!(live.latency.seams.is_empty());
    }

    #[test]
    fn camera_reset_restores_the_following_north_up_view() {
        let mut camera = CameraState {
            center: Vec2::splat(99.0),
            zoom: 2.0,
            rotation: 1.0,
            follow_position: false,
            follow_heading: true,
            align_track: true,
            smooth: false,
        };

        camera.reset(Vec2::new(3.0, 4.0));

        assert_eq!(camera.center, Vec2::new(3.0, 4.0));
        assert_eq!(camera.zoom, DEFAULT_ZOOM);
        assert_eq!(camera.rotation, 0.0);
        assert!(camera.follow_position);
        assert!(!camera.follow_heading);
        assert!(!camera.align_track);
        assert!(camera.smooth);
    }

    #[test]
    fn camera_smoothing_takes_the_short_way_across_pi() {
        let almost_pi = std::f32::consts::PI - 0.1;
        let almost_negative_pi = -std::f32::consts::PI + 0.1;
        assert!(smooth_angle(almost_pi, almost_negative_pi, 0.5) > almost_pi);
    }

    #[test]
    fn render_interpolation_blends_pose_and_wraps_yaw() {
        let previous = State {
            x: 2.0,
            yaw: std::f64::consts::PI - 0.2,
            speed: 4.0,
            ..Default::default()
        };
        let current = State {
            x: 6.0,
            y: 2.0,
            yaw: -std::f64::consts::PI + 0.2,
            speed: 8.0,
        };

        let rendered = interpolate_state(previous, current, 0.5);

        assert_eq!(rendered.x, 4.0);
        assert_eq!(rendered.y, 1.0);
        assert_eq!(rendered.speed, 6.0);
        assert!((rendered.yaw - std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn new_track_starts_with_ego_aligned_to_its_tangent() {
        let mut live = Live::default();
        live.acc = DT as f32 * 0.9;

        live.regenerate(2, PlannerKind::BezierIdm);

        let (_, track_yaw) = live.world.track.pose(live.world.ego.x);
        assert!((live.world.ego.yaw - track_yaw).abs() < 1e-12);
        assert_eq!(live.previous.ego.yaw, track_yaw);
        assert_eq!(live.acc, 0.0);
    }
}

pub(crate) fn draw(
    mut gizmos: Gizmos,
    state: Res<UiState>,
    mut live: NonSendMut<Live>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
    window: Single<&Window>,
    time: Res<Time>,
) {
    // Standard fixed-step interpolation. Rendering stays one simulation tick
    // behind so it can blend completed states without predicting physics.
    let render_alpha = if live.paused {
        1.0
    } else {
        (live.acc as f64 / DT).clamp(0.0, 1.0)
    };
    let ego = interpolate_state(live.previous.ego, live.world.ego, render_alpha);
    let (target_center, target_rotation) = if live.camera.align_track {
        let path = Path::new(&live.world.road.centerline);
        let (s, _) = path.project(ego);
        let (position, yaw) = path.pose_at(s);
        (
            Some(ppx(position)),
            Some(yaw as f32 - std::f32::consts::FRAC_PI_2),
        )
    } else {
        (
            live.camera.follow_position.then(|| px(&ego)),
            live.camera
                .follow_heading
                .then(|| ego.yaw as f32 - std::f32::consts::FRAC_PI_2),
        )
    };
    let blend = if live.camera.smooth {
        1.0 - (-CAMERA_SMOOTH_RATE * time.delta_secs()).exp()
    } else {
        1.0
    };
    if let Some(target) = target_center {
        live.camera.center = live.camera.center.lerp(target, blend);
    }
    if let Some(target) = target_rotation {
        live.camera.rotation = smooth_angle(live.camera.rotation, target, blend);
    }
    camera.translation = live.camera.center.extend(camera.translation.z);
    camera.rotation = Quat::from_rotation_z(live.camera.rotation);
    camera.scale = Vec3::splat(1.0 / live.camera.zoom);

    if state.show_grid {
        draw_grid(&mut gizmos, live.camera, &window);
    }

    let w = &live.world;

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
    for &(p, yaw, width) in &samples {
        let offset = [width * yaw.sin(), -width * yaw.cos()];
        gizmos.line_2d(
            ppx([p[0] - offset[0], p[1] - offset[1]]),
            ppx([p[0] + offset[0], p[1] + offset[1]]),
            Color::srgba(0.6, 0.6, 0.6, 0.2),
        );
    }

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

    if state.show_plan && !w.plan.is_empty() {
        gizmos.linestrip_2d(std::iter::once(&ego).chain(&w.plan).map(px), ACCENT);
    }
    draw_car(&mut gizmos, &ego, Color::WHITE);
    for actor in &w.actors {
        let state = live
            .previous
            .actors
            .iter()
            .find(|(id, _)| *id == actor.id)
            .map(|(_, previous)| {
                if (actor.state.x - previous.x).hypot(actor.state.y - previous.y)
                    > MAX_ACTOR_INTERPOLATION_M
                {
                    actor.state
                } else {
                    interpolate_state(*previous, actor.state, render_alpha)
                }
            })
            .unwrap_or(actor.state);
        draw_agent(
            &mut gizmos,
            &state,
            CAR_FOOTPRINT,
            Color::srgb(0.6, 0.6, 0.6),
        );
    }
}

fn interpolate_state(previous: State, current: State, alpha: f64) -> State {
    let yaw_delta = (current.yaw - previous.yaw + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    State {
        x: previous.x + (current.x - previous.x) * alpha,
        y: previous.y + (current.y - previous.y) * alpha,
        yaw: previous.yaw + yaw_delta * alpha,
        speed: previous.speed + (current.speed - previous.speed) * alpha,
    }
}

fn smooth_angle(current: f32, target: f32, blend: f32) -> f32 {
    let delta = (target - current + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    current + delta * blend
}

fn draw_grid(gizmos: &mut Gizmos, camera: CameraState, window: &Window) {
    let extent = window.width().hypot(window.height()) / camera.zoom;
    let wide_grid = camera.zoom < 0.25;
    let spacing = if wide_grid { 50.0 } else { 10.0 } * super::draw::PX_PER_M;
    let major_every = if wide_grid { 2 } else { 5 };
    let min_x = ((camera.center.x - extent) / spacing).floor() as i64;
    let max_x = ((camera.center.x + extent) / spacing).ceil() as i64;
    let min_y = ((camera.center.y - extent) / spacing).floor() as i64;
    let max_y = ((camera.center.y + extent) / spacing).ceil() as i64;
    for x in min_x..=max_x {
        let color = if x.rem_euclid(major_every) == 0 {
            Color::srgba(0.2, 0.48, 0.68, 0.11)
        } else {
            Color::srgba(0.2, 0.4, 0.55, 0.035)
        };
        let x = x as f32 * spacing;
        gizmos.line_2d(
            Vec2::new(x, camera.center.y - extent),
            Vec2::new(x, camera.center.y + extent),
            color,
        );
    }
    for y in min_y..=max_y {
        let color = if y.rem_euclid(major_every) == 0 {
            Color::srgba(0.2, 0.48, 0.68, 0.11)
        } else {
            Color::srgba(0.2, 0.4, 0.55, 0.035)
        };
        let y = y as f32 * spacing;
        gizmos.line_2d(
            Vec2::new(camera.center.x - extent, y),
            Vec2::new(camera.center.x + extent, y),
            color,
        );
    }
}
