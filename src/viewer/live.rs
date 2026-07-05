//! "Open world" mode: a realtime closed loop, judo/treetop style. The
//! planner and simulator run continuously ([`LiveWorld::tick`] at a fixed
//! rate, budgeted per frame), the user clicks anywhere on the procedural
//! street map to place a goal, and the ego routes there through the
//! traffic. All world logic lives in `nanoplan::world`; this module is the
//! Bevy plumbing: pacing, mouse input, camera, and gizmo drawing.

use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use nanoplan::PlannerKind;
use nanoplan::world::{LANE_OFFSET_M, LiveWorld, ROAD_HALF_WIDTH_M};

use super::draw::{ACCENT, PX_PER_M, draw_car, ppx, px};
use super::{DT, Mode, UiState};

const N_ACTORS: usize = 12;

/// Ticks of realtime allowed per rendered frame: enough to catch up from a
/// hitch, few enough that a slow planner lags gracefully instead of
/// freezing the UI trying to keep up.
const MAX_TICKS_PER_FRAME: usize = 3;

/// The open-world session: the `LiveWorld` plus viewer-side pacing and
/// camera state. `NonSend` because `LiveWorld` holds a `Box<dyn Planner>`.
pub(crate) struct Live {
    pub world: LiveWorld,
    pub seed: u64,
    pub paused: bool,
    pub zoom: f32,
    /// Realtime accumulator: whole `DT`s of it are consumed by ticks.
    acc: f32,
}

impl Live {
    pub fn regenerate(&mut self, seed: u64, planner: PlannerKind) {
        self.seed = seed;
        self.world = LiveWorld::new(seed, planner, N_ACTORS, DT);
    }
}

impl Default for Live {
    fn default() -> Self {
        Live {
            world: LiveWorld::new(1, PlannerKind::BezierIdm, N_ACTORS, DT),
            seed: 1,
            paused: false,
            zoom: 2.0,
            acc: 0.0,
        }
    }
}

/// Handle input and advance the world in realtime.
pub(crate) fn live_update(
    mut live: NonSendMut<Live>,
    state: Res<UiState>,
    time: Res<Time>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut wheel: MessageReader<MouseWheel>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<Camera2d>>,
) {
    if state.mode != Mode::Live {
        wheel.clear();
        return;
    }
    for ev in wheel.read() {
        let steps = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y / 50.0,
        };
        live.zoom = (live.zoom * 0.9f32.powf(steps)).clamp(0.4, 8.0);
    }
    live.world.set_planner(state.live_planner);
    live.world.target_speed = state.live_target_speed as f64;
    // click on the map (not the control panel) → place the goal
    if buttons.just_pressed(MouseButton::Left)
        && !state.pointer_over_ui
        && let Some(cursor) = window.cursor_position()
        && let Ok(hit) = camera.0.viewport_to_world_2d(camera.1, cursor)
    {
        let m = hit / PX_PER_M;
        live.world.set_goal([m.x as f64, m.y as f64]);
    }
    if live.paused {
        live.acc = 0.0;
        return;
    }
    live.acc = (live.acc + time.delta_secs()).min(0.3);
    let mut ticks = 0;
    while live.acc >= DT as f32 && ticks < MAX_TICKS_PER_FRAME {
        live.world.tick();
        live.acc -= DT as f32;
        ticks += 1;
    }
}

/// Draw the street map, the route and goal, the live plan preview, and
/// every vehicle; camera follows the ego at the current zoom.
pub(crate) fn draw_live(
    mut gizmos: Gizmos,
    state: Res<UiState>,
    live: NonSend<Live>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
    if state.mode != Mode::Live {
        return;
    }
    let w = &live.world;
    camera.translation = px(&w.ego).extend(camera.translation.z);
    camera.scale = Vec3::splat(live.zoom);

    let boundary = Color::srgb(0.55, 0.55, 0.55);
    let divider = Color::srgb(0.65, 0.55, 0.2);
    for &[a, b] in &w.map.edges {
        let (na, nb) = (w.map.nodes[a], w.map.nodes[b]);
        let len = (nb[0] - na[0]).hypot(nb[1] - na[1]).max(1e-9);
        let dir = [(nb[0] - na[0]) / len, (nb[1] - na[1]) / len];
        let right = [dir[1], -dir[0]];
        // hold the lines back from the intersections so they read as open
        // junctions instead of lines crossing through them
        let trim = (ROAD_HALF_WIDTH_M + 2.0).min(0.45 * len);
        let at = |s: f64, d: f64| {
            ppx([
                na[0] + dir[0] * s + right[0] * d,
                na[1] + dir[1] * s + right[1] * d,
            ])
        };
        for d in [-ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M] {
            gizmos.line_2d(at(trim, d), at(len - trim, d), boundary);
        }
        // dashed two-way divider down the road axis: 3 m dash, 3 m gap
        let mut s = trim;
        while s + 3.0 <= len - trim {
            gizmos.line_2d(at(s, 0.0), at(s + 3.0, 0.0), divider);
            s += 6.0;
        }
    }

    if let Some(road) = &w.road {
        gizmos.linestrip_2d(
            road.centerline.iter().map(|&p| ppx(p)),
            Color::srgb(0.25, 0.5, 0.35),
        );
    }
    if let Some(goal) = w.goal {
        let green = Color::srgb(0.25, 0.8, 0.45);
        gizmos.circle_2d(ppx(goal), 2.0 * PX_PER_M, green);
        gizmos.circle_2d(ppx(goal), 0.5 * PX_PER_M, green);
    }
    // the live plan, replanned this tick — the realtime analogue of the
    // scrub mode's future preview
    if !w.plan.is_empty() {
        gizmos.linestrip_2d(std::iter::once(&w.ego).chain(&w.plan).map(px), ACCENT);
        draw_car(&mut gizmos, w.plan.last().unwrap(), ACCENT.with_alpha(0.6));
    }
    draw_car(&mut gizmos, &w.ego, Color::WHITE);
    for actor in &w.actors {
        draw_car(&mut gizmos, &actor.state, Color::srgb(0.6, 0.6, 0.6));
    }
    // parked-and-waiting hint: a faint ring around the ego when goalless
    if w.goal.is_none() {
        gizmos.circle_2d(
            px(&w.ego),
            (LANE_OFFSET_M * 2.0) as f32 * PX_PER_M,
            Color::srgba(1.0, 1.0, 1.0, 0.2),
        );
    }
}
