//! "Open world" mode: a realtime closed loop, judo/treetop style. The
//! planner and simulator run continuously ([`LiveWorld::tick`] at a fixed
//! rate, budgeted per frame), the user clicks anywhere on the procedural
//! street map to place a goal, and the ego routes there through the
//! traffic. All world logic lives in `nanoplan::world`; this module is the
//! Bevy plumbing: pacing, mouse input, camera, and gizmo drawing.

use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use std::collections::HashMap;

use nanoplan::PlannerKind;
use nanoplan::world::{
    ActorKind, CROSSWALK_SETBACK_M, LANE_W_M, LiveWorld, POCKET_M, POCKET_TAPER_M, has_crosswalk,
    has_pocket, has_slip,
};

use super::draw::{ACCENT, PX_PER_M, draw_agent, draw_car, ppx, px};
use super::{DT, Mode, UiState};

/// Global cap on live traffic; the per-chunk spawner fills up to this.
const MAX_ACTORS: usize = 64;

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
        self.world = LiveWorld::new(seed, planner, MAX_ACTORS, DT);
    }
}

impl Default for Live {
    fn default() -> Self {
        Live {
            world: LiveWorld::new(1, PlannerKind::BezierIdm, MAX_ACTORS, DT),
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
    let lane_line = Color::srgb(0.45, 0.45, 0.45);
    // widest road entering each junction, for trimming the others back
    let mut node_w = vec![LANE_W_M; w.map.nodes.len()];
    for (ei, &[a, b]) in w.map.edges.iter().enumerate() {
        node_w[a] = node_w[a].max(w.map.half_width(ei));
        node_w[b] = node_w[b].max(w.map.half_width(ei));
    }
    for (ei, &[a, b]) in w.map.edges.iter().enumerate() {
        let (na, nb) = (w.map.nodes[a], w.map.nodes[b]);
        let len = (nb[0] - na[0]).hypot(nb[1] - na[1]).max(1e-9);
        let dir = [(nb[0] - na[0]) / len, (nb[1] - na[1]) / len];
        let right = [dir[1], -dir[0]];
        let hw = w.map.half_width(ei);
        // hold the lines back from the intersections so they read as open
        // junctions instead of lines crossing through them
        let (ta, tb) = (
            (node_w[a] + 2.0).min(0.45 * len),
            (node_w[b] + 2.0).min(0.45 * len),
        );
        let at = |s: f64, d: f64| {
            ppx([
                na[0] + dir[0] * s + right[0] * d,
                na[1] + dir[1] * s + right[1] * d,
            ])
        };
        // junction furniture on this street: turn pockets and crosswalks
        // per approach (a→b traffic rides d > 0 and approaches the b end)
        let (ca, cb) = (w.map.coords[a], w.map.coords[b]);
        let d_ab = [cb[0] - ca[0], cb[1] - ca[1]];
        let d_ba = [-d_ab[0], -d_ab[1]];
        let long = len > 45.0;
        let pocket_b = long && has_pocket(w.map.seed, cb, d_ab);
        let pocket_a = long && has_pocket(w.map.seed, ca, d_ba);
        // boundaries, stopping where a pocket flare takes over
        let zb = len - tb - POCKET_M;
        let za = ta + POCKET_M;
        gizmos.line_2d(
            at(ta, hw),
            at(if pocket_b { zb } else { len - tb }, hw),
            boundary,
        );
        gizmos.line_2d(
            at(if pocket_a { za } else { ta }, -hw),
            at(len - tb, -hw),
            boundary,
        );
        if pocket_b {
            // approach flares a lane wider; the freed inner lane is the pocket
            gizmos.line_2d(at(zb, hw), at(zb + POCKET_TAPER_M, hw + LANE_W_M), boundary);
            gizmos.line_2d(
                at(zb + POCKET_TAPER_M, hw + LANE_W_M),
                at(len - tb, hw + LANE_W_M),
                boundary,
            );
        }
        if pocket_a {
            gizmos.line_2d(
                at(za, -hw),
                at(za - POCKET_TAPER_M, -hw - LANE_W_M),
                boundary,
            );
            gizmos.line_2d(
                at(za - POCKET_TAPER_M, -hw - LANE_W_M),
                at(ta, -hw - LANE_W_M),
                boundary,
            );
        }
        // dashed lines: two-way divider down the axis, lane lines between
        // same-direction lanes, the old boundary as the pocket's lane line
        let mut dash = |d: f64, s0: f64, s1: f64, color: Color| {
            let mut s = s0;
            while s + 3.0 <= s1 {
                gizmos.line_2d(at(s, d), at(s + 3.0, d), color);
                s += 6.0;
            }
        };
        dash(0.0, ta, len - tb, divider);
        for lane in 1..w.map.lanes[ei] {
            dash(lane as f64 * LANE_W_M, ta, len - tb, lane_line);
            dash(-(lane as f64) * LANE_W_M, ta, len - tb, lane_line);
        }
        if pocket_b {
            dash(hw, zb + POCKET_TAPER_M, len - tb, lane_line);
        }
        if pocket_a {
            dash(-hw, ta, za - POCKET_TAPER_M, lane_line);
        }
        // crosswalk stripes, spanning the (possibly flared) roadway
        for (cross, s_c, near_b) in [
            (
                long && has_crosswalk(w.map.seed, cb, d_ab),
                len - CROSSWALK_SETBACK_M,
                true,
            ),
            (
                long && has_crosswalk(w.map.seed, ca, d_ba),
                CROSSWALK_SETBACK_M,
                false,
            ),
        ] {
            if !cross {
                continue;
            }
            let flare = if near_b { pocket_b } else { pocket_a };
            let (lo, hi) = if near_b {
                (-hw, hw + if flare { LANE_W_M } else { 0.0 })
            } else {
                (-hw - if flare { LANE_W_M } else { 0.0 }, hw)
            };
            let mut d = lo + 0.5;
            while d <= hi - 0.5 {
                gizmos.line_2d(
                    at(s_c - 1.2, d),
                    at(s_c + 1.2, d),
                    Color::srgb(0.7, 0.7, 0.7),
                );
                d += 1.5;
            }
        }
    }
    // slip lanes: a wide right-turn curve bypassing the junction proper
    let idx: HashMap<[i64; 2], usize> = w
        .map
        .coords
        .iter()
        .enumerate()
        .map(|(i, &c)| (c, i))
        .collect();
    let lane_of: HashMap<[usize; 2], f64> = w
        .map
        .edges
        .iter()
        .enumerate()
        .map(|(ei, &[a, b])| ([a.min(b), a.max(b)], w.map.half_width(ei)))
        .collect();
    for (ji, &jc) in w.map.coords.iter().enumerate() {
        for d_in in [[1i64, 0], [0, 1], [-1, 0], [0, -1]] {
            if !has_slip(w.map.seed, jc, d_in) {
                continue;
            }
            let d_out = [d_in[1], -d_in[0]]; // right turn
            let (Some(&ai), Some(&bi)) = (
                idx.get(&[jc[0] - d_in[0], jc[1] - d_in[1]]),
                idx.get(&[jc[0] + d_out[0], jc[1] + d_out[1]]),
            ) else {
                continue;
            };
            let (Some(&hw_in), Some(&hw_out)) = (
                lane_of.get(&[ai.min(ji), ai.max(ji)]),
                lane_of.get(&[bi.min(ji), bi.max(ji)]),
            ) else {
                continue;
            };
            let pj = w.map.nodes[ji];
            let unit_to = |q: [f64; 2], p: [f64; 2]| {
                let d = ((p[0] - q[0]).hypot(p[1] - q[1])).max(1e-9);
                [(p[0] - q[0]) / d, (p[1] - q[1]) / d]
            };
            let u_in = unit_to(w.map.nodes[ai], pj);
            let u_out = unit_to(pj, w.map.nodes[bi]);
            let set = node_w[ji] + 8.0;
            let p0 = [
                pj[0] - u_in[0] * set + u_in[1] * hw_in,
                pj[1] - u_in[1] * set - u_in[0] * hw_in,
            ];
            let p2 = [
                pj[0] + u_out[0] * set + u_out[1] * hw_out,
                pj[1] + u_out[1] * set - u_out[0] * hw_out,
            ];
            // control point: where the two boundary lines would meet
            let det = u_in[0] * u_out[1] - u_in[1] * u_out[0];
            if det.abs() < 0.2 {
                continue; // nearly straight: no corner to slip
            }
            let (rx, ry) = (p2[0] - p0[0], p2[1] - p0[1]);
            let t = (rx * u_out[1] - ry * u_out[0]) / det;
            let c = [p0[0] + u_in[0] * t, p0[1] + u_in[1] * t];
            gizmos.linestrip_2d(
                (0..=8).map(|k| {
                    let t = k as f64 / 8.0;
                    let mt = 1.0 - t;
                    ppx([
                        mt * mt * p0[0] + 2.0 * mt * t * c[0] + t * t * p2[0],
                        mt * mt * p0[1] + 2.0 * mt * t * c[1] + t * t * p2[1],
                    ])
                }),
                boundary,
            );
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
        let color = match actor.kind {
            ActorKind::Car => Color::srgb(0.6, 0.6, 0.6),
            ActorKind::Truck => Color::srgb(0.6, 0.5, 0.35),
            ActorKind::Bike => Color::srgb(0.4, 0.7, 0.7),
            ActorKind::Pedestrian => Color::srgb(0.85, 0.7, 0.45),
        };
        if actor.kind == ActorKind::Pedestrian {
            gizmos.circle_2d(px(&actor.state), 0.35 * PX_PER_M, color);
        } else {
            let sz = actor.kind.size_m();
            let size = Vec2::new(sz[0] as f32, sz[1] as f32);
            draw_agent(&mut gizmos, &actor.state, size, color);
        }
    }
    // parked-and-waiting hint: a faint ring around the ego when goalless
    if w.goal.is_none() {
        gizmos.circle_2d(
            px(&w.ego),
            (LANE_W_M * 2.0) as f32 * PX_PER_M,
            Color::srgba(1.0, 1.0, 1.0, 0.2),
        );
    }
}
