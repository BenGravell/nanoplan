//! Renders the scenario map, the ego/actor cars, and the future-preview
//! overlay via Bevy gizmos.

use bevy::prelude::*;
use nanoplan::planning::{Context, Diagnostics};
use nanoplan::scenarios::Road;
use nanoplan::{Control, Path, Scenario, State, step};

use super::rollouts::RolloutCache;
use super::{DT, Mode, Scenarios, UiState};

pub(crate) const PX_PER_M: f32 = 6.0;
/// Pacifica footprint from scenarios/nuplan/vehicle_parameters.py.
const CAR_SIZE_M: Vec2 = Vec2::new(5.176, 2.297);
pub(crate) const ACCENT: Color = Color::srgb(1.0, 0.25, 0.85);
const DIAG_POINT: Color = Color::srgb(0.95, 0.85, 0.2);
const DIAG_TRAJECTORY: Color = Color::srgb(0.2, 0.85, 0.95);

fn ctx<'a>(
    road: &'a Road,
    actors: &'a [State],
    horizon: usize,
    diagnostics: Option<&'a Diagnostics>,
) -> Context<'a> {
    Context {
        road,
        actors,
        ego_control: Control::default(),
        horizon,
        latency: None,
        diagnostics,
    }
}

fn rollout_controls(mut s: State, controls: &[Control]) -> Vec<State> {
    controls
        .iter()
        .map(|&u| {
            s = step(s, u, DT);
            s
        })
        .collect()
}

pub(crate) fn px(s: &State) -> Vec2 {
    Vec2::new(s.x as f32, s.y as f32) * PX_PER_M
}

pub(crate) fn ppx(p: [f64; 2]) -> Vec2 {
    Vec2::new(p[0] as f32, p[1] as f32) * PX_PER_M
}

/// Draw the scenario's map: road boundaries, centerline, lane divider,
/// crosswalks, cross streets, and the goal pose at the end of the route.
fn draw_map(gizmos: &mut Gizmos, sc: &Scenario) {
    let path = Path::new(&sc.centerline);
    let len = path.length();
    let line = |d: f64| {
        (0..)
            .map(move |i| i as f64 * 2.0)
            .take_while(move |s| *s <= len)
            .map(move |s| (s, d))
    };
    let boundary = Color::srgb(0.55, 0.55, 0.55);
    for d in [-sc.map.road_half_width, sc.map.road_half_width] {
        gizmos.linestrip_2d(line(d).map(|(s, d)| ppx(path.frenet_to_xy(s, d))), boundary);
    }
    gizmos.linestrip_2d(
        line(0.0).map(|(s, d)| ppx(path.frenet_to_xy(s, d))),
        Color::srgb(0.35, 0.35, 0.35),
    );
    for &s in &sc.map.cross_streets {
        // A straight perpendicular road through the main road at this
        // station, purely so a crossing/intersection actor has a road to
        // be driving on instead of appearing to cross empty space. `along`
        // reuses the main road's own heading as the cross street's width
        // axis; `perp` (rotated 90°) is the cross street's own direction
        // of travel.
        let (center, heading) = path.pose_at(s);
        let along = [heading.cos(), heading.sin()];
        let perp = [-heading.sin(), heading.cos()];
        const CROSS_HALF_LEN_M: f64 = 70.0;
        let pt = |t: f64, w: f64| {
            ppx([
                center[0] + perp[0] * t + along[0] * w,
                center[1] + perp[1] * t + along[1] * w,
            ])
        };
        for w in [-sc.map.road_half_width, sc.map.road_half_width] {
            gizmos.line_2d(pt(-CROSS_HALF_LEN_M, w), pt(CROSS_HALF_LEN_M, w), boundary);
        }
        gizmos.line_2d(
            pt(-CROSS_HALF_LEN_M, 0.0),
            pt(CROSS_HALF_LEN_M, 0.0),
            Color::srgb(0.35, 0.35, 0.35),
        );
    }
    if let Some(d) = sc.map.divider_d {
        // dashed divider between opposing lanes: 3 m dash, 3 m gap
        let mut s = 0.0;
        while s + 3.0 <= len {
            gizmos.line_2d(
                ppx(path.frenet_to_xy(s, d)),
                ppx(path.frenet_to_xy(s + 3.0, d)),
                Color::srgb(0.65, 0.55, 0.2),
            );
            s += 6.0;
        }
    }
    for &s in &sc.map.crosswalk_s {
        // stripes run along the road direction, spanning its width
        let mut d = -sc.map.road_half_width + 0.5;
        while d <= sc.map.road_half_width - 0.5 {
            gizmos.line_2d(
                ppx(path.frenet_to_xy(s - 1.5, d)),
                ppx(path.frenet_to_xy(s + 1.5, d)),
                Color::srgb(0.7, 0.7, 0.7),
            );
            d += 1.5;
        }
    }
    // scene goal pose (nuPlan scene.goal_ego_pose): end of the route
    let goal = ppx(path.frenet_to_xy(len, 0.0));
    let green = Color::srgb(0.25, 0.8, 0.45);
    gizmos.circle_2d(goal, 2.0 * PX_PER_M, green);
    gizmos.circle_2d(goal, 0.5 * PX_PER_M, green);
}

pub(crate) fn draw_car(gizmos: &mut Gizmos, s: &State, color: Color) {
    draw_agent(gizmos, s, CAR_SIZE_M, color);
}

/// A vehicle of arbitrary footprint (open-world trucks, bikes, ...).
pub(crate) fn draw_agent(gizmos: &mut Gizmos, s: &State, size_m: Vec2, color: Color) {
    let iso = Isometry2d::new(px(s), Rot2::radians(s.yaw as f32));
    gizmos.rect_2d(iso, size_m * PX_PER_M, color);
    // heading tick from center to front bumper
    let nose = iso * Vec2::new(size_m.x * PX_PER_M / 2.0, 0.0);
    gizmos.line_2d(iso * Vec2::ZERO, nose, color);
}

pub(crate) fn draw(
    mut gizmos: Gizmos,
    state: Res<UiState>,
    scenes: Res<Scenarios>,
    cache: Res<RolloutCache>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
    if state.mode != Mode::Scrub {
        return;
    }
    // undo any open-world zoom; scrub mode draws at fixed scale
    camera.scale = Vec3::ONE;
    let sc = &scenes.0[state.scenario];
    draw_map(&mut gizmos, sc);

    // Nothing simulated for the current selection yet (still queued/running):
    // just show the map and wait for the cache to fill in.
    let Some(rollout) = cache.0.get(&(state.scenario, state.planner)) else {
        return;
    };
    let idx = ((state.time_s as f64 / DT).round() as usize).min(rollout.ego.len() - 1);
    let ego = rollout.ego[idx];
    camera.translation = px(&ego).extend(camera.translation.z);

    draw_car(&mut gizmos, &ego, Color::WHITE);
    for actor in &rollout.actors {
        draw_car(&mut gizmos, &actor[idx], Color::srgb(0.6, 0.6, 0.6));
    }

    let k = (state.preview_s as f64 / DT).round() as usize;
    if k == 0 {
        return;
    }
    // planned ego future: replan from the scrubbed state, roll out k ticks.
    // Only ask for diagnostics when a checkbox wants them — recording is
    // extra work the planner otherwise skips entirely (see Context::diagnostics).
    let want_diag = state.show_diag_points || state.show_diag_trajectories;
    let recorder = Diagnostics::default();
    let current: Vec<State> = rollout.actors.iter().map(|t| t[idx]).collect();
    let road = sc.road(DT);
    let plan_ctx = ctx(&road, &current, k, want_diag.then_some(&recorder));
    let plan = state.planner.build().plan(ego, &plan_ctx);
    if want_diag {
        let diag = recorder.take();
        if state.show_diag_trajectories {
            for traj in &diag.trajectories {
                gizmos.linestrip_2d(traj.iter().copied().map(ppx), DIAG_TRAJECTORY);
            }
        }
        if state.show_diag_points {
            for &p in &diag.points {
                gizmos.circle_2d(ppx(p), 0.3 * PX_PER_M, DIAG_POINT);
            }
        }
    }
    let planned = rollout_controls(ego, &plan[..k.min(plan.len())]);
    gizmos.linestrip_2d(std::iter::once(&ego).chain(&planned).map(px), ACCENT);
    if let Some(last) = planned.last() {
        draw_car(&mut gizmos, last, ACCENT);
    }
    // predicted actor futures: constant velocity from the scrubbed state
    let dim = ACCENT.with_alpha(0.5);
    for actor in &rollout.actors {
        let predicted = rollout_controls(actor[idx], &vec![Control::default(); k]);
        gizmos.linestrip_2d(std::iter::once(&actor[idx]).chain(&predicted).map(px), dim);
        if let Some(last) = predicted.last() {
            draw_car(&mut gizmos, last, dim);
        }
    }
}
