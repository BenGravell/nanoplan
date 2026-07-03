//! Renders the scenario map, the ego/actor cars, and the future-preview
//! overlay via Bevy gizmos.

use bevy::prelude::*;
use nanoplan::planning::Context;
use nanoplan::{Control, Path, Scenario, State, step};

use super::sim::RolloutCache;
use super::{DT, Scenarios, UiState};

const PX_PER_M: f32 = 6.0;
/// Pacifica footprint from scenarios/nuplan/vehicle_parameters.py.
const CAR_SIZE_M: Vec2 = Vec2::new(5.176, 2.297);
const ACCENT: Color = Color::srgb(1.0, 0.25, 0.85);

fn ctx<'a>(sc: &'a Scenario, actors: &'a [State], horizon: usize) -> Context<'a> {
    Context {
        centerline: &sc.centerline,
        actors,
        target_speed: sc.target_speed,
        dt: DT,
        horizon,
        latency: None,
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

fn px(s: &State) -> Vec2 {
    Vec2::new(s.x as f32, s.y as f32) * PX_PER_M
}

fn ppx(p: [f64; 2]) -> Vec2 {
    Vec2::new(p[0] as f32, p[1] as f32) * PX_PER_M
}

/// Draw the scenario's map: road boundaries, centerline, lane divider,
/// crosswalks, and the goal pose at the end of the route.
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

fn draw_car(gizmos: &mut Gizmos, s: &State, color: Color) {
    let iso = Isometry2d::new(px(s), Rot2::radians(s.yaw as f32));
    gizmos.rect_2d(iso, CAR_SIZE_M * PX_PER_M, color);
    // heading tick from center to front bumper
    let nose = iso * Vec2::new(CAR_SIZE_M.x * PX_PER_M / 2.0, 0.0);
    gizmos.line_2d(iso * Vec2::ZERO, nose, color);
}

pub(crate) fn draw(
    mut gizmos: Gizmos,
    state: Res<UiState>,
    scenes: Res<Scenarios>,
    cache: Res<RolloutCache>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
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
    // planned ego future: replan from the scrubbed state, roll out k ticks
    let current: Vec<State> = rollout.actors.iter().map(|t| t[idx]).collect();
    let plan = state.planner.build().plan(ego, &ctx(sc, &current, k));
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
