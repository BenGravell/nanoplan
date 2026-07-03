//! Interactive viewer: scrub through a simulated scenario and preview the
//! planned ego future and predicted actor motion.

use std::collections::HashMap;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use nanoplan::planning::{Context, PLANNING_HORIZON_S};
use nanoplan::scenarios::{Actor, MapData};
use nanoplan::{
    Control, IncrementalSim, Metrics, Path, PlannerKind, Rollout, Scenario, State, simulate, step,
};
use web_time::Instant;

/// Scenario JSONs bundled into the binary, so they ship to the web build too.
const BUNDLED: [&str; 2] = [
    include_str!("../scenarios/json/braking_lead.json"),
    include_str!("../scenarios/json/cut_in.json"),
];

/// All scenarios the viewer offers: built-ins, bundled JSON (e.g. exported
/// from nuPlan logs), and — on desktop — any files or directories passed as
/// CLI args (see also the in-app scenario-loading widget in `ui()`).
fn all_scenarios() -> Vec<Scenario> {
    let mut all = scenarios();
    all.extend(
        BUNDLED
            .iter()
            .map(|s| serde_json::from_str(s).expect("bundled scenario is valid JSON")),
    );
    #[cfg(not(target_arch = "wasm32"))]
    for arg in std::env::args().skip(1) {
        match nanoplan::scenarios::load_path(std::path::Path::new(&arg)) {
            Ok(loaded) => all.extend(loaded),
            Err(e) => eprintln!("skipping scenario path {arg}: {e}"),
        }
    }
    all
}

const DT: f64 = 0.1;
const DURATION_S: f64 = 20.0;
const PREVIEW_MAX_S: f64 = PLANNING_HORIZON_S;
const PX_PER_M: f32 = 6.0;
/// Pacifica footprint from scenarios/nuplan/vehicle_parameters.py.
const CAR_SIZE_M: Vec2 = Vec2::new(5.176, 2.297);
const ACCENT: Color = Color::srgb(1.0, 0.25, 0.85);

fn straight_road() -> Vec<[f64; 2]> {
    (0..=45).map(|i| [i as f64 * 10.0 - 50.0, 0.0]).collect()
}

fn s_curve_road() -> Vec<[f64; 2]> {
    (0..=90)
        .map(|i| {
            let x = i as f64 * 5.0 - 50.0;
            [x, 8.0 * (x / 40.0).sin()]
        })
        .collect()
}

// ponytail: synthetic scenarios stand in for nuPlan logs until a loader exists
fn scenarios() -> Vec<Scenario> {
    let state = |x, y, yaw, speed| State { x, y, yaw, speed };
    let drive = |accel, curvature| Control { accel, curvature };
    let map = |divider_d, crosswalk_s| MapData {
        divider_d,
        crosswalk_s,
        ..Default::default()
    };
    let scenario = |name: &str, ego, actors, centerline, map| Scenario {
        name: name.into(),
        ego,
        actors,
        centerline,
        target_speed: 10.0,
        map,
    };
    vec![
        scenario(
            "offset start",
            state(0.0, 3.0, 0.0, 8.0),
            vec![],
            straight_road(),
            map(None, vec![]),
        ),
        scenario(
            "s-curve road",
            state(0.0, 0.0, 0.0, 8.0),
            vec![],
            s_curve_road(),
            map(None, vec![]),
        ),
        scenario(
            "stopped lead",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(60.0, 0.0, 0.0, 0.0),
                control: drive(0.0, 0.0),
                trajectory: vec![],
            }],
            straight_road(),
            map(None, vec![]),
        ),
        scenario(
            "oncoming",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(160.0, 4.0, std::f64::consts::PI, 8.0),
                control: drive(0.0, 0.0),
                trajectory: vec![],
            }],
            straight_road(),
            map(Some(2.0), vec![]),
        ),
        scenario(
            "crossing",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(80.0, -60.0, std::f64::consts::FRAC_PI_2, 6.0),
                control: drive(0.0, 0.0),
                trajectory: vec![],
            }],
            straight_road(),
            // crossing traffic at x = 80 → station 130 on a road starting at -50
            map(None, vec![130.0]),
        ),
        scenario(
            "curving lead",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(20.0, 0.0, 0.0, 8.0),
                // curves away; constant-velocity prediction visibly diverges
                control: drive(0.0, 0.02),
                trajectory: vec![],
            }],
            straight_road(),
            map(None, vec![]),
        ),
    ]
}

#[derive(Resource)]
struct Scenarios(Vec<Scenario>);

#[derive(Resource)]
struct UiState {
    scenario: usize,
    planner: PlannerKind,
    time_s: f32,
    preview_s: f32,
}

/// Finished closed-loop simulations, keyed by scenario + planner so
/// re-selecting a combo we've already simulated is instant.
#[derive(Resource, Default)]
struct RolloutCache(HashMap<(usize, PlannerKind), Rollout>);

/// A simulation in progress, time-sliced across frames so an expensive
/// planner (PI²-DDP) never blocks the UI thread — see `IncrementalSim`.
///
/// `IncrementalSim` holds a `Box<dyn Planner>` and an interior-mutable
/// latency recorder, neither of which are `Sync`, so this is a `NonSend`
/// resource rather than a regular one.
#[derive(Default)]
struct ActiveJob(Option<((usize, PlannerKind), IncrementalSim)>);

/// Per-frame wall-clock budget for stepping the active job.
const FRAME_BUDGET_MS: u64 = 8;

/// State for the in-app scenario-loading widget: type a path to a nuPlan
/// export (a `*.json` file or a directory of them) and load it live,
/// without relaunching with CLI args. Desktop only — wasm has no arbitrary
/// filesystem access.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Resource, Default)]
struct ScenarioLoader {
    path: String,
    status: Option<Result<String, String>>,
}

/// Fetches the pre-bundled nuPlan scenario set at startup — the web
/// equivalent of desktop's `load_path`/CLI args/"nuPlan path" widget, none
/// of which work without a filesystem. `scenarios/web_bundle.json` (built by
/// `tools/bundle_web_scenarios.py`, copied into `dist/` by Trunk's
/// `copy-file` directive in `index.html`) is fetched once as a single
/// compact JSON array — one HTTP request instead of one per scenario.
#[cfg(target_arch = "wasm32")]
mod web_scenarios {
    use super::{Scenario, Scenarios};
    use bevy::prelude::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    /// Relative, so it resolves under the page's own base path both for
    /// `trunk serve` (http://localhost:8080/) and the GitHub Pages deploy
    /// (.../nanoplan/).
    const BUNDLE_URL: &str = "web_bundle.json";

    /// Slot the spawned fetch task writes its result into, polled once a
    /// frame. Plain `Rc<RefCell<_>>` (wasm is single-threaded, so this can
    /// never be `Send`) means this can only ever be a `NonSend` resource.
    #[derive(Default)]
    pub struct WebScenarioFetch(Rc<RefCell<Option<Vec<Scenario>>>>);

    pub fn spawn_fetch(fetch: NonSend<WebScenarioFetch>) {
        let slot = fetch.0.clone();
        wasm_bindgen_futures::spawn_local(async move {
            *slot.borrow_mut() = Some(fetch_bundle().await);
        });
    }

    async fn fetch_bundle() -> Vec<Scenario> {
        let response = match gloo_net::http::Request::get(BUNDLE_URL).send().await {
            Ok(r) => r,
            Err(e) => {
                warn!("{BUNDLE_URL} fetch failed: {e}");
                return Vec::new();
            }
        };
        if !response.ok() {
            warn!("{BUNDLE_URL} fetch returned HTTP {}", response.status());
            return Vec::new();
        }
        match response.json::<Vec<Scenario>>().await {
            Ok(v) => v,
            Err(e) => {
                warn!("{BUNDLE_URL} parse failed: {e}");
                Vec::new()
            }
        }
    }

    /// Once a frame: if the fetch has landed, merge it into the scenario
    /// list. `take()` leaves `None` behind, so this is a no-op every frame
    /// after the first (cheap: one `Option` check).
    pub fn absorb_fetch(fetch: NonSend<WebScenarioFetch>, mut scenes: ResMut<Scenarios>) {
        let Some(loaded) = fetch.0.borrow_mut().take() else {
            return;
        };
        if !loaded.is_empty() {
            info!("loaded {} scenario(s) from {BUNDLE_URL}", loaded.len());
        }
        scenes.0.extend(loaded);
    }
}

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

fn main() {
    let scenes = all_scenarios();
    let mut cache = RolloutCache::default();
    cache.0.insert(
        (0, PlannerKind::Straight),
        simulate(&scenes[0], PlannerKind::Straight, DURATION_S, DT),
    );
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "nanoplan".into(),
            // fill the browser window on wasm; no effect on desktop
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .insert_resource(Scenarios(scenes))
    .insert_resource(UiState {
        scenario: 0,
        planner: PlannerKind::Straight,
        time_s: 0.0,
        preview_s: 0.0,
    })
    .insert_resource(cache)
    .init_non_send::<ActiveJob>()
    .add_systems(Startup, |mut commands: Commands| {
        commands.spawn(Camera2d);
    })
    .add_systems(EguiPrimaryContextPass, ui)
    .add_systems(Update, (step_active_job, draw).chain());
    #[cfg(not(target_arch = "wasm32"))]
    app.insert_resource(ScenarioLoader::default());
    #[cfg(target_arch = "wasm32")]
    app.init_non_send::<web_scenarios::WebScenarioFetch>()
        .add_systems(Startup, web_scenarios::spawn_fetch)
        .add_systems(Update, web_scenarios::absorb_fetch);
    app.run();
}

/// Advance the in-flight simulation (if any) by one frame's time budget,
/// so an expensive planner never blocks the UI thread. Once it finishes,
/// the result moves into the cache and the job slot frees up.
fn step_active_job(mut job: NonSendMut<ActiveJob>, mut cache: ResMut<RolloutCache>) {
    let Some((_, sim)) = &mut job.0 else { return };
    sim.step_until(Instant::now() + std::time::Duration::from_millis(FRAME_BUDGET_MS));
    if sim.is_done() {
        let (key, sim) = job.0.take().unwrap();
        cache.0.insert(key, sim.finish());
    }
}

#[cfg_attr(target_arch = "wasm32", allow(unused_mut))]
fn ui(
    mut contexts: EguiContexts,
    mut scenes: ResMut<Scenarios>,
    mut state: ResMut<UiState>,
    cache: Res<RolloutCache>,
    mut job: NonSendMut<ActiveJob>,
    #[cfg(not(target_arch = "wasm32"))] mut loader: ResMut<ScenarioLoader>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let prev = (state.scenario, state.planner);
    egui::Window::new("nanoplan").show(ctx, |ui| {
        egui::ComboBox::from_label("scenario")
            .selected_text(&scenes.0[state.scenario].name)
            .show_ui(ui, |ui| {
                for (i, sc) in scenes.0.iter().enumerate() {
                    ui.selectable_value(&mut state.scenario, i, &sc.name);
                }
            });
        egui::ComboBox::from_label("planner")
            .selected_text(state.planner.name())
            .show_ui(ui, |ui| {
                for kind in PlannerKind::ALL {
                    ui.selectable_value(&mut state.planner, kind, kind.name());
                }
            });
        #[cfg(not(target_arch = "wasm32"))]
        {
            ui.horizontal(|ui| {
                ui.label("nuPlan path:");
                ui.text_edit_singleline(&mut loader.path);
                if ui.button("Load").clicked() {
                    loader.status = Some(
                        match nanoplan::scenarios::load_path(std::path::Path::new(
                            loader.path.trim(),
                        )) {
                            Ok(loaded) if loaded.is_empty() => {
                                Err("no *.json scenarios found there".into())
                            }
                            Ok(loaded) => {
                                let n = loaded.len();
                                state.scenario = scenes.0.len();
                                scenes.0.extend(loaded);
                                Ok(format!(
                                    "loaded {n} scenario{}",
                                    if n == 1 { "" } else { "s" }
                                ))
                            }
                            Err(e) => Err(e.to_string()),
                        },
                    );
                }
            });
            if let Some(status) = &loader.status {
                let (color, msg) = match status {
                    Ok(msg) => (egui::Color32::from_rgb(120, 210, 140), msg),
                    Err(msg) => (egui::Color32::from_rgb(230, 100, 100), msg),
                };
                ui.colored_label(color, msg);
            }
        }
        ui.add(egui::Slider::new(&mut state.time_s, 0.0..=DURATION_S as f32).text("time [s]"));
        ui.add(
            egui::Slider::new(&mut state.preview_s, 0.0..=PREVIEW_MAX_S as f32)
                .text("future preview [s]"),
        );
        ui.separator();

        let key = (state.scenario, state.planner);
        match (cache.0.get(&key), &job.0) {
            (Some(rollout), _) => {
                let idx = (state.time_s as f64 / DT).round() as usize;
                let (tick_scores, tick_score) = rollout.metrics.at(idx);
                egui::Grid::new("metrics").show(ui, |ui| {
                    ui.label("");
                    ui.label("@t");
                    ui.label("agg");
                    ui.end_row();
                    for ((label, tick), avg) in Metrics::LABELS
                        .iter()
                        .zip(tick_scores)
                        .zip(rollout.metrics.aggregate)
                    {
                        ui.label(*label);
                        ui.label(format!("{tick:.2}"));
                        ui.label(format!("{avg:.2}"));
                        ui.end_row();
                    }
                    ui.strong("closed-loop score");
                    ui.strong(format!("{tick_score:.2}"));
                    ui.strong(format!("{:.2}", rollout.metrics.score));
                    ui.end_row();
                });
                ui.separator();
                ui.label("planner latency");
                egui::Grid::new("latency").show(ui, |ui| {
                    ui.label("seam");
                    ui.label("mean [ms]");
                    ui.label("max [ms]");
                    ui.end_row();
                    for seam in &rollout.latency.seams {
                        ui.label(seam.name);
                        ui.label(format!("{:.3}", seam.mean_ms()));
                        ui.label(format!("{:.3}", seam.max_ms));
                        ui.end_row();
                    }
                });
            }
            (None, Some((active_key, sim))) if *active_key == key => {
                ui.add(egui::ProgressBar::new(sim.progress()).text("simulating…"));
            }
            (None, _) => {
                if ui.button("Simulate").clicked() {
                    job.0 = Some((
                        key,
                        IncrementalSim::start(&scenes.0[key.0], key.1, DURATION_S, DT),
                    ));
                }
            }
        }
    });
    if (state.scenario, state.planner) != prev {
        state.time_s = 0.0;
    }
    // future preview active: frame the whole screen in the accent color
    if state.preview_s > 0.0 {
        let painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Foreground,
            egui::Id::new("preview_frame"),
        ));
        let accent = egui::Color32::from_rgb(255, 64, 217);
        painter.rect_stroke(
            ctx.content_rect(),
            0,
            egui::Stroke::new(10.0, accent),
            egui::StrokeKind::Inside,
        );
    }
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

fn draw(
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_scenarios_parse_and_simulate() {
        let scenes = all_scenarios();
        assert!(scenes.len() >= 8); // 6 built-ins + 2 bundled
        let bundled = &scenes[6];
        assert!(bundled.name.starts_with("nuplan:"));
        assert!(!bundled.actors[0].trajectory.is_empty());
        let r = simulate(bundled, PlannerKind::Straight, 2.0, DT);
        assert_eq!(r.ego.len(), 21);
    }
}
