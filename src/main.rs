//! Interactive viewer: scrub through a simulated scenario and preview the
//! planned ego future and predicted actor motion.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use nanoplan::{Context, Control, Metrics, PlannerKind, Simulator, State, metrics, step};

const DT: f64 = 0.1;
const DURATION_S: f64 = 20.0;
const PREVIEW_MAX_S: f64 = 5.0;
const PX_PER_M: f32 = 6.0;
/// Pacifica footprint from scenarios/nuplan/vehicle_parameters.py.
const CAR_SIZE_M: Vec2 = Vec2::new(5.176, 2.297);
const ACCENT: Color = Color::srgb(1.0, 0.25, 0.85);

/// A non-ego actor: initial state plus the constant control it drives with.
struct Actor {
    init: State,
    control: Control,
}

struct Scenario {
    name: &'static str,
    ego: State,
    actors: Vec<Actor>,
    centerline: Vec<[f64; 2]>,
    target_speed: f64,
}

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
    let scenario = |name, ego, actors, centerline| Scenario {
        name,
        ego,
        actors,
        centerline,
        target_speed: 10.0,
    };
    vec![
        scenario(
            "offset start",
            state(0.0, 3.0, 0.0, 8.0),
            vec![],
            straight_road(),
        ),
        scenario(
            "s-curve road",
            state(0.0, 0.0, 0.0, 8.0),
            vec![],
            s_curve_road(),
        ),
        scenario(
            "stopped lead",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(60.0, 0.0, 0.0, 0.0),
                control: drive(0.0, 0.0),
            }],
            straight_road(),
        ),
        scenario(
            "oncoming",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(160.0, 4.0, std::f64::consts::PI, 8.0),
                control: drive(0.0, 0.0),
            }],
            straight_road(),
        ),
        scenario(
            "crossing",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(80.0, -60.0, std::f64::consts::FRAC_PI_2, 6.0),
                control: drive(0.0, 0.0),
            }],
            straight_road(),
        ),
        scenario(
            "curving lead",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(20.0, 0.0, 0.0, 8.0),
                // curves away; constant-velocity prediction visibly diverges
                control: drive(0.0, 0.02),
            }],
            straight_road(),
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

/// Precomputed simulation: ego and actor states at every tick, plus the
/// nuPlan closed-loop metrics of the finished rollout.
#[derive(Resource)]
struct Rollout {
    ego: Vec<State>,
    actors: Vec<Vec<State>>,
    metrics: Metrics,
}

fn ctx<'a>(sc: &'a Scenario, actors: &'a [State], horizon: usize) -> Context<'a> {
    Context {
        centerline: &sc.centerline,
        actors,
        target_speed: sc.target_speed,
        dt: DT,
        horizon,
    }
}

fn compute_rollout(sc: &Scenario, kind: PlannerKind) -> Rollout {
    let steps = (DURATION_S / DT) as usize;
    let actors: Vec<Vec<State>> = sc
        .actors
        .iter()
        .map(|a| {
            let mut s = a.init;
            std::iter::once(s)
                .chain((0..steps).map(|_| {
                    s = step(s, a.control, DT);
                    s
                }))
                .collect()
        })
        .collect();
    let mut sim = Simulator {
        state: sc.ego,
        dt: DT,
    };
    let mut planner = kind.build();
    let mut ego = vec![sc.ego];
    ego.extend((0..steps).map(|i| {
        let current: Vec<State> = actors.iter().map(|t| t[i]).collect();
        sim.tick(planner.as_mut(), &ctx(sc, &current, 1))
    }));
    let metrics = metrics::evaluate(&ego, &actors, &sc.centerline, sc.target_speed, DT);
    Rollout {
        ego,
        actors,
        metrics,
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
    let scenes = scenarios();
    let rollout = compute_rollout(&scenes[0], PlannerKind::Straight);
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
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
        .insert_resource(rollout)
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
        .add_systems(EguiPrimaryContextPass, ui)
        .add_systems(Update, draw)
        .run();
}

fn ui(
    mut contexts: EguiContexts,
    scenes: Res<Scenarios>,
    mut state: ResMut<UiState>,
    mut rollout: ResMut<Rollout>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let prev = (state.scenario, state.planner);
    egui::Window::new("nanoplan").show(ctx, |ui| {
        egui::ComboBox::from_label("scenario")
            .selected_text(scenes.0[state.scenario].name)
            .show_ui(ui, |ui| {
                for (i, sc) in scenes.0.iter().enumerate() {
                    ui.selectable_value(&mut state.scenario, i, sc.name);
                }
            });
        egui::ComboBox::from_label("planner")
            .selected_text(state.planner.name())
            .show_ui(ui, |ui| {
                for kind in PlannerKind::ALL {
                    ui.selectable_value(&mut state.planner, kind, kind.name());
                }
            });
        ui.add(egui::Slider::new(&mut state.time_s, 0.0..=DURATION_S as f32).text("time [s]"));
        ui.add(
            egui::Slider::new(&mut state.preview_s, 0.0..=PREVIEW_MAX_S as f32)
                .text("future preview [s]"),
        );
        ui.separator();
        egui::Grid::new("metrics").show(ui, |ui| {
            for (label, value) in Metrics::LABELS.iter().zip(rollout.metrics.values()) {
                ui.label(*label);
                ui.label(format!("{value:.2}"));
                ui.end_row();
            }
            ui.strong("closed-loop score");
            ui.strong(format!("{:.2}", rollout.metrics.score));
            ui.end_row();
        });
    });
    if (state.scenario, state.planner) != prev {
        *rollout = compute_rollout(&scenes.0[state.scenario], state.planner);
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
    rollout: Res<Rollout>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
    let sc = &scenes.0[state.scenario];
    let idx = ((state.time_s as f64 / DT).round() as usize).min(rollout.ego.len() - 1);
    let ego = rollout.ego[idx];
    camera.translation = px(&ego).extend(camera.translation.z);

    gizmos.linestrip_2d(
        sc.centerline
            .iter()
            .map(|p| Vec2::new(p[0] as f32, p[1] as f32) * PX_PER_M),
        Color::srgb(0.35, 0.35, 0.35),
    );
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
