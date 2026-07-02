//! Interactive viewer: scrub through a simulated scenario and preview the
//! planned ego future and predicted actor motion.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use nanoplan::{Control, Planner, Simulator, State, StraightPlanner, step};

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
}

// ponytail: synthetic scenarios stand in for nuPlan logs until a loader exists
fn scenarios() -> Vec<Scenario> {
    let state = |x, y, yaw, speed| State { x, y, yaw, speed };
    let drive = |accel, curvature| Control { accel, curvature };
    vec![
        Scenario {
            name: "open road",
            ego: state(0.0, 0.0, 0.0, 8.0),
            actors: vec![],
        },
        Scenario {
            name: "oncoming",
            ego: state(0.0, 0.0, 0.0, 8.0),
            actors: vec![Actor {
                init: state(160.0, 4.0, std::f64::consts::PI, 8.0),
                control: drive(0.0, 0.0),
            }],
        },
        Scenario {
            name: "crossing",
            ego: state(0.0, 0.0, 0.0, 8.0),
            actors: vec![Actor {
                init: state(80.0, -60.0, std::f64::consts::FRAC_PI_2, 6.0),
                control: drive(0.0, 0.0),
            }],
        },
        Scenario {
            name: "curving lead",
            ego: state(0.0, 0.0, 0.0, 8.0),
            actors: vec![Actor {
                init: state(20.0, 0.0, 0.0, 8.0),
                // curves away; constant-velocity prediction visibly diverges
                control: drive(0.0, 0.02),
            }],
        },
    ]
}

#[derive(Resource)]
struct Scenarios(Vec<Scenario>);

#[derive(Resource)]
struct UiState {
    scenario: usize,
    time_s: f32,
    preview_s: f32,
}

/// Precomputed simulation: ego and actor states at every tick.
#[derive(Resource)]
struct Rollout {
    ego: Vec<State>,
    actors: Vec<Vec<State>>,
}

fn compute_rollout(sc: &Scenario) -> Rollout {
    let steps = (DURATION_S / DT) as usize;
    let mut sim = Simulator {
        state: sc.ego,
        dt: DT,
    };
    let mut planner = StraightPlanner { horizon: 1 };
    let mut ego = vec![sc.ego];
    ego.extend((0..steps).map(|_| sim.tick(&mut planner)));
    let actors = sc
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
    Rollout { ego, actors }
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
    let rollout = compute_rollout(&scenes[0]);
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin::default())
        .insert_resource(Scenarios(scenes))
        .insert_resource(UiState {
            scenario: 0,
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
    let prev = state.scenario;
    egui::Window::new("nanoplan").show(ctx, |ui| {
        egui::ComboBox::from_label("scenario")
            .selected_text(scenes.0[state.scenario].name)
            .show_ui(ui, |ui| {
                for (i, sc) in scenes.0.iter().enumerate() {
                    ui.selectable_value(&mut state.scenario, i, sc.name);
                }
            });
        ui.add(egui::Slider::new(&mut state.time_s, 0.0..=DURATION_S as f32).text("time [s]"));
        ui.add(
            egui::Slider::new(&mut state.preview_s, 0.0..=PREVIEW_MAX_S as f32)
                .text("future preview [s]"),
        );
    });
    if state.scenario != prev {
        *rollout = compute_rollout(&scenes.0[state.scenario]);
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
    rollout: Res<Rollout>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
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
    let plan = StraightPlanner { horizon: k }.plan(ego);
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
