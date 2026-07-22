//! Time-colored swept ego footprint.

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use colorgrad::Gradient;

use crate::common::differencing::forward_difference;
use crate::common::math::wrap_angle;
use crate::geometry::EGO_FOOTPRINT;
use crate::metrics::Metrics;
use crate::simulation::{MAX_TERMINAL_SPEED_MPS, State};
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};
#[cfg(test)]
use crate::viewer::colors::GUPPY_ORANGE;
use crate::viewer::{
    CarpetVisualization,
    colors::{CARPET_ALPHA, GUPPY, GUPPY_BLUE},
};

use super::super::screen::PX_PER_M;

const BAND_M: f64 = 0.5;
const FOOTPRINT_EPSILON_M: f64 = 1e-9;

#[derive(Resource)]
pub(crate) struct EgoCarpetMesh {
    handle: Handle<Mesh>,
    populated: bool,
}

#[derive(Clone, Copy)]
struct TimedState {
    state: State,
    time: f64,
}

#[derive(Clone, Copy)]
struct CrossSection {
    right: [f64; 2],
    left: [f64; 2],
    time: f64,
}

#[derive(Clone, Copy)]
struct CarpetPatch {
    rear: CrossSection,
    front: CrossSection,
    time: f64,
}

pub(crate) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let handle = meshes.add(empty_mesh());
    commands.spawn((
        Mesh2d(handle.clone()),
        MeshMaterial2d(materials.add(ColorMaterial::default())),
        Transform::from_xyz(0.0, 0.0, -0.5),
    ));
    commands.insert_resource(EgoCarpetMesh {
        handle,
        populated: false,
    });
}

pub(crate) fn draw(
    meshes: &mut Assets<Mesh>,
    carpet: &mut EgoCarpetMesh,
    ego: State,
    plan: &[State],
    dt: f64,
    visualization: CarpetVisualization,
    metrics: Option<&Metrics>,
) {
    let footprints = sample_footprints(ego, plan, dt);
    let patches = carpet_patches(&footprints);
    let values = visualization_values(ego, plan, dt, visualization, metrics);
    let colormap = match visualization {
        CarpetVisualization::Time => &*GUPPY_BLUE,
        CarpetVisualization::Speed => &*GUPPY_BLUE,
        _ => &*GUPPY,
    };

    let mut vertices = Vec::with_capacity(patches.len() * 6);
    let mut colors = Vec::with_capacity(vertices.capacity());
    for patch in patches {
        let index = (patch.time / dt).round() as usize;
        let sample = colormap.at(values[index.min(values.len() - 1)] as f32);
        let color = Color::srgba(sample.r, sample.g, sample.b, CARPET_ALPHA);
        push_patch(&mut vertices, &mut colors, patch, color);
    }

    let mut mesh = empty_mesh();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    if let Some(mut existing) = meshes.get_mut(&carpet.handle) {
        *existing = mesh;
        carpet.populated = true;
    }
}

pub(crate) fn clear(meshes: &mut Assets<Mesh>, carpet: &mut EgoCarpetMesh) {
    if !carpet.populated {
        return;
    }
    if let Some(mut mesh) = meshes.get_mut(&carpet.handle) {
        *mesh = empty_mesh();
        carpet.populated = false;
    }
}

fn visualization_values(
    ego: State,
    plan: &[State],
    dt: f64,
    visualization: CarpetVisualization,
    metrics: Option<&Metrics>,
) -> Vec<f64> {
    if let Some(metrics) = metrics {
        let values = match visualization {
            CarpetVisualization::Safety => metrics.per_tick.iter().map(|v| v[0]).collect(),
            CarpetVisualization::Progress => metrics.per_tick.iter().map(|v| v[1]).collect(),
            CarpetVisualization::Comfort => metrics.per_tick.iter().map(|v| v[2]).collect(),
            CarpetVisualization::Overall => metrics.score_per_tick.clone(),
            _ => vec![],
        };
        if !values.is_empty() {
            return values;
        }
    }

    let states: Vec<_> = std::iter::once(ego)
        .chain(plan.iter().skip(1).copied())
        .collect();
    let raw = match visualization {
        CarpetVisualization::Speed => states.iter().map(|state| state.speed).collect(),
        CarpetVisualization::Time => (0..states.len()).map(|i| i as f64 * dt).collect(),
        CarpetVisualization::LongitudinalAcceleration => {
            padded_forward(&states, |a, b| forward_difference(a.speed, b.speed, dt))
        }
        CarpetVisualization::LateralAcceleration => padded_forward(&states, |a, b| {
            let dvx = forward_difference(a.speed * a.yaw.cos(), b.speed * b.yaw.cos(), dt);
            let dvy = forward_difference(a.speed * a.yaw.sin(), b.speed * b.yaw.sin(), dt);
            -a.yaw.sin() * dvx + a.yaw.cos() * dvy
        }),
        CarpetVisualization::Curvature => padded_forward(&states, |a, b| {
            wrap_angle(b.yaw - a.yaw) / (a.speed.abs().max(0.1) * dt)
        }),
        _ => vec![0.0; states.len()],
    };
    let range = match visualization {
        CarpetVisualization::Speed => (0.0, *MAX_TERMINAL_SPEED_MPS),
        CarpetVisualization::Time => (
            0.0,
            (states.len().saturating_sub(1) as f64 * dt).max(f64::EPSILON),
        ),
        CarpetVisualization::LongitudinalAcceleration => (MIN_LON_ACCEL, MAX_LON_ACCEL),
        CarpetVisualization::LateralAcceleration => (-MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL),
        CarpetVisualization::Curvature => (-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE),
        _ => (0.0, 1.0),
    };
    raw.into_iter()
        .map(|value| ((value - range.0) / (range.1 - range.0)).clamp(0.0, 1.0))
        .collect()
}

fn padded_forward(states: &[State], f: impl Fn(&State, &State) -> f64) -> Vec<f64> {
    let mut result: Vec<_> = states
        .windows(2)
        .map(|pair| f(&pair[0], &pair[1]))
        .collect();
    result.push(result.last().copied().unwrap_or(0.0));
    result
}

fn sample_footprints(ego: State, plan: &[State], dt: f64) -> Vec<TimedState> {
    let mut samples = Vec::new();
    samples.push(TimedState {
        state: ego,
        time: 0.0,
    });
    let mut previous = ego;
    for (i, &next) in plan.iter().enumerate() {
        let translation = (next.x - previous.x).hypot(next.y - previous.y);
        let yaw_delta = wrap_angle(next.yaw - previous.yaw).abs();
        let corner_sweep = yaw_delta * 0.5 * EGO_FOOTPRINT.length.hypot(EGO_FOOTPRINT.width);
        let steps = ((translation + corner_sweep) / (0.5 * BAND_M))
            .ceil()
            .max(1.0) as usize;
        for step in 1..=steps {
            let alpha = step as f64 / steps as f64;
            samples.push(TimedState {
                state: interpolate_state(previous, next, alpha),
                time: (i as f64 + alpha) * dt,
            });
        }
        previous = next;
    }
    samples
}

fn carpet_patches(footprints: &[TimedState]) -> Vec<CarpetPatch> {
    footprint_stations(footprints)
        .into_iter()
        .filter_map(|station| cross_section(station, footprints))
        .collect::<Vec<_>>()
        .windows(2)
        .map(|sections| CarpetPatch {
            rear: sections[0],
            front: sections[1],
            time: 0.5 * (sections[0].time + sections[1].time),
        })
        .collect()
}

fn footprint_stations(footprints: &[TimedState]) -> Vec<State> {
    let Some(first) = footprints.first() else {
        return vec![];
    };
    let mut path: Vec<_> = footprints.iter().map(|sample| sample.state).collect();
    let mut terminal_front = footprints.last().unwrap().state;
    terminal_front.x += EGO_FOOTPRINT.length * terminal_front.yaw.cos();
    terminal_front.y += EGO_FOOTPRINT.length * terminal_front.yaw.sin();
    path.push(terminal_front);

    let mut stations = vec![first.state];
    let mut traversed = 0.0;
    let mut next_station = BAND_M;
    for pair in path.windows(2) {
        let distance = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
        while distance > f64::EPSILON && next_station <= traversed + distance {
            stations.push(interpolate_state(
                pair[0],
                pair[1],
                (next_station - traversed) / distance,
            ));
            next_station += BAND_M;
        }
        traversed += distance;
    }
    if stations.last().is_none_or(|station| {
        (station.x - terminal_front.x).hypot(station.y - terminal_front.y) > FOOTPRINT_EPSILON_M
    }) {
        stations.push(terminal_front);
    }
    stations
}

fn cross_section(station: State, footprints: &[TimedState]) -> Option<CrossSection> {
    let forward = [station.yaw.cos(), station.yaw.sin()];
    let left = [-forward[1], forward[0]];
    let mut right = f64::INFINITY;
    let mut leftmost = f64::NEG_INFINITY;
    let mut total_time = 0.0;
    let mut occupants = 0;

    for footprint in footprints {
        let footprint_corners = EGO_FOOTPRINT.corners(footprint.state.pose());
        let corners = [
            footprint_corners[0],
            footprint_corners[1],
            footprint_corners[3],
            footprint_corners[2],
        ];
        let local = corners.map(|point| {
            let delta = [point[0] - station.x, point[1] - station.y];
            [
                delta[0] * forward[0] + delta[1] * forward[1],
                delta[0] * left[0] + delta[1] * left[1],
            ]
        });
        let mut intersections = Vec::with_capacity(4);
        for i in 0..4 {
            let a = local[i];
            let b = local[(i + 1) % 4];
            if a[0].abs() <= FOOTPRINT_EPSILON_M {
                intersections.push(a[1]);
            }
            if (a[0] < -FOOTPRINT_EPSILON_M && b[0] > FOOTPRINT_EPSILON_M)
                || (a[0] > FOOTPRINT_EPSILON_M && b[0] < -FOOTPRINT_EPSILON_M)
            {
                let alpha = -a[0] / (b[0] - a[0]);
                intersections.push(a[1] + alpha * (b[1] - a[1]));
            }
        }
        if intersections.len() >= 2 {
            right = right.min(intersections.iter().copied().fold(f64::INFINITY, f64::min));
            leftmost = leftmost.max(
                intersections
                    .iter()
                    .copied()
                    .fold(f64::NEG_INFINITY, f64::max),
            );
            total_time += footprint.time;
            occupants += 1;
        }
    }
    (occupants > 0).then(|| CrossSection {
        right: [station.x + right * left[0], station.y + right * left[1]],
        left: [
            station.x + leftmost * left[0],
            station.y + leftmost * left[1],
        ],
        time: total_time / occupants as f64,
    })
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

fn empty_mesh() -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0; 3]; 3]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, vec![[0.0; 4]; 3]);
    mesh
}

fn push_patch(
    vertices: &mut Vec<[f32; 3]>,
    colors: &mut Vec<[f32; 4]>,
    patch: CarpetPatch,
    color: Color,
) {
    let point = |point: [f64; 2]| [point[0] as f32 * PX_PER_M, point[1] as f32 * PX_PER_M, 0.0];
    vertices.extend([
        point(patch.rear.left),
        point(patch.rear.right),
        point(patch.front.right),
        point(patch.rear.left),
        point(patch.front.right),
        point(patch.front.left),
    ]);
    colors.extend(std::iter::repeat_n(color.to_linear().to_f32_array(), 6));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viewer::DT;

    #[test]
    fn samples_overlap_even_at_terminal_speed() {
        let speed = *MAX_TERMINAL_SPEED_MPS;
        let ego = State {
            speed,
            ..Default::default()
        };
        let samples = sample_footprints(
            ego,
            &[State {
                x: speed * DT,
                speed,
                ..Default::default()
            }],
            DT,
        );

        assert!(samples.windows(2).all(|pair| {
            (pair[1].state.x - pair[0].state.x).hypot(pair[1].state.y - pair[0].state.y)
                <= EGO_FOOTPRINT.length * 0.5 + 1e-12
        }));
    }

    #[test]
    fn uses_mean_time_for_repeated_occupancy() {
        let footprints = [
            TimedState {
                state: State::default(),
                time: 0.0,
            },
            TimedState {
                state: State::default(),
                time: 2.0,
            },
        ];
        let patches = carpet_patches(&footprints);

        assert!(!patches.is_empty());
        assert!(patches.iter().all(|patch| patch.time == 1.0));
    }

    #[test]
    fn covers_rotated_footprints() {
        let state = State {
            yaw: 0.7,
            ..Default::default()
        };
        let patches = carpet_patches(&[TimedState { state, time: 0.0 }]);
        let rear = patches.first().unwrap().rear;
        let width = (rear.left[0] - rear.right[0]).hypot(rear.left[1] - rear.right[1]);

        assert!((width - EGO_FOOTPRINT.width).abs() < 1e-9);
    }

    #[test]
    fn carpet_includes_the_entire_current_and_terminal_footprints() {
        let terminal = State {
            x: EGO_FOOTPRINT.length * 2.0,
            ..Default::default()
        };
        let footprints = sample_footprints(State::default(), &[terminal], 1.0);

        let patches = carpet_patches(&footprints);
        let rear = patches
            .iter()
            .flat_map(|patch| [patch.rear.right[0], patch.rear.left[0]])
            .fold(f64::INFINITY, f64::min);
        let front = patches
            .iter()
            .flat_map(|patch| [patch.front.right[0], patch.front.left[0]])
            .fold(f64::NEG_INFINITY, f64::max);

        assert!(!patches.is_empty());
        assert!(rear <= FOOTPRINT_EPSILON_M);
        assert!(front >= terminal.x + EGO_FOOTPRINT.length - FOOTPRINT_EPSILON_M);
    }

    #[test]
    fn carpet_patches_are_disjoint() {
        use crate::geometry::polygons_overlap;

        let plan = [
            State::default(),
            State {
                x: 2.0,
                y: 1.0,
                yaw: 0.4,
                ..Default::default()
            },
        ];
        let footprints = sample_footprints(State::default(), &plan, 0.5);
        let patches = carpet_patches(&footprints);

        assert!(patches.windows(2).all(|pair| {
            pair[0].front.right == pair[1].rear.right && pair[0].front.left == pair[1].rear.left
        }));
        for i in 0..patches.len() {
            let a = [
                patches[i].rear.right,
                patches[i].rear.left,
                patches[i].front.left,
                patches[i].front.right,
            ];
            for b in patches.iter().skip(i + 2) {
                let b = [b.rear.right, b.rear.left, b.front.left, b.front.right];
                assert!(!polygons_overlap(&a, &b));
            }
        }
    }

    #[test]
    fn every_signal_visualization_is_normalized_for_each_planned_tick() {
        let ego = State::default();
        let plan = [State { speed: 2.0, ..ego }, State { speed: 3.0, ..ego }];
        for visualization in [
            CarpetVisualization::Speed,
            CarpetVisualization::Time,
            CarpetVisualization::LongitudinalAcceleration,
            CarpetVisualization::LateralAcceleration,
            CarpetVisualization::Curvature,
        ] {
            let values = visualization_values(ego, &plan, DT, visualization, None);
            assert_eq!(values.len(), plan.len());
            assert!(values.iter().all(|value| (0.0..=1.0).contains(value)));
        }
        assert_eq!(
            visualization_values(ego, &plan, DT, CarpetVisualization::Time, None),
            [0.0, 1.0]
        );
    }

    #[test]
    fn carpet_colormaps_match_metric_signedness() {
        assert_eq!(GUPPY.at(0.0).to_rgba8()[..3], [254, 107, 44]);
        assert_eq!(GUPPY.at(1.0).to_rgba8()[..3], [42, 182, 196]);
        assert_eq!(GUPPY_ORANGE.at(0.0).to_rgba8()[..3], [254, 107, 44]);
        assert_eq!(GUPPY_BLUE.at(0.0).to_rgba8()[..3], [42, 182, 196]);
        assert_eq!(GUPPY_ORANGE.at(1.0), GUPPY.at(0.5));
        assert_eq!(GUPPY_BLUE.at(1.0), GUPPY.at(0.5));
    }
}
