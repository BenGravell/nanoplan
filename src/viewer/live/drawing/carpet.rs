//! Signal-colored swept ego footprint.

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
#[cfg(test)]
const FOOTPRINT_EPSILON_M: f64 = 1e-9;
/// Maximum conservative allowance between rotating cross-sections.
const TURN_PADDING_M: f64 = 0.075;
/// Rotation, rather than translation, determines when an extra section is needed.
const MAX_YAW_STEP_RAD: f64 = 0.3 * BAND_M;

#[derive(Resource)]
pub(crate) struct EgoCarpetMesh {
    handle: Handle<Mesh>,
    populated: bool,
}

#[derive(Clone, Copy)]
struct TimedState {
    state: State,
    time: f64,
    arc_m: f64,
    center: [f64; 2],
    forward: [f64; 2],
    left: [f64; 2],
}

impl TimedState {
    fn new(state: State, time: f64, arc_m: f64) -> Self {
        let forward = [state.yaw.cos(), state.yaw.sin()];
        let left = [-forward[1], forward[0]];
        Self {
            state,
            time,
            arc_m,
            center: [
                state.x + 0.5 * EGO_FOOTPRINT.length * forward[0],
                state.y + 0.5 * EGO_FOOTPRINT.length * forward[1],
            ],
            forward,
            left,
        }
    }
}

#[derive(Clone, Copy)]
struct Station {
    state: State,
    arc_m: f64,
    slab_m: f64,
    padding_m: f64,
    turn_boundary: bool,
    forward: [f64; 2],
    left: [f64; 2],
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
) -> u64 {
    let footprints = sample_footprints(ego, plan, dt);
    let (patches, intersection_clocks) = carpet_patches_clocked(&footprints);
    let values = visualization_values(ego, plan, dt, visualization, metrics);
    let colormap = match visualization {
        CarpetVisualization::Time => &*GUPPY_BLUE,
        CarpetVisualization::Speed => &*GUPPY_BLUE,
        _ => &*GUPPY,
    };

    let mut vertices = Vec::with_capacity(patches.len() * 6);
    let mut colors = Vec::with_capacity(vertices.capacity());
    let patch_count = patches.len();
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
    footprints.len() as u64 + intersection_clocks + plan.len() as u64 + 2 * patch_count as u64
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
    samples.push(TimedState::new(ego, 0.0, 0.0));
    let mut previous = ego;
    let mut arc_m = 0.0;
    for (i, &next) in plan.iter().enumerate() {
        let translation = (next.x - previous.x).hypot(next.y - previous.y);
        let yaw_delta = wrap_angle(next.yaw - previous.yaw).abs();
        // Translation alone needs no subdivision: the trapezoid between two
        // equal-heading footprints is the exact straight sweep. Refine only
        // rotation, where the corners follow arcs rather than straight lines.
        let steps = (yaw_delta / MAX_YAW_STEP_RAD - 1e-12).ceil().max(1.0) as usize;
        for step in 1..=steps {
            let alpha = step as f64 / steps as f64;
            samples.push(TimedState::new(
                interpolate_state(previous, next, alpha),
                (i as f64 + alpha) * dt,
                arc_m + alpha * translation,
            ));
        }
        arc_m += translation;
        previous = next;
    }
    samples
}

#[cfg(test)]
fn carpet_patches(footprints: &[TimedState]) -> Vec<CarpetPatch> {
    carpet_patches_clocked(footprints).0
}

fn carpet_patches_clocked(footprints: &[TimedState]) -> (Vec<CarpetPatch>, u64) {
    let mut intersection_clocks = 0;
    let sections = footprint_stations(footprints)
        .into_iter()
        .map(|station| {
            let (section, clocks) = cross_section(station, footprints);
            intersection_clocks += clocks;
            section
        })
        .collect::<Vec<_>>();
    let patches = sections
        .windows(2)
        .filter_map(|sections| {
            let (Some(rear), Some(front)) = (sections[0], sections[1]) else {
                return None;
            };
            Some(CarpetPatch {
                rear,
                front,
                time: 0.5 * (rear.time + front.time),
            })
        })
        .collect();
    (patches, intersection_clocks)
}

fn footprint_stations(footprints: &[TimedState]) -> Vec<Station> {
    if footprints.is_empty() {
        return vec![];
    }
    let mut stations: Vec<_> = footprints
        .iter()
        .map(|sample| Station {
            state: sample.state,
            arc_m: sample.arc_m,
            slab_m: 0.0,
            padding_m: 0.0,
            turn_boundary: false,
            forward: sample.forward,
            left: sample.left,
        })
        .collect();
    let mut terminal_front = footprints.last().unwrap().state;
    terminal_front.x += EGO_FOOTPRINT.length * terminal_front.yaw.cos();
    terminal_front.y += EGO_FOOTPRINT.length * terminal_front.yaw.sin();
    stations.push(Station {
        state: terminal_front,
        arc_m: footprints.last().unwrap().arc_m + EGO_FOOTPRINT.length,
        slab_m: 0.0,
        padding_m: 0.0,
        turn_boundary: false,
        forward: footprints.last().unwrap().forward,
        left: footprints.last().unwrap().left,
    });
    for i in 0..stations.len() {
        let distance =
            |a: Station, b: Station| (a.state.x - b.state.x).hypot(a.state.y - b.state.y);
        let previous = i
            .checked_sub(1)
            .map(|previous| distance(stations[previous], stations[i]))
            .unwrap_or(0.0);
        let next = stations
            .get(i + 1)
            .map(|next| distance(stations[i], *next))
            .unwrap_or(0.0);
        stations[i].slab_m = if i == 0 {
            next
        } else if i + 1 == stations.len() {
            previous
        } else {
            0.5 * previous.max(next)
        };
        let turns_from_previous = i > 0
            && wrap_angle(stations[i].state.yaw - stations[i - 1].state.yaw).abs() > f64::EPSILON;
        let turns_to_next = stations.get(i + 1).is_some_and(|next| {
            wrap_angle(next.state.yaw - stations[i].state.yaw).abs() > f64::EPSILON
        });
        stations[i].turn_boundary = turns_from_previous || turns_to_next;
    }
    let padding_reach = EGO_FOOTPRINT.length + EGO_FOOTPRINT.width;
    let mut previous_turn_arc = f64::NEG_INFINITY;
    for station in &mut stations {
        if station.turn_boundary {
            previous_turn_arc = station.arc_m;
        }
        if station.arc_m - previous_turn_arc <= padding_reach + station.slab_m {
            station.padding_m = TURN_PADDING_M;
        }
    }
    let mut next_turn_arc = f64::INFINITY;
    for station in stations.iter_mut().rev() {
        if station.turn_boundary {
            next_turn_arc = station.arc_m;
        }
        if next_turn_arc - station.arc_m <= padding_reach + station.slab_m {
            station.padding_m = TURN_PADDING_M;
        }
    }
    stations
}

fn cross_section(station: Station, footprints: &[TimedState]) -> (Option<CrossSection>, u64) {
    let mut right = f64::INFINITY;
    let mut leftmost = f64::NEG_INFINITY;
    let mut total_time = 0.0;
    let mut occupants = 0;
    let padding_m = station.padding_m;

    let local = local_footprints(station, footprints);
    for footprint in local {
        let Some(interval) = footprint_lateral_interval(station, footprint) else {
            continue;
        };
        right = right.min(interval[0]);
        leftmost = leftmost.max(interval[1]);
        total_time += footprint.time;
        occupants += 1;
    }
    let section = (occupants > 0).then(|| CrossSection {
        right: [
            station.state.x + (right - padding_m) * station.left[0],
            station.state.y + (right - padding_m) * station.left[1],
        ],
        left: [
            station.state.x + (leftmost + padding_m) * station.left[0],
            station.state.y + (leftmost + padding_m) * station.left[1],
        ],
        time: total_time / occupants as f64,
    });
    (section, local.len() as u64)
}

fn local_footprints(station: Station, footprints: &[TimedState]) -> &[TimedState] {
    // A station can be occupied by an earlier rear pose whose body extends
    // forward into it. A small forward margin covers lateral motion on bends.
    // The terminal slab is one body long and therefore naturally grows this
    // to the two-body window needed to catch a turning nose at the end.
    let behind_m = EGO_FOOTPRINT.length + 0.5 * EGO_FOOTPRINT.width + station.slab_m;
    let ahead_m = EGO_FOOTPRINT.width + station.slab_m;
    let start = footprints.partition_point(|sample| sample.arc_m < station.arc_m - behind_m);
    let end = footprints.partition_point(|sample| sample.arc_m <= station.arc_m + ahead_m);
    &footprints[start..end]
}

fn footprint_lateral_interval(station: Station, footprint: &TimedState) -> Option<[f64; 2]> {
    let delta = [
        footprint.center[0] - station.state.x,
        footprint.center[1] - station.state.y,
    ];
    let project = |axis: [f64; 2]| delta[0] * axis[0] + delta[1] * axis[1];
    let support = |axis: [f64; 2]| {
        0.5 * EGO_FOOTPRINT.length
            * (footprint.forward[0] * axis[0] + footprint.forward[1] * axis[1]).abs()
            + 0.5
                * EGO_FOOTPRINT.width
                * (footprint.left[0] * axis[0] + footprint.left[1] * axis[1]).abs()
    };
    let longitudinal = project(station.forward);
    let longitudinal_radius = support(station.forward);
    (longitudinal - longitudinal_radius <= station.slab_m
        && longitudinal + longitudinal_radius >= -station.slab_m)
        .then(|| {
            let lateral = project(station.left);
            let lateral_radius = support(station.left);
            [lateral - lateral_radius, lateral + lateral_radius]
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
    use crate::planning::{Latency, LatencyStats};
    use crate::viewer::DT;

    #[test]
    fn straight_motion_uses_temporal_samples() {
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

        assert_eq!(samples.len(), 2);
    }

    #[test]
    fn uses_mean_time_for_repeated_occupancy() {
        let footprints = [
            TimedState::new(State::default(), 0.0, 0.0),
            TimedState::new(State::default(), 2.0, 0.0),
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
        let patches = carpet_patches(&[TimedState::new(state, 0.0, 0.0)]);
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
    fn hairpin_does_not_join_its_opposite_legs() {
        let radius = 5.0;
        let mut plan = (1..=20)
            .map(|x| State {
                x: x as f64,
                ..Default::default()
            })
            .collect::<Vec<_>>();
        plan.extend((1..=20).map(|i| {
            let angle = -std::f64::consts::FRAC_PI_2 + i as f64 / 20.0 * std::f64::consts::PI;
            State {
                x: 20.0 + radius * angle.cos(),
                y: radius + radius * angle.sin(),
                yaw: angle + std::f64::consts::FRAC_PI_2,
                ..Default::default()
            }
        }));
        plan.extend((0..20).rev().map(|x| State {
            x: x as f64,
            y: 2.0 * radius,
            yaw: std::f64::consts::PI,
            ..Default::default()
        }));

        let footprints = sample_footprints(State::default(), &plan, 0.1);
        let patch = carpet_patches(&footprints)
            .into_iter()
            .find(|patch| {
                let x = 0.25
                    * (patch.rear.left[0]
                        + patch.rear.right[0]
                        + patch.front.left[0]
                        + patch.front.right[0]);
                let y = 0.25
                    * (patch.rear.left[1]
                        + patch.rear.right[1]
                        + patch.front.left[1]
                        + patch.front.right[1]);
                (x - 10.5).abs() < BAND_M && y.abs() < BAND_M
            })
            .expect("outbound leg must contain a patch near x=10");

        assert!(
            [
                patch.rear.left[1],
                patch.rear.right[1],
                patch.front.left[1],
                patch.front.right[1],
            ]
            .into_iter()
            .all(|y| y.abs() < 0.5 * radius)
        );
    }

    #[test]
    fn carpet_contains_every_sampled_and_intermediate_footprint() {
        let plan = (1..=30)
            .map(|i| {
                let angle = i as f64 * 0.04;
                State {
                    x: 8.0 * angle.sin(),
                    y: 8.0 * (1.0 - angle.cos()),
                    yaw: angle,
                    ..Default::default()
                }
            })
            .collect::<Vec<_>>();
        assert_carpet_contains(State::default(), &plan);
    }

    #[test]
    fn carpet_contains_max_curvature_and_mixed_workloads() {
        for (name, ego, plan) in carpet_workloads().into_iter().skip(1) {
            assert_carpet_contains(ego, &plan);
            assert!(!name.is_empty());
        }
    }

    fn assert_carpet_contains(ego: State, plan: &[State]) {
        let footprints = sample_footprints(ego, plan, DT);
        let patches = carpet_patches(&footprints);
        let states =
            footprints
                .iter()
                .map(|sample| sample.state)
                .chain(footprints.windows(2).flat_map(|pair| {
                    [0.25, 0.5, 0.75]
                        .map(|alpha| interpolate_state(pair[0].state, pair[1].state, alpha))
                }));

        for state in states {
            let forward = [state.yaw.cos(), state.yaw.sin()];
            let left = [-forward[1], forward[0]];
            for longitudinal in [0.0, 0.25, 0.5, 0.75, 1.0] {
                for lateral in [-0.5, 0.0, 0.5] {
                    let point = [
                        state.x
                            + longitudinal * EGO_FOOTPRINT.length * forward[0]
                            + lateral * EGO_FOOTPRINT.width * left[0],
                        state.y
                            + longitudinal * EGO_FOOTPRINT.length * forward[1]
                            + lateral * EGO_FOOTPRINT.width * left[1],
                    ];
                    assert!(
                        patches.iter().any(|patch| point_in_patch(point, patch)),
                        "footprint point {point:?} at {state:?} is outside the carpet"
                    );
                }
            }
        }
    }

    #[test]
    fn long_carpet_work_scales_linearly() {
        let plan = (1..=100)
            .map(|i| State {
                x: i as f64 * 10.0,
                ..Default::default()
            })
            .collect::<Vec<_>>();
        let footprints = sample_footprints(State::default(), &plan, 0.1);
        let stations = footprint_stations(&footprints);
        let candidate_checks: usize = stations
            .iter()
            .map(|station| local_footprints(*station, &footprints).len())
            .sum();

        assert!(candidate_checks < stations.len() * 64);
        assert!(candidate_checks * 20 < stations.len() * footprints.len());
    }

    /// Manual optimized-build profiles for long, curved, and mixed sweeps.
    ///
    /// `cargo test --release profiles_carpet_workloads -- --ignored --nocapture`
    #[test]
    #[ignore = "wall-clock carpet profile"]
    fn profiles_carpet_workloads() {
        for (name, ego, plan) in carpet_workloads() {
            profile_carpet(name, ego, &plan);
        }
    }

    #[test]
    fn carpet_logical_clocks_are_stable_across_workloads() {
        for ((name, ego, plan), expected) in carpet_workloads().into_iter().zip([606, 1_585, 906]) {
            let mut meshes = Assets::<Mesh>::default();
            let mut carpet = EgoCarpetMesh {
                handle: meshes.add(empty_mesh()),
                populated: false,
            };
            let clocks = draw(
                &mut meshes,
                &mut carpet,
                ego,
                &plan,
                DT,
                CarpetVisualization::Time,
                None,
            );
            assert_eq!(clocks, expected, "{name}");
        }
    }

    fn carpet_workloads() -> [(&'static str, State, Vec<State>); 3] {
        let terminal_speed = *MAX_TERMINAL_SPEED_MPS;
        let straight = (
            "terminal_straight",
            State {
                speed: terminal_speed,
                ..Default::default()
            },
            (1..=100)
                .map(|tick| State {
                    x: tick as f64 * terminal_speed * DT,
                    speed: terminal_speed,
                    ..Default::default()
                })
                .collect::<Vec<_>>(),
        );
        let max_curvature_chicane = (
            "max_curvature_chicane",
            State {
                speed: 7.0,
                ..Default::default()
            },
            integrated_plan(7.0, |tick| {
                if (tick / 20) % 2 == 0 {
                    MAX_ABS_CURVATURE
                } else {
                    -MAX_ABS_CURVATURE
                }
            }),
        );
        let mixed = (
            "mixed",
            State {
                speed: 20.0,
                ..Default::default()
            },
            integrated_plan(20.0, |tick| match tick % 30 {
                0..=9 => 0.0,
                10..=19 => 0.025,
                _ => -0.025,
            }),
        );

        [straight, max_curvature_chicane, mixed]
    }

    fn profile_carpet(name: &'static str, ego: State, plan: &[State]) {
        let mut meshes = Assets::<Mesh>::default();
        let mut carpet = EgoCarpetMesh {
            handle: meshes.add(empty_mesh()),
            populated: false,
        };
        for _ in 0..20 {
            draw(
                &mut meshes,
                &mut carpet,
                ego,
                plan,
                DT,
                CarpetVisualization::Time,
                None,
            );
        }

        let recorder = Latency::default();
        let mut stats = LatencyStats::default();
        for _ in 0..500 {
            recorder.time("visualization.ego_carpet", || {
                let clocks = draw(
                    &mut meshes,
                    &mut carpet,
                    ego,
                    plan,
                    DT,
                    CarpetVisualization::Time,
                    None,
                );
                recorder.work(clocks);
            });
            stats.absorb(recorder.take());
        }
        let seam = &stats.seams[0];
        eprintln!(
            "{name:<24} calls {} mean {:.3} ms max {:.3} ms clocks {:.1}/{}",
            seam.calls,
            seam.mean_ms(),
            seam.max_ms,
            seam.mean_clocks(),
            seam.max_clocks,
        );

        if name == "max_curvature_chicane" {
            let recorder = Latency::default();
            let mut stages = LatencyStats::default();
            for _ in 0..500 {
                let footprints = recorder.time("carpet.sample", || {
                    let footprints = sample_footprints(ego, plan, DT);
                    recorder.work(footprints.len() as u64);
                    footprints
                });
                let patches = recorder.time("carpet.sections", || {
                    let (patches, clocks) = carpet_patches_clocked(&footprints);
                    recorder.work(clocks);
                    patches
                });
                let patch_count = patches.len();
                let values = recorder.time("carpet.values", || {
                    let values =
                        visualization_values(ego, plan, DT, CarpetVisualization::Time, None);
                    recorder.work(plan.len() as u64);
                    values
                });
                recorder.time("carpet.mesh", || {
                    let mut vertices = Vec::with_capacity(patches.len() * 6);
                    let mut colors = Vec::with_capacity(vertices.capacity());
                    for patch in patches {
                        let index = (patch.time / DT).round() as usize;
                        let sample = GUPPY_BLUE.at(values[index.min(values.len() - 1)] as f32);
                        push_patch(
                            &mut vertices,
                            &mut colors,
                            patch,
                            Color::srgba(sample.r, sample.g, sample.b, CARPET_ALPHA),
                        );
                    }
                    let mut mesh = empty_mesh();
                    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
                    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
                    *meshes.get_mut(&carpet.handle).unwrap() = mesh;
                    recorder.work(2 * patch_count as u64);
                });
                stages.absorb(recorder.take());
            }
            for stage in &stages.seams {
                eprintln!(
                    "  {:<22} mean {:.3} ms max {:.3} ms clocks {:.1}/{}",
                    stage.name,
                    stage.mean_ms(),
                    stage.max_ms,
                    stage.mean_clocks(),
                    stage.max_clocks,
                );
            }
        }
    }

    fn integrated_plan(speed: f64, curvature: impl Fn(usize) -> f64) -> Vec<State> {
        let mut state = State {
            speed,
            ..Default::default()
        };
        (0..100)
            .map(|tick| {
                let yaw_delta = speed * curvature(tick) * DT;
                let mid_yaw = state.yaw + 0.5 * yaw_delta;
                state.x += speed * DT * mid_yaw.cos();
                state.y += speed * DT * mid_yaw.sin();
                state.yaw = wrap_angle(state.yaw + yaw_delta);
                state
            })
            .collect()
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

    fn point_in_patch(point: [f64; 2], patch: &CarpetPatch) -> bool {
        let triangles = [
            [patch.rear.left, patch.rear.right, patch.front.right],
            [patch.rear.left, patch.front.right, patch.front.left],
        ];
        triangles.into_iter().any(|triangle| {
            let cross = |a: [f64; 2], b: [f64; 2]| {
                (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])
            };
            let signs = [
                cross(triangle[0], triangle[1]),
                cross(triangle[1], triangle[2]),
                cross(triangle[2], triangle[0]),
            ];
            signs.iter().all(|value| *value >= -1e-9) || signs.iter().all(|value| *value <= 1e-9)
        })
    }
}
