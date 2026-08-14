//! Signal-colored swept ego footprint.

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use colorgrad::Gradient;

use crate::common::interp::lerp_state;
use crate::common::kinematics::TrajectoryKinematics;
use crate::common::math::wrap_angle;
use crate::geometry::EGO_FOOTPRINT;
use crate::metrics::Metrics;
use crate::simulation::{MAX_TERMINAL_SPEED_MPS, Position, State};
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};
#[cfg(test)]
use crate::viewer::colors::GUPPY_ORANGE;
use crate::viewer::{
    CarpetVisualization,
    colors::{CARPET_ALPHA, GUPPY, GUPPY_BLUE},
};

use super::super::screen::PX_PER_M;
use super::config::EGO_CARPET_Z;

const BAND_M: f64 = 0.5;
#[cfg(test)]
const FOOTPRINT_EPSILON_M: f64 = 1e-9;
/// Maximum conservative allowance between rotating cross-sections.
const TURN_PADDING_M: f64 = 0.075;
/// Rotation, rather than translation, determines when an extra section is needed.
const MAX_YAW_STEP_RAD: f64 = 0.2 * BAND_M;

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
    center: Position,
    forward: Position,
    left: Position,
}

impl TimedState {
    fn new(state: State, time: f64, arc_m: f64) -> Self {
        let forward = Position::from_angle(state.pose.yaw);
        let left = Position::new(-forward.y, forward.x);
        Self {
            state,
            time,
            arc_m,
            center: Position::new(
                state.position().x + 0.5 * EGO_FOOTPRINT.length * forward.x,
                state.position().y + 0.5 * EGO_FOOTPRINT.length * forward.y,
            ),
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
    forward: Position,
    left: Position,
}

#[derive(Clone, Copy)]
struct CrossSection {
    right: Position,
    left: Position,
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
        Transform::from_xyz(0.0, 0.0, EGO_CARPET_Z),
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
    trajectory: &TrajectoryKinematics,
    visualization: CarpetVisualization,
    metrics: Option<&Metrics>,
) -> u64 {
    let plan = trajectory.states.get(1..).expect("carpet trajectory is non-empty");
    let footprints = sample_footprints(ego, plan, trajectory.dt);
    let (patches, intersection_clocks) = carpet_patches_clocked(&footprints);
    let values = visualization_values(trajectory, visualization, metrics);
    let colormap = match visualization {
        CarpetVisualization::Time => &*GUPPY_BLUE,
        CarpetVisualization::Speed => &*GUPPY_BLUE,
        _ => &*GUPPY,
    };

    let tick_colors = values
        .iter()
        .map(|value| {
            let sample = colormap.at(*value as f32);
            Color::srgba(sample.r, sample.g, sample.b, CARPET_ALPHA)
                .to_linear()
                .to_f32_array()
        })
        .collect::<Vec<_>>();
    let mut vertices = Vec::with_capacity(patches.len() * 6);
    let mut colors = Vec::with_capacity(vertices.capacity());
    let patch_count = patches.len();
    for patch in patches {
        let index = (patch.time / trajectory.dt).round() as usize;
        push_patch(
            &mut vertices,
            &mut colors,
            patch,
            tick_colors[index.min(tick_colors.len() - 1)],
        );
    }

    let mut mesh = empty_mesh();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    if let Some(mut existing) = meshes.get_mut(&carpet.handle) {
        *existing = mesh;
        carpet.populated = true;
    }
    footprints.len() as u64 + intersection_clocks + trajectory.len() as u64 + 2 * patch_count as u64
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
    trajectory: &TrajectoryKinematics,
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

    let raw = match visualization {
        CarpetVisualization::Speed => trajectory.states.iter().map(|state| state.speed).collect(),
        CarpetVisualization::Time => trajectory.time.clone(),
        CarpetVisualization::LongitudinalAcceleration => {
            trajectory.controls.iter().map(|control| control.acceleration).collect()
        }
        CarpetVisualization::LateralAcceleration => trajectory.lateral_acceleration.clone(),
        CarpetVisualization::Curvature => trajectory.controls.iter().map(|control| control.curvature).collect(),
        _ => vec![0.0; trajectory.len()],
    };
    let range = match visualization {
        CarpetVisualization::Speed => (0.0, *MAX_TERMINAL_SPEED_MPS),
        CarpetVisualization::Time => (0.0, trajectory.time.last().copied().unwrap_or(0.0).max(f64::EPSILON)),
        CarpetVisualization::LongitudinalAcceleration => (MIN_LON_ACCEL, MAX_LON_ACCEL),
        CarpetVisualization::LateralAcceleration => (-MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL),
        CarpetVisualization::Curvature => (-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE),
        _ => (0.0, 1.0),
    };
    raw.into_iter()
        .map(|value| ((value - range.0) / (range.1 - range.0)).clamp(0.0, 1.0))
        .collect()
}

fn sample_footprints(ego: State, plan: &[State], dt: f64) -> Vec<TimedState> {
    let mut samples = Vec::new();
    samples.push(TimedState::new(ego, 0.0, 0.0));
    let mut previous = ego;
    let mut arc_m = 0.0;
    for (i, &next) in plan.iter().enumerate() {
        let translation = next.position().distance(previous.position());
        let yaw_delta = wrap_angle(next.pose.yaw - previous.pose.yaw).abs();
        // Translation alone needs no subdivision: the trapezoid between two
        // equal-heading footprints is the exact straight sweep. Refine only
        // rotation, where the corners follow arcs rather than straight lines.
        let steps = (yaw_delta / MAX_YAW_STEP_RAD - 1e-12).ceil().max(1.0) as usize;
        for step in 1..=steps {
            let alpha = step as f64 / steps as f64;
            samples.push(TimedState::new(
                lerp_state(previous, next, alpha),
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
    let forward = Position::from_angle(terminal_front.pose.yaw);
    terminal_front.pose.position.x += EGO_FOOTPRINT.length * forward.x;
    terminal_front.pose.position.y += EGO_FOOTPRINT.length * forward.y;
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
        let distance = |a: Station, b: Station| a.state.position().distance(b.state.position());
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
        let turns_from_previous =
            i > 0 && wrap_angle(stations[i].state.pose.yaw - stations[i - 1].state.pose.yaw).abs() > f64::EPSILON;
        let turns_to_next = stations
            .get(i + 1)
            .is_some_and(|next| wrap_angle(next.state.pose.yaw - stations[i].state.pose.yaw).abs() > f64::EPSILON);
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
        right: Position::new(
            station.state.position().x + (right - padding_m) * station.left.x,
            station.state.position().y + (right - padding_m) * station.left.y,
        ),
        left: Position::new(
            station.state.position().x + (leftmost + padding_m) * station.left.x,
            station.state.position().y + (leftmost + padding_m) * station.left.y,
        ),
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
        footprint.center.x - station.state.position().x,
        footprint.center.y - station.state.position().y,
    ];
    let project = |axis: Position| delta[0] * axis.x + delta[1] * axis.y;
    let cos_relative = footprint.forward.x * station.forward.x + footprint.forward.y * station.forward.y;
    let sin_relative = footprint.forward.x * station.left.x + footprint.forward.y * station.left.y;
    let longitudinal_radius =
        0.5 * (EGO_FOOTPRINT.length * cos_relative.abs() + EGO_FOOTPRINT.width * sin_relative.abs());
    let longitudinal = project(station.forward);
    (longitudinal - longitudinal_radius <= station.slab_m && longitudinal + longitudinal_radius >= -station.slab_m)
        .then(|| {
            let lateral = project(station.left);
            let lateral_radius =
                0.5 * (EGO_FOOTPRINT.length * sin_relative.abs() + EGO_FOOTPRINT.width * cos_relative.abs());
            [lateral - lateral_radius, lateral + lateral_radius]
        })
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

fn push_patch(vertices: &mut Vec<[f32; 3]>, colors: &mut Vec<[f32; 4]>, patch: CarpetPatch, color: [f32; 4]) {
    let point = |point: Position| [point.x as f32 * PX_PER_M, point.y as f32 * PX_PER_M, 0.0];
    vertices.extend([
        point(patch.rear.left),
        point(patch.rear.right),
        point(patch.front.right),
        point(patch.rear.left),
        point(patch.front.right),
        point(patch.front.left),
    ]);
    colors.extend(std::iter::repeat_n(color, 6));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{Latency, LatencyStats};
    use crate::simulation::Control;
    use crate::viewer::DT;
    use bevy::mesh::VertexAttributeValues;

    fn trajectory(ego: State, plan: &[State], dt: f64) -> TrajectoryKinematics {
        let states: Vec<_> = std::iter::once(ego).chain(plan.iter().copied()).collect();
        let len = states.len();
        TrajectoryKinematics::new(states, vec![Control::default(); len], dt)
    }

    #[test]
    fn straight_motion_uses_temporal_samples() {
        let speed = *MAX_TERMINAL_SPEED_MPS;
        let ego = State {
            speed,
            ..Default::default()
        };
        let samples = sample_footprints(ego, &[State::from((Position::new(speed * DT, 0.0), 0.0, speed))], DT);

        assert_eq!(samples.len(), 2);
    }

    #[test]
    fn draw_starts_at_the_rendered_ego_footprint() {
        let planned_ego = State::from((Position::new(2.0, 0.0), 0.0, 1.0));
        let rendered_ego = State::from((Position::new(1.0, 0.0), 0.0, 1.0));
        let plan = [State::from((Position::new(4.0, 0.0), 0.0, 1.0))];
        let mut meshes = Assets::<Mesh>::default();
        let mut carpet = EgoCarpetMesh {
            handle: meshes.add(empty_mesh()),
            populated: false,
        };

        draw(
            &mut meshes,
            &mut carpet,
            rendered_ego,
            &trajectory(planned_ego, &plan, 1.0),
            CarpetVisualization::Time,
            None,
        );

        let mesh = meshes.get(&carpet.handle).unwrap();
        let VertexAttributeValues::Float32x3(vertices) = mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() else {
            panic!("carpet positions have the wrong format");
        };
        let rear = vertices.iter().map(|vertex| vertex[0]).fold(f32::INFINITY, f32::min);

        assert!((rear - PX_PER_M).abs() < 1e-3);
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
        let state = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 0.0), 0.7),
            0.0,
        );
        let patches = carpet_patches(&[TimedState::new(state, 0.0, 0.0)]);
        let rear = patches.first().unwrap().rear;
        let width = rear.left.distance(rear.right);

        assert!((width - EGO_FOOTPRINT.width).abs() < 1e-9);
    }

    #[test]
    fn carpet_includes_the_entire_current_and_terminal_footprints() {
        let terminal = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(EGO_FOOTPRINT.length * 2.0, 0.0), 0.0),
            0.0,
        );
        let footprints = sample_footprints(State::default(), &[terminal], 1.0);

        let patches = carpet_patches(&footprints);
        let rear = patches
            .iter()
            .flat_map(|patch| [patch.rear.right.x, patch.rear.left.x])
            .fold(f64::INFINITY, f64::min);
        let front = patches
            .iter()
            .flat_map(|patch| [patch.front.right.x, patch.front.left.x])
            .fold(f64::NEG_INFINITY, f64::max);

        assert!(!patches.is_empty());
        assert!(rear <= FOOTPRINT_EPSILON_M);
        assert!(front >= terminal.position().x + EGO_FOOTPRINT.length - FOOTPRINT_EPSILON_M);
    }

    #[test]
    fn carpet_patches_are_disjoint() {
        use crate::geometry::polygons_overlap;

        let plan = [
            State::default(),
            State::new(
                crate::simulation::Pose::new(crate::simulation::Position::new(2.0, 1.0), 0.4),
                0.0,
            ),
        ];
        let footprints = sample_footprints(State::default(), &plan, 0.5);
        let patches = carpet_patches(&footprints);

        assert!(
            patches
                .windows(2)
                .all(|pair| { pair[0].front.right == pair[1].rear.right && pair[0].front.left == pair[1].rear.left })
        );
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
            .map(|x| {
                State::new(
                    crate::simulation::Pose::new(crate::simulation::Position::new(x as f64, 0.0), 0.0),
                    0.0,
                )
            })
            .collect::<Vec<_>>();
        plan.extend((1..=20).map(|i| {
            let angle = -std::f64::consts::FRAC_PI_2 + i as f64 / 20.0 * std::f64::consts::PI;
            let unit = Position::from_angle(angle);
            State::new(
                crate::simulation::Pose::new(
                    Position::new(20.0 + radius * unit.x, radius + radius * unit.y),
                    angle + std::f64::consts::FRAC_PI_2,
                ),
                0.0,
            )
        }));
        plan.extend((0..20).rev().map(|x| {
            State::new(
                crate::simulation::Pose::new(
                    crate::simulation::Position::new(x as f64, 2.0 * radius),
                    std::f64::consts::PI,
                ),
                0.0,
            )
        }));

        let footprints = sample_footprints(State::default(), &plan, 0.1);
        let patch = carpet_patches(&footprints)
            .into_iter()
            .find(|patch| {
                let x = 0.25 * (patch.rear.left.x + patch.rear.right.x + patch.front.left.x + patch.front.right.x);
                let y = 0.25 * (patch.rear.left.y + patch.rear.right.y + patch.front.left.y + patch.front.right.y);
                (x - 10.5).abs() < BAND_M && y.abs() < BAND_M
            })
            .expect("outbound leg must contain a patch near x=10");

        assert!(
            [
                patch.rear.left.y,
                patch.rear.right.y,
                patch.front.left.y,
                patch.front.right.y,
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
                let unit = Position::from_angle(angle);
                State::new(
                    crate::simulation::Pose::new(Position::new(8.0 * unit.y, 8.0 * (1.0 - unit.x)), angle),
                    0.0,
                )
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

    #[test]
    fn sharp_turn_sections_limit_yaw_facets() {
        let radius = 1.0 / MAX_ABS_CURVATURE;
        let yaw_step = 7.0 * MAX_ABS_CURVATURE * DT;
        let plan = (1..=20)
            .map(|i| {
                let yaw = i as f64 * yaw_step;
                let forward = Position::from_angle(yaw);
                State::from((Position::new(radius * forward.y, radius * (1.0 - forward.x)), yaw, 7.0))
            })
            .collect::<Vec<_>>();
        let footprints = sample_footprints(
            State {
                speed: 7.0,
                ..Default::default()
            },
            &plan,
            DT,
        );

        assert!(footprints.windows(2).all(|pair| {
            wrap_angle(pair[1].state.pose.yaw - pair[0].state.pose.yaw).abs() <= MAX_YAW_STEP_RAD + f64::EPSILON
        }));
    }

    fn assert_carpet_contains(ego: State, plan: &[State]) {
        let footprints = sample_footprints(ego, plan, DT);
        let patches = carpet_patches(&footprints);
        let states = footprints.iter().map(|sample| sample.state).chain(
            footprints
                .windows(2)
                .flat_map(|pair| [0.25, 0.5, 0.75].map(|alpha| lerp_state(pair[0].state, pair[1].state, alpha))),
        );

        for state in states {
            let forward = Position::from_angle(state.pose.yaw);
            let left = Position::new(-forward.y, forward.x);
            for longitudinal in [0.0, 0.25, 0.5, 0.75, 1.0] {
                for lateral in [-0.5, 0.0, 0.5] {
                    let point = Position::new(
                        state.position().x
                            + longitudinal * EGO_FOOTPRINT.length * forward.x
                            + lateral * EGO_FOOTPRINT.width * left.x,
                        state.position().y
                            + longitudinal * EGO_FOOTPRINT.length * forward.y
                            + lateral * EGO_FOOTPRINT.width * left.y,
                    );
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
            .map(|i| {
                State::new(
                    crate::simulation::Pose::new(crate::simulation::Position::new(i as f64 * 10.0, 0.0), 0.0),
                    0.0,
                )
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
        for ((name, ego, plan), expected) in carpet_workloads().into_iter().zip([607, 5_377, 907]) {
            let mut meshes = Assets::<Mesh>::default();
            let mut carpet = EgoCarpetMesh {
                handle: meshes.add(empty_mesh()),
                populated: false,
            };
            let clocks = draw(
                &mut meshes,
                &mut carpet,
                ego,
                &trajectory(ego, &plan, DT),
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
                .map(|tick| {
                    State::new(
                        crate::simulation::Pose::new(
                            crate::simulation::Position::new(tick as f64 * terminal_speed * DT, 0.0),
                            0.0,
                        ),
                        terminal_speed,
                    )
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
                &trajectory(ego, plan, DT),
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
                    &trajectory(ego, plan, DT),
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
                    let trajectory = trajectory(ego, plan, DT);
                    let values = visualization_values(&trajectory, CarpetVisualization::Time, None);
                    recorder.work(plan.len() as u64);
                    values
                });
                recorder.time("carpet.mesh", || {
                    let tick_colors = values
                        .iter()
                        .map(|value| {
                            let sample = GUPPY_BLUE.at(*value as f32);
                            Color::srgba(sample.r, sample.g, sample.b, CARPET_ALPHA)
                                .to_linear()
                                .to_f32_array()
                        })
                        .collect::<Vec<_>>();
                    let mut vertices = Vec::with_capacity(patches.len() * 6);
                    let mut colors = Vec::with_capacity(vertices.capacity());
                    for patch in patches {
                        let index = (patch.time / DT).round() as usize;
                        push_patch(
                            &mut vertices,
                            &mut colors,
                            patch,
                            tick_colors[index.min(tick_colors.len() - 1)],
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
                let mid_yaw = state.pose.yaw + 0.5 * yaw_delta;
                let forward = Position::from_angle(mid_yaw);
                state.pose.position.x += speed * DT * forward.x;
                state.pose.position.y += speed * DT * forward.y;
                state.pose.yaw = wrap_angle(state.pose.yaw + yaw_delta);
                state
            })
            .collect()
    }

    #[test]
    fn every_signal_visualization_is_normalized_for_each_planned_tick() {
        let ego = State::default();
        let plan = [State { speed: 2.0, ..ego }, State { speed: 3.0, ..ego }];
        let trajectory = TrajectoryKinematics::new(
            vec![ego, plan[1]],
            vec![
                Control {
                    acceleration: -1.0,
                    curvature: -0.01,
                },
                Control {
                    acceleration: 2.0,
                    curvature: 0.02,
                },
            ],
            DT,
        );
        for visualization in [
            CarpetVisualization::Speed,
            CarpetVisualization::Time,
            CarpetVisualization::LongitudinalAcceleration,
            CarpetVisualization::LateralAcceleration,
            CarpetVisualization::Curvature,
        ] {
            let values = visualization_values(&trajectory, visualization, None);
            assert_eq!(values.len(), plan.len());
            assert!(values.iter().all(|value| (0.0..=1.0).contains(value)));
        }
        let normalize = |value: f64, min: f64, max: f64| (value - min) / (max - min);
        assert_eq!(
            visualization_values(&trajectory, CarpetVisualization::LongitudinalAcceleration, None),
            [
                normalize(-1.0, MIN_LON_ACCEL, MAX_LON_ACCEL),
                normalize(2.0, MIN_LON_ACCEL, MAX_LON_ACCEL),
            ]
        );
        assert_eq!(
            visualization_values(&trajectory, CarpetVisualization::LateralAcceleration, None),
            [
                normalize(0.0, -MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL),
                normalize(0.18, -MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL),
            ]
        );
        assert_eq!(
            visualization_values(&trajectory, CarpetVisualization::Time, None),
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

    fn point_in_patch(point: Position, patch: &CarpetPatch) -> bool {
        let triangles = [
            [patch.rear.left, patch.rear.right, patch.front.right],
            [patch.rear.left, patch.front.right, patch.front.left],
        ];
        triangles.into_iter().any(|triangle| {
            let cross = |a: Position, b: Position| (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
            let signs = [
                cross(triangle[0], triangle[1]),
                cross(triangle[1], triangle[2]),
                cross(triangle[2], triangle[0]),
            ];
            signs.iter().all(|value| *value >= -1e-9) || signs.iter().all(|value| *value <= 1e-9)
        })
    }
}
