use crate::geometry::RoadPolygon;
use crate::track::{ROAD_SAMPLE_STEP_M, Track};
use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;

use super::super::Live;
use super::super::screen::ppx;
use super::config::ROAD_SURFACE_Z;
use crate::viewer::colors::{
    ROAD_SURFACE, SUBDUED_TRACK_CENTERLINE, SUBDUED_TRACK_EDGE, TRACK_CENTERLINE, TRACK_EDGE,
    TRACK_STATION,
};

const VISIBLE_TRACK_BEHIND_M: f64 = 250.0;
const VISIBLE_TRACK_AHEAD_M: f64 = 750.0;

#[derive(Resource)]
pub(crate) struct RoadSurfaceMesh {
    handle: Handle<Mesh>,
    polygon: RoadPolygon,
}

pub(crate) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    live: NonSend<Live>,
) {
    let polygon = surface_polygon(&live.world.track, live.world.track_progress);
    let handle = meshes.add(surface_mesh(&polygon));
    commands.spawn((
        Mesh2d(handle.clone()),
        MeshMaterial2d(materials.add(ROAD_SURFACE)),
        Transform::from_xyz(0.0, 0.0, ROAD_SURFACE_Z),
    ));
    commands.insert_resource(RoadSurfaceMesh { handle, polygon });
}

pub(in crate::viewer::live) fn draw(
    gizmos: &mut Gizmos,
    meshes: &mut Assets<Mesh>,
    surface: &mut RoadSurfaceMesh,
    track: &Track,
    progress: f64,
    show_stations: bool,
    show_centerline: bool,
) {
    if let Some(length) = track.lap_length() {
        let polygon = track
            .road_polygon(0.0, length, ROAD_SAMPLE_STEP_M, true)
            .expect("track must form a valid road polygon");
        update_surface(meshes, surface, &polygon);
        draw_lines(
            gizmos,
            &polygon,
            true,
            show_centerline,
            SUBDUED_TRACK_EDGE,
            SUBDUED_TRACK_CENTERLINE,
        );
    }

    let polygon = track
        .road_polygon(
            progress - VISIBLE_TRACK_BEHIND_M,
            progress + VISIBLE_TRACK_AHEAD_M,
            ROAD_SAMPLE_STEP_M,
            false,
        )
        .expect("visible track must form a valid road polygon");

    if track.lap_length().is_none() {
        update_surface(meshes, surface, &polygon);
    }
    draw_lines(
        gizmos,
        &polygon,
        false,
        show_centerline,
        TRACK_EDGE,
        TRACK_CENTERLINE,
    );

    if show_stations {
        for (&right, &left) in polygon.right_boundary().iter().zip(polygon.left_boundary()) {
            gizmos.line_2d(ppx(right), ppx(left), TRACK_STATION);
        }
    }
}

fn surface_polygon(track: &Track, progress: f64) -> RoadPolygon {
    if let Some(length) = track.lap_length() {
        track
            .road_polygon(0.0, length, ROAD_SAMPLE_STEP_M, true)
            .expect("track must form a valid road polygon")
    } else {
        track
            .road_polygon(
                progress - VISIBLE_TRACK_BEHIND_M,
                progress + VISIBLE_TRACK_AHEAD_M,
                ROAD_SAMPLE_STEP_M,
                false,
            )
            .expect("visible track must form a valid road polygon")
    }
}

fn update_surface(meshes: &mut Assets<Mesh>, surface: &mut RoadSurfaceMesh, polygon: &RoadPolygon) {
    if !surface_needs_update(surface, polygon) {
        return;
    }
    if let Some(mut mesh) = meshes.get_mut(&surface.handle) {
        *mesh = surface_mesh(polygon);
        surface.polygon = polygon.clone();
    }
}

fn surface_needs_update(surface: &RoadSurfaceMesh, polygon: &RoadPolygon) -> bool {
    surface.polygon != *polygon
}

fn empty_surface_mesh() -> Mesh {
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
}

fn surface_mesh(road: &RoadPolygon) -> Mesh {
    let mut mesh = empty_surface_mesh();
    let positions = road
        .right_boundary()
        .iter()
        .zip(road.left_boundary())
        .flat_map(|(&right, &left)| {
            let right = ppx(right);
            let left = ppx(left);
            [[right.x, right.y, 0.0], [left.x, left.y, 0.0]]
        })
        .collect::<Vec<_>>();
    let indices = (0..road.segment_count())
        .flat_map(|i| {
            let next = (i + 1) % road.centerline().len();
            let (right, left) = (2 * i as u32, 2 * i as u32 + 1);
            let (next_right, next_left) = (2 * next as u32, 2 * next as u32 + 1);
            [right, next_right, next_left, right, next_left, left]
        })
        .collect::<Vec<_>>();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

fn draw_lines(
    gizmos: &mut Gizmos,
    road: &RoadPolygon,
    closed: bool,
    show_centerline: bool,
    edge: Color,
    centerline: Color,
) {
    for boundary in [road.right_boundary(), road.left_boundary()] {
        let line = boundary.iter().map(|&point| ppx(point));
        if closed {
            gizmos.lineloop_2d(line, edge);
        } else {
            gizmos.linestrip_2d(line, edge);
        }
    }
    if show_centerline {
        let line = road.centerline().iter().map(|&point| ppx(point));
        if closed {
            gizmos.lineloop_2d(line, centerline);
        } else {
            gizmos.linestrip_2d(line, centerline);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::mesh::VertexAttributeValues;

    #[test]
    fn surface_is_a_triangle_strip_over_the_shared_polygon() {
        let road = RoadPolygon::uniform(vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]], 2.0).unwrap();

        let mesh = surface_mesh(&road);
        let VertexAttributeValues::Float32x3(vertices) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap()
        else {
            panic!("surface positions have the wrong format");
        };
        let Indices::U32(indices) = mesh.indices().unwrap() else {
            panic!("surface indices have the wrong format");
        };

        assert_eq!(vertices.len(), 2 * road.centerline().len());
        assert_eq!(indices.len(), 6 * road.segment_count());
        assert_eq!(indices, &[0, 2, 3, 0, 3, 1, 2, 4, 5, 2, 5, 3]);
    }

    #[test]
    fn surface_updates_only_when_the_polygon_changes() {
        let polygon = RoadPolygon::uniform(vec![[0.0, 0.0], [10.0, 0.0]], 2.0).unwrap();
        let surface = RoadSurfaceMesh {
            handle: Handle::default(),
            polygon: polygon.clone(),
        };
        let changed = RoadPolygon::uniform(vec![[0.0, 0.0], [20.0, 0.0]], 2.0).unwrap();

        assert!(!surface_needs_update(&surface, &polygon));
        assert!(surface_needs_update(&surface, &changed));
    }
}
