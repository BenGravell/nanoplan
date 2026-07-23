use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;

use super::super::camera::CameraState;
use super::super::screen::PX_PER_M;
use super::config::GRID_Z;
use crate::viewer::colors::{GRID_MAJOR, GRID_MINOR};

const WIDE_GRID_ZOOM_THRESHOLD: f32 = 0.25;
const DEFAULT_GRID_SPACING_M: f32 = 10.0;
const WIDE_GRID_SPACING_M: f32 = 50.0;
const DEFAULT_MAJOR_LINE_INTERVAL: i64 = 5;
const WIDE_MAJOR_LINE_INTERVAL: i64 = 2;
const MAJOR_LINE_WIDTH_PX: f32 = 1.5;
const MINOR_LINE_WIDTH_PX: f32 = 0.75;

#[derive(Resource)]
pub(crate) struct GridMesh {
    handle: Handle<Mesh>,
    populated: bool,
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
        Transform::from_xyz(0.0, 0.0, GRID_Z),
    ));
    commands.insert_resource(GridMesh {
        handle,
        populated: false,
    });
}

pub(in crate::viewer::live) fn draw(
    meshes: &mut Assets<Mesh>,
    grid: &mut GridMesh,
    camera: CameraState,
    window: &Window,
) {
    let extent = window.width().hypot(window.height()) / camera.zoom;
    let wide_grid = camera.zoom < WIDE_GRID_ZOOM_THRESHOLD;
    let spacing = if wide_grid {
        WIDE_GRID_SPACING_M
    } else {
        DEFAULT_GRID_SPACING_M
    } * PX_PER_M;
    let major_every = if wide_grid {
        WIDE_MAJOR_LINE_INTERVAL
    } else {
        DEFAULT_MAJOR_LINE_INTERVAL
    };
    let min_x = ((camera.center.x - extent) / spacing).floor() as i64;
    let max_x = ((camera.center.x + extent) / spacing).ceil() as i64;
    let min_y = ((camera.center.y - extent) / spacing).floor() as i64;
    let max_y = ((camera.center.y + extent) / spacing).ceil() as i64;
    let mut vertices = Vec::with_capacity(((max_x - min_x + max_y - min_y + 2) * 4) as usize);
    let mut colors = Vec::with_capacity(vertices.capacity());
    for x in min_x..=max_x {
        let major = x.rem_euclid(major_every) == 0;
        let color = if major { GRID_MAJOR } else { GRID_MINOR };
        let x = x as f32 * spacing;
        push_rect(
            &mut vertices,
            &mut colors,
            Vec2::new(x, camera.center.y),
            Vec2::new(
                if major {
                    MAJOR_LINE_WIDTH_PX
                } else {
                    MINOR_LINE_WIDTH_PX
                } / camera.zoom,
                extent * 2.0,
            ),
            color,
        );
    }
    for y in min_y..=max_y {
        let major = y.rem_euclid(major_every) == 0;
        let color = if major { GRID_MAJOR } else { GRID_MINOR };
        let y = y as f32 * spacing;
        push_rect(
            &mut vertices,
            &mut colors,
            Vec2::new(camera.center.x, y),
            Vec2::new(
                extent * 2.0,
                if major {
                    MAJOR_LINE_WIDTH_PX
                } else {
                    MINOR_LINE_WIDTH_PX
                } / camera.zoom,
            ),
            color,
        );
    }

    let mut mesh = empty_mesh();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    if let Some(mut existing) = meshes.get_mut(&grid.handle) {
        *existing = mesh;
        grid.populated = true;
    }
}

pub(in crate::viewer::live) fn clear(meshes: &mut Assets<Mesh>, grid: &mut GridMesh) {
    if !grid.populated {
        return;
    }
    if let Some(mut mesh) = meshes.get_mut(&grid.handle) {
        *mesh = empty_mesh();
        grid.populated = false;
    }
}

fn empty_mesh() -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    // Bevy's mesh allocator cannot upload a zero-byte mesh. Keep one
    // degenerate triangle while the grid is hidden or not yet populated.
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0; 3]; 3]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, vec![[0.0; 4]; 3]);
    mesh
}

fn push_rect(
    vertices: &mut Vec<[f32; 3]>,
    colors: &mut Vec<[f32; 4]>,
    center: Vec2,
    size: Vec2,
    color: LinearRgba,
) {
    let half = size / 2.0;
    let corners = [
        [center.x - half.x, center.y - half.y, 0.0],
        [center.x + half.x, center.y - half.y, 0.0],
        [center.x + half.x, center.y + half.y, 0.0],
        [center.x - half.x, center.y - half.y, 0.0],
        [center.x + half.x, center.y + half.y, 0.0],
        [center.x - half.x, center.y + half.y, 0.0],
    ];
    vertices.extend(corners);
    colors.extend(std::iter::repeat_n(color.to_f32_array(), corners.len()));
}
