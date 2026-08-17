use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;

use super::super::camera::CameraState;
use super::super::screen::PX_PER_M;
use super::config::GRID_Z;
use crate::common::math::smoothstep;
use crate::viewer::colors::{GRID_MAJOR, GRID_MINOR};

const MINOR_GRID_SPACING_M: f32 = 10.0;
const MAJOR_GRID_MULTIPLE: u32 = 4;
const MAJOR_GRID_SPACING_M: f32 = MINOR_GRID_SPACING_M * MAJOR_GRID_MULTIPLE as f32;
const GRID_FADE_START_ZOOM: f32 = 0.2;
const GRID_FADE_END_ZOOM: f32 = 0.35;
const MAJOR_LINE_WIDTH_PX: f32 = 2.0;
const MINOR_LINE_WIDTH_PX: f32 = 1.0;

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
    let minor_opacity = minor_grid_opacity(camera.zoom);
    let major_opacity = 1.0 - minor_opacity;

    let mut vertices = Vec::new();
    let mut colors = Vec::new();

    // major grid
    push_grid(
        &mut vertices,
        &mut colors,
        camera.center,
        extent,
        MAJOR_GRID_SPACING_M * PX_PER_M,
        MAJOR_LINE_WIDTH_PX / camera.zoom,
        GRID_MAJOR,
        major_opacity,
    );

    // minor grid
    push_grid(
        &mut vertices,
        &mut colors,
        camera.center,
        extent,
        MINOR_GRID_SPACING_M * PX_PER_M,
        MINOR_LINE_WIDTH_PX / camera.zoom,
        GRID_MINOR,
        minor_opacity,
    );

    let mut mesh = empty_mesh();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    if let Some(mut existing) = meshes.get_mut(&grid.handle) {
        *existing = mesh;
        grid.populated = true;
    }
}

fn minor_grid_opacity(zoom: f32) -> f32 {
    smoothstep(((zoom - GRID_FADE_START_ZOOM) / (GRID_FADE_END_ZOOM - GRID_FADE_START_ZOOM)).into()) as f32
}

#[allow(clippy::too_many_arguments)]
fn push_grid(
    vertices: &mut Vec<[f32; 3]>,
    colors: &mut Vec<[f32; 4]>,
    center: Vec2,
    extent: f32,
    spacing: f32,
    line_width: f32,
    color: LinearRgba,
    opacity: f32,
) {
    if opacity <= 0.0 {
        return;
    }
    let min_x = ((center.x - extent) / spacing).floor() as i64;
    let max_x = ((center.x + extent) / spacing).ceil() as i64;
    let min_y = ((center.y - extent) / spacing).floor() as i64;
    let max_y = ((center.y + extent) / spacing).ceil() as i64;
    for x in min_x..=max_x {
        let x = x as f32 * spacing;
        push_rect(
            vertices,
            colors,
            Vec2::new(x, center.y),
            Vec2::new(line_width, extent * 2.0),
            color,
            opacity,
        );
    }
    for y in min_y..=max_y {
        let y = y as f32 * spacing;
        push_rect(
            vertices,
            colors,
            Vec2::new(center.x, y),
            Vec2::new(extent * 2.0, line_width),
            color,
            opacity,
        );
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
    opacity: f32,
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
    let mut color = color.to_f32_array();
    color[3] *= opacity;
    colors.extend(std::iter::repeat_n(color, corners.len()));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn major_grid_is_a_power_of_two_multiple_of_minor_grid() {
        assert!(MAJOR_GRID_MULTIPLE.is_power_of_two());
    }

    #[test]
    fn grid_levels_crossfade_smoothly() {
        assert_eq!(minor_grid_opacity(GRID_FADE_START_ZOOM), 0.0);
        assert!((minor_grid_opacity((GRID_FADE_START_ZOOM + GRID_FADE_END_ZOOM) / 2.0) - 0.5).abs() < 1e-6);
        assert_eq!(minor_grid_opacity(GRID_FADE_END_ZOOM), 1.0);
    }
}
