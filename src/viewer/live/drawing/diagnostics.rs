use crate::planning::DiagnosticsData;
use bevy::gizmos::config::GizmoConfigStore;
use bevy::prelude::*;

use super::super::Live;
use super::super::screen::{PX_PER_M, ppx};
use crate::viewer::colors::DIAGNOSTICS;

const POINT_RADIUS_M: f32 = 0.14;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct DiagnosticTrajectoryGizmos;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct DiagnosticPointGizmos;

pub(crate) fn configure(live: NonSend<Live>, mut configs: ResMut<GizmoConfigStore>) {
    configs.config_mut::<DiagnosticTrajectoryGizmos>().0.line.width = 1.5;
    configs.config_mut::<DiagnosticPointGizmos>().0.line.width = POINT_RADIUS_M * PX_PER_M * live.camera.zoom * 1.2;
}

pub(in crate::viewer::live) fn draw(
    trajectories: &mut Gizmos<DiagnosticTrajectoryGizmos>,
    points: &mut Gizmos<DiagnosticPointGizmos>,
    diagnostics: &DiagnosticsData,
    horizon_s: f64,
    show_trajectories: bool,
    show_points: bool,
) {
    if show_trajectories {
        for (trajectory, times) in diagnostics.trajectories.iter().zip(&diagnostics.trajectory_times) {
            for (segment, times) in trajectory.windows(2).zip(times.windows(2)) {
                if let Some((start, end)) = visible_segment(segment, times, horizon_s) {
                    trajectories.line_2d(ppx(start), ppx(end), DIAGNOSTICS);
                }
            }
        }
    }
    if show_points {
        for &point in &diagnostics.points {
            points.circle_2d(ppx(point), 0.5 * POINT_RADIUS_M * PX_PER_M, DIAGNOSTICS);
        }
    }
}

fn visible_segment(
    points: &[crate::simulation::Position],
    times: &[f64],
    horizon_s: f64,
) -> Option<(crate::simulation::Position, crate::simulation::Position)> {
    if times[0] >= horizon_s || horizon_s <= 0.0 {
        return None;
    }
    let end = if times[1] > horizon_s {
        crate::common::interp::lerp(points[0], points[1], (horizon_s - times[0]) / (times[1] - times[0]))
    } else {
        points[1]
    };
    Some((points[0], end))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::Position;

    #[test]
    fn clips_candidates_at_display_horizon_including_offset_edges() {
        let points = [Position::new(2.0, 0.0), Position::new(4.0, 0.0)];
        let times = [2.0, 4.0];
        assert_eq!(visible_segment(&points, &times, 0.0), None);
        assert_eq!(visible_segment(&points, &times, 1.0), None);
        assert_eq!(visible_segment(&points, &times, 2.0), None);
        assert_eq!(
            visible_segment(&points, &times, 3.0),
            Some((points[0], Position::new(3.0, 0.0)))
        );
        assert_eq!(visible_segment(&points, &times, 4.0), Some((points[0], points[1])));
        assert_eq!(visible_segment(&points, &times, 10.0), Some((points[0], points[1])));
    }
}
