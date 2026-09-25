use std::ops::Range;

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
    time_range: Range<f64>,
    show_trajectories: bool,
    show_points: bool,
) {
    if show_trajectories {
        for (trajectory, times) in diagnostics.trajectories.iter().zip(&diagnostics.trajectory_times) {
            for (segment, times) in trajectory.windows(2).zip(times.windows(2)) {
                if let Some((start, end)) = visible_segment(segment, times, &time_range) {
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
    time_range: &Range<f64>,
) -> Option<(crate::simulation::Position, crate::simulation::Position)> {
    if time_range.start >= time_range.end
        || times[1] <= time_range.start
        || times[0] >= time_range.end
        || times[1] <= times[0]
    {
        return None;
    }
    let at = |time: f64| crate::common::interp::lerp(points[0], points[1], (time - times[0]) / (times[1] - times[0]));
    Some((at(times[0].max(time_range.start)), at(times[1].min(time_range.end))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::Position;

    #[test]
    fn clips_candidates_at_display_horizon_including_offset_edges() {
        let points = [Position::new(2.0, 0.0), Position::new(4.0, 0.0)];
        let times = [2.0, 4.0];
        assert_eq!(visible_segment(&points, &times, &(0.0..0.0)), None);
        assert_eq!(visible_segment(&points, &times, &(0.0..1.0)), None);
        assert_eq!(visible_segment(&points, &times, &(0.0..2.0)), None);
        assert_eq!(
            visible_segment(&points, &times, &(0.0..3.0)),
            Some((points[0], Position::new(3.0, 0.0)))
        );
        assert_eq!(
            visible_segment(&points, &times, &(0.0..4.0)),
            Some((points[0], points[1]))
        );
        assert_eq!(
            visible_segment(&points, &times, &(0.0..10.0)),
            Some((points[0], points[1]))
        );
    }

    #[test]
    fn clips_elapsed_prefix_and_future_edges_without_moving_geometry() {
        let points = [Position::new(2.0, 0.0), Position::new(4.0, 0.0)];
        assert_eq!(
            visible_segment(&points, &[2.0, 4.0], &(3.0..3.5)),
            Some((Position::new(3.0, 0.0), Position::new(3.5, 0.0)))
        );
        assert_eq!(visible_segment(&points, &[2.0, 4.0], &(4.0..8.0)), None);
        assert_eq!(visible_segment(&points, &[2.0, 4.0], &(0.0..2.0)), None);
        assert_eq!(visible_segment(&points, &[2.0, 2.0], &(0.0..4.0)), None);
    }

    #[test]
    fn bezier_diagnostics_start_at_rendered_ego_through_ticks_pause_and_waiting() {
        use crate::planning::PlannerKind;
        use crate::viewer::live::rendering::{RenderSnapshot, rendered_ego, rendered_plan_age};
        use crate::world::{EgoStart, LiveWorld};
        let world = LiveWorld::with_track_at(
            1,
            1,
            PlannerKind::BezierToppra,
            0,
            0.1,
            EgoStart {
                speed: 20.0,
                ..Default::default()
            },
        );
        let mut live = Live {
            world,
            ..Default::default()
        };
        live.world.diagnostics_enabled = true;
        live.world.preview_ticks = 100;
        live.previous = RenderSnapshot::capture(&live.world);
        let check = |live: &Live| {
            let age = rendered_plan_age(live);
            let ego = rendered_ego(live).position();
            assert!(!live.world.diagnostics.trajectories.is_empty());
            for (points, times) in live
                .world
                .diagnostics
                .trajectories
                .iter()
                .zip(&live.world.diagnostics.trajectory_times)
            {
                let (start, _) = points
                    .windows(2)
                    .zip(times.windows(2))
                    .find_map(|(p, t)| visible_segment(p, t, &(age..age + 3.0)))
                    .unwrap();
                assert!(start.distance(ego) < 1e-8, "age {age}: start {start:?}, ego {ego:?}");
            }
        };
        for _ in 0..3 {
            assert!(live.tick());
            for fraction in [0.0, 0.25, 0.75, 1.0, 3.0] {
                live.acc = fraction * live.world.dt() as f32;
                check(&live);
            }
            live.toggle_pause();
            live.acc = 0.0;
            check(&live);
            live.toggle_pause();
            check(&live);
        }
    }
}
