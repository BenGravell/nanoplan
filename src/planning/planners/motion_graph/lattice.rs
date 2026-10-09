//! Multiple-parent connections and layered minimum-cost search.

use super::{Edge, Node, ZapIndex};
use crate::simulation::State;

pub(super) const MAX_PARENTS: usize = 6;

pub(super) fn parents(index: &ZapIndex, target: State) -> Vec<usize> {
    let mut parents = Vec::new();
    for point in index.0.nearest_neighbor_iter(&ZapIndex::point(target)) {
        // The yaw seam has three index entries for each parent.
        if !parents.contains(&point.data) {
            parents.push(point.data);
            if parents.len() == MAX_PARENTS {
                break;
            }
        }
    }
    parents
}

pub(super) fn best_incoming(nodes: &[Node], incoming: &[Edge]) -> usize {
    // Parent costs are settled before this layer: relax every incoming edge.
    (0..incoming.len())
        .min_by(|&a, &b| {
            let score = |i: usize| {
                let edge = &incoming[i];
                let parent = &nodes[edge.parent];
                (
                    parent.collides || edge.eval.collides,
                    parent.cost_to_come + edge.eval.cost,
                )
            };
            let (ca, a_cost) = score(a);
            let (cb, b_cost) = score(b);
            ca.cmp(&cb)
                .then(a_cost.total_cmp(&b_cost))
                .then(incoming[a].parent.cmp(&incoming[b].parent))
        })
        .expect("a node needs an incoming edge")
}

#[cfg(test)]
mod tests {
    use crate::common::geometry::barrier::collides_with_road_barrier;
    use crate::planning::Planner;
    use crate::planning::planners::motion_graph::{Connections, GraphPlanner};
    use crate::planning::{test_ctx, test_road, test_run, test_run_on};
    use crate::simulation::State;
    use crate::track::Path;
    use crate::track::Road;
    use crate::vehicle::MAX_LON_ACCEL;

    #[test]
    fn accelerates_on_an_open_straight_without_a_speed_profile() {
        let road = test_road(&[[-20.0, 0.0], [1_500.0, 0.0]]);
        let controls = GraphPlanner::new(Connections::Lattice).plan(
            State {
                speed: 5.0,
                ..Default::default()
            },
            &test_ctx(&road, &[]),
        );
        assert!(!controls.is_empty());
        assert!(
            controls[0].acceleration > MAX_LON_ACCEL - 0.1,
            "accel {}",
            controls[0].acceleration
        );
    }

    #[test]
    fn brakes_for_road_curvature() {
        let radius = 20.0;
        let centerline: Vec<crate::simulation::Position> = (0..=80)
            .map(|i| {
                let a = i as f64 * 0.02;
                crate::simulation::Position::new(radius * a.sin(), radius * (1.0 - a.cos()))
            })
            .collect();
        let road = Road::new(centerline, 5.5, 0.1);
        let controls = GraphPlanner::new(Connections::Lattice).plan(
            State {
                speed: 25.0,
                ..Default::default()
            },
            &test_ctx(&road, &[]),
        );
        assert!(controls[0].acceleration < 0.0);
    }

    #[test]
    fn stays_on_road_actor_free() {
        let trace = test_run(
            &mut GraphPlanner::new(Connections::Lattice),
            State::new(
                crate::simulation::Pose::new(crate::simulation::Position::new(0.0, 1.0), 0.0),
                8.0,
            ),
            &[],
            80,
        );
        assert!(trace.iter().all(|state| state.position().y.abs() <= 5.5));
        assert!(
            trace.last().unwrap().speed > 8.0,
            "final state {:?}",
            trace.last().unwrap()
        );
    }

    #[test]
    fn uses_the_road_width_through_a_corner() {
        let radius = 25.0;
        let mut centerline: Vec<crate::simulation::Position> = (0..=25)
            .map(|i| crate::simulation::Position::new(-50.0 + 4.0 * i as f64, 0.0))
            .collect();
        centerline.extend((1..=32).map(|i| {
            let a = std::f64::consts::FRAC_PI_2 * i as f64 / 32.0;
            crate::simulation::Position::new(50.0 + radius * a.sin(), radius * (1.0 - a.cos()))
        }));
        centerline.extend((1..=30).map(|i| crate::simulation::Position::new(50.0 + radius, radius + 4.0 * i as f64)));
        let road = Road::new(centerline, 7.0, 0.1);
        let trace = test_run_on(
            &mut GraphPlanner::new(Connections::Lattice),
            &road,
            State {
                speed: 18.0,
                ..Default::default()
            },
            &[],
            130,
        );
        let path = Path::new(road.centerline());
        let lateral_peak = trace
            .iter()
            .map(|state| path.project(state.position()).d.abs())
            .fold(0.0, f64::max);
        assert!(
            lateral_peak > 0.5,
            "planner remained on the centerline instead of using road width; final {:?}",
            trace.last().unwrap()
        );
        assert!(
            trace.iter().all(|state| !collides_with_road_barrier(*state, &road)),
            "racing trajectory contacted the road boundary"
        );
    }

    #[test]
    fn records_no_more_than_the_segment_budget() {
        let road = test_road(&[[-20.0, 0.0], [1_500.0, 0.0]]);
        let diagnostics = crate::planning::Diagnostics::default();
        let mut ctx = test_ctx(&road, &[]);
        ctx.diagnostics = Some(&diagnostics);
        GraphPlanner::new(Connections::Lattice).plan(
            State {
                speed: 8.0,
                ..Default::default()
            },
            &ctx,
        );
        let data = diagnostics.take();
        assert!(!data.trajectories.is_empty());
        assert!(data.trajectories.len() <= 1_000);
        assert_eq!(data.points.len(), data.trajectories.len());
        for (point, trajectory) in data.points.iter().zip(&data.trajectories) {
            assert_eq!(trajectory.last(), Some(point));
            assert!(
                trajectory.first() == Some(&crate::simulation::Position::default())
                    || data.points.iter().any(|node| trajectory.first() == Some(node)),
                "edge start {:?} is not a lattice node",
                trajectory.first()
            );
        }
    }

    #[test]
    fn reduced_budget_caps_search_work() {
        let road = test_road(&[[-20.0, 0.0], [1_500.0, 0.0]]);
        let diagnostics = crate::planning::Diagnostics::default();
        let mut ctx = test_ctx(&road, &[]);
        ctx.compute_budget = crate::planning::ComputeBudget::from_percent(10.0);
        ctx.diagnostics = Some(&diagnostics);
        GraphPlanner::new(Connections::Lattice).plan(
            State {
                speed: 8.0,
                ..Default::default()
            },
            &ctx,
        );
        assert!(diagnostics.take().trajectories.len() <= 100);
    }

    #[test]
    fn still_avoids_a_stopped_actor() {
        let obstacle = State::new(
            crate::simulation::Pose::new(crate::simulation::Position::new(40.0, 0.0), 0.0),
            0.0,
        );
        let trace = test_run(
            &mut GraphPlanner::new(Connections::Lattice),
            State {
                speed: 8.0,
                ..Default::default()
            },
            &[obstacle],
            120,
        );
        let min_gap = trace
            .iter()
            .map(|state| (state.position().x - obstacle.position().x).hypot(state.position().y - obstacle.position().y))
            .fold(f64::INFINITY, f64::min);
        assert!(min_gap > 2.0, "minimum actor gap {min_gap}");
    }
}
