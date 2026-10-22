//! Execution feasibility shared by sampled trajectory planners.

use crate::common::geometry::barrier::collides_with_road_barrier;
use crate::common::geometry::{EGO_FOOTPRINT, Footprint};
use crate::common::measure::dot;
use crate::constraints::collision::actor_collision;
use crate::constraints::{Constraint, Constraints, Kinodynamic, Sample};
use crate::planning::Context;
use crate::planning::planner_math::state_sample;
use crate::simulation::{Control, Position, State};

const ROAD_END_TOLERANCE_M: f64 = 1e-6;
// Clearance for rolling-window seams, matching the Frenet and CenterlineFollower planners.
const ROAD_FOOTPRINT: Footprint = Footprint::new(EGO_FOOTPRINT.length + 0.1, EGO_FOOTPRINT.width + 0.2);

/// Check requested commands and cached plant states, including swept footprint contact.
pub(crate) fn trajectory_is_feasible(states: &[State], controls: &[Control], ctx: &Context) -> bool {
    if controls.is_empty() || states.len() != controls.len() + 1 {
        return false;
    }
    if states.iter().any(|state| {
        !state.position().is_finite() || !state.pose.yaw.is_finite() || !state.speed.is_finite() || state.speed < -1e-9
    }) || controls
        .iter()
        .zip(states)
        .any(|(&control, state)| Kinodynamic.is_violated(&Sample::default().with_control(control, state.speed)))
    {
        return false;
    }
    let path = ctx.path();
    let ego = states[0];
    let mut station = ctx.project_ego(ego).s;
    let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, station);
    let (end, end_yaw) = path.pose_at(path.length());
    let forward = Position::from_angle(end_yaw);
    for (tick, pair) in states.windows(2).enumerate() {
        ctx.work(1);
        let [prev, state] = [pair[0], pair[1]];
        let (s, mut sample) = state_sample(
            path,
            &state,
            (tick + 1) as f64 * ctx.road.dt,
            Some(station + prev.speed * ctx.road.dt),
        );
        station = s;
        sample.road_bounds = Some(ctx.road.lateral_bounds_at(station));
        let beyond_end =
            dot((state.position() - end).xy(), forward.xy()) > 0.0 && station >= path.length() - ROAD_END_TOLERANCE_M;
        if beyond_end
            || collides_with_road_barrier(state, ctx.road)
            || constraints.is_transition_violated(prev, state, ROAD_FOOTPRINT, ctx.road, &sample)
            || actor_collision(state.pose, sample.t, ctx)
        {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::geometry::RoadPolygon;
    use crate::metrics;
    use crate::planning::{test_ctx, test_road};
    use crate::simulation::{rollout, world_step};
    use crate::track::Road;
    use crate::vehicle::MAX_LON_ACCEL;

    #[test]
    fn rejects_footprint_contacts_that_point_scoring_misses() {
        let road = test_road(&[[-20.0, 0.0], [200.0, 0.0]]);
        let ego = State::from((Position::new(0.0, road.half_width - 0.1), 0.0, 8.0));
        let control = Control::default();
        let next = world_step(ego, control, road.dt);
        let ctx = test_ctx(&road, &[]);
        let (_, sample) = state_sample(ctx.path(), &next, road.dt, None);
        let constraints = Constraints::new(road.half_width, &[], ctx.path(), ctx.project_ego(ego).s);
        assert!(!constraints.is_violated(&sample.with_control(control, ego.speed)));
        assert!(!trajectory_is_feasible(&[ego, next], &[control], &ctx));

        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let next = world_step(ego, control, road.dt);
        let actors = [State::from((Position::new(4.0, 0.0), 0.0, 0.0))];
        let ctx = test_ctx(&road, &actors);
        let (_, sample) = state_sample(ctx.path(), &next, road.dt, None);
        let constraints = Constraints::new(road.half_width, &actors, ctx.path(), ctx.project_ego(ego).s);
        assert!(!constraints.is_violated(&sample.with_control(control, ego.speed)));
        assert!(!trajectory_is_feasible(&[ego, next], &[control], &ctx));
    }

    #[test]
    fn uses_local_asymmetric_bounds_and_rejects_beyond_window() {
        let polygon = RoadPolygon::new(
            vec![Position::new(-20.0, 0.0), Position::new(200.0, 0.0)],
            vec![2.0, 2.0],
            vec![7.0, 7.0],
            false,
        )
        .unwrap();
        let road = Road::from_polygon(polygon, 0.1);
        let ctx = test_ctx(&road, &[]);
        let control = Control::default();
        // The global minimum width is 2 m; the left side actually has 7 m.
        let ego = State::from((Position::new(0.0, 4.0), 0.0, 8.0));
        assert!(trajectory_is_feasible(
            &[ego, world_step(ego, control, road.dt)],
            &[control],
            &ctx
        ));
        let ego = State::from((Position::new(199.9, 0.0), 0.0, 8.0));
        assert!(!trajectory_is_feasible(
            &[ego, world_step(ego, control, road.dt)],
            &[control],
            &ctx
        ));
    }

    #[test]
    fn feasibility_rejects_collisions_independently_of_progress_score() {
        let road = test_road(&[[-20.0, 0.0], [400.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let ctx = test_ctx(&road, &[]);
        let coast = rollout(ego, road.dt, 3, |_, _| Some(Control::default())).unwrap();
        let accelerated = rollout(ego, road.dt, 3, |_, _| {
            Some(Control {
                acceleration: MAX_LON_ACCEL,
                ..Default::default()
            })
        })
        .unwrap();
        let score = |states: &[State]| metrics::evaluate(states, ctx.path(), Some([20.0, 23.0]));
        assert!(score(&accelerated.states) > score(&coast.states));
        let actors = [ego];
        let blocked = test_ctx(&road, &actors);
        for candidate in [coast, accelerated] {
            assert!(trajectory_is_feasible(&candidate.states, &candidate.controls, &ctx));
            assert!(!trajectory_is_feasible(
                &candidate.states,
                &candidate.controls,
                &blocked
            ));
        }
    }
}
