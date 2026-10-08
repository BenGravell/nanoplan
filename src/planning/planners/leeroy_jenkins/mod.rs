//! Leeroy Jenkins: maximum acceleration and zero steering, always.

use crate::planning::{Context, Planner};
use crate::simulation::{Control, State};
use crate::vehicle::MAX_LON_ACCEL;

pub(crate) struct LeeroyJenkinsPlanner;

impl Planner for LeeroyJenkinsPlanner {
    fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
        vec![
            Control {
                acceleration: MAX_LON_ACCEL,
                curvature: 0.0
            };
            ctx.horizon
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_ctx, test_road, test_run_on};

    #[test]
    fn holds_heading_and_accelerates() {
        let yaw: f64 = 0.5;
        let ego = State::from((crate::simulation::Position::new(1.0, 2.0), yaw, 3.0));
        let forward = crate::simulation::Position::from_angle(yaw);
        let road = test_road(&[[1.0, 2.0], [1.0 + 2_000.0 * forward.x, 2.0 + 2_000.0 * forward.y]]);
        let ctx = test_ctx(&road, &[]);
        let controls = LeeroyJenkinsPlanner.plan(ego, &ctx);
        assert_eq!(controls.len(), ctx.horizon);
        assert!(
            controls
                .iter()
                .all(|u| u.acceleration == MAX_LON_ACCEL && u.curvature == 0.0)
        );
        let trace = test_run_on(&mut LeeroyJenkinsPlanner, &road, ego, &[], 100);
        let s = *trace.last().unwrap();
        assert_eq!(s.pose.yaw, yaw);
        assert!(s.speed > 3.0, "speed {}", s.speed);
        let along = (s.position().x - 1.0) * forward.x + (s.position().y - 2.0) * forward.y;
        assert!(along > 30.0, "along {along}");
    }
}
