//! Strawman planner: no steering and no throttle, always.

use crate::planning::{Context, Planner};
use crate::simulation::{Control, State};

pub(crate) struct StraightPlanner;

impl Planner for StraightPlanner {
    fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
        vec![Control::default(); ctx.horizon]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_road, test_run_on};

    #[test]
    fn holds_heading_and_coasts() {
        let yaw: f64 = 0.5;
        let ego = State::from((crate::simulation::Position::new(1.0, 2.0), yaw, 3.0));
        let forward = crate::simulation::Position::from_angle(yaw);
        let road = test_road(&[[1.0, 2.0], [1.0 + 100.0 * forward.x, 2.0 + 100.0 * forward.y]]);
        let trace = test_run_on(&mut StraightPlanner, &road, ego, &[], 100);
        let s = *trace.last().unwrap();
        assert_eq!(s.pose.yaw, yaw);
        assert!(s.speed < 3.0 && s.speed > 1.5, "speed {}", s.speed);
        let along = (s.position().x - 1.0) * forward.x + (s.position().y - 2.0) * forward.y;
        assert!(along > 20.0 && along < 30.0, "along {along}");
    }
}
