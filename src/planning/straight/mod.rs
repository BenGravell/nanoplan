//! Strawman planner: straight ahead at constant speed, always.

use crate::planning::{Context, Planner};
use crate::simulation::{Control, State};

pub struct StraightPlanner;

impl Planner for StraightPlanner {
    fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
        vec![Control::default(); ctx.horizon]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{test_ctx, test_road};
    use crate::simulation::Simulator;

    #[test]
    fn holds_heading_and_speed() {
        let mut sim = Simulator {
            state: State {
                x: 1.0,
                y: 2.0,
                yaw: 0.5,
                speed: 3.0,
            },
            dt: 0.1,
        };
        let road = test_road(&[[0.0, 0.0], [100.0, 0.0]]);
        let ctx = test_ctx(&road, &[]);
        let mut planner = StraightPlanner;
        for _ in 0..100 {
            sim.tick(&mut planner, &ctx);
        }
        let s = sim.state;
        assert_eq!((s.yaw, s.speed), (0.5, 3.0));
        // 100 ticks of 0.1 s at 3 m/s = 30 m along the initial heading
        assert!((s.x - (1.0 + 30.0 * 0.5f64.cos())).abs() < 1e-9);
        assert!((s.y - (2.0 + 30.0 * 0.5f64.sin())).abs() < 1e-9);
    }
}
