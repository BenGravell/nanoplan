//! Strawman planner: no steering and no throttle, always.

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
    fn holds_heading_and_coasts() {
        let yaw: f64 = 0.5;
        let mut sim = Simulator::new(
            State {
                x: 1.0,
                y: 2.0,
                yaw,
                speed: 3.0,
                ..Default::default()
            },
            0.1,
        );
        let road = test_road(&[
            [1.0, 2.0],
            [1.0 + 100.0 * yaw.cos(), 2.0 + 100.0 * yaw.sin()],
        ]);
        let ctx = test_ctx(&road, &[]);
        let mut planner = StraightPlanner;
        for _ in 0..100 {
            sim.tick(&mut planner, &ctx);
        }
        let s = sim.state;
        assert_eq!(s.yaw, yaw);
        assert!(s.speed < 3.0 && s.speed > 1.5, "speed {}", s.speed);
        let along = (s.x - 1.0) * yaw.cos() + (s.y - 2.0) * yaw.sin();
        assert!(along > 20.0 && along < 30.0, "along {along}");
    }
}
