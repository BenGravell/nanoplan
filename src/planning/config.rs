/// How far ahead planners with a genuine receding-horizon cost model
/// look when predicting collisions and optimizing a trajectory.
/// Not `Context::horizon`, which is just the requested length of the returned control trajectory.
pub(crate) const PLANNING_HORIZON_S: f64 = 10.0;
pub(crate) const PLANNING_DT_S: f64 = 0.1;
pub(crate) const PLANNING_TICKS: usize = (PLANNING_HORIZON_S / PLANNING_DT_S) as usize;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{Context, Planner, test_road, test_steps_on};
    use crate::simulation::{Control, State};
    use crate::vehicle::{MAX_TERMINAL_SPEED_MPS, MIN_LON_ACCEL};

    struct MaxDecelStop;

    impl Planner for MaxDecelStop {
        fn plan(&mut self, _ego: State, _ctx: &Context) -> Vec<Control> {
            vec![Control {
                acceleration: MIN_LON_ACCEL,
                curvature: 0.0,
            }]
        }
    }

    #[test]
    fn planning_horizon_covers_max_decel_stop_from_top_speed() {
        const ROAD_BUFFER_LENGTH_M: f64 = 20.0;

        // A long straight must leave enough preview to brake for a sharp corner.
        let ego = State {
            speed: *MAX_TERMINAL_SPEED_MPS,
            ..Default::default()
        };
        let mut road = test_road(&[
            [-ROAD_BUFFER_LENGTH_M, 0.0],
            [ego.speed * PLANNING_HORIZON_S + ROAD_BUFFER_LENGTH_M, 0.0],
        ]);
        road.dt = PLANNING_DT_S;
        let stop = test_steps_on(&mut MaxDecelStop, &road, ego, &[], PLANNING_TICKS)
            .enumerate()
            .find(|(_, state)| state.speed <= 0.0);
        if let Some((index, _)) = stop {
            let stop_time = (index + 1) as f64 * PLANNING_DT_S;
            println!("Time to stop: {stop_time:.1} s; planning horizon: {PLANNING_HORIZON_S:.1} s");
        }
        assert!(
            stop.is_some(),
            "maximum braking from top speed must reach or cross zero speed within the planning horizon ({PLANNING_HORIZON_S} s, {PLANNING_TICKS} ticks)"
        );
    }
}
