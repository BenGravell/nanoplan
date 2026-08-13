use crate::simulation::State;

/// How far ahead planners with a genuine receding-horizon cost model (lattice, PI²-DDP, RRT*)
/// look when predicting collisions and optimizing a trajectory.
/// Not `Context::horizon`, which is just the requested length of the returned control trajectory.
pub(crate) const PLANNING_HORIZON_S: f64 = 10.0;
pub(crate) const PLANNING_DT_S: f64 = 0.1;
pub(crate) const PLANNING_TICKS: usize = (PLANNING_HORIZON_S / PLANNING_DT_S) as usize;

const WARM_START_MAX_POSITION_ERROR_M: f64 = 1.0;

pub(crate) fn warm_start_matches(expected_next: State, ego: State) -> bool {
    (expected_next.x - ego.x).hypot(expected_next.y - ego.y) < WARM_START_MAX_POSITION_ERROR_M
}
