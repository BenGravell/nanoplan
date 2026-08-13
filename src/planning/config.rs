/// How far ahead planners with a genuine receding-horizon cost model (lattice, PI²-DDP, RRT*)
/// look when predicting collisions and optimizing a trajectory.
/// Not `Context::horizon`, which is just the requested length of the returned control trajectory.
pub(crate) const PLANNING_HORIZON_S: f64 = 10.0;
pub(crate) const PLANNING_DT_S: f64 = 0.1;
pub(crate) const PLANNING_TICKS: usize = (PLANNING_HORIZON_S / PLANNING_DT_S) as usize;
