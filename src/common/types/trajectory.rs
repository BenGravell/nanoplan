use super::{Control, State};

/// Trajectory of vehicle states and the controls producing each transition.
pub(crate) struct Trajectory {
    pub(crate) states: Vec<State>,
    pub(crate) controls: Vec<Control>,
}
