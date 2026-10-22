use super::super::{Control, State};

/// Trajectory of vehicle states and the controls producing each transition.
#[derive(Debug, Clone)]
pub(crate) struct Trajectory {
    pub(crate) states: Vec<State>,
    pub(crate) controls: Vec<Control>,
}
