/// Position relative to a reference path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrenetPosition {
    /// Station along the reference path, in metres.
    pub(crate) s: f64,
    /// Signed lateral offset from the reference path, in metres.
    pub(crate) d: f64,
}
