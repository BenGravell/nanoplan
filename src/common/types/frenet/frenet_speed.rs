/// Velocity in a Frenet frame, in metres per second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrenetSpeed {
    /// Rate of change of station along the reference path.
    pub(crate) s: f64,
    /// Rate of change of signed lateral offset.
    pub(crate) d: f64,
}
