/// Signed longitudinal and lateral accelerations in the vehicle frame, in m/s².
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct FrenetAcceleration {
    /// Tangential acceleration along the direction of travel.
    pub(crate) s: f64,
    /// Normal acceleration, positive to the left.
    pub(crate) d: f64,
}
