/// Signed longitudinal and lateral accelerations in the vehicle frame, in m/s².
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Accelerations {
    pub(crate) lon: f64,
    pub(crate) lat: f64,
}
