#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Control {
    pub(crate) acceleration: f64,
    pub(crate) curvature: f64,
}

impl Control {
    pub(crate) const fn is_finite(self) -> bool {
        self.acceleration.is_finite() && self.curvature.is_finite()
    }
}
