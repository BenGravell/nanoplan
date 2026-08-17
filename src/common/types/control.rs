use super::vector::V2;

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Control {
    pub(crate) acceleration: f64,
    pub(crate) curvature: f64,
}

impl From<V2> for Control {
    fn from(v: V2) -> Self {
        Control {
            acceleration: v[0],
            curvature: v[1],
        }
    }
}
