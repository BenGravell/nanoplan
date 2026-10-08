//! Planner-specific math helpers.

use crate::common::types::FrenetPosition;
use crate::constraints::Sample;
use crate::simulation::State;
use crate::track::Path;

/// Station radius for trajectory-state projection around a supplied hint.
pub(crate) const STATE_SAMPLE_PROJECTION_RADIUS_M: f64 = 15.0;

pub(crate) fn state_sample(path: &Path, x: &State, t_s: f64, s_hint: Option<f64>) -> (f64, Sample) {
    let p = x.position();
    let FrenetPosition { s, d } = match s_hint {
        Some(h) => path.project_near(p, h, STATE_SAMPLE_PROJECTION_RADIUS_M),
        None => path.project(p),
    };
    (
        s,
        Sample {
            position: p,
            lateral: d,
            road_bounds: None,
            station: s,
            control: None,
            t: t_s,
        },
    )
}
