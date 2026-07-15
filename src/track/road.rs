//! Finite road windows consumed by planners and simulation.

use crate::geometry::barrier::{Barrier, road_side_barriers};

/// The finite planning window sampled from the active track.
#[derive(Debug, Clone)]
pub(crate) struct Road {
    pub(crate) centerline: Vec<[f64; 2]>,
    pub(crate) target_speed: f64,
    pub(crate) half_width: f64,
    pub(crate) barriers: Vec<Barrier>,
    pub(crate) dt: f64,
}

impl Road {
    pub(crate) fn new(
        centerline: Vec<[f64; 2]>,
        target_speed: f64,
        half_width: f64,
        dt: f64,
    ) -> Self {
        let barriers = road_side_barriers(&centerline, half_width);
        Self {
            centerline,
            target_speed,
            half_width,
            barriers,
            dt,
        }
    }
}
