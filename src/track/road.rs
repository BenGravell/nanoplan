//! Finite road windows consumed by planners and simulation.

use crate::geometry::barrier::{Barrier, road_side_barriers};

/// The finite planning window sampled from the active track.
#[derive(Debug, Clone)]
pub struct Road {
    pub centerline: Vec<[f64; 2]>,
    pub target_speed: f64,
    pub half_width: f64,
    pub barriers: Vec<Barrier>,
    pub dt: f64,
}

impl Road {
    pub fn new(centerline: Vec<[f64; 2]>, target_speed: f64, half_width: f64, dt: f64) -> Self {
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
