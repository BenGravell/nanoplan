//! Finite road windows consumed by planners and simulation.

use crate::geometry::RoadPolygon;
use crate::geometry::barrier::{Barrier, road_side_barriers};

/// The finite planning window sampled from the active track.
#[derive(Debug, Clone)]
pub(crate) struct Road {
    polygon: RoadPolygon,
    pub(crate) target_speed: f64,
    pub(crate) half_width: f64,
    barriers: Vec<Barrier>,
    pub(crate) dt: f64,
}

impl Road {
    #[cfg(test)]
    pub(crate) fn new(
        centerline: Vec<[f64; 2]>,
        target_speed: f64,
        half_width: f64,
        dt: f64,
    ) -> Self {
        let polygon = RoadPolygon::uniform(centerline, half_width)
            .expect("road needs a finite positive width and at least two distinct stations");
        Self::from_polygon(polygon, target_speed, dt)
    }

    pub(crate) fn from_polygon(polygon: RoadPolygon, target_speed: f64, dt: f64) -> Self {
        let half_width = polygon
            .right_widths()
            .iter()
            .chain(polygon.left_widths())
            .copied()
            .reduce(f64::min)
            .expect("road polygon needs widths");
        let barriers = road_side_barriers(&polygon);
        Self {
            polygon,
            target_speed,
            half_width,
            barriers,
            dt,
        }
    }

    pub(crate) fn centerline(&self) -> &[[f64; 2]] {
        self.polygon.centerline()
    }

    pub(crate) fn polygon(&self) -> &RoadPolygon {
        &self.polygon
    }

    pub(crate) fn barriers(&self) -> &[Barrier] {
        &self.barriers
    }
}
