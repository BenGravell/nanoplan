//! Finite road windows consumed by planners and simulation.

use crate::geometry::RoadPolygon;
use crate::geometry::barrier::{Barrier, road_side_barriers};

/// The finite planning window sampled from the active track.
#[derive(Debug, Clone)]
pub(crate) struct Road {
    polygon: RoadPolygon,
    stations: Vec<f64>,
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
        let mut stations = Vec::with_capacity(polygon.centerline().len());
        stations.push(0.0);
        for pair in polygon.centerline().windows(2) {
            stations.push(
                stations.last().copied().unwrap()
                    + (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]),
            );
        }
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
            stations,
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

    /// Signed usable road bounds at centerline station `s`: right is
    /// negative and left is positive. Unlike [`Road::half_width`], this
    /// preserves local and asymmetric source widths.
    pub(crate) fn lateral_bounds_at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.stations.last().copied().unwrap_or(0.0));
        let i = self
            .stations
            .partition_point(|&station| station < s)
            .clamp(1, self.stations.len() - 1);
        let ds = self.stations[i] - self.stations[i - 1];
        let u = (s - self.stations[i - 1]) / ds.max(1e-9);
        let lerp = |values: &[f64]| values[i - 1] + u * (values[i] - values[i - 1]);
        (
            -lerp(self.polygon.right_widths()),
            lerp(self.polygon.left_widths()),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_lateral_bounds_interpolate_each_side() {
        let polygon = RoadPolygon::new(
            vec![[0.0, 0.0], [10.0, 0.0]],
            vec![2.0, 4.0],
            vec![3.0, 7.0],
            false,
        )
        .unwrap();
        let road = Road::from_polygon(polygon, 10.0, 0.1);
        assert_eq!(road.lateral_bounds_at(5.0), (-3.0, 5.0));
    }
}
