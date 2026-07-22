use crate::common::measure::dot;
use crate::simulation::Pose;

/// Rectangular footprint dimensions in meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Footprint {
    pub(crate) length: f64,
    pub(crate) width: f64,
}

impl Footprint {
    pub(crate) const fn new(length: f64, width: f64) -> Self {
        Self { length, width }
    }

    /// Geometric center for a pose whose position is the rear of the vehicle.
    pub(crate) fn center(self, pose: Pose) -> Pose {
        Pose::new(
            pose.x + 0.5 * self.length * pose.yaw.cos(),
            pose.y + 0.5 * self.length * pose.yaw.sin(),
            pose.yaw,
        )
    }

    /// World-space corners for a pose whose position is the rear of the vehicle.
    pub(crate) fn corners(self, pose: Pose) -> [[f64; 2]; 4] {
        let forward = [pose.yaw.cos(), pose.yaw.sin()];
        let left = [-forward[1], forward[0]];
        let front = [
            pose.x + self.length * forward[0],
            pose.y + self.length * forward[1],
        ];
        let half_width = self.width / 2.0;
        [
            [pose.x + half_width * left[0], pose.y + half_width * left[1]],
            [pose.x - half_width * left[0], pose.y - half_width * left[1]],
            [
                front[0] + half_width * left[0],
                front[1] + half_width * left[1],
            ],
            [
                front[0] - half_width * left[0],
                front[1] - half_width * left[1],
            ],
        ]
    }

    /// Furthest extent from the rear reference point along a world-space axis.
    pub(crate) fn support(self, yaw: f64, axis: [f64; 2]) -> f64 {
        let n = axis[0].hypot(axis[1]).max(1e-9);
        let axis = [axis[0] / n, axis[1] / n];
        let center = 0.5 * self.length * (axis[0] * yaw.cos() + axis[1] * yaw.sin());
        center + self.support_radius(yaw, axis)
    }

    /// Half-extent of this rectangle along a world-space axis.
    pub(crate) fn support_radius(self, yaw: f64, axis: [f64; 2]) -> f64 {
        let n = axis[0].hypot(axis[1]).max(1e-9);
        let axis = [axis[0] / n, axis[1] / n];
        let forward = [yaw.cos(), yaw.sin()];
        let left = [-forward[1], forward[0]];
        0.5 * self.length * dot(axis, forward).abs() + 0.5 * self.width * dot(axis, left).abs()
    }
}
