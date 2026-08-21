use crate::common::measure::dot;
use crate::simulation::{Pose, Position};

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
        let forward = Position::from_angle(pose.yaw);
        Pose::new(
            Position::new(
                pose.position.x + 0.5 * self.length * forward.x,
                pose.position.y + 0.5 * self.length * forward.y,
            ),
            pose.yaw,
        )
    }

    /// World-space corners for a pose whose position is the rear of the vehicle.
    pub(crate) fn corners(self, pose: Pose) -> [Position; 4] {
        let rear = Position::from(pose);
        let forward = Position::from_angle(pose.yaw);
        let left = Position::new(-forward.y, forward.x);
        let front = Position::new(rear.x + self.length * forward.x, rear.y + self.length * forward.y);
        let half_width = self.width / 2.0;
        [
            Position::new(rear.x + half_width * left.x, rear.y + half_width * left.y),
            Position::new(rear.x - half_width * left.x, rear.y - half_width * left.y),
            Position::new(front.x + half_width * left.x, front.y + half_width * left.y),
            Position::new(front.x - half_width * left.x, front.y - half_width * left.y),
        ]
    }

    /// Furthest extent from the rear reference point along a world-space axis.
    #[cfg(test)]
    pub(crate) fn support(self, yaw: f64, axis: [f64; 2]) -> f64 {
        let n = axis[0].hypot(axis[1]).max(1e-9);
        let axis = [axis[0] / n, axis[1] / n];
        let forward = Position::from_angle(yaw);
        let center = 0.5 * self.length * (axis[0] * forward.x + axis[1] * forward.y);
        center + self.support_radius(yaw, axis)
    }

    /// Half-extent of this rectangle along a world-space axis.
    pub(crate) fn support_radius(self, yaw: f64, axis: [f64; 2]) -> f64 {
        let n = axis[0].hypot(axis[1]).max(1e-9);
        let axis = [axis[0] / n, axis[1] / n];
        let forward = Position::from_angle(yaw);
        let left = Position::new(-forward.y, forward.x);
        0.5 * self.length * dot(axis, forward.xy()).abs() + 0.5 * self.width * dot(axis, left.xy()).abs()
    }
}
