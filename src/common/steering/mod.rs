//! Analytic steering and conversion to vehicle commands.

pub(crate) mod cartesian;
pub(crate) mod constraint_handling;
pub(crate) mod frenet;
pub(crate) mod motion;

pub(crate) use constraint_handling::{ConstraintHandlingStrategy, motion_to_control};
pub(crate) use motion::Motion;

use crate::common::kinematics::{curvature_from_cartesian, longitudinal_acceleration_from_cartesian};
use crate::common::types::{Acceleration, Control, Speed};

const MOTION_SPEED_EPSILON_MPS: f64 = 1e-6;

/// Recover net controls from velocity and acceleration in a common orthonormal frame.
/// Stationary samples need a coordinate-specific fallback because curvature is undefined.
fn control_from_derivatives(velocity: Speed, acceleration: Acceleration) -> Option<Control> {
    if velocity.norm() <= MOTION_SPEED_EPSILON_MPS {
        return None;
    }

    Some(Control {
        acceleration: longitudinal_acceleration_from_cartesian(velocity, acceleration),
        curvature: curvature_from_cartesian(velocity, acceleration),
    })
}
