//! Analytic steering and conversion to vehicle commands.

pub(crate) mod cartesian;
pub(crate) mod frenet;

use crate::common::kinematics::{clamp_control, commanded_accel_for_net, commanded_accel_to_stop};
use crate::common::types::{Control, State};
use crate::constraints::{Constraint, Kinodynamic, Sample};

#[derive(Clone, Copy)]
pub(crate) enum ConstraintHandlingStrategy {
    Reject,
    Clamp,
}

pub(crate) fn motion_to_control(
    (state, mut control): (State, Control),
    actual: State,
    dt: f64,
    strategy: ConstraintHandlingStrategy,
) -> Option<Control> {
    if matches!(strategy, ConstraintHandlingStrategy::Reject) {
        reject(control, state.speed)?;
    }

    control.acceleration = commanded_accel_for_net(control.acceleration, actual.speed);
    if matches!(strategy, ConstraintHandlingStrategy::Clamp) {
        control = clamp(control, actual.speed, dt);
    }
    Some(control)
}

fn reject(control: Control, speed: f64) -> Option<()> {
    let command = Control {
        acceleration: commanded_accel_for_net(control.acceleration, speed),
        ..control
    };
    (!Kinodynamic.is_violated(&Sample::default().with_control(command, speed))).then_some(())
}

fn clamp(mut control: Control, speed: f64, dt: f64) -> Control {
    control.acceleration = control.acceleration.max(commanded_accel_to_stop(speed, dt));
    clamp_control(control, speed)
}

/// Return the motion sample only if every state and control component is finite.
fn check_finite(state: State, control: Control) -> Option<(State, Control)> {
    (state.position().is_finite()
        && state.pose.yaw.is_finite()
        && state.speed.is_finite()
        && control.acceleration.is_finite()
        && control.curvature.is_finite())
    .then_some((state, control))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::types::Position;

    #[test]
    fn check_finite_rejects_each_nonfinite_component() {
        let sample = |v: [f64; 6]| {
            check_finite(
                State::from((Position::new(v[0], v[1]), v[2], v[3])),
                Control {
                    acceleration: v[4],
                    curvature: v[5],
                },
            )
        };
        assert!(sample([1.0; 6]).is_some());
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for component in 0..6 {
                let mut values = [1.0; 6];
                values[component] = value;
                assert!(sample(values).is_none(), "component {component}: {value}");
            }
        }
    }
}
