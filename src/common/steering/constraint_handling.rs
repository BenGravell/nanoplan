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
