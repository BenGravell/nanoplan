#[allow(
    clippy::module_inception,
    reason = "Keep the Trajectory type beside its related trajectory types"
)]
mod trajectory;
mod trajectory_kinematics;

pub(crate) use trajectory::Trajectory;
pub(crate) use trajectory_kinematics::TrajectoryKinematics;
