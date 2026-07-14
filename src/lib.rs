//! Ultra minimalist motion planner for car-like vehicles.
//!
//! Core components:
//! - [`barrier`]: physical roadside barrier segments and ego collision response
//! - [`planning`]: the planner interface and one module per planner
//! - [`simulation`]: the kinematic model and closed-loop simulator
//! - [`geometry`]: shared collision/rendering footprints
//! - [`metrics`]: nuPlan closed-loop quality metrics, one module per metric
//! - [`prediction`]: shared actor state prediction
//! - [`track`]: the endless procedural track and shared path geometry
//! - [`vehicle`]: global ego capability and resistance constants
//! - [`world`]: realtime endless-track demo

pub mod barrier;
pub mod geometry;
pub(crate) mod math;
pub mod metrics;
pub mod planning;
pub mod prediction;
pub(crate) mod rng;
pub mod simulation;
pub mod track;
pub mod vehicle;
pub mod world;

pub use barrier::{BARRIER_RESTITUTION, Barrier, collide_with_barriers, road_side_barriers};
pub use geometry::{
    BIKE_FOOTPRINT, CAR_COLLISION_RADIUS_M, CAR_FOOTPRINT, EGO_COLLISION_RADIUS_M, EGO_FOOTPRINT,
    Footprint, PEDESTRIAN_FOOTPRINT, TRUCK_FOOTPRINT,
};
pub use metrics::Metrics;
pub use planning::{
    BasicPlanner, BezierToppraPlanner, Cem, Context, IlqrPlanner, LatticePlanner, Mppi,
    Pi2DdpPlanner, Planner, PlannerKind, PredictiveSampling, RrtPlanner, RrtStarPlanner,
    SamplingPlanner, StraightPlanner, TreetopPlanner,
};
pub use simulation::{Control, Pose, Position, Simulator, State};
pub use track::{Path, Road, Track};
