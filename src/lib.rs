//! Ultra minimalist motion planner for car-like vehicles.
//!
//! Core components:
//! - [`barrier`]: physical roadside barrier segments and ego collision response
//! - [`planning`]: the planner interface and one module per planner
//! - [`simulation`]: the kinematic model and closed-loop simulator
//! - [`geometry`]: shared collision/rendering footprints
//! - [`metrics`]: nuPlan closed-loop quality metrics, one module per metric
//! - [`prediction`]: shared actor state prediction
//! - [`scenarios`]: scenario data, road geometry, loading, and generation
//! - [`vehicle`]: global ego capability and resistance constants
//! - [`world`]: infinite chunked procedural street world, mixed traffic, realtime interactive world

pub mod barrier;
pub mod geometry;
pub(crate) mod math;
pub mod metrics;
pub mod planning;
pub mod prediction;
pub(crate) mod rng;
pub mod routing;
pub mod scenarios;
pub mod simulation;
pub mod tuning;
pub mod vehicle;
pub mod world;

pub use barrier::{BARRIER_RESTITUTION, Barrier, collide_with_barriers, road_side_barriers};
pub use geometry::{
    BIKE_FOOTPRINT, CAR_COLLISION_RADIUS_M, CAR_FOOTPRINT, EGO_COLLISION_RADIUS_M, EGO_FOOTPRINT,
    Footprint, PEDESTRIAN_FOOTPRINT, TRUCK_FOOTPRINT,
};
pub use metrics::Metrics;
pub use planning::model::step as planner_step;
pub use planning::{
    BasicPlanner, BezierIdmPlanner, Cem, Context, IlqrPlanner, LatticePlanner, Mppi, Pi2DdpPlanner,
    Planner, PlannerKind, PredictiveSampling, RrtPlanner, RrtStarPlanner, SamplingPlanner,
    StraightPlanner, TreetopPlanner,
};
pub use scenarios::{Path, Road, Scenario};
pub use simulation::{
    Control, IncrementalSim, Pose, Position, Rollout, Simulator, State, simulate,
};
