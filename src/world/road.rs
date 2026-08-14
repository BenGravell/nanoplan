use crate::common::kinematics::net_longitudinal_accel;
use crate::planning::PLANNING_HORIZON_S;
use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::track::{ROAD_SAMPLE_STEP_M, Road, Track};
use crate::vehicle::MAX_LON_ACCEL;

const ROAD_BEHIND_M: f64 = 50.0;
pub(super) const ROAD_AHEAD_M: f64 = 250.0;
const ROAD_LOOKAHEAD_MARGIN_M: f64 = 25.0;

fn planning_lookahead_m(mut speed: f64, dt: f64) -> f64 {
    let ticks = (PLANNING_HORIZON_S / dt).ceil() as usize;
    let mut reachable = 0.0;
    for _ in 0..ticks {
        reachable += speed.max(0.0) * dt;
        speed = (speed + net_longitudinal_accel(MAX_LON_ACCEL, speed) * dt).max(0.0);
    }

    // The planner only evaluates PLANNING_HORIZON_S. Extending the road by
    // the stopping distance *after* that horizon made its barrier scans grow
    // with a trajectory it could never select, especially on fast straights.
    reachable + ROAD_LOOKAHEAD_MARGIN_M
}

pub(super) fn road_window(track: &Track, x: f64, speed: f64, dt: f64, reachability_sized: bool) -> Road {
    let ahead = if reachability_sized {
        planning_lookahead_m(speed, dt)
    } else {
        ROAD_AHEAD_M
    };
    let polygon = track
        .road_polygon(x - ROAD_BEHIND_M, x + ahead, ROAD_SAMPLE_STEP_M, false)
        .expect("track road window must form a valid polygon");
    Road::from_polygon(polygon, *MAX_TERMINAL_SPEED_MPS, dt)
}

pub(super) fn full_circuit_road(track: &Track, dt: f64) -> Road {
    let length = track
        .lap_length()
        .expect("the live driving world requires a closed circuit");
    let polygon = track
        .road_polygon(0.0, length, ROAD_SAMPLE_STEP_M, true)
        .expect("track road must form a valid closed polygon");
    Road::from_polygon(polygon, *MAX_TERMINAL_SPEED_MPS, dt)
}
