use crate::common::kinematics::net_longitudinal_accel;
use crate::planning::PLANNING_HORIZON_S;
use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::track::{ROAD_SAMPLE_STEP_M, Road, Track};
use crate::vehicle::MAX_LON_ACCEL;

const ROAD_BEHIND_M: f64 = 50.0;
const ROAD_LOOKAHEAD_MARGIN_M: f64 = 25.0;
pub(super) const ROAD_REFRESH_DISTANCE_M: f64 = 20.0;

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

pub(super) fn road_window(track: &Track, x: f64, speed: f64, dt: f64) -> Road {
    let ahead = planning_lookahead_m(speed, dt);
    let polygon = track
        .road_polygon(x - ROAD_BEHIND_M, x + ahead, ROAD_SAMPLE_STEP_M, false)
        .expect("track road window must form a valid polygon");
    // Convert the anchor's sample index to chord length along this window.
    // Track progress and polyline arc length differ slightly on bends.
    let anchor_index = x / ROAD_SAMPLE_STEP_M - ((x - ROAD_BEHIND_M) / ROAD_SAMPLE_STEP_M).floor();
    let anchor_station = polygon
        .centerline()
        .windows(2)
        .enumerate()
        .map(|(i, pair)| pair[0].distance(pair[1]) * (anchor_index - i as f64).clamp(0.0, 1.0))
        .sum();
    let mut road = Road::from_polygon(polygon, *MAX_TERMINAL_SPEED_MPS, dt);
    road.ego_projection_window = Some((anchor_station, ROAD_REFRESH_DISTANCE_M + ROAD_SAMPLE_STEP_M));
    road
}

pub(super) fn needs_road_window_update(road: &Road, distance_from_anchor: f64, speed: f64) -> bool {
    let remaining = road.length() - ROAD_BEHIND_M - distance_from_anchor;
    distance_from_anchor.abs() >= ROAD_REFRESH_DISTANCE_M
        || remaining < planning_lookahead_m(speed, road.dt) - ROAD_LOOKAHEAD_MARGIN_M
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::test_ctx;
    use crate::simulation::{Position, State};
    use crate::track::{TRACK_CATALOG, TRACK_PRESETS};

    #[test]
    fn ego_projection_window_covers_refresh_interval_on_every_track() {
        for index in 0..TRACK_PRESETS.len() + TRACK_CATALOG.len() {
            let track = Track::from_catalog(index);
            let lap = track.lap_length().unwrap();
            // Include negative, fractional, and wrapped anchors. High speed
            // makes the small circuits repeat inside the planning window.
            for i in -1..=64 {
                let anchor = lap * i as f64 / 64.0 + 0.37;
                let road = road_window(&track, anchor, 40.0, 0.1);
                let (hint, radius) = road.ego_projection_window.unwrap();
                let ctx = test_ctx(&road, &[]);
                assert_eq!(radius, ROAD_REFRESH_DISTANCE_M + ROAD_SAMPLE_STEP_M);
                // project_near includes the segments crossing either bound.
                assert!(lap > 2.0 * (radius + ROAD_SAMPLE_STEP_M));
                for delta in [-19.99, 0.0, 19.99] {
                    let progress = anchor + delta;
                    let first = ((anchor - ROAD_BEHIND_M) / ROAD_SAMPLE_STEP_M).floor();
                    let sample_index = progress / ROAD_SAMPLE_STEP_M - first;
                    let expected: f64 = road
                        .centerline()
                        .windows(2)
                        .enumerate()
                        .map(|(j, pair)| pair[0].distance(pair[1]) * (sample_index - j as f64).clamp(0.0, 1.0))
                        .sum();
                    assert!((expected - hint).abs() < radius);
                    let (center, yaw) = track.pose(progress);
                    for offset in [-0.5, 0.0, 0.5] {
                        let position = center
                            + Position::from_angle(yaw + std::f64::consts::FRAC_PI_2)
                                * (offset * track.half_width(progress));
                        let ego = State::from((position, yaw, 40.0));
                        let projected = ctx.project_ego(ego).0;
                        assert!(
                            (projected - expected).abs() < 0.5,
                            "track {index}, anchor {anchor}, delta {delta}, offset {offset}: {projected} vs {expected}"
                        );
                    }
                }
            }
            assert!(full_circuit_road(&track, 0.1).ego_projection_window.is_none());
        }
    }
}
