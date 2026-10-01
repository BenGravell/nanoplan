#[cfg(test)]
use crate::track::ROAD_SAMPLE_STEP_M;
pub(super) use crate::track::prepared::{ROAD_REFRESH_DISTANCE_M, needs_road_window_update};
use crate::track::{Road, Track};

pub(super) fn road_window(track: &Track, x: f64, speed: f64, dt: f64) -> Road {
    track.prepared().window(x, speed, dt)
}

pub(super) fn full_circuit_road(track: &Track, dt: f64) -> Road {
    let mut road = track.prepared().collision.clone();
    road.dt = dt;
    road
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
                    let expected = hint + delta;
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
