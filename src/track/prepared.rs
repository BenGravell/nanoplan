//! Track-selection preparation. Memory is O(lap length + maximum lookahead),
//! not one independently processed road per possible planning window.
use super::{ROAD_SAMPLE_STEP_M, Road, RoadView, Track};
use crate::common::geometry::RoadPolygon;
use crate::common::kinematics::net_longitudinal_accel;
use crate::planning::{PLANNING_DT_S, PLANNING_HORIZON_S};
use crate::simulation::MAX_TERMINAL_SPEED_MPS;
use crate::vehicle::MAX_LON_ACCEL;

pub(crate) const ROAD_BEHIND_M: f64 = 50.0;
pub(crate) const VISIBLE_TRACK_BEHIND_M: f64 = 250.0;
pub(crate) const VISIBLE_TRACK_AHEAD_M: f64 = 750.0;
const ROAD_LOOKAHEAD_MARGIN_M: f64 = 25.0;
pub(crate) const ROAD_REFRESH_DISTANCE_M: f64 = 20.0;

#[derive(Debug)]
pub(crate) struct PreparedRoads {
    pub(crate) planning: Road,
    pub(crate) collision: Road,
    lap: f64,
    samples: usize,
    prefix: usize,
}

impl PreparedRoads {
    pub(crate) fn new(track: &Track, initial_speed: f64, dt: f64) -> Self {
        let lap = track.lap_length().unwrap();
        let samples = (lap / ROAD_SAMPLE_STEP_M).ceil() as usize;
        // Keep the existing metre grid. The final (shorter) segment closes
        // the lap; later laps reuse these exact stations and boundary miters.
        let step = ROAD_SAMPLE_STEP_M;
        let points = (0..samples).map(|i| track.point(i as f64 * step)).collect();
        let (right, left) = (0..samples).map(|i| track.widths(i as f64 * step)).unzip();
        let polygon = RoadPolygon::new(points, right, left, true).expect("prepared track must be a valid road");
        let geometry = track.reference_geometry(0.0, lap, step, true);
        let prefix = (VISIBLE_TRACK_BEHIND_M.max(ROAD_BEHIND_M) / step).ceil() as usize + 1;
        // Keep enough repeated geometry for the selected starting conditions
        // as well as normal driving, including the window refresh margin.
        let reach = *MAX_TERMINAL_SPEED_MPS * PLANNING_HORIZON_S
            + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2)
            + ROAD_LOOKAHEAD_MARGIN_M
            + ROAD_REFRESH_DISTANCE_M;
        let reach = reach
            .max(planning_lookahead_m(initial_speed.max(*MAX_TERMINAL_SPEED_MPS), dt) + ROAD_REFRESH_DISTANCE_M)
            .max(VISIBLE_TRACK_AHEAD_M);
        let count = prefix + sample_index(lap + reach, lap, samples).ceil() as usize + 2;
        let mut planning = Road::from_polygon(polygon.repeated(-(prefix as isize), count), PLANNING_DT_S);
        planning.set_reference_geometry(
            (0..count)
                .map(|i| geometry[(i + samples - prefix % samples) % samples])
                .collect(),
        );
        planning.prepare();
        let mut collision = Road::from_polygon(polygon, PLANNING_DT_S);
        collision.set_reference_geometry(geometry[..samples].to_vec());
        collision.prepare();
        Self {
            planning,
            collision,
            lap,
            samples,
            prefix,
        }
    }

    pub(crate) fn visible_polygon(&self, progress: f64) -> RoadPolygon {
        if self.lap <= VISIBLE_TRACK_BEHIND_M + VISIBLE_TRACK_AHEAD_M {
            return self.collision.polygon().clone();
        }
        let x = wrapped_progress(progress, self.lap);
        let at = |p| sample_index(p, self.lap, self.samples) + self.prefix as f64;
        self.planning
            .polygon()
            .window(at(x - VISIBLE_TRACK_BEHIND_M).floor() as usize..at(x + VISIBLE_TRACK_AHEAD_M).ceil() as usize + 1)
    }

    pub(crate) fn window(&self, x: f64, speed: f64, dt: f64) -> Road {
        let x = wrapped_progress(x, self.lap);
        let at = |progress| sample_index(progress, self.lap, self.samples) + self.prefix as f64;
        let anchor = at(x);
        let start = at(x - ROAD_BEHIND_M).floor().max(0.0) as usize;
        let end = at(x + planning_lookahead_m(speed, dt)).ceil() as usize + 1;
        let station = self.planning.station_at_sample(anchor) - self.planning.station_at_sample(start as f64);
        let mut road = self.planning.with_view(RoadView {
            range: start..end,
            ego_projection_window: Some((station, ROAD_REFRESH_DISTANCE_M + ROAD_SAMPLE_STEP_M)),
        });
        road.dt = dt;
        road
    }
}

fn wrapped_progress(progress: f64, lap: f64) -> f64 {
    let local = progress.rem_euclid(lap);
    if local < 1e-9 || lap - local < 1e-9 { 0.0 } else { local }
}

fn sample_index(progress: f64, lap: f64, samples: usize) -> f64 {
    let local = progress.rem_euclid(lap);
    let last = (samples - 1) as f64 * ROAD_SAMPLE_STEP_M;
    let index = if local <= last {
        local / ROAD_SAMPLE_STEP_M
    } else {
        (samples - 1) as f64 + (local - last) / (lap - last)
    };
    let index = (progress / lap).floor() * samples as f64 + index;
    // Keep floor/ceil window bounds stable when accumulated lap arithmetic
    // lands a few ulps either side of an exact sample.
    if (index - index.round()).abs() < 1e-9 {
        index.round()
    } else {
        index
    }
}

fn planning_lookahead_m(mut speed: f64, dt: f64) -> f64 {
    let ticks = (PLANNING_HORIZON_S / dt).ceil() as usize;
    let mut reachable = 0.0;
    for _ in 0..ticks {
        reachable += speed.max(0.0) * dt;
        speed = (speed + net_longitudinal_accel(MAX_LON_ACCEL, speed) * dt).max(0.0);
    }
    reachable + ROAD_LOOKAHEAD_MARGIN_M
}

pub(crate) fn needs_road_window_update(road: &Road, distance_from_anchor: f64, speed: f64) -> bool {
    let remaining = road.length() - ROAD_BEHIND_M - distance_from_anchor;
    distance_from_anchor.abs() >= ROAD_REFRESH_DISTANCE_M
        || remaining < planning_lookahead_m(speed, road.dt) - ROAD_LOOKAHEAD_MARGIN_M
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::kinematics::TrajectoryKinematics;
    use crate::planning::{ComputeBudget, Context, latency::geometry_build_clocks};
    use crate::simulation::{Control, State};

    #[test]
    fn prepared_tracks_build_no_geometry_for_windows_contexts_metrics_or_rendering() {
        for index in 0..crate::track::TRACK_PRESETS.len() + crate::track::TRACK_CATALOG.len() {
            let track = Track::from_catalog(index);
            let prepared = track.prepared();
            let builds = geometry_build_clocks();
            for anchor in [
                -2.0 * prepared.lap - 0.2,
                -0.1,
                0.0,
                prepared.lap - 0.1,
                3.0 * prepared.lap + 50.0,
            ] {
                for speed in [0.0, 40.0, *MAX_TERMINAL_SPEED_MPS] {
                    let road = prepared.window(anchor, speed, 0.1);
                    let cloned = road.clone();
                    let worker_view = prepared.planning.with_view(cloned.view());
                    assert_eq!(road.centerline(), worker_view.centerline());
                    assert_eq!(road.polygon(), worker_view.polygon());
                    let (position, yaw) = track.pose(anchor);
                    let ego = State::from((position, yaw, speed));
                    let ctx = Context::new(&worker_view, &[], 100, ComputeBudget::NOMINAL, None, None);
                    let start = ctx.project_ego(ego);
                    assert!((start.s - road.ego_projection_window.unwrap().0).abs() < 0.1);
                    let path = ctx.path();
                    assert!(path.project_near(position, start.s, 21.0).d.abs() < 0.1);
                    road.closest_centerline_segment(position).unwrap();
                    let trajectory = TrajectoryKinematics::new(vec![ego; 2], vec![Control::default(); 2], road.dt);
                    crate::metrics::evaluate(&trajectory.states, trajectory.dt, &road.path(), None);
                    let visible = prepared.visible_polygon(anchor);
                    assert!(visible.centerline().len() > 2);
                }
            }
            assert_eq!(
                geometry_build_clocks(),
                builds,
                "track {index}: runtime rebuilt static geometry"
            );
        }
    }

    #[test]
    fn custom_high_speed_is_prepared_before_windows_are_requested() {
        let track = Track::from_catalog(1);
        let speed = 1_000.0;
        let prepared = track.prepare(speed, 0.1);
        let builds = geometry_build_clocks();
        for speed in [speed, *MAX_TERMINAL_SPEED_MPS, 0.0] {
            let road = prepared.window(prepared.lap - 0.1, speed, 0.1);
            assert!(road.length() >= ROAD_BEHIND_M + planning_lookahead_m(speed, 0.1) - 1.0);
        }
        assert_eq!(geometry_build_clocks(), builds);
    }

    #[test]
    fn lap_seam_views_reuse_identical_geometry_and_cover_braking_horizon() {
        let track = Track::from_catalog(1);
        let prepared = track.prepared();
        for anchor in [-0.2, 0.0, prepared.lap - 0.2, prepared.lap + 0.2] {
            let a = prepared.window(anchor, *MAX_TERMINAL_SPEED_MPS, 0.1);
            let b = prepared.window(anchor + 10.0 * prepared.lap, *MAX_TERMINAL_SPEED_MPS, 0.1);
            assert_eq!(a.centerline(), b.centerline());
            assert!(a.polygon().same_geometry(b.polygon()));
            assert!(a.length() > *MAX_TERMINAL_SPEED_MPS * PLANNING_HORIZON_S);
        }
    }
}
