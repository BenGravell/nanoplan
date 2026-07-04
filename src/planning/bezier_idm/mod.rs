//! Steer toward the lane centerline along a cubic Bezier path, with the
//! Intelligent Driver Model for the speed profile.

use crate::planning::{Context, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State};

/// Intelligent Driver Model acceleration. `lead` is (gap, lead speed).
fn idm_accel(v: f64, target: f64, lead: Option<(f64, f64)>) -> f64 {
    const HEADWAY_S: f64 = 1.5;
    const MAX_ACCEL: f64 = 1.5;
    const COMFORT_DECEL: f64 = 2.0;
    const MIN_GAP_M: f64 = 2.0;
    let free = 1.0 - (v / target.max(0.1)).powi(4);
    let interaction = match lead {
        Some((gap, lead_v)) => {
            let desired_gap = MIN_GAP_M
                + (v * HEADWAY_S + v * (v - lead_v) / (2.0 * (MAX_ACCEL * COMFORT_DECEL).sqrt()))
                    .max(0.0);
            (desired_gap / gap.max(0.1)).powi(2)
        }
        None => 0.0,
    };
    MAX_ACCEL * (free - interaction)
}

/// Nearest in-lane actor ahead of station `s0`: (gap, speed along the lane).
fn lead_vehicle(path: &Path, s0: f64, actors: &[State]) -> Option<(f64, f64)> {
    const CAR_LENGTH_M: f64 = 5.0;
    actors
        .iter()
        .filter_map(|a| {
            let (s, d) = path.project([a.x, a.y]);
            let (_, lane_yaw) = path.pose_at(s);
            (d.abs() < 2.0 && s > s0)
                .then(|| (s - s0 - CAR_LENGTH_M, a.speed * (a.yaw - lane_yaw).cos()))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

pub struct BezierIdmPlanner;

impl Planner for BezierIdmPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0) = ctx.time("route", || {
            let path = Path::new(&ctx.road.centerline);
            let (s0, _) = path.project([ego.x, ego.y]);
            (path, s0)
        });
        // custom seam: fitting the lane-return Bezier
        let b = ctx.time("bezier_fit", || {
            let lookahead = (3.0 * ego.speed).max(15.0);
            let (end, end_yaw) = path.pose_at(s0 + lookahead);
            let l3 = lookahead / 3.0;
            // ends tangent to the ego heading and the lane heading
            [
                [ego.x, ego.y],
                [ego.x + l3 * ego.yaw.cos(), ego.y + l3 * ego.yaw.sin()],
                [end[0] - l3 * end_yaw.cos(), end[1] - l3 * end_yaw.sin()],
                end,
            ]
        });
        // custom seam: scanning the actors for the in-lane lead
        let mut lead = ctx.time("lead_search", || lead_vehicle(&path, s0, ctx.actors));
        let mut v = ego.speed;
        let mut t = 0.0;
        ctx.time("extract", || {
            (0..ctx.horizon)
                .map(|_| {
                    let accel = idm_accel(v, ctx.road.target_speed, lead);
                    let u = Control {
                        accel,
                        curvature: bezier_curvature(&b, t),
                    };
                    v = (v + accel * ctx.road.dt).max(0.0);
                    let d1 = bezier_d1(&b, t);
                    t = (t + v * ctx.road.dt / d1[0].hypot(d1[1]).max(1e-6)).min(1.0);
                    if let Some((gap, lead_v)) = &mut lead {
                        *gap = (*gap + (*lead_v - v) * ctx.road.dt).max(0.0);
                    }
                    u
                })
                .collect()
        })
    }
}

fn bezier_d1(p: &[[f64; 2]; 4], t: f64) -> [f64; 2] {
    let mt = 1.0 - t;
    let c = [3.0 * mt * mt, 6.0 * mt * t, 3.0 * t * t];
    [
        c[0] * (p[1][0] - p[0][0]) + c[1] * (p[2][0] - p[1][0]) + c[2] * (p[3][0] - p[2][0]),
        c[0] * (p[1][1] - p[0][1]) + c[1] * (p[2][1] - p[1][1]) + c[2] * (p[3][1] - p[2][1]),
    ]
}

fn bezier_d2(p: &[[f64; 2]; 4], t: f64) -> [f64; 2] {
    let mt = 1.0 - t;
    [
        6.0 * mt * (p[2][0] - 2.0 * p[1][0] + p[0][0])
            + 6.0 * t * (p[3][0] - 2.0 * p[2][0] + p[1][0]),
        6.0 * mt * (p[2][1] - 2.0 * p[1][1] + p[0][1])
            + 6.0 * t * (p[3][1] - 2.0 * p[2][1] + p[1][1]),
    ]
}

fn bezier_curvature(p: &[[f64; 2]; 4], t: f64) -> f64 {
    let d1 = bezier_d1(p, t);
    let d2 = bezier_d2(p, t);
    let speed = d1[0].hypot(d1[1]).max(1e-6);
    (d1[0] * d2[1] - d1[1] * d2[0]) / speed.powi(3)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::test_run;

    #[test]
    fn converges_to_centerline_and_target_speed() {
        let ego = State {
            y: 3.0,
            speed: 5.0,
            ..Default::default()
        };
        let trace = test_run(&mut BezierIdmPlanner, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.3, "offset {}", end.y);
        assert!((end.speed - 10.0).abs() < 0.5, "speed {}", end.speed);
    }

    #[test]
    fn stops_behind_stopped_lead() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let lead = State {
            x: 50.0,
            ..Default::default()
        };
        let trace = test_run(&mut BezierIdmPlanner, ego, &[lead], 300);
        let end = trace.last().unwrap();
        assert!(end.speed < 0.5, "speed {}", end.speed);
        assert!(end.x < 45.0, "x {}", end.x); // stopped short of the lead
    }
}
