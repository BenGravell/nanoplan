//! Cubic Bezier lane return with a TOPP-RA speed parameterization.

use crate::common::kinematics::longitudinal_resistance_accel;
use crate::common::math::wrap_angle;
use crate::geometry::barrier::collide_with_road_barriers;
use crate::geometry::{CAR_COLLISION_RADIUS_M, EGO_COLLISION_RADIUS_M};
use crate::planning::policy::centerline_feedback;
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::prediction::predict;
use crate::simulation::curvature_limit;
use crate::simulation::{Control, State, world_step};
use crate::track::Path;
use crate::vehicle::{
    AERO_DRAG_ACCEL_COEFFICIENT, MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL,
    MIN_LON_ACCEL, ROLLING_RESISTANCE_ACCEL,
};

const GRID_STEPS: usize = 100;

pub(crate) struct BezierToppraPlanner;

impl Planner for BezierToppraPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0) = ctx.time("route", || {
            let path = Path::new(ctx.road.centerline());
            let (s0, _) = path.project(ego.position());
            ctx.work(ctx.road.centerline().len() as u64);
            (path, s0)
        });
        let lookahead = (3.0 * ego.speed).max(15.0);
        let b = ctx.time("bezier_fit", || {
            let bezier = fit_bezier(ego, &path, s0, lookahead);
            ctx.work(1);
            bezier
        });
        let horizon_m = (ego.speed.max(0.0) * PLANNING_HORIZON_S
            + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2))
        .max(lookahead)
        .min(path.length() - s0);
        let ds = (horizon_m / GRID_STEPS as f64).max(1e-3);

        let speed2 = ctx.time("optimize", || {
            let mut max_speed2 = (0..=GRID_STEPS)
                .map(|i| {
                    ctx.work(1);
                    let d = i as f64 * ds;
                    let curvature = planned_curvature(&b, &path, s0, lookahead, d);
                    let lateral = if curvature.abs() > 1e-9 {
                        MAX_ABS_LAT_ACCEL / curvature.abs()
                    } else {
                        f64::INFINITY
                    };
                    if curvature.abs() > MAX_ABS_CURVATURE {
                        0.0
                    } else {
                        ctx.road.target_speed.powi(2).min(lateral)
                    }
                })
                .collect::<Vec<_>>();

            // Dynamic collision constraints are tightened monotonically from
            // the arrival times of the current parameterization.
            let mut speed2 =
                toppra_profile_clocked(ego.speed.max(0.0).powi(2), ds, &max_speed2, ctx);
            for _ in 0..2 {
                let times = arrival_times_clocked(ds, &speed2, ctx);
                let mut changed = false;
                for (i, &time) in times.iter().enumerate().skip(1) {
                    let d = i as f64 * ds;
                    let xy = planned_position(&b, &path, s0, lookahead, d);
                    if ctx.actors.iter().any(|actor| {
                        ctx.work(1);
                        let p = predict(actor, &path, time);
                        (xy[0] - p.x).hypot(xy[1] - p.y)
                            < EGO_COLLISION_RADIUS_M + CAR_COLLISION_RADIUS_M
                    }) {
                        let stop = i - 1;
                        if max_speed2[stop] != 0.0 {
                            max_speed2[stop] = 0.0;
                            changed = true;
                        }
                    }
                }
                if !changed {
                    break;
                }
                speed2 = toppra_profile_clocked(ego.speed.max(0.0).powi(2), ds, &max_speed2, ctx);
            }

            // Feed the first full-footprint road collision back into the
            // speed envelope until the complete rollout is feasible.
            for _ in 0..GRID_STEPS {
                let controls =
                    extract_controls_clocked(ego, ctx, &path, s0, &b, lookahead, ds, &speed2);
                let mut state = ego;
                let mut distance = 0.0;
                let collision = controls.into_iter().find_map(|u| {
                    ctx.work(1);
                    let prev = state;
                    distance += state.speed.max(0.0) * ctx.road.dt;
                    state = world_step(state, u, ctx.road.dt);
                    (collide_with_road_barriers(
                        prev,
                        state,
                        crate::geometry::EGO_FOOTPRINT,
                        ctx.road,
                    ) != state)
                        .then_some(distance)
                });
                let Some(distance) = collision else { break };
                let collision_index = ((distance / ds) as usize)
                    .saturating_sub(1)
                    .min(GRID_STEPS - 1);
                let Some(stop) = (0..=collision_index).rev().find(|&i| max_speed2[i] != 0.0) else {
                    break;
                };
                max_speed2[stop] = 0.0;
                speed2 = toppra_profile_clocked(ego.speed.max(0.0).powi(2), ds, &max_speed2, ctx);
            }

            speed2
        });
        ctx.time("extract", || {
            extract_controls_clocked(ego, ctx, &path, s0, &b, lookahead, ds, &speed2)
        })
    }
}

fn toppra_profile_clocked(
    start_speed2: f64,
    ds: f64,
    max_speed2: &[f64],
    ctx: &Context,
) -> Vec<f64> {
    let speed2 = toppra_profile(start_speed2, ds, max_speed2);
    ctx.work(2 * (max_speed2.len() - 1) as u64);
    speed2
}

/// Scalar TOPP-RA: backward controllable intervals, then maximum-control
/// forward propagation. `x = speed²`, `u = path acceleration` and
/// `x[i+1] = x[i] + 2 ds u[i]` (Pham & Pham, 2017, Algorithm 1).
fn toppra_profile(start_speed2: f64, ds: f64, max_speed2: &[f64]) -> Vec<f64> {
    let n = max_speed2.len() - 1;
    let mut controllable = max_speed2.to_vec();
    for i in (0..n).rev() {
        let next = predecessor_limit(controllable[i + 1], ds);
        controllable[i] = controllable[i].min(next.max(0.0));
    }

    let mut x = vec![0.0; n + 1];
    x[0] = start_speed2;
    for i in 0..n {
        let resistance = longitudinal_resistance_accel(x[i].sqrt());
        let lo = (x[i] + 2.0 * ds * (MIN_LON_ACCEL - resistance)).max(0.0);
        let hi = (x[i] + 2.0 * ds * (MAX_LON_ACCEL - resistance)).max(0.0);
        x[i + 1] = hi.min(controllable[i + 1]).max(lo);
    }
    x
}

fn predecessor_limit(next_speed2: f64, ds: f64) -> f64 {
    // Forward-speed resistance is rolling + aero*x, hence this is the closed
    // form of max x whose hardest braking can reach `next_speed2`.
    (next_speed2 - 2.0 * ds * (MIN_LON_ACCEL - ROLLING_RESISTANCE_ACCEL))
        / (1.0 - 2.0 * ds * AERO_DRAG_ACCEL_COEFFICIENT)
}

fn arrival_times(ds: f64, speed2: &[f64]) -> Vec<f64> {
    let mut times = vec![0.0; speed2.len()];
    for i in 0..speed2.len() - 1 {
        let average = 0.5 * (speed2[i].sqrt() + speed2[i + 1].sqrt());
        times[i + 1] = times[i] + ds / average.max(1e-3);
    }
    times
}

fn arrival_times_clocked(ds: f64, speed2: &[f64], ctx: &Context) -> Vec<f64> {
    let times = arrival_times(ds, speed2);
    ctx.work((speed2.len() - 1) as u64);
    times
}

#[allow(clippy::too_many_arguments)]
fn extract_controls_clocked(
    ego: State,
    ctx: &Context,
    path: &Path,
    s0: f64,
    b: &[[f64; 2]; 4],
    lookahead: f64,
    ds: f64,
    speed2: &[f64],
) -> Vec<Control> {
    let controls = extract_controls(ego, ctx, path, s0, b, lookahead, ds, speed2);
    ctx.work(ctx.horizon as u64);
    controls
}

#[allow(clippy::too_many_arguments)]
fn extract_controls(
    ego: State,
    ctx: &Context,
    path: &Path,
    s0: f64,
    b: &[[f64; 2]; 4],
    lookahead: f64,
    ds: f64,
    speed2: &[f64],
) -> Vec<Control> {
    let mut state = ego;
    let mut distance = 0.0;
    (0..ctx.horizon)
        .map(|_| {
            let i = ((distance / ds) as usize).min(speed2.len() - 2);
            let path_accel = (speed2[i + 1] - speed2[i]) / (2.0 * ds);
            let feedback = centerline_feedback(path, &state, state.speed);
            let u = Control {
                acceleration: (path_accel + longitudinal_resistance_accel(state.speed))
                    .clamp(MIN_LON_ACCEL, MAX_LON_ACCEL),
                curvature: (planned_curvature(b, path, s0, lookahead, distance)
                    + feedback.curvature)
                    .clamp(-curvature_limit(state.speed), curvature_limit(state.speed)),
            };
            distance += state.speed.max(0.0) * ctx.road.dt;
            state = world_step(state, u, ctx.road.dt);
            u
        })
        .collect()
}

fn fit_bezier(ego: State, path: &Path, s0: f64, lookahead: f64) -> [[f64; 2]; 4] {
    let (end, end_yaw) = path.pose_at(s0 + lookahead);
    let l3 = lookahead / 3.0;
    [
        [ego.x, ego.y],
        [ego.x + l3 * ego.yaw.cos(), ego.y + l3 * ego.yaw.sin()],
        [end[0] - l3 * end_yaw.cos(), end[1] - l3 * end_yaw.sin()],
        end,
    ]
}

fn planned_position(
    b: &[[f64; 2]; 4],
    path: &Path,
    s0: f64,
    lookahead: f64,
    distance: f64,
) -> [f64; 2] {
    if distance <= lookahead {
        bezier_point(b, distance / lookahead)
    } else {
        path.pose_at(s0 + distance).0
    }
}

fn planned_curvature(
    b: &[[f64; 2]; 4],
    path: &Path,
    s0: f64,
    lookahead: f64,
    distance: f64,
) -> f64 {
    if distance <= lookahead {
        bezier_curvature(b, distance / lookahead)
    } else {
        // The live road polyline is sampled about every 15 m; straddling one
        // whole sample interval avoids treating each vertex as an impulse.
        let half_window = 7.5;
        let (_, before) = path.pose_at(s0 + distance - half_window);
        let (_, after) = path.pose_at(s0 + distance + half_window);
        wrap_angle(after - before) / (2.0 * half_window)
    }
}

fn bezier_point(p: &[[f64; 2]; 4], t: f64) -> [f64; 2] {
    let mt = 1.0 - t;
    let c = [mt.powi(3), 3.0 * mt * mt * t, 3.0 * mt * t * t, t.powi(3)];
    [
        c.iter().zip(p).map(|(c, p)| c * p[0]).sum(),
        c.iter().zip(p).map(|(c, p)| c * p[1]).sum(),
    ]
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
    fn backward_pass_brakes_for_a_zero_speed_constraint() {
        let mut limits = vec![400.0; 11];
        limits[5] = 0.0;
        let x = toppra_profile(10.0_f64.powi(2), 2.0, &limits);
        assert!(x[5] < 1e-9, "speed² {}", x[5]);
        assert!(x[..5].windows(2).any(|w| w[1] < w[0]));
    }

    #[test]
    fn converges_to_centerline_and_target_speed() {
        let ego = State {
            y: 3.0,
            speed: 5.0,
            ..Default::default()
        };
        let trace = test_run(&mut BezierToppraPlanner, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.3, "offset {}", end.y);
        assert!((end.speed - 10.0).abs() < 0.5, "speed {}", end.speed);
    }

    #[test]
    fn stops_behind_stopped_actor() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let actor = State {
            x: 50.0,
            ..Default::default()
        };
        let trace = test_run(&mut BezierToppraPlanner, ego, &[actor], 300);
        let end = trace.last().unwrap();
        assert!(end.speed < 0.5, "speed {}", end.speed);
        assert!(
            end.x <= actor.x - crate::geometry::EGO_FOOTPRINT.length + 1e-9,
            "x {}",
            end.x
        );
    }

    #[test]
    fn follows_a_slower_lead_without_contact() {
        use crate::geometry::{CAR_FOOTPRINT, EGO_FOOTPRINT, footprints_overlap};
        use crate::planning::{test_ctx, test_road};
        use crate::simulation::CommandLimiter;

        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let mut planner = BezierToppraPlanner;
        let mut limiter = CommandLimiter::new();
        let mut ego = State::default();
        let mut lead = State {
            x: 55.0,
            speed: 7.0,
            ..Default::default()
        };

        for tick in 0..300 {
            lead.x += lead.speed * road.dt;
            let actors = [lead];
            let controls = planner.plan(ego, &test_ctx(&road, &actors));
            ego = limiter.step(ego, controls[0], road.dt);
            assert!(
                !footprints_overlap(ego.pose(), EGO_FOOTPRINT, lead.pose(), CAR_FOOTPRINT),
                "contact at tick {tick}: ego {ego:?}, lead {lead:?}"
            );
        }
    }

    #[test]
    fn generated_track_predictions_stay_inside_road_for_full_horizon() {
        use crate::geometry::barrier::collides_with_road_barrier;
        use crate::simulation::MAX_TERMINAL_SPEED_MPS;
        use crate::track::{Road, Track};

        for seed in 0..10 {
            let track = Track::new(seed);
            let lap = track.lap_length().unwrap();
            for n in 0..20 {
                let progress = lap * n as f64 / 20.0;
                let road = Road::new(
                    track.centerline(progress - 50.0, progress + 250.0, 15.0),
                    *MAX_TERMINAL_SPEED_MPS,
                    track.half_width(progress),
                    0.1,
                );
                let (p, yaw) = track.pose(progress);
                let ego = State {
                    x: p[0],
                    y: p[1],
                    yaw,
                    speed: 20.0,
                };
                let ctx = Context {
                    road: &road,
                    actors: &[],
                    horizon: 100,
                    latency: None,
                    diagnostics: None,
                };
                let path = Path::new(road.centerline());
                let mut state = ego;
                for (tick, control) in BezierToppraPlanner.plan(ego, &ctx).into_iter().enumerate() {
                    state = world_step(state, control, road.dt);
                    let (s, d) = path.project(state.position());
                    assert!(
                        !collides_with_road_barrier(state, &road),
                        "seed {seed} progress {progress} width {} tick {tick} d {d} heading_err {} control {control:?} state {state:?}",
                        road.half_width,
                        wrap_angle(state.yaw - path.pose_at(s).1)
                    );
                }
            }
        }
    }
}
