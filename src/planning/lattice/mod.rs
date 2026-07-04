//! EM/lattice-style planner. Samples a deterministic grid of (station,
//! lateral) points in the road Frenet frame, connects successive layers into
//! a tree with cubic-in-time lateral segments, assigns node costs (offset,
//! smoothness, predicted-obstacle proximity), and picks the best path.

use crate::planning::cost::{self, Sample};
use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State};
use crate::wrap_angle;

pub struct LatticePlanner;

const LATERALS_M: [f64; 9] = [-3.5, -2.625, -1.75, -0.875, 0.0, 0.875, 1.75, 2.625, 3.5];
const STATION_LAYERS: usize = 5;
const SAMPLES_PER_SEGMENT: usize = 8;

/// Turn a sampled position trajectory (spaced `dt` apart, starting one tick
/// after the ego) into controls for the kinematic model.
fn xy_to_controls(ego: State, pts: &[[f64; 2]], dt: f64) -> Vec<Control> {
    let mut v = ego.speed;
    let mut yaw = ego.yaw;
    let mut prev = [ego.x, ego.y];
    pts.iter()
        .map(|&p| {
            let ds = (p[0] - prev[0]).hypot(p[1] - prev[1]);
            let new_v = ds / dt;
            let new_yaw = if ds > 1e-6 {
                (p[1] - prev[1]).atan2(p[0] - prev[0])
            } else {
                yaw
            };
            let u = Control {
                accel: (new_v - v) / dt,
                curvature: if ds > 1e-6 {
                    wrap_angle(new_yaw - yaw) / ds
                } else {
                    0.0
                },
            };
            (v, yaw, prev) = (new_v, new_yaw, p);
            u
        })
        .collect()
}

impl Planner for LatticePlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let (path, s0, d0) = ctx.time("route", || {
            let path = Path::new(ctx.centerline);
            let (s0, d0) = path.project([ego.x, ego.y]);
            (path, s0, d0)
        });
        // ponytail: constant-speed profile; couple IDM into the lattice when needed
        let v = ego.speed.clamp(2.0, ctx.target_speed.max(2.0));
        // STATION_LAYERS evenly spaced layers reaching out to the full
        // prediction horizon at the assumed cruise speed
        let stations_m: [f64; STATION_LAYERS] = std::array::from_fn(|i| {
            v * PLANNING_HORIZON_S * (i + 1) as f64 / STATION_LAYERS as f64
        });
        // initial lateral rate, expressed per unit of segment parameter u; the
        // first segment must honor it or every replan restarts the swerve at
        // zero slope and the executed path lags the plan into obstacles
        let (_, lane_yaw) = path.pose_at(s0);
        let m0_first = ego.speed * wrap_angle(ego.yaw - lane_yaw).sin() * (stations_m[0] / v);
        // cubic Hermite in u with start slope m0 and flat end
        let d_shape = |da: f64, db: f64, m0: f64, u: f64| {
            let (u2, u3) = (u * u, u * u * u);
            (2.0 * u3 - 3.0 * u2 + 1.0) * da + (u3 - 2.0 * u2 + u) * m0 + (3.0 * u2 - 2.0 * u3) * db
        };

        // cost of one lattice edge: planner-specific lateral-smoothness and
        // centerline-pull terms (structural to the DP search itself) plus
        // the shared cost of each sampled point, timed under the "cost"
        // seam so it's comparable across planners. Curvature at each point
        // is a numerical estimate off the last three sampled points
        // (`cost::curvature_of`) — the lattice has no closed-form curve to
        // evaluate directly, unlike RRT*'s steering function.
        let edge_cost = |sa: f64, da: f64, sb: f64, db: f64, m0: f64| -> f64 {
            let mut total = 2.0 * (db - da).powi(2); // lateral smoothness
            let mut prev2: Option<[f64; 2]> = None;
            let mut prev1 = path.frenet_to_xy(sa, da);
            for i in 1..=SAMPLES_PER_SEGMENT {
                let u = i as f64 / SAMPLES_PER_SEGMENT as f64;
                let s = sa + (sb - sa) * u;
                let d = d_shape(da, db, m0, u);
                total += d * d / SAMPLES_PER_SEGMENT as f64; // stay near centerline
                let p = path.frenet_to_xy(s, d);
                let curvature = prev2.map_or(0.0, |p0| cost::curvature_of(p0, prev1, p));
                let sample = Sample {
                    xy: p,
                    lateral: d,
                    speed: v,
                    curvature,
                    t: (s - s0) / v, // time when the ego gets there
                    ..Default::default()
                };
                let point = ctx.time("cost", || {
                    cost::point_cost(&sample, ctx.target_speed, ctx.actors)
                });
                if point.is_infinite() {
                    return f64::INFINITY;
                }
                total += point;
                prev2 = Some(prev1);
                prev1 = p;
            }
            total
        };

        // DP over layers: the lattice is a layered DAG, so dynamic programming
        // finds the exact best path (A* would just add bookkeeping).
        // (the nested "cost" seam accounts for most of this)
        if let Some(diag) = ctx.diagnostics {
            diag.record_point(path.frenet_to_xy(s0, d0)); // tree root
        }
        let mut prev: Vec<(f64, f64, Vec<f64>)> = vec![(d0, 0.0, vec![])]; // (d, cost, laterals so far)
        ctx.time("optimize", || {
            for (layer, ds) in stations_m.iter().enumerate() {
                let sa = s0
                    + if layer == 0 {
                        0.0
                    } else {
                        stations_m[layer - 1]
                    };
                let sb = s0 + ds;
                prev = LATERALS_M
                    .iter()
                    .map(|&db| {
                        if let Some(diag) = ctx.diagnostics {
                            diag.record_point(path.frenet_to_xy(sb, db));
                        }
                        prev.iter()
                            .map(|(da, c, laterals)| {
                                let mut l = laterals.clone();
                                l.push(db);
                                let m0 = if layer == 0 { m0_first } else { 0.0 };
                                let edge_cost = c + edge_cost(sa, *da, sb, db, m0);
                                if let Some(diag) = ctx.diagnostics {
                                    // sample the cubic Hermite connector between the
                                    // two grid nodes, for the diagnostic overlay
                                    let traj = (0..=SAMPLES_PER_SEGMENT)
                                        .map(|i| {
                                            let u = i as f64 / SAMPLES_PER_SEGMENT as f64;
                                            let s = sa + (sb - sa) * u;
                                            let d = d_shape(*da, db, m0, u);
                                            path.frenet_to_xy(s, d)
                                        })
                                        .collect();
                                    diag.record_trajectory(traj);
                                }
                                (db, edge_cost, l)
                            })
                            .min_by(|a, b| a.1.total_cmp(&b.1))
                            .unwrap()
                    })
                    .collect();
            }
        });
        let (_, best_cost, laterals) = prev.into_iter().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
        if best_cost.is_infinite() {
            // every path collides: brake straight ahead
            return vec![
                Control {
                    accel: -4.0,
                    curvature: 0.0,
                };
                ctx.horizon
            ];
        }

        // sample the winning path over time; d is cubic in t on each segment
        let s_max = *stations_m.last().unwrap();
        ctx.time("extract", || {
            let pts: Vec<[f64; 2]> = (1..=ctx.horizon.max((s_max / (v * ctx.dt)) as usize))
                .map(|i| {
                    let s_rel = (v * ctx.dt * i as f64).min(s_max);
                    let seg = stations_m.iter().position(|&m| s_rel <= m).unwrap();
                    let (sa, da) = if seg == 0 {
                        (0.0, d0)
                    } else {
                        (stations_m[seg - 1], laterals[seg - 1])
                    };
                    let u = (s_rel - sa) / (stations_m[seg] - sa);
                    let m0 = if seg == 0 { m0_first } else { 0.0 };
                    let d = d_shape(da, laterals[seg], m0, u);
                    path.frenet_to_xy(s0 + s_rel, d)
                })
                .collect();
            xy_to_controls(ego, &pts, ctx.dt)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::test_run;

    #[test]
    fn stays_on_empty_centerline() {
        let ego = State {
            y: 1.5,
            speed: 8.0,
            ..Default::default()
        };
        let trace = test_run(&mut LatticePlanner, ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.5, "offset {}", end.y);
    }

    #[test]
    fn swerves_around_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = test_run(&mut LatticePlanner, ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        let end = trace.last().unwrap();
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(end.x > 60.0, "did not pass the obstacle, x {}", end.x);
    }

    #[test]
    fn records_diagnostics_when_requested() {
        use crate::planning::Diagnostics;

        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let diag = Diagnostics::default();
        let mut ctx = crate::planning::test_ctx(&[[-20.0, 0.0], [400.0, 0.0]], &[]);
        ctx.diagnostics = Some(&diag);
        LatticePlanner.plan(ego, &ctx);
        let data = diag.take();
        // tree root + STATION_LAYERS * LATERALS_M.len() grid nodes
        assert_eq!(data.points.len(), 1 + STATION_LAYERS * LATERALS_M.len());
        // layer 0 has 1 predecessor, layers 1.. have LATERALS_M.len() each
        let edges = LATERALS_M.len() + (STATION_LAYERS - 1) * LATERALS_M.len() * LATERALS_M.len();
        assert_eq!(data.trajectories.len(), edges);
        assert!(
            data.trajectories
                .iter()
                .all(|t| t.len() == SAMPLES_PER_SEGMENT + 1)
        );
    }
}
