//! EM/lattice-style planner. Samples a deterministic grid of (station,
//! lateral) points in the road Frenet frame, connects successive layers into
//! a tree with cubic-in-time lateral segments, assigns node costs (offset,
//! smoothness, predicted-obstacle proximity), and picks the best path.

use crate::planning::{Context, PLANNING_HORIZON_S, Planner};
use crate::scenarios::Path;
use crate::simulation::{Control, State};
use crate::wrap_angle;

pub struct LatticePlanner;

const LATERALS_M: [f64; 5] = [-3.5, -1.75, 0.0, 1.75, 3.5];
const SAMPLES_PER_SEGMENT: usize = 8;
// center-to-center; slightly over one car width plus margin
const COLLISION_RADIUS_M: f64 = 2.5;

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
        // three evenly spaced layers reaching out to the full prediction
        // horizon at the assumed cruise speed
        let stations_m = [
            v * PLANNING_HORIZON_S / 3.0,
            v * PLANNING_HORIZON_S * 2.0 / 3.0,
            v * PLANNING_HORIZON_S,
        ];
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

        // cost of one lattice edge, integrating over sampled points;
        // custom seam: the hot loop of the DP search
        let edge_cost = |sa: f64, da: f64, sb: f64, db: f64, m0: f64| -> f64 {
            ctx.time("edge_costs", || {
                let mut cost = 2.0 * (db - da).powi(2); // lateral smoothness
                for i in 1..=SAMPLES_PER_SEGMENT {
                    let u = i as f64 / SAMPLES_PER_SEGMENT as f64;
                    let s = sa + (sb - sa) * u;
                    let d = d_shape(da, db, m0, u);
                    cost += d * d / SAMPLES_PER_SEGMENT as f64; // stay near centerline
                    let p = path.frenet_to_xy(s, d);
                    let t = (s - s0) / v; // time when the ego gets there
                    for a in ctx.actors {
                        // constant-velocity prediction of the actor
                        let q = [
                            a.x + a.speed * a.yaw.cos() * t,
                            a.y + a.speed * a.yaw.sin() * t,
                        ];
                        let gap = (p[0] - q[0]).hypot(p[1] - q[1]);
                        if gap < COLLISION_RADIUS_M {
                            return f64::INFINITY;
                        }
                        cost += 20.0 / (gap * gap);
                    }
                }
                cost
            })
        };

        // DP over layers: the lattice is a layered DAG, so dynamic programming
        // finds the exact best path (A* would just add bookkeeping).
        // (the nested edge_costs seam accounts for most of this)
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
                        prev.iter()
                            .map(|(da, c, laterals)| {
                                let mut l = laterals.clone();
                                l.push(db);
                                let m0 = if layer == 0 { m0_first } else { 0.0 };
                                (db, c + edge_cost(sa, *da, sb, db, m0), l)
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
        ctx.time("extract", || {
            let pts: Vec<[f64; 2]> = (1..=ctx.horizon.max((stations_m[2] / (v * ctx.dt)) as usize))
                .map(|i| {
                    let s_rel = (v * ctx.dt * i as f64).min(stations_m[2]);
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
}
