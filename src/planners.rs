//! Planners beyond the strawman, plus the path/Frenet helpers they share.

use crate::{Context, Control, Planner, State};

/// A polyline path with arc-length lookup and Frenet projection.
pub struct Path {
    pts: Vec<[f64; 2]>,
    /// Cumulative arc length at each point.
    s: Vec<f64>,
}

impl Path {
    pub fn new(pts: &[[f64; 2]]) -> Self {
        let mut s = vec![0.0];
        for w in pts.windows(2) {
            s.push(s.last().unwrap() + dist(w[0], w[1]));
        }
        Path {
            pts: pts.to_vec(),
            s,
        }
    }

    pub fn length(&self) -> f64 {
        *self.s.last().unwrap()
    }

    /// Position and heading at arc length `s` (clamped to the path).
    pub fn pose_at(&self, s: f64) -> ([f64; 2], f64) {
        let s = s.clamp(0.0, self.length());
        let i = self
            .s
            .partition_point(|&x| x < s)
            .clamp(1, self.pts.len() - 1);
        let (a, b) = (self.pts[i - 1], self.pts[i]);
        let seg = (self.s[i] - self.s[i - 1]).max(1e-9);
        let u = (s - self.s[i - 1]) / seg;
        let pos = [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u];
        (pos, (b[1] - a[1]).atan2(b[0] - a[0]))
    }

    /// Frenet coordinates of a point: (arc length, signed lateral offset).
    /// Positive offset is left of the path.
    pub fn project(&self, p: [f64; 2]) -> (f64, f64) {
        let mut best = (0.0, f64::INFINITY);
        for (i, w) in self.pts.windows(2).enumerate() {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len2 = (dx * dx + dy * dy).max(1e-12);
            let u = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0);
            let q = [a[0] + dx * u, a[1] + dy * u];
            let d = dist(p, q);
            if d < best.1.abs() {
                // sign from the cross product of segment direction and offset
                let cross = dx * (p[1] - q[1]) - dy * (p[0] - q[0]);
                best = (self.s[i] + len2.sqrt() * u, d.copysign(cross));
            }
        }
        best
    }

    pub fn frenet_to_xy(&self, s: f64, d: f64) -> [f64; 2] {
        let (p, yaw) = self.pose_at(s);
        [p[0] - d * yaw.sin(), p[1] + d * yaw.cos()]
    }
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Turn a sampled position trajectory (spaced `dt` apart, starting one tick
/// after the ego) into controls for the kinematic model.
fn xy_to_controls(ego: State, pts: &[[f64; 2]], dt: f64) -> Vec<Control> {
    let mut v = ego.speed;
    let mut yaw = ego.yaw;
    let mut prev = [ego.x, ego.y];
    pts.iter()
        .map(|&p| {
            let ds = dist(prev, p);
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

/// Planner 1: steer toward the lane centerline along a cubic Bezier path,
/// with IDM for the speed profile.
pub struct BezierIdmPlanner;

impl Planner for BezierIdmPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = Path::new(ctx.centerline);
        let (s0, _) = path.project([ego.x, ego.y]);
        let lookahead = (3.0 * ego.speed).max(15.0);
        let (end, end_yaw) = path.pose_at(s0 + lookahead);
        let l3 = lookahead / 3.0;
        // ends tangent to the ego heading and the lane heading
        let b = [
            [ego.x, ego.y],
            [ego.x + l3 * ego.yaw.cos(), ego.y + l3 * ego.yaw.sin()],
            [end[0] - l3 * end_yaw.cos(), end[1] - l3 * end_yaw.sin()],
            end,
        ];
        let mut lead = lead_vehicle(&path, s0, ctx.actors);
        let mut v = ego.speed;
        let mut t = 0.0;
        (0..ctx.horizon)
            .map(|_| {
                let accel = idm_accel(v, ctx.target_speed, lead);
                let u = Control {
                    accel,
                    curvature: bezier_curvature(&b, t),
                };
                v = (v + accel * ctx.dt).max(0.0);
                let d1 = bezier_d1(&b, t);
                t = (t + v * ctx.dt / d1[0].hypot(d1[1]).max(1e-6)).min(1.0);
                if let Some((gap, lead_v)) = &mut lead {
                    *gap = (*gap + (*lead_v - v) * ctx.dt).max(0.0);
                }
                u
            })
            .collect()
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

/// Planner 2: EM/lattice-style. Samples a deterministic grid of (station,
/// lateral) points in the road Frenet frame, connects successive layers into
/// a tree with cubic-in-time lateral segments, assigns node costs (offset,
/// smoothness, predicted-obstacle proximity), and picks the best path.
pub struct LatticePlanner;

const STATIONS_M: [f64; 3] = [15.0, 30.0, 45.0];
const LATERALS_M: [f64; 5] = [-3.5, -1.75, 0.0, 1.75, 3.5];
const SAMPLES_PER_SEGMENT: usize = 8;
// center-to-center; slightly over one car width plus margin
const COLLISION_RADIUS_M: f64 = 2.5;

impl Planner for LatticePlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        let path = Path::new(ctx.centerline);
        let (s0, d0) = path.project([ego.x, ego.y]);
        // ponytail: constant-speed profile; couple IDM into the lattice when needed
        let v = ego.speed.clamp(2.0, ctx.target_speed.max(2.0));
        // initial lateral rate, expressed per unit of segment parameter u; the
        // first segment must honor it or every replan restarts the swerve at
        // zero slope and the executed path lags the plan into obstacles
        let (_, lane_yaw) = path.pose_at(s0);
        let m0_first = ego.speed * wrap_angle(ego.yaw - lane_yaw).sin() * (STATIONS_M[0] / v);
        // cubic Hermite in u with start slope m0 and flat end
        let d_shape = |da: f64, db: f64, m0: f64, u: f64| {
            let (u2, u3) = (u * u, u * u * u);
            (2.0 * u3 - 3.0 * u2 + 1.0) * da + (u3 - 2.0 * u2 + u) * m0 + (3.0 * u2 - 2.0 * u3) * db
        };

        // cost of one lattice edge, integrating over sampled points
        let edge_cost = |sa: f64, da: f64, sb: f64, db: f64, m0: f64| -> f64 {
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
                    let gap = dist(p, q);
                    if gap < COLLISION_RADIUS_M {
                        return f64::INFINITY;
                    }
                    cost += 20.0 / (gap * gap);
                }
            }
            cost
        };

        // DP over layers: the lattice is a layered DAG, so dynamic programming
        // finds the exact best path (A* would just add bookkeeping).
        let mut prev: Vec<(f64, f64, Vec<f64>)> = vec![(d0, 0.0, vec![])]; // (d, cost, laterals so far)
        for (layer, ds) in STATIONS_M.iter().enumerate() {
            let sa = s0
                + if layer == 0 {
                    0.0
                } else {
                    STATIONS_M[layer - 1]
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
        let pts: Vec<[f64; 2]> = (1..=ctx.horizon.max((STATIONS_M[2] / (v * ctx.dt)) as usize))
            .map(|i| {
                let s_rel = (v * ctx.dt * i as f64).min(STATIONS_M[2]);
                let seg = STATIONS_M.iter().position(|&m| s_rel <= m).unwrap();
                let (sa, da) = if seg == 0 {
                    (0.0, d0)
                } else {
                    (STATIONS_M[seg - 1], laterals[seg - 1])
                };
                let u = (s_rel - sa) / (STATIONS_M[seg] - sa);
                let m0 = if seg == 0 { m0_first } else { 0.0 };
                let d = d_shape(da, laterals[seg], m0, u);
                path.frenet_to_xy(s0 + s_rel, d)
            })
            .collect();
        xy_to_controls(ego, &pts, ctx.dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Simulator, test_ctx};

    const CENTERLINE: [[f64; 2]; 2] = [[-20.0, 0.0], [400.0, 0.0]];

    fn run(planner: &mut dyn Planner, ego: State, actors: &[State], ticks: usize) -> Vec<State> {
        let mut sim = Simulator {
            state: ego,
            dt: 0.1,
        };
        (0..ticks)
            .map(|_| sim.tick(planner, &test_ctx(&CENTERLINE, actors)))
            .collect()
    }

    #[test]
    fn frenet_roundtrip() {
        let path = Path::new(&CENTERLINE);
        let (s, d) = path.project([10.0, 2.5]);
        assert!((s - 30.0).abs() < 1e-9 && (d - 2.5).abs() < 1e-9);
        assert_eq!(path.frenet_to_xy(s, d), [10.0, 2.5]);
    }

    #[test]
    fn bezier_idm_converges_to_centerline_and_target_speed() {
        let ego = State {
            y: 3.0,
            speed: 5.0,
            ..Default::default()
        };
        let trace = run(&mut BezierIdmPlanner, ego, &[], 200);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.3, "offset {}", end.y);
        assert!((end.speed - 10.0).abs() < 0.5, "speed {}", end.speed);
    }

    #[test]
    fn bezier_idm_stops_behind_stopped_lead() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let lead = State {
            x: 50.0,
            ..Default::default()
        };
        let trace = run(&mut BezierIdmPlanner, ego, &[lead], 300);
        let end = trace.last().unwrap();
        assert!(end.speed < 0.5, "speed {}", end.speed);
        assert!(end.x < 45.0, "x {}", end.x); // stopped short of the lead
    }

    #[test]
    fn lattice_stays_on_empty_centerline() {
        let ego = State {
            y: 1.5,
            speed: 8.0,
            ..Default::default()
        };
        let trace = run(&mut LatticePlanner, ego, &[], 150);
        let end = trace.last().unwrap();
        assert!(end.y.abs() < 0.5, "offset {}", end.y);
    }

    #[test]
    fn lattice_swerves_around_stopped_obstacle() {
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let obstacle = State {
            x: 40.0,
            ..Default::default()
        };
        let trace = run(&mut LatticePlanner, ego, &[obstacle], 150);
        let min_gap = trace
            .iter()
            .map(|s| (s.x - 40.0).hypot(s.y))
            .fold(f64::INFINITY, f64::min);
        let end = trace.last().unwrap();
        assert!(min_gap > 2.0, "min gap {min_gap}");
        assert!(end.x > 60.0, "did not pass the obstacle, x {}", end.x);
    }
}
