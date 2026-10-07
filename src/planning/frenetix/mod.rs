//! Section III.B polynomial sampling from https://arxiv.org/abs/2402.01443.
//! Reduced-degree variant: longitudinal cubics crossed with lateral cubics.

use crate::common::geometry::wrap_angle;
use crate::common::interp::lerp;
use crate::common::kinematics::commanded_accel_for_net;
use crate::common::types::FrenetPosition;
use crate::constraints::Constraints;
use crate::planning::controls::stop_controls;
use crate::planning::steering::cubic_coeffs;
use crate::planning::{ComputeBudget, Context, PLANNING_HORIZON_S, Planner};
use crate::simulation::{Control, State, clamp_control, world_step};
use crate::track::Path;
use crate::vehicle::MAX_LON_ACCEL;

pub(crate) struct FrenetixPlanner;

const NOMINAL_GRID_SAMPLES: usize = 11;
const TERMINAL_LATERAL_SPEEDS_MPS: [f64; 3] = [-0.5, 0.0, 0.5];

struct Polynomial {
    c: [f64; 4],
}

impl Polynomial {
    /// Position, velocity, acceleration at time `t`.
    fn at(&self, t: f64) -> [f64; 3] {
        let [c0, c1, c2, c3] = self.c;
        let p = c0 + t * (c1 + t * (c2 + t * c3));
        let v = c1 + t * (2.0 * c2 + t * 3.0 * c3);
        let a = 2.0 * c2 + t * 6.0 * c3;
        [p, v, a]
    }
}

struct Motion {
    longitudinal: Polynomial,
    lateral: Polynomial,
}

impl Motion {
    fn control(&self, path: &Path, time: f64) -> Option<Control> {
        let [s, sv, sa] = self.longitudinal.at(time);
        let [d, dv, da] = self.lateral.at(time);
        if !(0.0..=path.length()).contains(&s) || sv < -1e-8 {
            return None;
        }
        let k = path.curvature_at(s);
        let dk = path.sharpness_at(s);
        let scale = 1.0 - k * d;
        if scale <= 0.1 {
            return None;
        }
        // Cartesian velocity and acceleration in the reference tangent/normal frame.
        let vx = scale * sv;
        let ax = scale * sa - dk * d * sv * sv - 2.0 * k * sv * dv;
        let ay = da + k * scale * sv * sv;
        let speed = vx.hypot(dv);
        let (acceleration, curvature) = if speed > 1e-6 {
            ((vx * ax + dv * ay) / speed, (vx * ay - dv * ax) / speed.powi(3))
        } else {
            (ax, 0.0)
        };
        (acceleration.is_finite() && curvature.is_finite()).then_some(Control {
            acceleration,
            curvature,
        })
    }
}

impl Planner for FrenetixPlanner {
    fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
        if ctx.horizon == 0 {
            return Vec::new();
        }
        let path = ctx.time("route", || ctx.path());
        let FrenetPosition { s: s0, d: d0 } = path.project(ego.position());
        let heading = wrap_angle(ego.pose.yaw - path.heading_at(s0));
        let scale = 1.0 - path.curvature_at(s0) * d0;
        if scale <= 0.1 || ego.speed < 0.0 || heading.cos() < 0.0 {
            return stop_controls(ego, ctx, ctx.horizon);
        }

        // Current/initial lat and lon state
        let lon = [s0, ego.speed * heading.cos() / scale];
        let lat = [d0, ego.speed * heading.sin()];

        let width = (ctx.road.half_width - crate::common::geometry::EGO_FOOTPRINT.width / 2.0).max(0.0);

        let duration = PLANNING_HORIZON_S;
        // Include accelerating endpoints; speed * duration alone only covers
        // coasting distance and forces faster candidates to brake again.
        let ahead = ego.speed * duration + 0.5 * MAX_LON_ACCEL * duration.powi(2);
        let samples = grid_samples(ctx.compute_budget);
        let stations = grid(s0, (s0 + ahead).min(path.length()), samples);
        let laterals = grid(-width, width, samples);
        let ticks = (duration / ctx.road.dt).ceil() as usize;
        let speeds = grid(
            0.0,
            crate::simulation::speed_after_max_accel(ego.speed, ticks, ctx.road.dt),
            samples,
        );
        let mut best = None;
        ctx.time("fit", || {
            for station in stations {
                for speed in speeds.clone() {
                    for lateral in laterals.clone() {
                        for lateral_speed in TERMINAL_LATERAL_SPEEDS_MPS {
                            let motion = Motion {
                                longitudinal: Polynomial {
                                    c: cubic_coeffs(lon[0], lon[1], station, speed, duration),
                                },
                                lateral: Polynomial {
                                    c: cubic_coeffs(lat[0], lat[1], lateral, lateral_speed, duration),
                                },
                            };
                            if let Some((cost, controls)) = evaluate(ego, ctx, &motion, ticks)
                                && best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost)
                            {
                                best = Some((cost, controls));
                            }
                        }
                    }
                }
            }
        });
        best.map(|(_, mut controls)| {
            controls.truncate(ctx.horizon);
            controls
        })
        .unwrap_or_else(|| stop_controls(ego, ctx, ctx.horizon))
    }
}

fn grid_samples(budget: ComputeBudget) -> usize {
    // Three variable-density axes; lateral velocity always has three samples.
    (budget.scale(NOMINAL_GRID_SAMPLES.pow(3), 8) as f64).cbrt().round() as usize
}

fn grid(lo: f64, hi: f64, samples: usize) -> impl Iterator<Item = f64> + Clone {
    (0..samples).map(move |i| lerp(lo, hi, i as f64 / (samples - 1) as f64))
}

fn evaluate(ego: State, ctx: &Context, motion: &Motion, ticks: usize) -> Option<(f64, Vec<Control>)> {
    let path = ctx.path();
    let dt = ctx.road.dt;
    let constraints = Constraints::new(ctx.road.half_width, ctx.actors, path, ego.speed, ctx.project_ego(ego).s);
    let mut actual = ego;
    let mut controls = Vec::with_capacity(ticks);
    let mut points = ctx.diagnostics.map(|_| vec![ego.position()]);
    let mut cost = 0.0;

    for tick in 0..ticks {
        let time = (tick + 1) as f64 * dt;
        let control_time = ((tick as f64 + 0.5) * dt).min(PLANNING_HORIZON_S);
        let mut control = motion.control(path, control_time)?;
        control.acceleration = commanded_accel_for_net(control.acceleration.max(-actual.speed / dt), actual.speed);
        let control = clamp_control(control, actual.speed);
        let control_speed = actual.speed;
        actual = world_step(actual, control, dt);
        let (s, mut sample) = crate::planning::planner_math::state_sample(
            path,
            &actual,
            time,
            Some(motion.longitudinal.at(time.min(PLANNING_HORIZON_S))[0]),
        );
        sample = sample.with_control(control, control_speed);
        sample.road_bounds = Some(ctx.road.lateral_bounds_at(s));
        cost += ctx.time("cost", || constraints.point_cost(&sample));
        ctx.work(1);
        if !cost.is_finite() {
            return None;
        }
        controls.push(control);
        if let Some(points) = &mut points {
            points.push(actual.position());
        }
    }
    if let (Some(diag), Some(points)) = (ctx.diagnostics, points) {
        diag.record_trajectory(points);
    }
    Some((cost, controls))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::{ComputeBudget, Diagnostics, test_ctx, test_road};
    use crate::simulation::{Pose, Position};

    #[test]
    fn accelerates_on_an_empty_straight_at_cruising_speeds() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let controls: Vec<_> = [0.0, 10.0, 20.0, 40.0]
            .into_iter()
            .map(|speed| {
                let ego = State {
                    speed,
                    ..Default::default()
                };
                (speed, FrenetixPlanner.plan(ego, &test_ctx(&road, &[]))[0])
            })
            .collect();
        assert!(
            controls
                .iter()
                .all(|(_, u)| u.acceleration > 0.8 * crate::vehicle::MAX_LON_ACCEL),
            "empty-road first controls: {controls:?}"
        );
    }

    #[test]
    fn candidate_count_scales_approximately_with_budget() {
        let nominal = NOMINAL_GRID_SAMPLES.pow(3) * TERMINAL_LATERAL_SPEEDS_MPS.len();
        let mut previous = 0;
        for percent in crate::planning::COMPUTE_BUDGET_BREAKPOINTS {
            let budget = ComputeBudget::from_percent(percent);
            let count = grid_samples(budget).pow(3) * TERMINAL_LATERAL_SPEEDS_MPS.len();
            let expected = nominal as f64 * percent as f64 / 100.0;
            assert!(
                (count as f64 / expected - 1.0).abs() < 0.25,
                "{percent}%: {count} vs {expected}"
            );
            assert!(count > previous);
            previous = count;
        }
        assert_eq!(grid_samples(ComputeBudget::NOMINAL), NOMINAL_GRID_SAMPLES);
    }

    #[test]
    fn polynomial_boundary_conditions() {
        let start = [2.0, 3.0];
        for duration in [0.5, 2.0, PLANNING_HORIZON_S] {
            for lateral_speed in TERMINAL_LATERAL_SPEEDS_MPS {
                let lateral = Polynomial {
                    c: cubic_coeffs(start[0], start[1], -1.0, lateral_speed, duration),
                };
                assert_eq!(lateral.at(0.0)[..2], start);
                let end = lateral.at(duration);
                assert!((end[0] + 1.0).abs() < 1e-8);
                assert!((end[1] - lateral_speed).abs() < 1e-8);
                assert!(lateral.at(0.0)[2].abs() > 0.0);
            }
            let target_position = start[0] + 6.0 * duration;
            let longitudinal = Polynomial {
                c: cubic_coeffs(start[0], start[1], target_position, 8.0, duration),
            };
            assert_eq!(longitudinal.at(0.0)[..2], start);
            let end = longitudinal.at(duration);
            assert!((end[0] - target_position).abs() < 1e-8);
            assert!((end[1] - 8.0).abs() < 1e-8);
            assert!(longitudinal.c[3].abs() > 1e-8);
            assert!(longitudinal.at(0.0)[2] > end[2] && end[2] > 0.0);
        }
    }

    #[test]
    fn evaluates_shared_horizon_independently_of_requested_controls() {
        for dt in [0.1, 0.2, 0.3] {
            let mut road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
            road.dt = dt;
            let diagnostics = Diagnostics::default();
            let mut ctx = Context::new(&road, &[], 1, ComputeBudget::NOMINAL, None, Some(&diagnostics));
            let short = FrenetixPlanner.plan(State::default(), &ctx);
            assert_eq!(short.len(), 1);
            assert!(short[0].acceleration > 0.0);
            let trajectories = diagnostics.take().trajectories;
            assert!(!trajectories.is_empty());
            let ticks = (PLANNING_HORIZON_S / dt).ceil() as usize;
            assert!(trajectories.iter().all(|points| points.len() == ticks + 1));
            ctx.horizon = ticks;
            let long = FrenetixPlanner.plan(State::default(), &ctx);
            assert_eq!(long.len(), ticks);
            assert_eq!(short[0], long[0]);
            assert!(
                long.iter()
                    .all(|u| u.acceleration.is_finite() && u.curvature.is_finite())
            );
            ctx.horizon = 0;
            assert!(FrenetixPlanner.plan(State::default(), &ctx).is_empty());
        }
    }

    #[test]
    fn shared_cost_rejects_a_collision_beyond_the_output_horizon() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let ego = State {
            speed: 8.0,
            ..Default::default()
        };
        let actor = State::new(Pose::new(Position::new(40.0, 0.0), 0.0), 0.0);
        let motion = Motion {
            longitudinal: Polynomial {
                c: cubic_coeffs(20.0, 8.0, 20.0 + 8.0 * PLANNING_HORIZON_S, 8.0, PLANNING_HORIZON_S),
            },
            lateral: Polynomial {
                c: cubic_coeffs(0.0, 0.0, 0.0, 0.0, 2.0),
            },
        };
        let ticks = (PLANNING_HORIZON_S / road.dt) as usize;
        assert!(evaluate(ego, &test_ctx(&road, &[]), &motion, ticks).is_some());
        assert!(evaluate(ego, &test_ctx(&road, &[actor]), &motion, ticks).is_none());
    }

    #[test]
    fn lateral_motion_steers_toward_its_target_and_stays_finite_at_rest() {
        let road = test_road(&[[-20.0, 0.0], [2_000.0, 0.0]]);
        let path = Path::new(road.centerline());
        let motion = Motion {
            longitudinal: Polynomial {
                c: cubic_coeffs(20.0, 8.0, 38.4, 10.0, 2.0),
            },
            lateral: Polynomial {
                c: cubic_coeffs(2.0, 0.0, 0.0, 0.0, 2.0),
            },
        };
        assert!(motion.control(&path, 0.5).unwrap().curvature < 0.0);
        let stopped = Motion {
            longitudinal: Polynomial {
                c: cubic_coeffs(20.0, 0.0, 20.0, 0.0, 2.0),
            },
            lateral: Polynomial {
                c: cubic_coeffs(0.0, 0.0, 0.0, 0.0, 2.0),
            },
        };
        assert_eq!(stopped.control(&path, 0.0), Some(Control::default()));
    }
}
