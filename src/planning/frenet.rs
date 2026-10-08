//! Shared Frenet cubic motion, Cartesian transformations, and reachable target sampling.

use crate::common::geometry::{EGO_FOOTPRINT, angle_delta};
use crate::common::interp::lerp;
use crate::common::kinematics::{commanded_accel_for_net, net_longitudinal_accel};
use crate::common::measure::dot;
use crate::common::polynomial::CubicPolynomial;
use crate::common::types::{Control, Position, State};
use crate::planning::ComputeBudget;
use crate::planning::planner_math::STATE_SAMPLE_PROJECTION_RADIUS_M;
use crate::simulation::world_step;
use crate::track::Path;
use crate::vehicle::{MAX_LON_ACCEL, MIN_LON_ACCEL};

const STATION_SAMPLES_PER_INTERVAL: usize = 9;
const SPEED_SAMPLES_PER_INTERVAL: usize = 5;
const LATERAL_SAMPLES: usize = 11;
const MIN_FRENET_SCALE: f64 = 0.1;
const STATION_SPEED_TOLERANCE_MPS: f64 = 1e-8;
const MOTION_SPEED_EPSILON_MPS: f64 = 1e-6;
const ACCELERATION_BISECTION_ITERATIONS: usize = 32;
const SUSTAINED_ACCELERATION_FRACTIONS: [f64; 3] = [0.5, 0.75, 1.0];
const UPPER_SAMPLE_SPACING_EXPONENT: i32 = 2;
const TERMINAL_LATERAL_SPEEDS_MPS: [f64; 3] = [-0.5, 0.0, 0.5];

/// Position and velocity boundary conditions in the road's Frenet chart.
pub(crate) fn frenet_boundary(path: &Path, state: State, station_hint: f64) -> Option<([f64; 2], [f64; 2])> {
    let projected = path.project_near(state.position(), station_hint, STATE_SAMPLE_PROJECTION_RADIUS_M);
    let heading = angle_delta(path.heading_at(projected.s), state.pose.yaw);
    let scale = 1.0 - path.curvature_at(projected.s) * projected.d;
    if scale <= MIN_FRENET_SCALE || state.speed < 0.0 || heading.cos() < 0.0 {
        return None;
    }
    Some((
        [projected.s, state.speed * heading.cos() / scale],
        [projected.d, state.speed * heading.sin()],
    ))
}

pub(crate) struct Motion {
    pub(crate) longitudinal: CubicPolynomial,
    pub(crate) lateral: CubicPolynomial,
}

impl Motion {
    /// Transform Frenet position and derivatives into Cartesian state and net controls.
    pub(crate) fn at(&self, path: &Path, t: f64) -> Option<(State, Control)> {
        let [s, sv, sa] = self.longitudinal.at(t);
        let [d, dv, da] = self.lateral.at(t);
        if !(0.0..=path.length()).contains(&s) || sv < -STATION_SPEED_TOLERANCE_MPS {
            return None;
        }
        let k = path.curvature_at(s);
        let dk = path.sharpness_at(s);
        let scale = 1.0 - k * d;
        if scale <= MIN_FRENET_SCALE {
            return None;
        }
        let vx = scale * sv;
        let ax = scale * sa - dk * d * sv * sv - 2.0 * k * sv * dv;
        let ay = da + k * scale * sv * sv;
        let velocity = Position::new(vx, dv);
        let acceleration = Position::new(ax, ay);
        let speed = velocity.norm();
        let (acceleration, curvature) = if speed > MOTION_SPEED_EPSILON_MPS {
            (
                dot(velocity.xy(), acceleration.xy()) / speed,
                velocity.cross(acceleration) / speed.powi(3),
            )
        } else {
            (ax, 0.0)
        };
        let heading = path.heading_at(s);
        let left = Position::from_angle(heading + std::f64::consts::FRAC_PI_2);
        let state = State::from((path.pose_at(s).0 + left * d, heading + velocity.angle(), speed));
        let control = Control {
            acceleration,
            curvature,
        };
        (state.position().is_finite()
            && state.pose.yaw.is_finite()
            && speed.is_finite()
            && acceleration.is_finite()
            && curvature.is_finite())
        .then_some((state, control))
    }
}

fn sampling_scale(budget: ComputeBudget) -> f64 {
    let nominal = (2 * STATION_SAMPLES_PER_INTERVAL) * (2 * SPEED_SAMPLES_PER_INTERVAL) * LATERAL_SAMPLES;
    (budget.scale(nominal, 1) as f64 / nominal as f64).cbrt()
}

fn lateral_sample_count(budget: ComputeBudget) -> usize {
    // Odd counts always include the centerline, including at the lowest budget.
    2 * (((LATERAL_SAMPLES / 2) as f64 * sampling_scale(budget)).round() as usize).max(1) + 1
}

pub(crate) fn lateral_targets(initial_lateral: f64, half_width: f64, budget: ComputeBudget) -> Vec<(f64, f64)> {
    let width = (half_width - EGO_FOOTPRINT.width / 2.0).max(0.0);
    let count = lateral_sample_count(budget);
    std::iter::once((-initial_lateral, 0.0))
        .chain((0..count).flat_map(|i| {
            let lateral = lerp(-width, width, i as f64 / (count - 1) as f64);
            TERMINAL_LATERAL_SPEEDS_MPS.into_iter().map(move |v| (lateral, v))
        }))
        .collect()
}

fn sample_counts(budget: ComputeBudget) -> (usize, usize) {
    // Scale the three position/speed axes together; lateral velocity keeps three samples.
    let scale = sampling_scale(budget);
    let station_count = ((STATION_SAMPLES_PER_INTERVAL as f64 * scale).round() as usize).max(2);
    let speed_count = ((SPEED_SAMPLES_PER_INTERVAL as f64 * scale).round() as usize).max(2);
    (station_count, speed_count)
}

/// Integrate speed along the prescribed lateral cubic using the road's Frenet metric.
/// This is a sampling envelope; the actual cubics still undergo all feasibility checks.
fn frenet_endpoint(
    (path, s0, lateral): (&Path, f64, &CubicPolynomial),
    dt: f64,
    duration: f64,
    mut speed_step: impl FnMut(f64, f64) -> (f64, f64),
) -> (f64, f64) {
    let mut station = s0;
    let mut terminal_speed = 0.0;
    let station_speed = |s, t, speed: f64| {
        let [d, _, _] = lateral.at(t);
        // Allocate all speed to station for a conservative distance envelope, leaving
        // room for lateral recovery. Motion::at rejects singular charts and checks
        // the actual combined motion; this denominator floor only bounds sampling.
        speed / (1.0 - path.curvature_at(s) * d).max(MIN_FRENET_SCALE)
    };
    for tick in 0..(duration / dt).ceil() as usize {
        let t = tick as f64 * dt;
        let step_dt = dt.min(duration - t);
        let (speed, end_speed) = speed_step(t, step_dt);
        let middle = station + 0.5 * step_dt * station_speed(station, t, speed);
        station += step_dt * station_speed(middle, t + 0.5 * step_dt, speed);
        terminal_speed = end_speed;
    }
    let lateral_speed = lateral.at(duration)[1];
    let terminal_tangent_speed = (terminal_speed * terminal_speed - lateral_speed * lateral_speed)
        .max(0.0)
        .sqrt();
    (station - s0, station_speed(station, duration, terminal_tangent_speed))
}

/// Independent station/speed grid plus paired sustained-acceleration endpoints.
pub(crate) fn longitudinal_targets(
    initial_speed: f64,
    dt: f64,
    budget: ComputeBudget,
    duration: f64,
    frame: (&Path, f64, &CubicPolynomial),
) -> Vec<(f64, f64)> {
    let rollout = |acceleration| {
        let mut state = State {
            speed: initial_speed,
            ..Default::default()
        };
        frenet_endpoint(frame, dt, duration, |_, step_dt| {
            let speed = state.speed;
            state = world_step(
                state,
                Control {
                    acceleration,
                    curvature: 0.0,
                },
                step_dt,
            );
            state.speed = state.speed.max(0.0);
            (speed, state.speed)
        })
    };
    let nominal = rollout(0.0);
    let min = rollout(MIN_LON_ACCEL);
    let max = rollout(MAX_LON_ACCEL);

    let samples = |min, nominal, max, count| {
        // Retain dense near-coasting samples for slow rolling through tight bends.
        (0..count)
            .map(move |i| lerp(min, nominal, i as f64 / count as f64))
            .chain((0..count).map(move |i| {
                lerp(
                    nominal,
                    max,
                    (i as f64 / (count - 1) as f64).powi(UPPER_SAMPLE_SPACING_EXPONENT),
                )
            }))
    };
    let (station_count, speed_count) = sample_counts(budget);
    let stations = samples(min.0, nominal.0, max.0, station_count);
    let speeds = samples(min.1, nominal.1, max.1, speed_count);
    let mut targets =
        Vec::with_capacity(stations.clone().count() * speeds.clone().count() + SUSTAINED_ACCELERATION_FRACTIONS.len());
    for station in stations {
        for speed in speeds.clone() {
            targets.push((station, speed));
        }
    }
    // Find a constant net acceleration whose terminal drag-compensated command
    // remains within the thrust limit. On a straight this fits a quadratic exactly,
    // avoiding the acceleration overshoot of a cubic fitted to a full-thrust rollout.
    let (mut low, mut high) = (0.0, net_longitudinal_accel(MAX_LON_ACCEL, initial_speed).max(0.0));
    for _ in 0..ACCELERATION_BISECTION_ITERATIONS {
        let acceleration = 0.5 * (low + high);
        if commanded_accel_for_net(acceleration, initial_speed + acceleration * duration) <= MAX_LON_ACCEL {
            low = acceleration;
        } else {
            high = acceleration;
        }
    }
    for fraction in SUSTAINED_ACCELERATION_FRACTIONS {
        let acceleration = low * fraction;
        targets.push(frenet_endpoint(frame, dt, duration, |t, step_dt| {
            (
                initial_speed + acceleration * (t + 0.5 * step_dt),
                initial_speed + acceleration * (t + step_dt),
            )
        }));
    }
    targets
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::geometry::wrap_angle;
    use crate::constraints::{Constraint, Kinodynamic, Sample};
    use crate::planning::{PLANNING_HORIZON_S, test_ctx, test_road};

    fn straight_targets(speed: f64, dt: f64, budget: ComputeBudget, duration: f64) -> Vec<(f64, f64)> {
        let path = Path::new(&[Position::new(0.0, 0.0), Position::new(2000.0, 0.0)]);
        longitudinal_targets(speed, dt, budget, duration, (&path, 0.0, &CubicPolynomial([0.0; 4])))
    }

    #[test]
    fn frenet_cubics_match_boundaries_and_follow_a_curved_reference() {
        for end_velocity in TERMINAL_LATERAL_SPEEDS_MPS {
            let polynomial = CubicPolynomial::from_boundary(2.0, 0.3, -1.0, end_velocity, 3.0);
            assert_eq!(polynomial.at(0.0)[..2], [2.0, 0.3]);
            let end = polynomial.at(3.0);
            assert!((end[0] + 1.0).abs() < 1e-10);
            assert!((end[1] - end_velocity).abs() < 1e-10);
        }
        let radius = 40.0;
        let points: Vec<_> = (0..1800)
            .map(|i| {
                let angle = i as f64 * 0.002;
                Position::new(radius * angle.cos(), radius * angle.sin())
            })
            .collect();
        let path = Path::new(&points);
        let motion = Motion {
            longitudinal: CubicPolynomial([10.0, 8.0, 0.0, 0.0]),
            lateral: CubicPolynomial([2.0, 0.0, 0.0, 0.0]),
        };
        for t in [0.0, 2.0, 5.0, 10.0] {
            let (state, control) = motion.at(&path, t).unwrap();
            // A constant Frenet offset traces the concentric circle, not a Cartesian chord.
            assert!((state.position().x.hypot(state.position().y) - 38.0).abs() < 1e-4);
            assert!((state.speed - 7.6).abs() < 1e-8);
            assert!((control.curvature - 1.0 / 38.0).abs() < 1e-8);
            assert!(control.acceleration.abs() < 1e-8);
            let tangent = state.position().y.atan2(state.position().x) + std::f64::consts::FRAC_PI_2;
            assert!(wrap_angle(state.pose.yaw - tangent).abs() < 1e-8);
        }
        // Frenet chart singularities and backwards station motion are not valid trajectories.
        let singular = Motion {
            lateral: CubicPolynomial([41.0, 0.0, 0.0, 0.0]),
            ..motion
        };
        assert!(singular.at(&path, 1.0).is_none());
        let reverse = Motion {
            longitudinal: CubicPolynomial([10.0, -1.0, 0.0, 0.0]),
            ..singular
        };
        assert!(reverse.at(&path, 1.0).is_none());
    }

    #[test]
    fn frenet_transform_accounts_for_changing_reference_curvature() {
        let points: Vec<_> = (0..3000)
            .map(|i| {
                let x = i as f64 * 0.05;
                Position::new(x, 0.01 * x * x)
            })
            .collect();
        let path = Path::new(&points);
        let motion = Motion {
            longitudinal: CubicPolynomial([10.0, 8.0, 0.15, 0.0]),
            lateral: CubicPolynomial([2.0, 0.3, 0.1, 0.0]),
        };
        let t = 1.234;
        let epsilon = 1e-5;
        let (before, _) = motion.at(&path, t - epsilon).unwrap();
        let (state, control) = motion.at(&path, t).unwrap();
        let (after, _) = motion.at(&path, t + epsilon).unwrap();
        assert!(path.sharpness_at(motion.longitudinal.at(t)[0]).abs() > 1e-5);
        let acceleration = (after.speed - before.speed) / (2.0 * epsilon);
        let curvature = wrap_angle(after.pose.yaw - before.pose.yaw) / (2.0 * epsilon * state.speed);
        assert!((control.acceleration - acceleration).abs() < 1e-6);
        assert!((control.curvature - curvature).abs() < 1e-5);
    }

    #[test]
    fn targets_use_relative_station_and_resistance_over_the_planning_horizon() {
        let speed = 8.0;
        let targets = straight_targets(speed, 0.1, ComputeBudget::NOMINAL, PLANNING_HORIZON_S);
        let nominal = (0..100).fold(
            State {
                speed,
                ..Default::default()
            },
            |state, _| world_step(state, Control::default(), 0.1),
        );
        assert!(targets.contains(&(nominal.position().x, nominal.speed)));
        assert!(nominal.position().x < speed * PLANNING_HORIZON_S);
        assert!(nominal.speed < speed);
        assert_eq!(targets.first().unwrap().1, 0.0);
        assert!(targets.first().unwrap().0 > 0.0);
        let station_count = 2 * STATION_SAMPLES_PER_INTERVAL;
        let speed_count = 2 * SPEED_SAMPLES_PER_INTERVAL;
        assert_eq!(
            targets.len(),
            station_count * speed_count + SUSTAINED_ACCELERATION_FRACTIONS.len()
        );
        let mut stations: Vec<_> = targets[..station_count * speed_count]
            .iter()
            .map(|target| target.0)
            .collect();
        stations.dedup();
        let mut speeds: Vec<_> = targets[..speed_count].iter().map(|target| target.1).collect();
        speeds.dedup();
        assert_eq!(stations.len(), station_count);
        assert_eq!(speeds.len(), speed_count);
        assert_eq!(
            stations.iter().filter(|&&s| s < nominal.position().x).count(),
            STATION_SAMPLES_PER_INTERVAL
        );
        assert_eq!(
            speeds.iter().filter(|&&v| v < nominal.speed).count(),
            SPEED_SAMPLES_PER_INTERVAL
        );
        for station in stations {
            for &speed in &speeds {
                assert!(targets.contains(&(station, speed)));
            }
        }
        let end = (0..100).fold(
            State {
                speed,
                ..Default::default()
            },
            |state, _| {
                world_step(
                    state,
                    Control {
                        acceleration: MAX_LON_ACCEL,
                        curvature: 0.0,
                    },
                    0.1,
                )
            },
        );
        assert_eq!(targets[station_count * speed_count - 1], (end.position().x, end.speed));
        assert!(end.position().x < speed * PLANNING_HORIZON_S + 0.5 * MAX_LON_ACCEL * PLANNING_HORIZON_S.powi(2));
    }

    #[test]
    fn station_bounds_follow_inside_and_outside_offsets() {
        let radius = 200.0;
        for turn in [-1.0, 1.0] {
            let points: Vec<_> = (0..2001)
                .map(|i| {
                    let angle = i as f64 / radius;
                    Position::new(radius * angle.sin(), turn * radius * (1.0 - angle.cos()))
                })
                .collect();
            let path = Path::new(&points);
            let straight = straight_targets(8.0, 0.1, ComputeBudget::NOMINAL, PLANNING_HORIZON_S);
            for offset in [-5.0, 5.0] {
                let lateral = CubicPolynomial([offset, 0.0, 0.0, 0.0]);
                let curved = longitudinal_targets(
                    8.0,
                    0.1,
                    ComputeBudget::NOMINAL,
                    PLANNING_HORIZON_S,
                    (&path, 10.0, &lateral),
                );
                let scale = 1.0 - turn * offset / radius;
                for ((distance, speed), (straight_distance, straight_speed)) in curved.iter().zip(&straight) {
                    assert!((distance * scale - straight_distance).abs() < 1e-7);
                    assert!((speed * scale - straight_speed).abs() < 1e-7);
                }
            }
        }
    }

    #[test]
    fn station_envelope_tracks_changing_curvature_and_lateral_velocity() {
        // Integrate a clothoid with k(s) = 0.001 + 0.00001*s.
        let mut point = Position::new(0.0, 0.0);
        let mut points = vec![point];
        for i in 0..5000 {
            let s = (i as f64 + 0.5) * 0.1;
            point = point + Position::from_angle(0.001 * s + 0.000005 * s * s) * 0.1;
            points.push(point);
        }
        let path = Path::new(&points);
        let lateral = CubicPolynomial([5.0, 0.0, 0.0, 0.0]);
        let (distance, speed) = frenet_endpoint((&path, 10.0, &lateral), 0.1, 10.0, |_, _| (20.0, 20.0));
        let s = 10.0 + distance;
        let integral = |s: f64| 0.995 * s - 0.000025 * s * s;
        assert!((integral(s) - integral(10.0) - 200.0).abs() < 1e-4);
        assert!((speed * (0.995 - 0.00005 * s) - 20.0).abs() < 1e-6);
        let straight = Path::new(&[Position::new(0.0, 0.0), Position::new(500.0, 0.0)]);
        let lateral = CubicPolynomial([0.0, 3.0, 0.0, 0.0]);
        let (s, speed) = frenet_endpoint((&straight, 0.0, &lateral), 0.1, 10.0, |_, _| (5.0, 5.0));
        assert!((s - 50.0).abs() < 1e-10);
        assert!((speed - 4.0).abs() < 1e-10);
    }

    #[test]
    fn candidate_count_scales_with_compute_budget() {
        let nominal = straight_targets(8.0, 0.1, ComputeBudget::NOMINAL, PLANNING_HORIZON_S);
        let nominal_target =
            nominal[STATION_SAMPLES_PER_INTERVAL * 2 * SPEED_SAMPLES_PER_INTERVAL + SPEED_SAMPLES_PER_INTERVAL];
        let mut previous = 0;
        for percent in crate::planning::COMPUTE_BUDGET_BREAKPOINTS {
            let targets = straight_targets(8.0, 0.1, ComputeBudget::from_percent(percent), PLANNING_HORIZON_S);
            let count = targets.len()
                * (1 + lateral_sample_count(ComputeBudget::from_percent(percent)) * TERMINAL_LATERAL_SPEEDS_MPS.len());
            assert!(count > previous, "{percent}%: {count}");
            previous = count;
            assert_eq!(targets.first(), nominal.first());
            assert_eq!(targets.last(), nominal.last());
            assert!(targets.contains(&nominal_target));
            assert!(
                targets
                    .iter()
                    .all(|(station, speed)| station.is_finite() && speed.is_finite())
            );
        }
    }

    #[test]
    fn sustained_acceleration_targets_survive_strict_limits() {
        let road = test_road(&[[-20.0, 0.0], [2000.0, 0.0]]);
        for speed in [2.0, 8.0, 20.0] {
            let targets = straight_targets(speed, 0.1, ComputeBudget::NOMINAL, PLANNING_HORIZON_S);
            for &(distance, terminal_speed) in &targets[targets.len() - SUSTAINED_ACCELERATION_FRACTIONS.len()..] {
                let motion = Motion {
                    longitudinal: CubicPolynomial::from_boundary(
                        20.0,
                        speed,
                        20.0 + distance,
                        terminal_speed,
                        PLANNING_HORIZON_S,
                    ),
                    lateral: CubicPolynomial([0.0; 4]),
                };
                for tick in 0..=100 {
                    let (state, control) = motion.at(test_ctx(&road, &[]).path(), tick as f64 * 0.1).unwrap();
                    let command = Control {
                        acceleration: commanded_accel_for_net(control.acceleration, state.speed),
                        ..control
                    };
                    assert!(!Kinodynamic.is_violated(&Sample::default().with_control(command, state.speed)));
                }
            }
        }
    }
}
