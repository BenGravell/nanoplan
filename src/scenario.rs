//! Scenario definitions and batch simulation.
//!
//! Scenarios are plain data (serde JSON) so large batches can come from
//! anywhere: the built-in synthetic generator, or real nuPlan logs exported
//! with tools/export_nuplan_scenarios.py.

use serde::{Deserialize, Serialize};

use crate::{Context, Control, Metrics, PlannerKind, Simulator, State, metrics, step, wrap_angle};

/// A timestamped state along a logged actor trajectory.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    /// Seconds since the start of the scenario.
    pub t: f64,
    #[serde(flatten)]
    pub state: State,
}

/// A non-ego actor. With a logged `trajectory` the actor replays it;
/// otherwise it integrates `init` under the constant `control`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Actor {
    pub init: State,
    #[serde(default)]
    pub control: Control,
    /// Logged trajectory to replay (e.g. from a nuPlan log); must be sorted
    /// by time. Overrides `control` when non-empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub trajectory: Vec<Waypoint>,
}

/// Replayed state at time `t`: linear interpolation between waypoints
/// (shortest-arc for yaw), the first waypoint before the log starts, and
/// constant velocity beyond its end.
fn replay(trajectory: &[Waypoint], t: f64) -> State {
    let first = trajectory[0];
    if t <= first.t {
        return first.state;
    }
    let last = trajectory[trajectory.len() - 1];
    if t >= last.t {
        let s = last.state;
        let dt = t - last.t;
        return State {
            x: s.x + s.speed * s.yaw.cos() * dt,
            y: s.y + s.speed * s.yaw.sin() * dt,
            ..s
        };
    }
    let i = trajectory.partition_point(|w| w.t <= t).max(1);
    let (a, b) = (trajectory[i - 1], trajectory[i]);
    let u = (t - a.t) / (b.t - a.t).max(1e-9);
    State {
        x: a.state.x + (b.state.x - a.state.x) * u,
        y: a.state.y + (b.state.y - a.state.y) * u,
        yaw: a.state.yaw + wrap_angle(b.state.yaw - a.state.yaw) * u,
        speed: a.state.speed + (b.state.speed - a.state.speed) * u,
    }
}

/// Environmental data mirroring the nuPlan map/scenario elements
/// (drivable area edges, lane boundary, crosswalks).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapData {
    /// Lateral offset of the road boundaries (drivable area edge).
    pub road_half_width: f64,
    /// Lateral offset of a dashed lane divider, when an opposing lane exists.
    pub divider_d: Option<f64>,
    /// Stations (arc length along the centerline) of crosswalk bands.
    pub crosswalk_s: Vec<f64>,
}

impl Default for MapData {
    fn default() -> Self {
        MapData {
            // matches the drivable-area bound in the metrics
            road_half_width: 5.5,
            divider_d: None,
            crosswalk_s: vec![],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    pub name: String,
    pub ego: State,
    #[serde(default)]
    pub actors: Vec<Actor>,
    /// Lane centerline / route reference path.
    pub centerline: Vec<[f64; 2]>,
    #[serde(default = "default_target_speed")]
    pub target_speed: f64,
    #[serde(default)]
    pub map: MapData,
}

fn default_target_speed() -> f64 {
    10.0
}

/// A finished closed-loop simulation: ego and actor states at every tick,
/// plus the metrics of the rollout.
pub struct Rollout {
    pub ego: Vec<State>,
    pub actors: Vec<Vec<State>>,
    pub metrics: Metrics,
}

/// Run a planner closed-loop through a scenario.
pub fn simulate(sc: &Scenario, kind: PlannerKind, duration_s: f64, dt: f64) -> Rollout {
    let steps = (duration_s / dt) as usize;
    let actors: Vec<Vec<State>> = sc
        .actors
        .iter()
        .map(|a| {
            if a.trajectory.is_empty() {
                let mut s = a.init;
                std::iter::once(s)
                    .chain((0..steps).map(|_| {
                        s = step(s, a.control, dt);
                        s
                    }))
                    .collect()
            } else {
                (0..=steps)
                    .map(|i| replay(&a.trajectory, i as f64 * dt))
                    .collect()
            }
        })
        .collect();
    let mut sim = Simulator { state: sc.ego, dt };
    let mut planner = kind.build();
    let mut ego = vec![sc.ego];
    ego.extend((0..steps).map(|i| {
        let current: Vec<State> = actors.iter().map(|t| t[i]).collect();
        let ctx = Context {
            centerline: &sc.centerline,
            actors: &current,
            target_speed: sc.target_speed,
            dt,
            horizon: 1,
        };
        sim.tick(planner.as_mut(), &ctx)
    }));
    let metrics = metrics::evaluate(&ego, &actors, &sc.centerline, sc.target_speed, dt);
    Rollout {
        ego,
        actors,
        metrics,
    }
}

/// Load every `*.json` scenario in a directory (non-recursive), sorted by
/// file name.
pub fn load_dir(dir: &std::path::Path) -> std::io::Result<Vec<Scenario>> {
    let mut paths: Vec<_> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    paths
        .into_iter()
        .map(|p| {
            let text = std::fs::read_to_string(&p)?;
            serde_json::from_str(&text)
                .map_err(|e| std::io::Error::other(format!("{}: {e}", p.display())))
        })
        .collect()
}

/// Generate `count` randomized scenario variations: lead vehicles at varying
/// gaps and speeds, oncoming and crossing traffic, and curved roads.
/// Deterministic in `seed`, standing in for nuPlan logs at batch scale.
pub fn synthetic_batch(count: usize, seed: u64) -> Vec<Scenario> {
    let mut rng = crate::Rng(seed | 1);
    (0..count)
        .map(|i| {
            let ego_speed = rng.range(5.0, 12.0);
            let amplitude = rng.range(0.0, 10.0);
            let wavelength = rng.range(60.0, 140.0);
            let centerline: Vec<[f64; 2]> = (0..=90)
                .map(|k| {
                    let x = k as f64 * 5.0 - 50.0;
                    [
                        x,
                        amplitude * (x / wavelength * std::f64::consts::TAU).sin(),
                    ]
                })
                .collect();
            let (kind, actors) = match i % 4 {
                0 => (
                    "lead",
                    vec![Actor {
                        init: State {
                            x: rng.range(25.0, 80.0),
                            speed: rng.range(0.0, 6.0),
                            ..Default::default()
                        },
                        control: Control::default(),
                        trajectory: vec![],
                    }],
                ),
                1 => (
                    "oncoming",
                    vec![Actor {
                        init: State {
                            x: rng.range(120.0, 200.0),
                            y: 4.0,
                            yaw: std::f64::consts::PI,
                            speed: rng.range(4.0, 10.0),
                        },
                        control: Control::default(),
                        trajectory: vec![],
                    }],
                ),
                2 => (
                    "crossing",
                    vec![Actor {
                        init: State {
                            x: rng.range(50.0, 110.0),
                            y: -60.0,
                            yaw: std::f64::consts::FRAC_PI_2,
                            speed: rng.range(3.0, 8.0),
                        },
                        control: Control::default(),
                        trajectory: vec![],
                    }],
                ),
                _ => ("open", vec![]),
            };
            Scenario {
                name: format!("{kind}-{i:03}"),
                ego: State {
                    y: rng.range(-2.0, 2.0),
                    speed: ego_speed,
                    ..Default::default()
                },
                actors,
                centerline,
                target_speed: 10.0,
                map: MapData::default(),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_batch_is_deterministic() {
        let a = synthetic_batch(12, 7);
        let b = synthetic_batch(12, 7);
        assert_eq!(a.len(), 12);
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.name, y.name);
            assert_eq!(x.ego, y.ego);
        }
    }

    #[test]
    fn scenario_json_round_trip_with_defaults() {
        let json = r#"{
            "name": "minimal",
            "ego": {"x": 0.0, "y": 0.0, "speed": 8.0},
            "centerline": [[-10.0, 0.0], [100.0, 0.0]]
        }"#;
        let sc: Scenario = serde_json::from_str(json).unwrap();
        assert_eq!(sc.target_speed, 10.0);
        assert!(sc.actors.is_empty());
        let back: Scenario = serde_json::from_str(&serde_json::to_string(&sc).unwrap()).unwrap();
        assert_eq!(back.ego, sc.ego);
    }

    #[test]
    fn replay_interpolates_and_extrapolates() {
        let wp = |t, x, speed| Waypoint {
            t,
            state: State {
                x,
                speed,
                ..Default::default()
            },
        };
        let actor = Actor {
            init: wp(0.0, 0.0, 10.0).state,
            control: Control::default(),
            trajectory: vec![wp(0.0, 0.0, 10.0), wp(1.0, 10.0, 10.0)],
        };
        let sc = Scenario {
            name: "replay".into(),
            ego: State {
                y: 4.0,
                speed: 1.0,
                ..Default::default()
            },
            actors: vec![actor],
            centerline: vec![[-10.0, 0.0], [100.0, 0.0]],
            target_speed: 10.0,
            map: MapData::default(),
        };
        let r = simulate(&sc, crate::PlannerKind::Straight, 2.0, 0.1);
        // interpolated halfway through the log
        assert!((r.actors[0][5].x - 5.0).abs() < 1e-9);
        // constant velocity beyond the log's end
        assert!((r.actors[0][20].x - 20.0).abs() < 1e-9);
    }

    #[test]
    fn trajectory_json_round_trip() {
        let json = r#"{
            "name": "logged",
            "ego": {"x": 0.0, "y": 0.0, "speed": 8.0},
            "actors": [{
                "init": {"x": 30.0, "y": 0.0},
                "trajectory": [
                    {"t": 0.0, "x": 30.0, "y": 0.0, "yaw": 0.0, "speed": 5.0},
                    {"t": 0.5, "x": 32.5, "y": 0.0, "yaw": 0.0, "speed": 5.0}
                ]
            }],
            "centerline": [[-10.0, 0.0], [100.0, 0.0]]
        }"#;
        let sc: Scenario = serde_json::from_str(json).unwrap();
        assert_eq!(sc.actors[0].trajectory.len(), 2);
        let text = serde_json::to_string(&sc).unwrap();
        let back: Scenario = serde_json::from_str(&text).unwrap();
        assert_eq!(back.actors[0].trajectory, sc.actors[0].trajectory);
    }

    #[test]
    fn simulate_produces_full_rollout_and_metrics() {
        let sc = &synthetic_batch(1, 3)[0];
        let r = simulate(sc, crate::PlannerKind::Straight, 5.0, 0.1);
        assert_eq!(r.ego.len(), 51);
        assert_eq!(r.metrics.per_tick.len(), 51);
    }
}
