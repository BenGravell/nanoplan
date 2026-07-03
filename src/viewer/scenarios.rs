//! Where the viewer's scenarios come from: six built-in synthetic examples,
//! JSON bundled into the binary at compile time, and (desktop only) any
//! files/directories passed as CLI args.

use nanoplan::scenarios::{Actor, MapData};
use nanoplan::{Control, Scenario, State};

/// Scenario JSONs bundled into the binary, so they ship to the web build too.
const BUNDLED: [&str; 2] = [
    include_str!("../../scenarios/json/braking_lead.json"),
    include_str!("../../scenarios/json/cut_in.json"),
];

/// All scenarios the viewer offers: built-ins, bundled JSON (e.g. exported
/// from nuPlan logs), and — on desktop — any files or directories passed as
/// CLI args (see also the in-app scenario-loading widget in `ui`).
pub(crate) fn all_scenarios() -> Vec<Scenario> {
    let mut all = scenarios();
    all.extend(
        BUNDLED
            .iter()
            .map(|s| serde_json::from_str(s).expect("bundled scenario is valid JSON")),
    );
    #[cfg(not(target_arch = "wasm32"))]
    for arg in std::env::args().skip(1) {
        match nanoplan::scenarios::load_path(std::path::Path::new(&arg)) {
            Ok(loaded) => all.extend(loaded),
            Err(e) => eprintln!("skipping scenario path {arg}: {e}"),
        }
    }
    all
}

fn straight_road() -> Vec<[f64; 2]> {
    (0..=45).map(|i| [i as f64 * 10.0 - 50.0, 0.0]).collect()
}

fn s_curve_road() -> Vec<[f64; 2]> {
    (0..=90)
        .map(|i| {
            let x = i as f64 * 5.0 - 50.0;
            [x, 8.0 * (x / 40.0).sin()]
        })
        .collect()
}

// ponytail: synthetic scenarios stand in for nuPlan logs until a loader exists
fn scenarios() -> Vec<Scenario> {
    let state = |x, y, yaw, speed| State { x, y, yaw, speed };
    let drive = |accel, curvature| Control { accel, curvature };
    let map = |divider_d, crosswalk_s| MapData {
        divider_d,
        crosswalk_s,
        ..Default::default()
    };
    let scenario = |name: &str, ego, actors, centerline, map| Scenario {
        name: name.into(),
        ego,
        actors,
        centerline,
        target_speed: 10.0,
        map,
    };
    vec![
        scenario(
            "offset start",
            state(0.0, 3.0, 0.0, 8.0),
            vec![],
            straight_road(),
            map(None, vec![]),
        ),
        scenario(
            "s-curve road",
            state(0.0, 0.0, 0.0, 8.0),
            vec![],
            s_curve_road(),
            map(None, vec![]),
        ),
        scenario(
            "stopped lead",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(60.0, 0.0, 0.0, 0.0),
                control: drive(0.0, 0.0),
                trajectory: vec![],
            }],
            straight_road(),
            map(None, vec![]),
        ),
        scenario(
            "oncoming",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(160.0, 4.0, std::f64::consts::PI, 8.0),
                control: drive(0.0, 0.0),
                trajectory: vec![],
            }],
            straight_road(),
            map(Some(2.0), vec![]),
        ),
        scenario(
            "crossing",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(80.0, -60.0, std::f64::consts::FRAC_PI_2, 6.0),
                control: drive(0.0, 0.0),
                trajectory: vec![],
            }],
            straight_road(),
            // crossing traffic at x = 80 → station 130 on a road starting at -50
            map(None, vec![130.0]),
        ),
        scenario(
            "curving lead",
            state(0.0, 0.0, 0.0, 8.0),
            vec![Actor {
                init: state(20.0, 0.0, 0.0, 8.0),
                // curves away; constant-velocity prediction visibly diverges
                control: drive(0.0, 0.02),
                trajectory: vec![],
            }],
            straight_road(),
            map(None, vec![]),
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use nanoplan::{PlannerKind, simulate};

    #[test]
    fn bundled_scenarios_parse_and_simulate() {
        let scenes = all_scenarios();
        assert!(scenes.len() >= 8); // 6 built-ins + 2 bundled
        let bundled = &scenes[6];
        assert!(bundled.name.starts_with("nuplan:"));
        assert!(!bundled.actors[0].trajectory.is_empty());
        let r = simulate(bundled, PlannerKind::Straight, 2.0, super::super::DT);
        assert_eq!(r.ego.len(), 21);
    }
}
