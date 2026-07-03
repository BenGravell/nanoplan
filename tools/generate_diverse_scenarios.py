#!/usr/bin/env python3
"""Generate a large, diverse set of synthetic scenarios for the web build.

There's no real nuPlan corpus checked into this repo (the dataset requires
registration — see docs/USAGE.md#exporting-real-nuplan-scenarios), so the web
build ships with an empty scenarios/web_bundle.json by default. This script
is a stand-in: it procedurally generates scenarios spanning a wide range of
road geometries, speeds, and agent interactions, tagged with nuPlan-style
scenario-type names (see scenarios/nuplan/nuplan_schema.md's scenario_tag
table), and writes them straight into scenarios/web/ for
tools/bundle_web_scenarios.py to bundle — so the web viewer has a rich
scenario set available immediately, with no upload required.

Every actor is a fully scripted 20s/10Hz trajectory (the same replay format
as scenarios/json/cut_in.json), computed by simulating a control profile
(accel, curvature) through the exact kinematic step nanoplan's own physics
uses (src/simulation/mod.rs::step) — so a "braking to a stop" or "cutting
into the lane" actor behaves exactly as advertised, with speed clamped at
zero rather than drifting negative the way a raw constant Control would.

"Environmental conditions" (wet road, night, fog, school zone) are not
physically modeled anywhere in nanoplan (no weather/visibility/friction
system exists) — they're approximated the only way the schema allows: a
reduced target_speed, applied post-hoc to a random subset of scenarios
across every category, and called out in the scenario name. Treat it as
flavor/difficulty variation, not a claim that the viewer renders weather.

Usage:
  python3 tools/generate_diverse_scenarios.py [--out scenarios/web] [--variations 5] [--seed 1]
  python3 tools/bundle_web_scenarios.py
"""

import argparse
import json
import math
import random
import shutil
from pathlib import Path

DT = 0.1
DURATION_S = 20.0
N_TICKS = round(DURATION_S / DT)


# --- road geometry -----------------------------------------------------


class Path2D:
    """Builds a centerline polyline with a moving pen: `.straight(len)` and
    `.arc(radius, sweep_deg, direction)` segments chain together, each
    continuing from the previous segment's end position/heading — the same
    way a car's own path would connect a straightaway into a turn."""

    def __init__(self, x=-50.0, y=0.0, heading=0.0, step=5.0):
        self.x, self.y, self.heading, self.step = x, y, heading, step
        self.pts = [[round(x, 2), round(y, 2)]]

    def straight(self, length):
        n = max(1, round(length / self.step))
        for _ in range(n):
            self.x += self.step * math.cos(self.heading)
            self.y += self.step * math.sin(self.heading)
            self.pts.append([round(self.x, 2), round(self.y, 2)])
        return self

    def arc(self, radius, sweep_deg, direction):
        """direction: +1 curves left, -1 curves right."""
        sweep = math.radians(sweep_deg)
        n = max(12, round(abs(sweep) * radius / self.step))
        dtheta = direction * sweep / n
        ds = radius * abs(dtheta)
        for _ in range(n):
            self.heading += dtheta / 2
            self.x += ds * math.cos(self.heading)
            self.y += ds * math.sin(self.heading)
            self.heading += dtheta / 2
            self.pts.append([round(self.x, 2), round(self.y, 2)])
        return self

    def build(self):
        return self.pts


def sine_centerline(rng, length=450.0, amplitude=(1.0, 8.0), wavelength=(70.0, 150.0), x0=-50.0, step=5.0):
    a = rng.uniform(*amplitude)
    wl = rng.uniform(*wavelength)
    n = round(length / step)
    pts = []
    for i in range(n + 1):
        x = x0 + i * step
        pts.append([round(x, 2), round(a * math.sin(x / wl * math.tau), 2)])
    return pts


# --- actor motion --------------------------------------------------------


def _waypoint(t, state):
    x, y, yaw, speed = state
    return {"t": round(t, 2), "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4), "speed": round(speed, 3)}


def scripted_actor(x0, y0, yaw0, speed0, control_fn):
    """Simulate `control_fn(t, [x, y, yaw, speed]) -> (accel, curvature)`
    over the full scenario duration with nanoplan's own kinematic step,
    clamping speed at zero, and package the result as a replayed Actor."""
    state = [x0, y0, yaw0, max(0.0, speed0)]
    traj = [_waypoint(0.0, state)]
    for i in range(1, N_TICKS + 1):
        t = (i - 1) * DT
        accel, curvature = control_fn(t, state)
        x, y, yaw, speed = state
        state = [
            x + speed * math.cos(yaw) * DT,
            y + speed * math.sin(yaw) * DT,
            yaw + speed * curvature * DT,
            max(0.0, speed + accel * DT),
        ]
        traj.append(_waypoint(i * DT, state))
    return {"init": {k: traj[0][k] for k in ("x", "y", "yaw", "speed")}, "trajectory": traj}


def simple_actor(x, y, yaw, speed, accel=0.0, curvature=0.0):
    """A constant-control actor (straight line, or a steady turn/closing
    speed) — cheaper than a scripted trajectory when no clamping or
    maneuver is needed."""
    actor = {"init": {"x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 4), "speed": round(speed, 2)}}
    if accel != 0.0 or curvature != 0.0:
        actor["control"] = {"accel": round(accel, 3), "curvature": round(curvature, 5)}
    return actor


def brake_to_stop(decel, brake_start=0.0):
    def fn(t, state):
        if t < brake_start or state[3] <= 0.0:
            return (0.0, 0.0)
        return (-decel, 0.0)

    return fn


def lane_shift(curve_start, curve_duration, curvature, decel_start=None, decel=0.0, min_speed=0.0):
    """Cut-in / merge maneuver: hold straight, curve laterally for
    `curve_duration` seconds to change lanes (mirrors scenarios/json/cut_in.json),
    then optionally decelerate. `curvature`'s sign picks which way it turns."""

    def fn(t, state):
        c = curvature if curve_start <= t < curve_start + curve_duration else 0.0
        a = -decel if (decel_start is not None and t >= decel_start and state[3] > min_speed) else 0.0
        return (a, c)

    return fn


# --- scenario assembly ---------------------------------------------------


def mk_scenario(name, ego, actors, centerline, target_speed, map_overrides=None):
    m = {"road_half_width": 5.5, "divider_d": None, "crosswalk_s": []}
    if map_overrides:
        m.update(map_overrides)
    return {
        "name": name,
        "ego": ego,
        "actors": actors,
        "centerline": centerline,
        "target_speed": round(target_speed, 2),
        "map": m,
    }


def ego_state(speed, y=0.0, yaw=0.0, x=0.0):
    return {"x": x, "y": round(y, 2), "yaw": round(yaw, 4), "speed": round(speed, 2)}


# --- categories: (nuPlan-style scenario type, generator(rng) -> scenario) --


def gen_stationary_lead(rng):
    speed = rng.uniform(6.0, 12.0)
    gap = rng.uniform(30.0, 90.0)
    centerline = Path2D().straight(300).build()
    actors = [simple_actor(gap, 0.0, 0.0, 0.0)]
    return mk_scenario(
        "stationary_lead", ego_state(speed), actors, centerline, target_speed=speed + rng.uniform(0.0, 2.0)
    )


def gen_stopping_with_lead(rng):
    ego_speed = rng.uniform(7.0, 13.0)
    lead_speed = rng.uniform(5.0, 10.0)
    gap = rng.uniform(25.0, 50.0)
    decel = rng.uniform(1.5, 3.5)
    brake_start = rng.uniform(1.0, 4.0)
    centerline = Path2D().straight(300).build()
    actors = [scripted_actor(gap, 0.0, 0.0, lead_speed, brake_to_stop(decel, brake_start))]
    return mk_scenario("stopping_with_lead", ego_state(ego_speed), actors, centerline, target_speed=ego_speed)


def gen_following_lead_at_speed(rng):
    ego_speed = rng.uniform(8.0, 16.0)
    lead_speed = ego_speed + rng.uniform(-2.0, 1.0)
    gap = rng.uniform(20.0, 60.0)
    centerline = Path2D().straight(350).build()
    actors = [simple_actor(gap, 0.0, 0.0, max(0.5, lead_speed))]
    return mk_scenario(
        "following_lead_at_speed", ego_state(ego_speed), actors, centerline, target_speed=max(ego_speed, lead_speed)
    )


def gen_starting_left_turn(rng):
    radius = rng.uniform(25.0, 55.0)
    sweep = rng.uniform(45.0, 100.0)
    speed = rng.uniform(4.0, 9.0)
    centerline = Path2D().straight(rng.uniform(15.0, 35.0)).arc(radius, sweep, +1).straight(30.0).build()
    return mk_scenario("starting_left_turn", ego_state(speed), [], centerline, target_speed=speed)


def gen_starting_right_turn(rng):
    radius = rng.uniform(25.0, 55.0)
    sweep = rng.uniform(45.0, 100.0)
    speed = rng.uniform(4.0, 9.0)
    centerline = Path2D().straight(rng.uniform(15.0, 35.0)).arc(radius, sweep, -1).straight(30.0).build()
    return mk_scenario("starting_right_turn", ego_state(speed), [], centerline, target_speed=speed)


def gen_high_lateral_acceleration(rng):
    radius = rng.uniform(12.0, 22.0)
    sweep = rng.uniform(70.0, 120.0)
    direction = rng.choice([-1, 1])
    speed = rng.uniform(7.0, 12.0)
    centerline = Path2D().straight(20.0).arc(radius, sweep, direction).straight(20.0).build()
    return mk_scenario("high_lateral_acceleration", ego_state(speed), [], centerline, target_speed=speed)


def gen_s_curve_road(rng):
    radius = rng.uniform(30.0, 60.0)
    sweep = rng.uniform(35.0, 70.0)
    speed = rng.uniform(6.0, 11.0)
    centerline = (
        Path2D()
        .straight(20.0)
        .arc(radius, sweep, +1)
        .arc(radius, sweep, -1)
        .straight(30.0)
        .build()
    )
    return mk_scenario("s_curve_road", ego_state(speed), [], centerline, target_speed=speed)


def gen_roundabout_like_curve(rng):
    radius = rng.uniform(20.0, 35.0)
    sweep = rng.uniform(160.0, 260.0)
    direction = rng.choice([-1, 1])
    speed = rng.uniform(4.0, 7.0)
    centerline = Path2D().straight(20.0).arc(radius, sweep, direction).straight(20.0).build()
    return mk_scenario("traversing_roundabout", ego_state(speed), [], centerline, target_speed=speed)


def gen_pedestrian_crossing(rng):
    ego_speed = rng.uniform(4.0, 9.0)
    crossing_x = rng.uniform(40.0, 70.0)
    ped_speed = rng.uniform(1.0, 1.8)
    y0 = -rng.uniform(4.0, 7.0)
    centerline = Path2D().straight(300).build()
    actors = [simple_actor(crossing_x, y0, math.pi / 2, ped_speed)]
    return mk_scenario(
        "waiting_for_pedestrian_to_cross",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"crosswalk_s": [round(crossing_x, 1)]},
    )


def gen_crossing_traffic(rng):
    ego_speed = rng.uniform(6.0, 12.0)
    crossing_x = rng.uniform(50.0, 110.0)
    actor_speed = rng.uniform(3.0, 9.0)
    from_left = rng.choice([True, False])
    y0 = -60.0 if from_left else 60.0
    yaw0 = math.pi / 2 if from_left else -math.pi / 2
    centerline = Path2D().straight(300).build()
    actors = [simple_actor(crossing_x, y0, yaw0, actor_speed)]
    return mk_scenario(
        "traversing_intersection", ego_state(ego_speed), actors, centerline, target_speed=ego_speed
    )


def gen_crossing_traffic_high_speed(rng):
    ego_speed = rng.uniform(10.0, 16.0)
    crossing_x = rng.uniform(70.0, 130.0)
    actor_speed = rng.uniform(9.0, 14.0)
    centerline = Path2D().straight(320).build()
    actors = [simple_actor(crossing_x, -60.0, math.pi / 2, actor_speed)]
    return mk_scenario(
        "high_speed_crossing_traffic", ego_state(ego_speed), actors, centerline, target_speed=ego_speed
    )


def gen_oncoming_traffic(rng):
    ego_speed = rng.uniform(6.0, 12.0)
    actor_speed = rng.uniform(4.0, 11.0)
    actor_x = rng.uniform(100.0, 220.0)
    lane_offset = rng.uniform(3.0, 4.5)
    centerline = Path2D().straight(350).build()
    actors = [simple_actor(actor_x, lane_offset, math.pi, actor_speed)]
    return mk_scenario(
        "on_coming_traffic",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"divider_d": 0.0},
    )


def gen_oncoming_on_narrow_road(rng):
    ego_speed = rng.uniform(4.0, 8.0)
    actor_speed = rng.uniform(4.0, 8.0)
    actor_x = rng.uniform(90.0, 160.0)
    centerline = Path2D().straight(300).build()
    actors = [simple_actor(actor_x, 2.6, math.pi, actor_speed)]
    return mk_scenario(
        "oncoming_traffic_narrow_road",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"road_half_width": 3.2, "divider_d": 0.0},
    )


def gen_bidirectional_intersection(rng):
    ego_speed = rng.uniform(6.0, 11.0)
    centerline = Path2D().straight(320).build()
    oncoming = simple_actor(rng.uniform(120.0, 200.0), 3.5, math.pi, rng.uniform(4.0, 9.0))
    crossing = simple_actor(rng.uniform(50.0, 100.0), -60.0, math.pi / 2, rng.uniform(3.0, 8.0))
    return mk_scenario(
        "on_intersection_bidirectional_traffic",
        ego_state(ego_speed),
        [oncoming, crossing],
        centerline,
        target_speed=ego_speed,
        map_overrides={"divider_d": 0.0},
    )


def gen_near_multiple_vehicles(rng):
    ego_speed = rng.uniform(6.0, 11.0)
    centerline = Path2D().straight(350).build()
    lead = simple_actor(rng.uniform(30.0, 60.0), 0.0, 0.0, rng.uniform(3.0, 8.0))
    oncoming = simple_actor(rng.uniform(130.0, 210.0), 3.5, math.pi, rng.uniform(4.0, 9.0))
    crossing = simple_actor(rng.uniform(70.0, 120.0), 60.0, -math.pi / 2, rng.uniform(3.0, 7.0))
    return mk_scenario(
        "near_multiple_vehicles",
        ego_state(ego_speed),
        [lead, oncoming, crossing],
        centerline,
        target_speed=ego_speed,
        map_overrides={"divider_d": 0.0},
    )


def gen_congested_stop_and_go(rng):
    ego_speed = rng.uniform(5.0, 9.0)
    centerline = Path2D().straight(300).build()
    actors = []
    gap = rng.uniform(20.0, 30.0)
    for k in range(3):
        gap += rng.uniform(18.0, 28.0)
        speed = rng.uniform(3.0, 7.0)
        decel = rng.uniform(1.0, 2.5)
        brake_start = rng.uniform(0.5, 5.0) + k * 1.5
        actors.append(scripted_actor(gap, 0.0, 0.0, speed, brake_to_stop(decel, brake_start)))
    return mk_scenario("congested_stop_and_go", ego_state(ego_speed), actors, centerline, target_speed=ego_speed)


def gen_changing_lane_cut_in(rng):
    ego_speed = rng.uniform(7.0, 12.0)
    actor_speed = ego_speed + rng.uniform(-1.0, 2.0)
    from_left = rng.choice([True, False])
    y0 = rng.uniform(3.0, 4.5) * (1 if from_left else -1)
    curvature = rng.uniform(0.005, 0.011) * (-1 if from_left else 1)
    curve_start = rng.uniform(1.0, 3.0)
    curve_duration = rng.uniform(1.5, 2.5)
    x0 = rng.uniform(20.0, 40.0)
    centerline = Path2D().straight(350).build()
    actor = scripted_actor(x0, y0, 0.0, actor_speed, lane_shift(curve_start, curve_duration, curvature))
    return mk_scenario("changing_lane_cut_in", ego_state(ego_speed), [actor], centerline, target_speed=ego_speed)


def gen_merging_onto_highway(rng):
    ego_speed = rng.uniform(15.0, 22.0)
    actor_speed = rng.uniform(12.0, 20.0)
    from_left = rng.choice([True, False])
    y0 = rng.uniform(3.5, 5.0) * (1 if from_left else -1)
    curvature = rng.uniform(0.004, 0.008) * (-1 if from_left else 1)
    curve_start = rng.uniform(1.5, 4.0)
    curve_duration = rng.uniform(2.5, 4.0)
    x0 = rng.uniform(30.0, 55.0)
    centerline = Path2D().straight(400).build()
    actor = scripted_actor(x0, y0, 0.0, actor_speed, lane_shift(curve_start, curve_duration, curvature))
    return mk_scenario(
        "merging_onto_highway",
        ego_state(ego_speed),
        [actor],
        centerline,
        target_speed=ego_speed,
        map_overrides={"road_half_width": 7.0},
    )


def gen_high_speed_highway_cruise(rng):
    ego_speed = rng.uniform(18.0, 25.0)
    centerline = sine_centerline(rng, length=500.0, amplitude=(0.5, 3.0), wavelength=(150.0, 260.0))
    actors = []
    if rng.random() < 0.5:
        actors = [simple_actor(rng.uniform(80.0, 160.0), 0.0, 0.0, ego_speed + rng.uniform(-3.0, 1.0))]
    return mk_scenario(
        "high_speed_highway_cruise",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"road_half_width": 7.0},
    )


def gen_low_speed_residential(rng):
    ego_speed = rng.uniform(2.5, 6.0)
    centerline = Path2D().straight(20.0).arc(rng.uniform(18.0, 30.0), rng.uniform(30.0, 60.0), rng.choice([-1, 1])).straight(20.0).build()
    actors = []
    if rng.random() < 0.4:
        actors = [simple_actor(rng.uniform(20.0, 45.0), 0.0, 0.0, 0.0)]
    return mk_scenario(
        "low_speed_residential_street",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"road_half_width": 3.5},
    )


def gen_school_zone(rng):
    ego_speed = rng.uniform(3.0, 6.0)
    crossing_x = rng.uniform(35.0, 60.0)
    centerline = Path2D().straight(280).build()
    actors = [simple_actor(crossing_x, -3.5, math.pi / 2, rng.uniform(0.8, 1.4))]
    return mk_scenario(
        "school_zone_pedestrian",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"road_half_width": 3.8, "crosswalk_s": [round(crossing_x, 1)]},
    )


def gen_narrow_construction_zone(rng):
    ego_speed = rng.uniform(4.0, 8.0)
    centerline = Path2D().straight(300).build()
    actors = [simple_actor(rng.uniform(30.0, 55.0), 0.0, 0.0, rng.uniform(0.0, 1.5))]
    return mk_scenario(
        "narrow_construction_zone",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"road_half_width": 2.8},
    )


def gen_accelerating_from_stop(rng):
    target_speed = rng.uniform(9.0, 16.0)
    centerline = sine_centerline(rng, length=350.0, amplitude=(0.0, 3.0), wavelength=(100.0, 180.0))
    return mk_scenario("accelerating_from_a_stop", ego_state(0.0), [], centerline, target_speed=target_speed)


CATEGORIES = [
    gen_stationary_lead,
    gen_stopping_with_lead,
    gen_following_lead_at_speed,
    gen_starting_left_turn,
    gen_starting_right_turn,
    gen_high_lateral_acceleration,
    gen_s_curve_road,
    gen_roundabout_like_curve,
    gen_pedestrian_crossing,
    gen_crossing_traffic,
    gen_crossing_traffic_high_speed,
    gen_oncoming_traffic,
    gen_oncoming_on_narrow_road,
    gen_bidirectional_intersection,
    gen_near_multiple_vehicles,
    gen_congested_stop_and_go,
    gen_changing_lane_cut_in,
    gen_merging_onto_highway,
    gen_high_speed_highway_cruise,
    gen_low_speed_residential,
    gen_school_zone,
    gen_narrow_construction_zone,
    gen_accelerating_from_stop,
]

# (label, target_speed multiplier) — approximates the driving-condition axis
# by derating the reference cruise speed; see the module docstring for why
# this (and not literal weather) is what "environmental conditions" means
# here.
CONDITIONS = [
    ("wet_road", 0.82),
    ("night_low_visibility", 0.75),
    ("dense_fog", 0.68),
    ("school_zone_caution", 0.6),
]


def apply_condition(rng, scenario, probability=0.35):
    if rng.random() >= probability:
        return scenario
    label, factor = rng.choice(CONDITIONS)
    scenario["target_speed"] = round(scenario["target_speed"] * factor, 2)
    scenario["name"] = f"{scenario['name']} [{label}]"
    return scenario


def generate(variations, seed):
    scenarios = []
    for gen_fn in CATEGORIES:
        category = gen_fn.__name__.removeprefix("gen_")
        for i in range(variations):
            rng = random.Random(f"{seed}:{category}:{i}")
            scenario = gen_fn(rng)
            scenario["name"] = f"nuplan: {scenario['name']}-{i:03}"
            scenario = apply_condition(rng, scenario)
            scenario["_filename"] = f"{category}-{i:03}.json"
            scenarios.append(scenario)
    return scenarios


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="scenarios/web", help="output directory (default: scenarios/web)")
    ap.add_argument("--variations", type=int, default=5, help="variations per category (default: 5)")
    ap.add_argument("--seed", type=int, default=1, help="RNG seed (default: 1)")
    ap.add_argument("--clean", action="store_true", help="remove the output directory first")
    args = ap.parse_args()

    out = Path(args.out)
    if args.clean and out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    scenarios = generate(args.variations, args.seed)
    for scenario in scenarios:
        filename = scenario.pop("_filename")
        (out / filename).write_text(json.dumps(scenario, indent=1))

    print(f"wrote {len(scenarios)} scenarios ({len(CATEGORIES)} categories x {args.variations} variations) to {out}")


if __name__ == "__main__":
    main()
