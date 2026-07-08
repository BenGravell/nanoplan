#!/usr/bin/env python3
"""Generate the repo's diverse scenario corpus as CommonRoad XML.

Writes procedurally generated scenarios — spanning a wide range of road
geometries, speeds, and agent interactions — into scenarios/commonroad/ in
the CommonRoad 2020a format (https://commonroad.in.tum.de): a lanelet
network, dynamic/static obstacles with full state trajectories, and a
planning problem. Unlike nuPlan logs (registration-gated, no
redistribution), these files are original to this repo and ship in it
freely; tools/export_commonroad_scenarios.py converts them (or any real
CommonRoad scenario) to the nanoplan JSON the viewer and batch runner load.

Every obstacle carries a fully scripted 20s/10Hz trajectory, computed by
simulating an acceleration/curvature profile through the same kinematic
model nanoplan uses (src/simulation/mod.rs::step) — so a
"braking to a stop" or "cutting into the lane" obstacle behaves exactly as
advertised, with speed clamped at zero rather than drifting negative under a
constant acceleration state.

Environmental conditions (rain, night, fog) are expressed the CommonRoad
way — a location/environment element — plus a derated goal-velocity
interval, applied to a random subset of scenarios. nanoplan has no
weather/visibility model, so after conversion this surfaces only as a
reduced target_speed and a name suffix: flavor/difficulty variation, not a
claim that the viewer renders weather.

Besides the randomized categories, two fixed "classic" scenarios
(ZAM_BrakingLead-1_1_T-1, ZAM_CutIn-1_1_T-1) are always generated — their
conversions are checked in as scenarios/json/*.json and compiled into the
viewer binary.

Usage:
  python3 tools/generate_diverse_scenarios.py [--out scenarios/commonroad] [--variations 1] [--seed 1]
  python3 tools/export_commonroad_scenarios.py scenarios/commonroad scenarios/web
  python3 tools/bundle_web_scenarios.py
"""

import argparse
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

DT = 0.1
DURATION_S = 20.0
N_TICKS = round(DURATION_S / DT)

# Every curved-geometry generator's centerline must stay straight for at
# least this long before it starts curving. The ego is placed at the fixed
# origin (0, 0) — same convention scenarios/json/*.json uses, and every
# actor placement below assumes — while every Path2D centerline starts at
# x=-50 (road context *behind* the ego). A lead-in shorter than 50 m lets
# the arc begin before x=0, so the ego (still pinned at the origin) lands
# off the road entirely instead of on the straight run — the exact bug
# behind a generated `traversing_roundabout` scenario starting off-road.
LEAD_IN_M = 55.0


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


def _waypoint(t, state, accel=0.0, curvature=0.0):
    x, y, yaw, speed = state
    return {
        "t": round(t, 2),
        "x": round(x, 3),
        "y": round(y, 3),
        "yaw": round(yaw, 4),
        "speed": round(speed, 3),
        "accel": round(accel, 3),
        "curvature": round(curvature, 5),
    }


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
        traj.append(_waypoint(i * DT, state, accel, curvature))
    return {"init": {k: traj[0][k] for k in ("x", "y", "yaw", "speed")}, "trajectory": traj}


def simple_actor(x, y, yaw, speed, accel=0.0, curvature=0.0):
    """A constant-actuator actor (straight line, or a steady turn/closing
    speed) — cheaper than a scripted trajectory when no clamping or
    maneuver is needed.

    `x` here is an absolute world coordinate (matching the ego, always at
    the origin) — NOT the same number as a Frenet *station* along the
    centerline, which for every `Path2D().straight(...)`-based generator in
    this file starts 50 m behind the ego at x=-50. Placing a `crosswalk_s`
    or `cross_streets` marker at a crossing actor's own `x` value without
    subtracting `centerline[0][0]` first draws it 50 m short of where the
    actor actually crosses — a bug found once already (see
    `LEAD_IN_M`/`ego_state`'s docstring for the ego-position version of the
    same mix-up) and worth not repeating."""
    actor = {"init": {"x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 4), "speed": round(speed, 2)}}
    if accel != 0.0 or curvature != 0.0:
        actor["init"]["accel"] = round(accel, 3)
        actor["init"]["curvature"] = round(curvature, 5)
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
    m = {"road_half_width": 5.5, "divider_d": None, "crosswalk_s": [], "cross_streets": []}
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
    """Every actor/lead/crossing placement below is an absolute x
    coordinate measured as "ahead of the ego", implicitly treating the ego
    as sitting at the origin — same convention the hand-authored
    scenarios/json/*.json files use (a centerline that starts at x=-50,
    i.e. extends *behind* the ego for road context, with the ego itself at
    x=0). Every `Path2D`-based centerline below must keep an initial
    straight run of at least 50 m (matching Path2D's default start x=-50)
    before any curving starts, so x=0 always falls on that straight run —
    see the `LEAD_IN_M` comment on the curved generators for what happens
    if it doesn't."""
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
    centerline = Path2D().straight(LEAD_IN_M + rng.uniform(0.0, 20.0)).arc(radius, sweep, +1).straight(30.0).build()
    return mk_scenario("starting_left_turn", ego_state(speed), [], centerline, target_speed=speed)


def gen_starting_right_turn(rng):
    radius = rng.uniform(25.0, 55.0)
    sweep = rng.uniform(45.0, 100.0)
    speed = rng.uniform(4.0, 9.0)
    centerline = Path2D().straight(LEAD_IN_M + rng.uniform(0.0, 20.0)).arc(radius, sweep, -1).straight(30.0).build()
    return mk_scenario("starting_right_turn", ego_state(speed), [], centerline, target_speed=speed)


def gen_high_lateral_acceleration(rng):
    radius = rng.uniform(12.0, 22.0)
    sweep = rng.uniform(70.0, 120.0)
    direction = rng.choice([-1, 1])
    speed = rng.uniform(7.0, 12.0)
    centerline = Path2D().straight(LEAD_IN_M).arc(radius, sweep, direction).straight(20.0).build()
    return mk_scenario("high_lateral_acceleration", ego_state(speed), [], centerline, target_speed=speed)


def gen_s_curve_road(rng):
    radius = rng.uniform(30.0, 60.0)
    sweep = rng.uniform(35.0, 70.0)
    speed = rng.uniform(6.0, 11.0)
    centerline = (
        Path2D()
        .straight(LEAD_IN_M)
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
    centerline = Path2D().straight(LEAD_IN_M).arc(radius, sweep, direction).straight(20.0).build()
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
        map_overrides={"crosswalk_s": [round(crossing_x - centerline[0][0], 1)]},
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
        "traversing_intersection",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"cross_streets": [round(crossing_x - centerline[0][0], 1)]},
    )


def gen_crossing_traffic_high_speed(rng):
    ego_speed = rng.uniform(10.0, 16.0)
    crossing_x = rng.uniform(70.0, 130.0)
    actor_speed = rng.uniform(9.0, 14.0)
    centerline = Path2D().straight(320).build()
    actors = [simple_actor(crossing_x, -60.0, math.pi / 2, actor_speed)]
    return mk_scenario(
        "high_speed_crossing_traffic",
        ego_state(ego_speed),
        actors,
        centerline,
        target_speed=ego_speed,
        map_overrides={"cross_streets": [round(crossing_x - centerline[0][0], 1)]},
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
    crossing_x = rng.uniform(50.0, 100.0)
    crossing = simple_actor(crossing_x, -60.0, math.pi / 2, rng.uniform(3.0, 8.0))
    return mk_scenario(
        "on_intersection_bidirectional_traffic",
        ego_state(ego_speed),
        [oncoming, crossing],
        centerline,
        target_speed=ego_speed,
        map_overrides={"divider_d": 0.0, "cross_streets": [round(crossing_x - centerline[0][0], 1)]},
    )


def gen_near_multiple_vehicles(rng):
    ego_speed = rng.uniform(6.0, 11.0)
    centerline = Path2D().straight(350).build()
    lead = simple_actor(rng.uniform(30.0, 60.0), 0.0, 0.0, rng.uniform(3.0, 8.0))
    oncoming = simple_actor(rng.uniform(130.0, 210.0), 3.5, math.pi, rng.uniform(4.0, 9.0))
    crossing_x = rng.uniform(70.0, 120.0)
    crossing = simple_actor(crossing_x, 60.0, -math.pi / 2, rng.uniform(3.0, 7.0))
    return mk_scenario(
        "near_multiple_vehicles",
        ego_state(ego_speed),
        [lead, oncoming, crossing],
        centerline,
        target_speed=ego_speed,
        map_overrides={"divider_d": 0.0, "cross_streets": [round(crossing_x - centerline[0][0], 1)]},
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
    centerline = (
        Path2D()
        .straight(LEAD_IN_M)
        .arc(rng.uniform(18.0, 30.0), rng.uniform(30.0, 60.0), rng.choice([-1, 1]))
        .straight(20.0)
        .build()
    )
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
        map_overrides={"road_half_width": 3.8, "crosswalk_s": [round(crossing_x - centerline[0][0], 1)]},
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

# --- the two fixed classics (compiled into the viewer binary) -------------


def s_shift(start, half, c, decel_start=None, decel=0.0, min_speed=0.0):
    """A proper S-shaped lane change: curve one way for `half` seconds, then
    the opposite way for `half` seconds so the heading levels back out,
    then optionally decelerate."""

    def fn(t, state):
        if start <= t < start + half:
            cur = -c
        elif start + half <= t < start + 2 * half:
            cur = c
        else:
            cur = 0.0
        a = -decel if (decel_start is not None and t >= decel_start and state[3] > min_speed) else 0.0
        return (a, cur)

    return fn


def classic_braking_lead():
    """A lead at 40 m cruising at the ego's speed, braking to a stop at
    t=2 s — the successor of the original hand-authored braking_lead.json."""
    centerline = Path2D().straight(450).build()
    actor = scripted_actor(40.0, 0.0, 0.0, 8.0, brake_to_stop(2.0, 2.0))
    return mk_scenario("braking_lead", ego_state(8.0), [actor], centerline, target_speed=10.0)


def classic_cut_in():
    """A faster vehicle in the left lane cutting into the ego's lane at
    t=2 s, then slowing to 7 m/s — the successor of the original
    hand-authored cut_in.json."""
    centerline = Path2D().straight(450).build()
    actor = scripted_actor(
        25.0, 4.0, 0.0, 9.0, s_shift(2.0, 1.5, 0.022, decel_start=5.0, decel=1.0, min_speed=7.0)
    )
    return mk_scenario(
        "cut_in", ego_state(8.0), [actor], centerline, target_speed=10.0, map_overrides={"divider_d": 2.0}
    )


CLASSICS = [("BrakingLead", classic_braking_lead), ("CutIn", classic_cut_in)]

# CommonRoad location/environment for the driving-condition axis, plus a
# goal-velocity derate; see the module docstring for what this does and
# doesn't mean.
CONDITIONS = [
    ({"time": "12:00:00", "timeOfDay": "day", "weather": "light_rain", "underground": "wet"}, 0.82),
    ({"time": "23:00:00", "timeOfDay": "night", "weather": "sunny", "underground": "clean"}, 0.75),
    ({"time": "12:00:00", "timeOfDay": "day", "weather": "fog", "underground": "clean"}, 0.68),
]


def apply_condition(rng, scenario, probability=0.35):
    if rng.random() >= probability:
        return scenario
    environment, factor = rng.choice(CONDITIONS)
    scenario["target_speed"] = round(scenario["target_speed"] * factor, 2)
    scenario["environment"] = environment
    return scenario


# --- CommonRoad 2020a XML writer ------------------------------------------
#
# Field-name/structure reference: the official XSD, shipped in the
# commonroad-io package (commonroad/common/xml_definition_files/
# XML_commonRoad_XSD.xsd, BSD-3-Clause).

FILE_DATE = "2026-07-06"  # fixed so regeneration is byte-identical
LANE_HALF_DEFAULT = 1.85  # half of a standard 3.7 m lane

# scenarioTags elements, in the XSD's required sequence order
TAG_ORDER = [
    "highway", "urban", "cut_in", "intersection", "lane_following",
    "multi_lane", "no_oncoming_traffic", "oncoming_traffic",
    "parallel_lanes", "simulated", "single_lane", "emergency_braking",
]


def _el(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        e.text = str(text)
    return e


def _point(parent, tag, x, y):
    p = _el(parent, tag)
    _el(p, "x", round(x, 3))
    _el(p, "y", round(y, 3))
    return p


def _exact(parent, tag, value, digits=4):
    _el(_el(parent, tag), "exact", round(value, digits))


def _interval(parent, tag, lo, hi, digits=4):
    e = _el(parent, tag)
    _el(e, "intervalStart", round(lo, digits))
    _el(e, "intervalEnd", round(hi, digits))


def headings(pts):
    """Per-vertex heading of a polyline (averaged over adjacent segments)."""
    seg = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
    out = [seg[0]]
    for h0, h1 in zip(seg, seg[1:]):
        d = (h1 - h0 + math.pi) % math.tau - math.pi
        out.append(h0 + d / 2)
    out.append(seg[-1])
    return out


def offset_polyline(pts, d):
    """The polyline shifted laterally by `d` (positive = left)."""
    return [
        [x - d * math.sin(h), y + d * math.cos(h)]
        for (x, y), h in zip(pts, headings(pts))
    ]


def _bound(parent, tag, pts, marking):
    b = _el(parent, tag)
    for x, y in pts:
        _point(b, "point", x, y)
    _el(b, "lineMarking", marking)


def _is_opposite(actor):
    yaw = actor["init"].get("yaw", 0.0)
    return abs((yaw + math.pi) % math.tau - math.pi) > math.pi / 2


def add_lanelets(root, scenario):
    """The road as lanelets: the ego lane (id 100), one adjacent lane (101)
    when the scenario has a divider, a crosswalk lanelet per crosswalk_s
    (120+), and a crossing road per cross_streets (140+). Returns the ego
    lane's half-width (for the goal region)."""
    center = scenario["centerline"]
    m = scenario["map"]
    hw, div = m["road_half_width"], m["divider_d"]
    lane_type = "highway" if hw >= 6.5 else "urban"

    if div is None:
        b = hw
        ego = _el(root, "lanelet", id=100)
        _bound(ego, "leftBound", offset_polyline(center, b), "solid")
        _bound(ego, "rightBound", offset_polyline(center, -b), "solid")
        _el(ego, "laneletType", lane_type)
    else:
        b = div if div >= 1.0 else LANE_HALF_DEFAULT
        outer = max(hw, b + 3.0)
        # the adjacent lane sits on the side its traffic starts on
        off_lane = [a for a in scenario["actors"] if abs(a["init"]["y"]) > b]
        side = 1 if not off_lane or off_lane[0]["init"]["y"] > 0 else -1
        opposite = any(_is_opposite(a) for a in scenario["actors"])
        direction = "opposite" if opposite else "same"

        ego = _el(root, "lanelet", id=100)
        near = offset_polyline(center, side * b)
        far = offset_polyline(center, -side * b)
        if side > 0:
            _bound(ego, "leftBound", near, "dashed")
            _bound(ego, "rightBound", far, "solid")
        else:
            _bound(ego, "leftBound", far, "solid")
            _bound(ego, "rightBound", near, "dashed")
        _el(ego, "adjacentLeft" if side > 0 else "adjacentRight", ref=101, drivingDir=direction)
        _el(ego, "laneletType", lane_type)

        adj = _el(root, "lanelet", id=101)
        adj_outer = offset_polyline(center, side * outer)
        if opposite:
            # bounds run along the *opposite* lane's own travel direction
            left, right = (near, adj_outer) if side > 0 else (adj_outer, near)
            _bound(adj, "leftBound", left[::-1], "dashed" if side > 0 else "solid")
            _bound(adj, "rightBound", right[::-1], "solid" if side > 0 else "dashed")
            _el(adj, "adjacentLeft" if side > 0 else "adjacentRight", ref=100, drivingDir="opposite")
        else:
            left, right = (adj_outer, near) if side > 0 else (near, adj_outer)
            _bound(adj, "leftBound", left, "solid" if side > 0 else "dashed")
            _bound(adj, "rightBound", right, "dashed" if side > 0 else "solid")
            _el(adj, "adjacentRight" if side > 0 else "adjacentLeft", ref=100, drivingDir="same")
        _el(adj, "laneletType", lane_type)

    # crosswalks and cross streets only appear on straight roads (y = 0), so
    # a station converts to x by adding the centerline's start x
    x0 = center[0][0]
    span = hw + 2.0
    for i, s in enumerate(m["crosswalk_s"]):
        x = x0 + s
        cw = _el(root, "lanelet", id=120 + i)
        _bound(cw, "leftBound", [[x - 1.5, -span], [x - 1.5, 0.0], [x - 1.5, span]], "unknown")
        _bound(cw, "rightBound", [[x + 1.5, -span], [x + 1.5, 0.0], [x + 1.5, span]], "unknown")
        _el(cw, "laneletType", "crosswalk")
    for i, s in enumerate(m["cross_streets"]):
        x = x0 + s
        ys = [y * 5.0 for y in range(-16, 17)]  # -80..80, 5 m spacing
        street = _el(root, "lanelet", id=140 + i)
        _bound(street, "leftBound", [[x - 3.5, y] for y in ys], "solid")
        _bound(street, "rightBound", [[x + 3.5, y] for y in ys], "solid")
        _el(street, "laneletType", lane_type)
    return b


def rolled_out(actor):
    """Every obstacle gets a full trajectory: scripted actors already have
    one; constant-actuator actors are integrated with the same kinematic
    model (an obstacle in CommonRoad is a trajectory, not a control law)."""
    if "trajectory" in actor:
        return actor["trajectory"]
    init = actor["init"]
    fn = lambda t, s: (init.get("accel", 0.0), init.get("curvature", 0.0))  # noqa: E731
    return scripted_actor(init["x"], init["y"], init.get("yaw", 0.0), init["speed"], fn)["trajectory"]


def add_obstacles(root, scenario):
    for i, actor in enumerate(scenario["actors"]):
        init = actor["init"]
        speed = init.get("speed", 0.0)
        yaw = init.get("yaw", 0.0)
        moving = speed > 0.0 or actor.get("control") or actor.get("trajectory")
        pedestrian = moving and speed <= 2.0 and abs(math.sin(yaw)) > 0.7

        if not moving:
            obs = _el(root, "staticObstacle", id=200 + i)
            _el(obs, "type", "parkedVehicle")
            shape = _el(obs, "shape")
            rect = _el(shape, "rectangle")
            _el(rect, "length", 4.5)
            _el(rect, "width", 1.8)
        else:
            obs = _el(root, "dynamicObstacle", id=200 + i)
            _el(obs, "type", "pedestrian" if pedestrian else "car")
            shape = _el(obs, "shape")
            if pedestrian:
                _el(_el(shape, "circle"), "radius", 0.35)
            else:
                rect = _el(shape, "rectangle")
                _el(rect, "length", 4.5)
                _el(rect, "width", 1.8)

        state = _el(obs, "initialState")
        _point(_el(state, "position"), "point", init["x"], init["y"])
        _exact(state, "orientation", yaw)
        _el(_el(state, "time"), "exact", 0)
        _exact(state, "velocity", speed, 3)

        if moving:
            trajectory = _el(obs, "trajectory")
            for wp in rolled_out(actor)[1:]:
                st = _el(trajectory, "state")
                _point(_el(st, "position"), "point", wp["x"], wp["y"])
                _exact(st, "orientation", wp["yaw"])
                _el(_el(st, "time"), "exact", round(wp["t"] / DT))
                _exact(st, "velocity", wp["speed"], 3)


def add_planning_problem(root, scenario, lane_half):
    ego = scenario["ego"]
    center = scenario["centerline"]
    problem = _el(root, "planningProblem", id=900)
    init = _el(problem, "initialState")
    _point(_el(init, "position"), "point", ego["x"], ego["y"])
    _exact(init, "velocity", ego["speed"], 3)
    _exact(init, "orientation", ego.get("yaw", 0.0))
    _exact(init, "yawRate", 0.0)
    _exact(init, "slipAngle", 0.0)
    _el(_el(init, "time"), "exact", 0)

    goal = _el(problem, "goalState")
    _interval(goal, "time", 0, N_TICKS, 0)
    pos = _el(goal, "position")
    rect = _el(pos, "rectangle")
    _el(rect, "length", 20.0)
    _el(rect, "width", round(min(2 * lane_half, 8.0), 2))
    _el(rect, "orientation", round(headings(center)[-1], 4))
    _point(rect, "center", *center[-1])
    ts = scenario["target_speed"]
    _interval(goal, "velocity", max(0.0, ts - 1.0), ts + 1.0, 2)


def scenario_tags(scenario, category):
    m = scenario["map"]
    tags = {"simulated", "highway" if m["road_half_width"] >= 6.5 else "urban"}
    tags.add("multi_lane" if m["divider_d"] is not None else "single_lane")
    if m["divider_d"] is not None:
        tags.add("parallel_lanes")
    if m["cross_streets"]:
        tags.add("intersection")
    if any(_is_opposite(a) for a in scenario["actors"]):
        tags.add("oncoming_traffic")
    else:
        tags.add("no_oncoming_traffic")
    if "cut_in" in category or "merging" in category:
        tags.add("cut_in")
    if "lead" in category or "stop_and_go" in category:
        tags.add("lane_following")
    if "stopping" in category:
        tags.add("emergency_braking")
    return [t for t in TAG_ORDER if t in tags]


def to_commonroad_xml(scenario, benchmark_id, category):
    root = ET.Element(
        "commonRoad",
        commonRoadVersion="2020a",
        benchmarkID=benchmark_id,
        date=FILE_DATE,
        author="nanoplan",
        affiliation="nanoplan (https://github.com/BenGravell/nanoplan)",
        source="synthetic (tools/generate_diverse_scenarios.py)",
        timeStepSize=str(DT),
    )
    location = _el(root, "location")
    _el(location, "geoNameId", -999)
    _el(location, "gpsLatitude", 999)
    _el(location, "gpsLongitude", 999)
    if "environment" in scenario:
        env = _el(location, "environment")
        for tag in ("time", "timeOfDay", "weather", "underground"):
            _el(env, tag, scenario["environment"][tag])
    tags = _el(root, "scenarioTags")
    for t in scenario_tags(scenario, category):
        _el(tags, t)
    lane_half = add_lanelets(root, scenario)
    add_obstacles(root, scenario)
    add_planning_problem(root, scenario, lane_half)
    tree = ET.ElementTree(root)
    ET.indent(tree, space=" ")
    return tree


def camel(category):
    return "".join(part.capitalize() for part in category.split("_"))


def generate(variations, seed):
    """(filename, ElementTree) for every classic and category variation."""
    out = []
    for name, classic_fn in CLASSICS:
        benchmark_id = f"ZAM_{name}-1_1_T-1"
        out.append((benchmark_id, to_commonroad_xml(classic_fn(), benchmark_id, name.lower())))
    for gen_fn in CATEGORIES:
        category = gen_fn.__name__.removeprefix("gen_")
        for i in range(variations):
            rng = random.Random(f"{seed}:{category}:{i}")
            scenario = apply_condition(rng, gen_fn(rng))
            benchmark_id = f"ZAM_{camel(category)}-1_{i + 1}_T-1"
            out.append((benchmark_id, to_commonroad_xml(scenario, benchmark_id, category)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="scenarios/commonroad", help="output directory (default: scenarios/commonroad)")
    ap.add_argument("--variations", type=int, default=1, help="variations per category (default: 1)")
    ap.add_argument("--seed", type=int, default=1, help="RNG seed (default: 1)")
    ap.add_argument("--clean", action="store_true", help="remove the output directory's *.xml files first")
    args = ap.parse_args()

    out = Path(args.out)
    if args.clean and out.exists():
        for old in out.glob("*.xml"):
            old.unlink()
    out.mkdir(parents=True, exist_ok=True)

    scenarios = generate(args.variations, args.seed)
    for benchmark_id, tree in scenarios:
        tree.write(out / f"{benchmark_id}.xml", encoding="UTF-8", xml_declaration=True)

    print(
        f"wrote {len(scenarios)} scenarios ({len(CLASSICS)} classics + "
        f"{len(CATEGORIES)} categories x {args.variations} variations) to {out}"
    )


if __name__ == "__main__":
    main()
