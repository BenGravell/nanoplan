#!/usr/bin/env python3
"""Export CommonRoad scenarios into nanoplan JSON.

Reads CommonRoad 2020a XML files (https://commonroad.in.tum.de) with only the
Python standard library — no commonroad-io install needed. The XML format
does not determine a scenario's redistribution rights; see the separate
sources and licenses in scenarios/commonroad/README.md. For every input
scenario it writes one JSON file that nanoplan's batch runner and viewer can
load:

  - centerline: the route through the lanelet network, from the lanelet
    containing the planning problem's initial state to the goal, following
    successors (falling back to the longest successor chain when the goal
    isn't position-based or isn't reachable)
  - ego: the planning problem's initial state
  - actors: every static and dynamic obstacle; dynamic ones carry their
    logged state trajectory (replayed in simulation; nanoplan extrapolates
    constant velocity past the trajectory's end)
  - target_speed: the midpoint of the goal state's velocity interval, else
    the ego's initial speed
  - map: road half-width and lane divider derived from the route lanelets'
    bounds and adjacencies; crosswalk-type lanelets and other roads crossing
    the route become the crosswalk/cross-street markers the viewer draws

CommonRoad scenarios carry no expert ego trajectory, so no `expert` field is
written — expert demonstrations for the cost-weight autotuner come from
locally exported nuPlan logs instead (tools/export_nuplan_scenarios.py).

Usage:
  python3 tools/export_commonroad_scenarios.py SRC OUT_DIR

SRC is a single CommonRoad *.xml file or a directory of them (non-recursive),
e.g. scenarios/commonroad/, or scenarios downloaded from commonroad.in.tum.de.

Then run the batch:
  cargo run --release --bin batch -- --count 0 --dir OUT_DIR
"""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# lanelet types that are never part of the ego's driving route
NON_ROUTE_TYPES = {"crosswalk", "sidewalk", "border", "restricted", "restricted_area"}


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _points(elem):
    return [
        [float(p.findtext("x")), float(p.findtext("y"))]
        for p in elem.findall("point")
    ]


def _scalar(elem, default=None):
    """A CommonRoad exact-or-interval value, collapsed to one number."""
    if elem is None:
        return default
    exact = elem.findtext("exact")
    if exact is not None:
        return float(exact)
    lo, hi = elem.findtext("intervalStart"), elem.findtext("intervalEnd")
    if lo is None or hi is None:
        return default
    return (float(lo) + float(hi)) / 2.0


def _project(polyline, p):
    """(arc length, signed lateral offset) of p onto a polyline —
    the same Frenet projection as src/scenarios/mod.rs::Path::project
    (positive offset is left of the path)."""
    best = (0.0, math.inf)
    s0 = 0.0
    for a, b in zip(polyline, polyline[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        len2 = max(dx * dx + dy * dy, 1e-12)
        u = min(max(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2, 0.0), 1.0)
        q = [a[0] + dx * u, a[1] + dy * u]
        d = _dist(p, q)
        if d < abs(best[1]):
            cross = dx * (p[1] - q[1]) - dy * (p[0] - q[0])
            best = (s0 + math.sqrt(len2) * u, math.copysign(d, cross))
        s0 += math.sqrt(len2)
    return best


def _heading(polyline, i):
    a, b = polyline[max(0, i - 1)], polyline[min(len(polyline) - 1, i)]
    if a == b and len(polyline) > 1:
        a, b = polyline[0], polyline[1]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _wrap(a):
    return (a + math.pi) % math.tau - math.pi


class Lanelet:
    def __init__(self, elem):
        self.id = int(elem.get("id"))
        self.left = _points(elem.find("leftBound"))
        self.right = _points(elem.find("rightBound"))
        n = min(len(self.left), len(self.right))
        self.center = [
            [(l[0] + r[0]) / 2.0, (l[1] + r[1]) / 2.0]
            for l, r in zip(self.left[:n], self.right[:n])
        ]
        self.successors = [int(s.get("ref")) for s in elem.findall("successor")]
        adj_l, adj_r = elem.find("adjacentLeft"), elem.find("adjacentRight")
        self.adjacent = [int(a.get("ref")) for a in (adj_l, adj_r) if a is not None]
        self.adj_left = int(adj_l.get("ref")) if adj_l is not None else None
        self.adj_right = int(adj_r.get("ref")) if adj_r is not None else None
        self.types = {t.text for t in elem.findall("laneletType")}

    def contains(self, p):
        """Point-in-polygon over leftBound + reversed rightBound (ray cast)."""
        ring = self.left + self.right[::-1]
        inside = False
        for a, b in zip(ring, ring[1:] + ring[:1]):
            if (a[1] > p[1]) != (b[1] > p[1]):
                x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
                if p[0] < x:
                    inside = not inside
        return inside

    def width(self):
        i = len(self.center) // 2
        return _dist(self.left[min(i, len(self.left) - 1)], self.right[min(i, len(self.right) - 1)])


def route_lanelets(lanelets, ego_xy, ego_yaw, goal_xy):
    """BFS over successor edges from the lanelet under the ego to the one
    under the goal; without a (reachable) goal, the deepest chain wins."""
    drivable = {l.id: l for l in lanelets.values() if not (l.types & NON_ROUTE_TYPES)}

    def containing(p, yaw=None):
        hits = [l for l in drivable.values() if l.contains(p)]
        if yaw is not None:
            aligned = [
                l for l in hits
                if abs(_wrap(_heading(l.center, 1) - yaw)) < math.pi / 2
            ]
            hits = aligned or hits
        if hits:
            return hits[0]
        # fall back to the nearest centerline
        return min(
            drivable.values(),
            key=lambda l: min(_dist(p, c) for c in l.center),
            default=None,
        )

    start = containing(ego_xy, ego_yaw)
    if start is None:
        return []
    goal = containing(goal_xy) if goal_xy else None
    parent, order = {start.id: None}, [start.id]
    for lid in order:  # BFS; `order` grows while iterating
        if goal is not None and lid == goal.id:
            break
        for nxt in drivable[lid].successors:
            if nxt in drivable and nxt not in parent:
                parent[nxt] = lid
                order.append(nxt)
    end = goal.id if goal is not None and goal.id in parent else order[-1]
    path = []
    while end is not None:
        path.append(drivable[end])
        end = parent[end]
    return path[::-1]


def map_data(lanelets, route, centerline):
    """MapData (road half-width, divider, crosswalks, cross streets) from
    the lanelet network, relative to the route centerline."""
    route_ids = {l.id for l in route}
    adjacent_ids = {a for l in route for a in l.adjacent}

    half_widths, dividers = [], []
    for l in route:
        i = len(l.center) // 2
        c = l.center[i]
        own_left = min(_dist(c, p) for p in l.left)
        own_right = min(_dist(c, p) for p in l.right)
        left_ext = own_left + (
            lanelets[l.adj_left].width() if l.adj_left in lanelets else 0.0
        )
        right_ext = own_right + (
            lanelets[l.adj_right].width() if l.adj_right in lanelets else 0.0
        )
        half_widths.append(max(left_ext, right_ext))
        if l.adj_left is not None:
            dividers.append(own_left)
        elif l.adj_right is not None:
            dividers.append(-own_right)
    half_width = round(sorted(half_widths)[len(half_widths) // 2], 2) if half_widths else 5.5
    divider = round(sorted(dividers)[len(dividers) // 2], 2) if dividers else None

    crosswalk_s, cross_streets = [], []
    for l in lanelets.values():
        if l.id in route_ids or l.id in adjacent_ids:
            continue
        mid = l.center[len(l.center) // 2]
        s, d = _project(centerline, mid)
        if "crosswalk" in l.types:
            if abs(d) < half_width + 3.0:
                crosswalk_s.append(round(s, 1))
        elif not (l.types & NON_ROUTE_TYPES):
            # another road crossing ours: its centerline passes through the
            # route at a steep angle — drawn by the viewer so crossing
            # traffic has a visible road to be on
            projections = [(_project(centerline, c), i) for i, c in enumerate(l.center)]
            (s, d), i = min(projections, key=lambda t: abs(t[0][1]))
            if abs(d) < half_width:
                rel = abs(_wrap(_heading(l.center, max(i, 1)) - _heading(centerline, 1)))
                if math.pi / 4 < rel < 3 * math.pi / 4:
                    cross_streets.append(round(s, 1))
    return {
        "road_half_width": half_width,
        "divider_d": divider,
        "crosswalk_s": sorted(set(crosswalk_s)),
        "cross_streets": sorted(set(cross_streets)),
    }


def state_waypoint(state, dt):
    t = float(state.findtext("time/exact", "0")) * dt
    pos = state.find("position/point")
    return {
        "t": round(t, 3),
        "x": round(float(pos.findtext("x")), 3),
        "y": round(float(pos.findtext("y")), 3),
        "yaw": round(_scalar(state.find("orientation"), 0.0), 4),
        "speed": round(_scalar(state.find("velocity"), 0.0), 3),
    }


def actors(root, dt):
    out = []
    for obs in root.findall("staticObstacle"):
        wp = state_waypoint(obs.find("initialState"), dt)
        out.append({"init": {k: wp[k] for k in ("x", "y", "yaw", "speed")}})
    for obs in root.findall("dynamicObstacle"):
        trajectory = [state_waypoint(obs.find("initialState"), dt)]
        for st in obs.findall("trajectory/state"):
            trajectory.append(state_waypoint(st, dt))
        init = {k: trajectory[0][k] for k in ("x", "y", "yaw", "speed")}
        actor = {"init": init}
        if len(trajectory) > 1:
            actor["trajectory"] = trajectory
        out.append(actor)
    return out


def goal_center(goal_state):
    pos = goal_state.find("position")
    if pos is None:
        return None
    for shape in ("rectangle", "circle"):
        center = pos.find(f"{shape}/center")
        if center is not None:
            return [float(center.findtext("x")), float(center.findtext("y"))]
    point = pos.find("point")
    if point is not None:
        return [float(point.findtext("x")), float(point.findtext("y"))]
    return None  # lanelet-ref goals are resolved by the caller

def goal_lanelet_ref(goal_state):
    ref = goal_state.find("position/lanelet")
    return int(ref.get("ref")) if ref is not None else None


def condition_suffix(root):
    """A `[fog]`-style display suffix when the scenario declares non-default
    environment conditions (CommonRoad location/environment)."""
    env = root.find("location/environment")
    if env is None:
        return ""
    parts = []
    weather = env.findtext("weather")
    if weather and weather != "sunny":
        parts.append(weather)
    if env.findtext("timeOfDay") == "night":
        parts.append("night")
    return f" [{'_'.join(parts)}]" if parts else ""


def convert(xml_path):
    root = ET.parse(xml_path).getroot()
    if root.tag != "commonRoad":
        raise ValueError(f"{xml_path}: not a CommonRoad scenario (root <{root.tag}>)")
    dt = float(root.get("timeStepSize", "0.1"))
    lanelets = {l.id: l for l in map(Lanelet, root.findall("lanelet"))}
    problem = root.find("planningProblem")
    if problem is None or not lanelets:
        raise ValueError(f"{xml_path}: no planning problem or no lanelets")

    init = problem.find("initialState")
    pos = init.find("position/point")
    ego = {
        "x": round(float(pos.findtext("x")), 3),
        "y": round(float(pos.findtext("y")), 3),
        "yaw": round(_scalar(init.find("orientation"), 0.0), 4),
        "speed": round(_scalar(init.find("velocity"), 0.0), 3),
    }

    goal = problem.find("goalState")
    goal_xy = goal_center(goal) if goal is not None else None
    if goal_xy is None and goal is not None:
        ref = goal_lanelet_ref(goal)
        if ref in lanelets:
            goal_xy = lanelets[ref].center[-1]

    route = route_lanelets(lanelets, [ego["x"], ego["y"]], ego["yaw"], goal_xy)
    if not route:
        raise ValueError(f"{xml_path}: no drivable lanelet under the ego")
    centerline = []
    for l in route:
        for p in l.center:
            if not centerline or _dist(p, centerline[-1]) > 1e-6:
                centerline.append(p)

    speed = _scalar(goal.find("velocity")) if goal is not None else None
    if speed is None:
        speed = ego["speed"] if ego["speed"] > 3.0 else 10.0

    return {
        "name": root.get("benchmarkID", Path(xml_path).stem) + condition_suffix(root),
        "ego": ego,
        "actors": actors(root, dt),
        "centerline": [[round(x, 3), round(y, 3)] for x, y in centerline],
        "target_speed": round(speed, 2),
        "map": map_data(lanelets, route, centerline),
    }


def export(src, out_dir):
    src = Path(src)
    paths = sorted(src.glob("*.xml")) if src.is_dir() else [src]
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written = 0
    for path in paths:
        scenario = convert(path)
        (out / f"{path.stem}.json").write_text(json.dumps(scenario, indent=1))
        written += 1
    print(f"wrote {written} scenario(s) to {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("src", help="CommonRoad *.xml file, or a directory of them")
    ap.add_argument("out", help="output directory for scenario JSON files")
    args = ap.parse_args()
    export(args.src, args.out)
