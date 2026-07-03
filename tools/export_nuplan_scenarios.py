#!/usr/bin/env python3
"""Export scenarios from a nuPlan log database into nanoplan JSON.

Reads a nuPlan sqlite log (see scenarios/nuplan/nuplan_schema.md) with only
the Python standard library — no nuplan-devkit install needed. For every
tagged scenario in the log it writes one JSON file that nanoplan's batch
runner and viewer can load:

  - centerline: the expert ego route over the horizon (the map itself is not
    in the log database, so the driven route stands in as the lane reference)
  - ego: the ego pose and speed at the scenario's anchor frame
  - actors: vehicles present at the anchor frame, each with its logged
    trajectory over the horizon (replayed in simulation; nanoplan
    extrapolates constant velocity past the log's end)
  - target_speed: the expert's 85th-percentile speed over the horizon

Usage:
  python3 tools/export_nuplan_scenarios.py LOG.db OUT_DIR \
      [--horizon 20] [--max 100] [--types stopping_with_lead,...]

Then run the batch:
  cargo run --release --bin batch -- --count 0 --dir OUT_DIR
"""

import argparse
import json
import math
import sqlite3
import statistics
from pathlib import Path


def quaternion_yaw(qw, qx, qy, qz):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def export(db_path, out_dir, horizon_s, max_scenarios, types):
    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    tags = db.execute(
        """SELECT st.type AS type, hex(st.lidar_pc_token) AS token,
                  lp.timestamp AS t0
           FROM scenario_tag st JOIN lidar_pc lp ON st.lidar_pc_token = lp.token
           ORDER BY lp.timestamp"""
    ).fetchall()
    if types:
        tags = [t for t in tags if t["type"] in types]

    written = 0
    for tag in tags:
        if written >= max_scenarios:
            break
        t0, t1 = tag["t0"], tag["t0"] + int(horizon_s * 1e6)
        ego_rows = db.execute(
            """SELECT ep.x, ep.y, ep.qw, ep.qx, ep.qy, ep.qz, ep.vx, ep.vy
               FROM lidar_pc lp JOIN ego_pose ep ON lp.ego_pose_token = ep.token
               WHERE lp.timestamp BETWEEN ? AND ? ORDER BY lp.timestamp""",
            (t0, t1),
        ).fetchall()
        if len(ego_rows) < 10:
            continue

        # expert route, downsampled to ~2 m spacing, as the lane reference
        centerline = []
        for r in ego_rows:
            p = [r["x"], r["y"]]
            if not centerline or math.dist(p, centerline[-1]) >= 2.0:
                centerline.append(p)
        if len(centerline) < 2:
            continue  # ego is parked; no route to follow

        # translate everything to the scenario's local origin: nuPlan uses
        # global map coordinates (hundreds of km), which break f32 rendering
        first = ego_rows[0]
        ox, oy = first["x"], first["y"]
        centerline = [[round(x - ox, 3), round(y - oy, 3)] for x, y in centerline]
        ego = {
            "x": 0.0,
            "y": 0.0,
            "yaw": quaternion_yaw(first["qw"], first["qx"], first["qy"], first["qz"]),
            "speed": math.hypot(first["vx"], first["vy"]),
        }

        # vehicles present at the anchor frame, with their logged trajectories
        track_tokens = db.execute(
            """SELECT hex(lb.track_token) AS tt
               FROM lidar_box lb
               JOIN track tr ON lb.track_token = tr.token
               JOIN category c ON tr.category_token = c.token
               WHERE lb.lidar_pc_token = (SELECT lidar_pc_token FROM scenario_tag
                                          WHERE hex(lidar_pc_token) = ? LIMIT 1)
                 AND c.name = 'vehicle'""",
            (tag["token"],),
        ).fetchall()
        actors = []
        for row in track_tokens:
            boxes = db.execute(
                """SELECT lp.timestamp AS ts, lb.x, lb.y, lb.yaw, lb.vx, lb.vy
                   FROM lidar_box lb JOIN lidar_pc lp ON lb.lidar_pc_token = lp.token
                   WHERE hex(lb.track_token) = ? AND lp.timestamp BETWEEN ? AND ?
                   ORDER BY lp.timestamp""",
                (row["tt"], t0, t1),
            ).fetchall()
            if not boxes:
                continue
            trajectory = []
            for b in boxes:
                t = (b["ts"] - t0) / 1e6
                # downsample to the 10 Hz simulation rate
                if trajectory and t - trajectory[-1]["t"] < 0.099:
                    continue
                trajectory.append(
                    {
                        "t": round(t, 3),
                        "x": round(b["x"] - ox, 3),
                        "y": round(b["y"] - oy, 3),
                        "yaw": b["yaw"],
                        "speed": math.hypot(b["vx"], b["vy"]),
                    }
                )
            init = {k: trajectory[0][k] for k in ("x", "y", "yaw", "speed")}
            actors.append({"init": init, "trajectory": trajectory})

        speeds = sorted(math.hypot(r["vx"], r["vy"]) for r in ego_rows)
        target_speed = max(3.0, statistics.quantiles(speeds, n=20)[16])  # p85

        scenario = {
            "name": f"{tag['type']}_{tag['token'][:8].lower()}",
            "ego": ego,
            "actors": actors,
            "centerline": centerline,
            "target_speed": round(target_speed, 2),
        }
        path = out / f"{scenario['name']}.json"
        path.write_text(json.dumps(scenario, indent=1))
        written += 1

    print(f"wrote {written} scenarios to {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("db", help="nuPlan log database (.db sqlite file)")
    ap.add_argument("out", help="output directory for scenario JSON files")
    ap.add_argument("--horizon", type=float, default=20.0, help="seconds per scenario")
    ap.add_argument("--max", type=int, default=100, help="max scenarios to export")
    ap.add_argument("--types", default="", help="comma-separated scenario types to keep")
    args = ap.parse_args()
    export(
        args.db,
        args.out,
        args.horizon,
        args.max,
        {t for t in args.types.split(",") if t},
    )
