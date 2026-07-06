#!/usr/bin/env python3
"""Combine scenario JSON files into one compact bundle for the web viewer.

The web build has no filesystem, so it can't browse a directory the way the
desktop viewer does (see scenarios::load_path / the "scenario path" widget in
main.rs). Instead it fetches one static file at startup — this script builds
that file: every *.json scenario in a directory, combined into a single
minified JSON array. One HTTP request instead of N, and no per-file
whitespace/formatting overhead.

Usage:
  python3 tools/bundle_web_scenarios.py [SRC_DIR] [OUT_FILE]

Defaults: SRC_DIR=scenarios/web, OUT_FILE=scenarios/web_bundle.json

SRC_DIR is deliberately not scenarios/json/ (those two files are already
compiled into every build, native and wasm, via include_str! in main.rs —
bundling them again here would just show duplicate entries in the viewer's
dropdown). scenarios/web/ is the staging directory for whatever the *web
build only* should fetch at runtime — by default the converted CommonRoad
corpus this repo ships. Do NOT stage nuPlan exports for a deployed bundle:
the nuPlan dataset license doesn't permit redistribution (local desktop use
only).

The checked-in scenarios/web_bundle.json is produced by:
  python3 tools/generate_diverse_scenarios.py --clean       # scenarios/commonroad/*.xml
  python3 tools/export_commonroad_scenarios.py scenarios/commonroad scenarios/web
  rm scenarios/web/ZAM_BrakingLead-1_1_T-1.json scenarios/web/ZAM_CutIn-1_1_T-1.json
  python3 tools/bundle_web_scenarios.py
  trunk build --release --public-url /nanoplan/   # copies web_bundle.json into dist/
(the two rm'd scenarios are the ones already compiled in via scenarios/json/)
"""

import argparse
import json
import sys
from pathlib import Path


def bundle(src_dir, out_file):
    src = Path(src_dir)
    paths = sorted(p for p in src.glob("*.json") if p.resolve() != Path(out_file).resolve()) if src.is_dir() else []
    scenarios = []
    for p in paths:
        with open(p) as f:
            scenarios.append(json.load(f))
    with open(out_file, "w") as f:
        json.dump(scenarios, f, separators=(",", ":"))
    return len(scenarios), Path(out_file).stat().st_size


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src_dir", nargs="?", default="scenarios/web")
    ap.add_argument("out_file", nargs="?", default="scenarios/web_bundle.json")
    args = ap.parse_args()

    default_src = ap.get_default("src_dir")
    if not Path(args.src_dir).is_dir() and args.src_dir != default_src:
        sys.exit(f"no such directory: {args.src_dir}")
    count, size = bundle(args.src_dir, args.out_file)
    print(f"wrote {count} scenario(s), {size} bytes, to {args.out_file}")


if __name__ == "__main__":
    main()
