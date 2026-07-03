#!/usr/bin/env python3
"""Profile `cargo build` and report the slowest crates.

A thin wrapper around cargo's built-in `--timings` report: runs the build,
finds the HTML report cargo just wrote, and prints the units that took
longest to compile plus overall wall time and parallelism. Useful for
answering "why is this build slow" without opening the HTML report by hand.

Usage:
  python3 tools/build_timings.py                 # incremental build, bin target
  python3 tools/build_timings.py --clean         # cargo clean first: a true
                                                  # from-scratch baseline
  python3 tools/build_timings.py --top 30 -- --release --bin batch
                                                  # extra args go to cargo build
"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPORT_DIR = Path("target/cargo-timings")


def run_build(extra_args):
    before = set(REPORT_DIR.glob("cargo-timing-*.html")) if REPORT_DIR.is_dir() else set()
    subprocess.run(["cargo", "build", "--timings", *extra_args], check=True)
    after = set(REPORT_DIR.glob("cargo-timing-*.html"))
    new = after - before
    if not new:
        sys.exit(f"no new report appeared in {REPORT_DIR}")
    return max(new, key=lambda p: p.stat().st_mtime)


def unit_data(report):
    text = report.read_text()
    m = re.search(r"const UNIT_DATA = (\[.*?\]);\n", text, re.S)
    if not m:
        sys.exit(f"couldn't find UNIT_DATA in {report}")
    return json.loads(m.group(1))


def summarize(units, top):
    total_wall = max(u["start"] + u["duration"] for u in units)
    total_cpu = sum(u["duration"] for u in units)
    print(f"wall time:   {total_wall:7.1f}s")
    print(f"cpu time:    {total_cpu:7.1f}s  ({total_cpu / total_wall:.1f}x parallelism)")
    print(f"units built: {len(units)}")
    print()
    print(f"slowest {top} units:")
    print(f"{'crate':30s} {'dur (s)':>8s} {'ends at (s)':>12s}")
    for u in sorted(units, key=lambda u: -u["duration"])[:top]:
        print(f"{u['name'][:30]:30s} {u['duration']:8.2f} {u['start'] + u['duration']:12.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clean", action="store_true", help="cargo clean first, for a from-scratch baseline")
    ap.add_argument("--top", type=int, default=20, help="how many slowest units to list (default: 20)")
    ap.add_argument("cargo_args", nargs=argparse.REMAINDER, help="extra args, after --, forwarded to `cargo build`")
    args = ap.parse_args()

    extra = args.cargo_args[1:] if args.cargo_args[:1] == ["--"] else args.cargo_args
    if args.clean:
        subprocess.run(["cargo", "clean"], check=True)

    report = run_build(extra)
    print(f"report: {report}\n")
    summarize(unit_data(report), args.top)


if __name__ == "__main__":
    main()
