#!/usr/bin/env python3
"""Apply Clippy fixes, but never cache a run that changed the working tree."""

import os
from pathlib import Path
import subprocess
import sys


def snapshot():
    # Include ignored/generated sources too, without scanning Cargo's target directory.
    paths = subprocess.check_output([
        "git", "ls-files", "--cached", "--others", "-z", "--",
        "Cargo.toml", "Cargo.lock", "build.rs", "src", "tests", "examples", "benches", "tools",
    ]).split(b"\0")
    return {
        name: path.read_bytes() if path.is_file() else None
        for name in set(paths) - {b""}
        for path in [Path(os.fsdecode(name))]
    }


if __name__ == "__main__":
    before = snapshot()
    result = subprocess.run(sys.argv[1:])
    if result.returncode:
        raise SystemExit(result.returncode)
    if snapshot() != before:
        # mise only caches successful runs. Reintroduced warnings must be fixed again.
        print("Clippy changed files; review and stage the fixes, then retry.", flush=True)
        raise SystemExit(1)
