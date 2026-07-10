#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["rich"]
# ///
"""Cache the clearly TUM-authored CommonRoad scenarios and nanoplan JSON.

The final stdout line is the converted directory, so it composes with:

  cargo run -- "$(uv run tools/load_commonroad_scenarios.py)"
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    from rich.console import Console
    from rich.progress import track
    from rich.table import Table
except ModuleNotFoundError:
    sys.exit("Rich is required; run `uv run tools/load_commonroad_scenarios.py`")

from export_commonroad_scenarios import convert

UPSTREAM = "https://gitlab.lrz.de/tum-cps/commonroad-scenarios.git"
COMMIT = "2416dc27faf3ee36b6799f42712da72a34a2c8fd"
SCENARIO_DIR = Path("scenarios/recorded/hand-crafted")
OPEN_SOURCES = {"hand-crafted", "Specification-based scenario synthesis"}
OPEN_AFFILIATIONS = {
    "Technical University of Munich",
    "Technical University of Munich, Germany",
}
EXPECTED_SCENARIOS = 619


def default_cache_dir():
    override = os.environ.get("NANOPLAN_CACHE_DIR")
    if override:
        return Path(override).expanduser()
    if sys.platform == "darwin":
        return Path.home() / "Library/Caches/nanoplan/commonroad"
    if os.name == "nt":
        return Path(os.environ.get("LOCALAPPDATA", Path.home())) / "nanoplan/commonroad"
    return (
        Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
        / "nanoplan/commonroad"
    )


def git(*args):
    result = subprocess.run(
        ["git", *map(str, args)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout.strip()


def checkout(cache_dir):
    repo = cache_dir / COMMIT[:12] / "upstream"
    if repo.exists():
        if git("-C", repo, "rev-parse", "HEAD") != COMMIT:
            raise RuntimeError(f"unexpected checkout in {repo}; remove it and retry")
        if git("-C", repo, "status", "--porcelain", "--untracked-files=no"):
            raise RuntimeError(f"modified checkout in {repo}; remove it and retry")
        return repo, True

    tmp = repo.with_name("upstream.tmp")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.parent.mkdir(parents=True, exist_ok=True)
    try:
        git("init", "--quiet", tmp)
        git("-C", tmp, "remote", "add", "origin", UPSTREAM)
        git("-C", tmp, "sparse-checkout", "init", "--no-cone")
        git("-C", tmp, "sparse-checkout", "set", "LICENSE.txt", SCENARIO_DIR)
        git("-C", tmp, "fetch", "--depth", "1", "--filter=blob:none", "origin", COMMIT)
        git("-C", tmp, "checkout", "--quiet", "--detach", "FETCH_HEAD")
        tmp.replace(repo)
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    return repo, False


def selected_scenarios(directory):
    selected = []
    for path in sorted(directory.glob("*.xml")):
        _, root = next(ET.iterparse(path, events=("start",)))
        if (
            root.get("source") in OPEN_SOURCES
            and root.get("affiliation") in OPEN_AFFILIATIONS
        ):
            selected.append(path)
    return selected


def convert_one(source, target, converter=convert):
    if target.exists():
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(converter(source), indent=1))
    temporary.replace(target)
    return True


def load(cache_dir, console):
    with console.status("[bold cyan]Checking CommonRoad source cache…"):
        repo, source_hit = checkout(cache_dir)

    paths = selected_scenarios(repo / SCENARIO_DIR)
    if len(paths) != EXPECTED_SCENARIOS:
        raise RuntimeError(
            f"expected {EXPECTED_SCENARIOS} open scenarios, found {len(paths)}"
        )
    converter_file = Path(__file__).with_name("export_commonroad_scenarios.py")
    selection = "\0".join(sorted(OPEN_SOURCES | OPEN_AFFILIATIONS)).encode()
    converter_key = hashlib.sha256(converter_file.read_bytes() + selection).hexdigest()[
        :12
    ]
    output = cache_dir / COMMIT[:12] / f"json-{converter_key}"
    missing = [path for path in paths if not (output / f"{path.stem}.json").exists()]
    if missing:
        for path in track(missing, description="Converting scenarios", console=console):
            convert_one(path, output / f"{path.stem}.json")
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(repo / "LICENSE.txt", output / "LICENSE.txt")

    table = Table(title="CommonRoad cache", show_header=False)
    table.add_row("Source", "cached" if source_hit else "downloaded")
    table.add_row("Selected", str(len(paths)))
    table.add_row(
        "Converted", f"{len(missing)} new, {len(paths) - len(missing)} cached"
    )
    table.add_row("Directory", str(output))
    console.print(table)
    return output


def self_test():
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        source = root / "source"
        source.mkdir()
        (source / "open.xml").write_text(
            '<commonRoad source="hand-crafted" affiliation="Technical University of Munich"/>'
        )
        (source / "restricted.xml").write_text(
            '<commonRoad source="Bing Maps" affiliation="Technical University of Munich"/>'
        )
        assert [path.name for path in selected_scenarios(source)] == ["open.xml"]
        calls = []
        target = root / "out/open.json"

        def fake(path):
            calls.append(path)
            return {"name": path.stem}

        assert convert_one(source / "open.xml", target, fake)
        assert not convert_one(source / "open.xml", target, fake)
        assert len(calls) == 1 and json.loads(target.read_text()) == {"name": "open"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", type=Path, default=default_cache_dir())
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("self-test passed")
        return

    console = Console(stderr=True)
    try:
        output = load(args.cache_dir.expanduser(), console)
    except (OSError, RuntimeError, ET.ParseError, ValueError) as error:
        console.print(f"[bold red]CommonRoad cache failed:[/] {error}")
        raise SystemExit(1)
    print(output)


if __name__ == "__main__":
    main()
