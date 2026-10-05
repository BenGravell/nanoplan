#!/usr/bin/env python3
"""Exercise hook caching and automatic fixes: python3 tools/test_cargo_hooks.py.

Uses temporary projects, the installed mise/pre-commit, and a tiny Rust crate.
Set MISE_BIN to an absolute executable path if mise's launcher needs bypassing.
"""

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import tomllib
import unittest


ROOT = Path(__file__).resolve().parents[1]
MISE = os.environ.get("MISE_BIN", shutil.which("mise"))
FLAGS = ["--all-targets", "--all-features", "--", "-D", "warnings"]


class HookTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="nanoplan-hooks-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.env = dict(os.environ, MISE_CACHE_DIR=str(self.root / "cache"),
                        MISE_STATE_DIR=str(self.root / "state"),
                        MISE_TRUSTED_CONFIG_PATHS=str(self.root),
                        MISE_TASK_CACHE="read-write", MISE_TASK_RUN_AUTO_INSTALL="false",
                        PRE_COMMIT_HOME=str(self.root / "pre-commit-cache"))

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def run_command(self, *args, success=True, env=None):
        result = subprocess.run(args, cwd=self.root, env=env or self.env,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(result.returncode == 0, success, result.stdout)
        return result.stdout

    def cached_project(self):
        self.mise_project(test_command="python3 fake_test.py")
        self.write("fake_test.py", '''
from pathlib import Path
with Path("runs").open("a") as log:
    log.write("run\\n")
if Path("src/lib.rs").read_text() == "fail":
    raise SystemExit(1)
Path("viewer-renders").mkdir(exist_ok=True)
Path("viewer-renders/result.png").write_text("rendered")
''')
        self.write("src/lib.rs", "ok")
        self.write("src/required.rs", "required")

    def mise_project(self, test_command=None):
        text = (ROOT / "mise.toml").read_text()
        # Exercise the real declarations without installing unrelated developer tools.
        text = re.sub(r"(?ms)^\[tools\]\n.*?(?=^\[)", "", text)
        if test_command:
            run = tomllib.loads(text)["tasks"]["hook-test"]["run"]
            text = text.replace(json.dumps(run), json.dumps(test_command), 1)
        self.write("mise.toml", text)
        self.write("tools/cargo_hook_context.py", (ROOT / "tools/cargo_hook_context.py").read_text())
        self.write("tools/clippy_fix.py", (ROOT / "tools/clippy_fix.py").read_text())
        self.write(".gitignore", "target/\ncache/\nstate/\npre-commit-cache/\nviewer-renders/\nruns\nbin/\ncargo-home/\n")

    def task(self, *args, **kwargs):
        return self.run_command(MISE, "run", *args, "hook-test", **kwargs)

    def runs(self):
        return len((self.root / "runs").read_text().splitlines())

    def test_content_cache_invalidation_and_output_restoration(self):
        self.cached_project()
        self.task()
        self.task()
        self.write("src/README.md", "Documentation only")
        os.utime(self.root / "src/lib.rs", None)
        self.task()
        self.assertEqual(self.runs(), 1, "unchanged content and docs should reuse results")
        shutil.rmtree(self.root / "viewer-renders")
        self.task()
        self.assertEqual(self.runs(), 1)
        self.assertEqual((self.root / "viewer-renders/result.png").read_text(), "rendered")

        source = self.root / "src/lib.rs"
        timestamp = source.stat().st_mtime_ns
        source.write_text("changed with the same timestamp")
        os.utime(source, ns=(timestamp, timestamp))
        self.task()
        (self.root / "src/required.rs").unlink()
        self.task()
        for path in ("src/untracked.rs", "assets/font.ttf", "web/colors.css", "Cargo.lock", ".cargo/config.toml"):
            self.write(path, "[build]\njobs = 2\n" if path == ".cargo/config.toml" else "new input")
            self.task()
        self.assertEqual(self.runs(), 8, "content edits, additions, deletions and assets must invalidate")

    def test_context_changes_failures_and_forced_runs(self):
        self.cached_project()
        self.env["CARGO_HOME"] = str(self.root / "cargo-home")
        self.task()
        self.task(env=dict(self.env, RUSTFLAGS="--cfg changed"))
        self.task(env=dict(self.env, WGPU_BACKEND="gl"))
        self.task(env=dict(self.env, LANG="nanoplan-test-locale"))
        self.write("cargo-home/config.toml", "[build]\njobs = 2\n")
        self.task()
        # Same executable path, different active compiler version.
        self.env["PATH"] = str(self.root / "bin") + os.pathsep + self.env["PATH"]
        rustc = self.write("bin/rustc", "#!/bin/sh\nprintf 'rustc fixture 1\\n'\n")
        rustc.chmod(0o755)
        self.task()
        rustc.write_text("#!/bin/sh\nprintf 'rustc fixture 2\\n'\n")
        self.task()
        self.assertEqual(self.runs(), 7)
        self.write("src/lib.rs", "fail")
        self.task(success=False)
        self.task(success=False)
        self.assertEqual(self.runs(), 9, "failures must never become cache hits")
        self.write("src/lib.rs", "ok")
        self.task()
        self.assertEqual(self.runs(), 9, "returning to tested inputs should reuse their success")
        self.task("--force")
        self.task(env=dict(self.env, MISE_TASK_CACHE="write-only"))
        self.assertEqual(self.runs(), 11)

    def test_pre_commit_hashes_the_staged_snapshot(self):
        self.cached_project()
        self.write(".pre-commit-config.yaml", f'''repos:
  - repo: local
    hooks:
      - id: cargo-test
        name: cargo test
        entry: {MISE} run hook-test
        language: system
        always_run: true
        pass_filenames: false
''')
        self.write("README.md", "Initial documentation")
        self.run_command("git", "init", "-q")
        self.run_command("git", "add", ".")
        self.run_command("git", "-c", "user.name=Hook Test", "-c", "user.email=hook-test@example.invalid",
                         "commit", "-qm", "fixture")
        self.task()
        self.write("README.md", "Staged documentation")
        self.run_command("git", "add", "README.md")
        self.write("src/lib.rs", "fail")
        self.run_command("pre-commit", "run", "cargo-test")
        self.assertEqual(self.runs(), 1)
        self.assertEqual((self.root / "src/lib.rs").read_text(), "fail", "unstaged edits must be restored")
        self.run_command("git", "add", "src/lib.rs")
        self.run_command("pre-commit", "run", "cargo-test", success=False)
        self.assertEqual(self.runs(), 2)

    def test_clippy_fixes_reintroduced_warnings_and_rejects_unfixable_ones(self):
        self.mise_project()
        self.write("Cargo.toml", '[package]\nname = "hook-fixture"\nversion = "0.1.0"\nedition = "2024"\n')
        bad_source = "pub fn number() -> i32 { let n = 42; return n; }\n"
        self.write("src/lib.rs", bad_source)
        self.run_command("git", "init", "-q")
        self.run_command("git", "add", ".")
        fixer = [MISE, "run", "hook-clippy-fix"]
        for _ in range(2):
            self.write("src/lib.rs", bad_source)
            self.run_command(*fixer, success=False)
            self.assertNotEqual((self.root / "src/lib.rs").read_text(), bad_source)
            self.run_command("cargo", "clippy", *FLAGS)
        self.run_command(*fixer)
        unchanged = self.run_command(*fixer)
        self.assertIn("sources up-to-date, skipping", unchanged)
        with (self.root / ".gitignore").open("a") as ignore:
            ignore.write("src/generated.rs\n")
        self.write("src/lib.rs", "pub mod generated;\n")
        for _ in range(2):
            self.write("src/generated.rs", bad_source)
            self.run_command(*fixer, success=False)
            self.assertNotEqual((self.root / "src/generated.rs").read_text(), bad_source)
            self.run_command(*fixer)
        (self.root / "src/generated.rs").unlink()
        self.write("src/lib.rs", "pub fn number(value: i32) -> i32 { value }\n")
        self.run_command(*fixer)
        self.write("clippy.toml", 'disallowed-names = ["value"]\n')
        self.run_command(*fixer, success=False)
        self.run_command("cargo", "clippy", *FLAGS, success=False)
        self.write("clippy.toml", 'disallowed-names = ["placeholder"]\n')
        self.run_command(*fixer)
        self.write("clippy.toml", 'disallowed-names = ["value"]\n')
        self.run_command(*fixer, success=False)
        self.run_command("cargo", "clippy", *FLAGS, success=False)
        self.env["CLIPPY_CONF_DIR"] = str(self.root / "clippy-config")
        self.write("clippy-config/clippy.toml", 'disallowed-names = ["placeholder"]\n')
        self.run_command(*fixer)
        self.write("clippy-config/clippy.toml", 'disallowed-names = ["value"]\n')
        self.run_command(*fixer, success=False)
        self.write("src/lib.rs", "pub unsafe fn undocumented() {}\n")
        self.run_command(*fixer, success=False)
        self.run_command("cargo", "clippy", *FLAGS, success=False)


if __name__ == "__main__":
    unittest.main()
