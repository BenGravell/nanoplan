#!/usr/bin/env python3
"""Report inputs mise cannot declare directly: ancestors and variable prefixes.

mise hashes command-input output without printing or retaining it.
"""

import json
import os
from pathlib import Path


ENV_PREFIXES = (
    "CARGO_", "RUST", "CLIPPY_", "CC", "CXX", "AR", "CFLAGS", "CXXFLAGS", "CPPFLAGS",
    "LDFLAGS", "LD_", "DYLD_", "LIB", "PKG_CONFIG",
)
if os.environ.get("MISE_TASK_NAME") == "hook-test":
    ENV_PREFIXES += ("WGPU", "VK_", "MESA", "EGL", "GL")

root = Path.cwd()
clippy_root = Path(os.environ.get("CLIPPY_CONF_DIR", root)).resolve()
configs = {
    str(path): path.read_text()
    for parents, names in (
        (root.parents, (".cargo/config", ".cargo/config.toml")),
        (clippy_root.parents, ("clippy.toml", ".clippy.toml")),
    )
    for parent in parents
    for name in names
    if (path := parent / name).is_file()
}
print(json.dumps({
    "config": configs,
    "env": {key: value for key, value in os.environ.items() if key.startswith(ENV_PREFIXES)},
}, sort_keys=True))
