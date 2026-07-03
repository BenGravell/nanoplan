# Setup

## Prerequisites

- **Rust**, stable channel, **1.95 or newer** (bevy 0.19 requires it). If
  `rustup` is already installed, you don't need to do anything else —
  [`rust-toolchain.toml`](../rust-toolchain.toml) pins the channel and adds
  the `wasm32-unknown-unknown` target automatically the first time you run
  `cargo` in this repo.
- **Python 3** with the standard library only, if you plan to export real
  nuPlan log scenarios (`tools/export_nuplan_scenarios.py` uses `sqlite3`
  from the standard library — no `pip install` and no `nuplan-devkit`
  needed).

No other language toolchains are required. There is no JavaScript build step
outside of what [Trunk](#web-wasm-build) drives.

## Native build

```sh
git clone <repo-url> nanoplan && cd nanoplan
cargo build            # debug build of both binaries (nanoplan, batch)
cargo test             # ~27 tests across all four components
cargo run              # launch the interactive viewer
```

The first build compiles bevy from source and takes a few minutes; after
that, incremental builds are fast. `[profile.dev]` in
[`Cargo.toml`](../Cargo.toml) sets `opt-level = 1` for this crate's own code
and `opt-level = 3` for dependencies — the standard bevy debug-profile
tradeoff that keeps iteration snappy without sacrificing runtime frame rate.

## Profiling build times

```sh
python3 tools/build_timings.py            # incremental build, reports the report path + slowest crates
python3 tools/build_timings.py --clean    # cargo clean first: a true from-scratch baseline
python3 tools/build_timings.py --top 30 -- --release --bin batch   # extra args go to `cargo build`
```

A thin wrapper around cargo's built-in `--timings` report (`target/cargo-timings/cargo-timing.html`):
runs the build, then prints wall time, CPU time (i.e. the parallelism
factor), and the slowest individual crates to compile. Used to find two
concrete wins already applied in this repo:

- **`2d_bevy_render` looked minimal but wasn't.** It routes through bevy's
  `common_api` feature bundle, which pulls in `bevy_ui`, `bevy_ui_render`,
  `bevy_animation`, `bevy_scene`, and every picking backend regardless — none
  of which this app uses. Listing the individual render features instead
  (see [`Cargo.toml`](../Cargo.toml)) dropped all of them from the build.
- **`bevy_egui`'s own `default` feature set does the same thing**: its
  `bevy_ui` feature (on by default) drags `bevy_ui`/`bevy_ui_render` back in
  even with them off on the `bevy` dependency, for embedding egui inside
  bevy_ui layouts (also unused here). `default-features = false` plus an
  explicit list (`manage_clipboard`, `default_fonts`, `render`, `picking`)
  fixed it.

Together these cut a clean `cargo build` from **739.6s wall / 2872.1s CPU
(3.9x parallelism, 467 units)** to **505.9s wall / 1991.0s CPU (3.9x
parallelism, 410 units)** on a 4-core machine — a **31.6% wall-time
reduction** — mostly by removing 57 whole compilation units from the
dependency graph (`bevy_ui`, `bevy_ui_render`, `bevy_animation`, `bevy_text`,
`webbrowser`, and their transitive deps) rather than compiling the same code
faster; `[profile.dev] debug = "line-tables-only"` (instead of full
debuginfo) accounts for the rest, trading per-variable debugger info for
faster codegen and linking (panic/backtrace file:line info is unaffected).

### Linux system dependencies

Bevy's windowing and audio backends link against system libraries that
aren't always present in minimal containers. If `cargo build` fails at the
link step (not the compile step), install:

```sh
sudo apt-get install -y --no-install-recommends \
    libwayland-dev libxkbcommon-dev libudev-dev libasound2-dev pkg-config
```

To *run* the viewer in a headless environment (CI, a container with no
display), you additionally need a software renderer and a virtual display:

```sh
sudo apt-get install -y --no-install-recommends \
    libxkbcommon-x11-0 mesa-vulkan-drivers libegl1 xvfb
```

Then run under `xvfb-run cargo run`, or start `Xvfb` yourself and point
`DISPLAY` at it.

## Web (wasm) build

The viewer also targets `wasm32-unknown-unknown` and deploys to GitHub Pages
on every push to `main` (see [docs/USAGE.md#web-deploy](USAGE.md#web-deploy)
for the CI side). To build or serve it locally:

```sh
rustup target add wasm32-unknown-unknown   # no-op if rust-toolchain.toml already added it
cargo install trunk
trunk serve                                # http://localhost:8080, rebuilds on change
trunk build --release --public-url /nanoplan/   # static output in dist/
```

[`index.html`](../index.html) is Trunk's entry point. `data-bin="nanoplan"`
pins Trunk to the viewer binary — without it, Trunk refuses to guess between
`nanoplan` and the `batch` binary and the build fails with *"found more than
one target artifact"*.

### wasm-specific notes

- **Bevy and bevy_egui features are trimmed** in
  [`Cargo.toml`](../Cargo.toml) to individual render features (`bevy_render`,
  `bevy_sprite_render`, `bevy_gizmos_render`, ...) rather than the coarse
  `2d_bevy_render` bundle, and `bevy_egui` has `default-features = false` —
  see the comments there for why (the bundle and bevy_egui's defaults each
  pull in bevy_ui/bevy_ui_render/bevy_animation/bevy_scene, unused here).
  This keeps the wasm binary from ballooning; a full-feature bevy wasm build
  easily hits 30-50 MB unoptimized. `[profile.release]` additionally sets
  `opt-level = "s"`, thin LTO, and stripped debuginfo. See
  [Profiling build times](#profiling-build-times) for how this was found.
- **Timing** (`src/planning/latency.rs`) uses the [`web-time`](https://docs.rs/web-time)
  crate instead of `std::time::Instant`, which panics on wasm.
- **`wasm-opt` version matters.** Trunk's own `wasm-opt` download (invoked
  via `data-wasm-opt="s"` in `index.html`) works correctly. If you instead
  run a system-packaged `wasm-opt` (e.g. Ubuntu's binaryen 108) over the
  output, the binary can come out corrupted and fail at runtime with
  `WebAssembly.Table.grow(): failed to grow table`. Stick to Trunk's
  pipeline or a recent binaryen (130+) if you optimize by hand.
- Directories passed as CLI arguments to load extra scenarios
  (`cargo run -- some_dir/`) are a desktop-only feature — the wasm build has
  no filesystem, so that code path is `#[cfg(not(target_arch = "wasm32"))]`.
  Instead, the wasm build fetches `scenarios/web_bundle.json` at startup
  (`main.rs`'s `web_scenarios` module, wasm-only, using `gloo-net` +
  `wasm_bindgen_futures::spawn_local` — polled once a frame the same way
  `IncrementalSim`'s async simulation jobs are). Build it with
  `python3 tools/bundle_web_scenarios.py` after exporting scenarios into
  `scenarios/web/`; Trunk's `copy-file` directive in `index.html` copies the
  result into `dist/`. See [docs/USAGE.md#scenario-sources](USAGE.md#scenario-sources).

## Batch runner

The batch runner is a second binary in the same crate; no extra setup:

```sh
cargo run --release --bin batch -- --count 20
```

See [docs/USAGE.md#batch-evaluation](USAGE.md#batch-evaluation) for flags and
output format.

## Exporting real nuPlan scenarios

`tools/export_nuplan_scenarios.py` reads a nuPlan log `.db` (SQLite) file
directly — you do not need to install `nuplan-devkit` or download the full
nuPlan dataset's maps to use it, only a log file:

```sh
python3 tools/export_nuplan_scenarios.py path/to/log.db out_dir
```

See [docs/USAGE.md#exporting-real-nuplan-scenarios](USAGE.md#exporting-real-nuplan-scenarios)
for details and flags.

## Verifying the setup

```sh
cargo fmt --check
cargo clippy --all-targets
cargo test
cargo check --bin nanoplan --target wasm32-unknown-unknown
```

All four should be clean on a working setup. The last one catches
wasm-incompatible code (like `std::time::Instant`) without needing a full
Trunk build.
