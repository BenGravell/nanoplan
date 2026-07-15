# Setup

Install the Rust toolchain selected by `rust-toolchain.toml`, then build:

```bash
cargo build --release
cargo test
```

Linux builds need the normal Bevy window-system development libraries for
X11 or Wayland. For the web viewer, install `trunk` and the wasm target:

```bash
rustup target add wasm32-unknown-unknown
cargo install trunk
trunk serve --release
```

The first app startup downloads the real-track catalog from a pinned upstream
revision before opening the viewer. Later desktop starts use a single file in
`$XDG_CACHE_HOME/nanoplan` (or the platform cache directory); web starts use a
single `localStorage` entry. Delete that entry to force a fresh download.
The spectral model is trained from that cache in memory on every startup; no
model artifact derived from the upstream data is part of the application
payload.
