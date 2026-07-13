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

The demo has no external maps, datasets, Python conversion tools, or runtime
network requests.
