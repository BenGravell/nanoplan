# Usage

## Desktop app

Run the desktop app:

```bash
cargo run --release
```

## Web app

Run the web app:

```bash
trunk serve --release
```

## Profiling

The `profile` tool runs end-to-end laps and prints statistics for command-line profiling.
Planner, track, and lap count (including fractions) are configurable:

```bash
cargo run --release --bin profile -- --planner lattice --track small --laps 0.3
```

## Tests

Run tests:

```bash
cargo test
```
