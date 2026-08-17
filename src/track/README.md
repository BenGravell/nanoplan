# Tracks

Nanoplan provides pre-generated and preset circuits.

## Pre-generated Tracks

The offline `pregenerate-tracks` tool concurrently downloads pinned source anchors from the
[TUM racetrack database](https://github.com/TUMFTM/racetrack-database).
It applies Nanoplan's closed cubic spline fitting, one-metre arc-length resampling, curvature-aware width correction,
and intersection validation, then saves the resulting points as human-readable CSV under `data/`.

Runtime embeds and parses only these transformed points.
It performs no network requests, spline fitting, width correction, or track validation.
Both test presets are baked through the same spline pipeline.
Regenerate them explicitly with:

```sh
cargo run --features track-pregeneration --bin pregenerate-tracks
```

The downloaded source CSV files are transient and are not included in this repository.

## Preset Tracks

`presets.rs` constructs deterministic closed test tracks.

### Test Track (large)

A wide, long straight, large superellipse end caps, and a narrower return leg that alternates straights with
increasingly tight corners.
A lengthy repeatable circuit for high-speed and cornering stress tests.

### Test Track (small)

A compact circuit made from two straights and two superellipse end caps.
A quick repeatable circuit for planner and simulation tests.

## Processing

Offline, raw centerline anchors are first joined by a closed cubic spline and resampled at a fine arc-length spacing.
The resulting fine polyline plus its interpolated right/left widths is converted into the shared
`geometry::RoadPolygon`: source stations, continuous mitered boundary polylines, and strip quads.
The viewer triangulates that polygon for the road surface, while simulation barriers use the exact same boundary
segments.

Segment-intersection checks reject non-simple shapes.

## Contents

```text
track/
├── README.md           this document
├── catalog.rs          metadata and checked-in baked point inclusion
├── circuit.rs          closed-circuit baking, interpolation, and projection
├── data/               spline-processed point data used at runtime
├── mod.rs              module wiring
├── path.rs             arc-length lookup and Frenet projection
├── pregenerate.rs      offline concurrent downloader and spline baker
├── presets.rs          deterministic procedurally constructed test circuits
├── road.rs             finite planner and simulation road windows
└── track.rs            public preset/baked track API
```
