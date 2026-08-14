# Tracks

Nanoplan provides procedurally generated, preset, and real-world tracks.

## Real-world Tracks

The real-world tracks come from the [TUM racetrack database](https://github.com/TUMFTM/racetrack-database).

After opening the viewer with a built-in track, `loader.rs` loads the real-world tracks from a single cache entry or downloads them from a pinned revision of the [TUM racetrack database](https://github.com/TUMFTM/racetrack-database).
It installs the downloaded source catalog immediately, then parses and validates a circuit when the user selects it.
The first startup downloads the catalog in the background; later desktop starts use one combined file in a cache directory, while web starts use one `localStorage` entry.
Delete that file or entry to force a fresh download.

The source CSV files are not included in this repository.
Downloaded data remains subject to its upstream license.

## Procedurally Generated Tracks

`trained_model.json` contains Fourier spectra learned from the real-world tracks' signed curvature and left and right road widths.
`model.rs` loads those checked-in coefficients directly.
Procedural generation builds a hybrid spectrum by drawing each harmonic from a different training profile, then randomizes its phase.
The phase is shared between curvature and widths to preserve their cross-spectra and characteristic balance of straights and corners.
A harmonic solve closes each candidate.
Candidates whose signed-curvature shape has at least 0.95 cyclic correlation with any training circuit are rejected, including reversed or mirrored matches.

## Preset Tracks

`presets.rs` constructs deterministic closed test tracks.

### Test Track (large)

A wide, long straight, large superellipse end caps, and a narrower return leg that alternates straights with increasingly tight corners.
A lengthy repeatable circuit for high-speed and cornering stress tests.

### Test Track (small)
A compact circuit made from two straights and two superellipse end caps.
A quick repeatable circuit for planner and simulation tests.

## Processing

Raw centerline anchors are first joined by a closed cubic spline and resampled at a fine arc-length spacing.
The resulting fine polyline plus its interpolated right/left widths is converted into the shared `geometry::RoadPolygon`: source stations, continuous mitered boundary polylines, and strip quads.
The viewer triangulates that polygon for the road surface, while simulation barriers use the exact same boundary segments.

Segment-intersection checks reject non-simple shapes.

## Contents

```text
track/
├── README.md           this document
├── catalog.rs          canonical metadata and atomically loaded runtime state
├── circuit.rs          closed-circuit parsing, interpolation, and projection
├── loader.rs           startup download and platform cache
├── model.rs            pretrained spectral model loader and generator
├── mod.rs              module wiring and catalog installation
├── path.rs             arc-length lookup and Frenet projection
├── presets.rs          deterministic procedurally constructed test circuits
├── road.rs             finite planner and simulation road windows
├── track.rs            public generated/preset/downloaded track API
└── trained_model.json  model coefficients and provenance
```
