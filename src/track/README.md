# Tracks

Nanoplan supports generated and downloaded simple closed circuits. The first
selector entry is deterministic for a seed; the remaining entries are real
circuits used directly in the viewer.

At startup, `loader.rs` downloads 24 non-self-intersecting circuits from a
pinned revision of the
[TUM racetrack database](https://github.com/TUMFTM/racetrack-database). It
stores one combined cache file on desktop or one `localStorage` entry on the
web, then validates and parses every circuit before the viewer starts.

`trained_model.json` contains Fourier spectra learned from the pinned circuits'
signed curvature and left and right road widths. `model.rs` loads those checked-in
coefficients directly. Generation applies shared phase perturbations to preserve
the curvature/width cross-spectra and characteristic balance of straights and
corners. A harmonic solve closes each candidate, and segment-intersection checks
reject non-simple shapes.

Raw centerline anchors are first joined by a closed cubic spline and resampled
at approximately one-metre arc-length spacing. The resulting fine polyline plus
its interpolated right/left widths is converted into the shared
`geometry::RoadPolygon`: source stations, continuous mitered boundary polylines,
and strip quads. The viewer triangulates that polygon for the road surface,
while simulation barriers use the exact same boundary segments.

```text
track/
├── catalog.rs canonical metadata and atomically loaded runtime state
├── circuit.rs closed-circuit parsing, interpolation, and projection
├── loader.rs  startup download and platform cache
├── model.rs   pretrained spectral model loader and generator
├── mod.rs     module wiring and catalog installation
├── path.rs    arc-length lookup and Frenet projection
├── road.rs    finite planner and simulation road windows
├── track.rs   public generated/downloaded track API
├── README.md  this document
└── trained_model.json checked-in Fourier coefficients and provenance
```

The source CSV files are not included in this repository. Downloaded data
remains subject to its upstream license; the derived trained coefficients are
distributed in `trained_model.json` so startup does not retrain the model.
