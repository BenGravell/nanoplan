# Tracks

Nanoplan supports generated and downloaded simple closed circuits. The first
selector entry is deterministic for a seed; the remaining entries are real
circuits used both directly and as runtime training data.

At startup, `loader.rs` downloads 24 non-self-intersecting circuits from a
pinned revision of the
[TUM racetrack database](https://github.com/TUMFTM/racetrack-database). It
stores one combined cache file on desktop or one `localStorage` entry on the
web, then validates and parses every circuit before the viewer starts.

`model.rs` resamples every circuit by arc length and learns Fourier spectra for
signed curvature and the left and right road widths. Generation applies shared
phase perturbations to preserve the curvature/width cross-spectra and the
characteristic balance of straights and corners. A harmonic solve closes each
candidate, and segment-intersection checks reject non-simple shapes.

```text
track/
├── catalog.rs canonical metadata and atomically loaded runtime state
├── circuit.rs closed-circuit parsing, interpolation, and projection
├── loader.rs  startup download and platform cache
├── model.rs   runtime spectral trainer and generator
├── mod.rs     module wiring and catalog installation
├── path.rs    arc-length lookup and Frenet projection
├── road.rs    finite planner and simulation road windows
├── track.rs   public generated/downloaded track API
└── README.md  this document
```

The source CSV files are not included in this repository. Downloaded data
remains subject to its upstream license, and no trained coefficients or
generated data are distributed: the model is rebuilt in memory at startup.
