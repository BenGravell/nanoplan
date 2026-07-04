# 0004 — Unify the desktop/web scenario loaders behind a `ScenarioSource` trait

- **Status**: Accepted
- **Pattern**: Strategy / Adapter
  ([refactoring.guru](https://refactoring.guru/design-patterns/strategy),
  [Adapter](https://refactoring.guru/design-patterns/adapter))
- **Affects**: `src/viewer/loader.rs` (new), `src/viewer/ui.rs`, `src/viewer/web.rs`, `src/viewer/mod.rs`

## Context

The viewer lets a user load extra scenarios at runtime. The two platforms do
this differently: desktop types a filesystem path (`load_path`); wasm opens
the browser's native file picker (async, into a polled slot). Both are
implementations of one concept — "load some scenarios from the user" — but
they were stitched into the `ui()` system as:

- a `#[cfg]`-gated `loader` parameter (`ScenarioLoader` on desktop,
  `WebScenarioLoader` on wasm), **and**
- two `#[cfg]`-gated blocks inside the `ui()` body, one per platform.

The two blocks independently converged on the same shape — a
`status: Option<Result<String, String>>` field, a "merge loaded scenarios,
select the first new one" step, and a green/red `colored_label` render — and
the status-rendering match was **copy-pasted verbatim** in both. The web side
also carried a separate `absorb_load` ECS system duplicating the desktop
merge logic. Adding a third way to load (drag-and-drop, a URL field) meant a
third `cfg` block and a third copy of the shared tail.

## Decision

Introduce one seam both platforms implement, and confine the `#[cfg]` split
to *choosing which implementation* — the shared merge/status logic lives once:

```rust
// src/viewer/loader.rs
pub(crate) trait ScenarioSource {
    /// Render this platform's widget; return Some(result) only on a load
    /// that landed this frame.
    fn widget(&mut self, ui: &mut egui::Ui) -> Option<LoadResult>;
}

pub(crate) struct Loader {
    pub source: Box<dyn ScenarioSource>,   // DesktopLoader | WebScenarioLoader
    pub status: Option<Result<String, String>>,
}
```

- `DesktopLoader` (in `loader.rs`) and `WebScenarioLoader` (in `web.rs`,
  where the wasm-only `rfd`/`Rc` machinery already lives) each implement
  `ScenarioSource`. `Loader::default()` picks the concrete type behind one
  `#[cfg]`.
- `ui()` takes a single `NonSendMut<Loader>`, calls `loader.source.widget(ui)`,
  and owns the platform-independent tail exactly once: extend `Scenarios`,
  jump the selection to the first new scenario, format the "loaded N" message,
  and render the status line.
- The web `absorb_load` system is deleted; its logic was the duplicate the
  shared tail now covers. `widget()` polls the async slot itself.

## Consequences

**Good**

- The status-rendering match and the merge/select logic exist once, not
  twice. `ui()` no longer contains platform `#[cfg]` blocks — only
  `Loader::default()` does.
- A new load source (drag-and-drop, URL) is a new `ScenarioSource` impl, not a
  third `cfg` block and a third copy of the tail.

**Costs / trade-offs**

- `Loader` is a `NonSend` resource (the web source holds `Rc`s, which aren't
  `Send`). This matches the existing pattern — `ActiveJob` and the web fetch
  slots are already `NonSend` for the same reason — so it adds no new concept.
- One `Box<dyn ScenarioSource>` indirection on the load path. This is a
  once-per-click UI action; the virtual call is irrelevant.
- The trait returns `Option<LoadResult>` so each platform can encode its own
  "nothing happened this frame" (desktop: button not clicked; web: dialog
  still open or cancelled) — a slightly subtle contract, documented on the
  trait method.

## Alternatives considered

- **Keep the two `cfg` blocks.** Rejected: that's the duplication this ADR
  removes; it scales badly with a third source.
- **A runtime enum instead of a trait object.** Rejected: the two variants
  are compile-time mutually exclusive by target arch, so a `dyn` behind one
  `cfg` selection is cleaner than an enum whose arms are each `cfg`-gated
  anyway.
- **Merge inside each source** (have `widget()` mutate `Scenarios`/`UiState`
  directly). Rejected: that would re-duplicate the merge/select logic into
  each impl — the exact thing being consolidated. Sources return *data*; the
  shared caller applies it.

## Verification

Builds clean on both `x86_64-unknown-linux-gnu` and
`wasm32-unknown-unknown`; full test suite and `cargo clippy --all-targets`
green.
