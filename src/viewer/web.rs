//! Fetches the pre-bundled nuPlan scenario set at startup — the web
//! equivalent of desktop's `load_path`/CLI args/"nuPlan path" widget, none
//! of which work without a filesystem. `scenarios/web_bundle.json` (built by
//! `tools/bundle_web_scenarios.py`, copied into `dist/` by Trunk's
//! `copy-file` directive in `index.html`) is fetched once as a single
//! compact JSON array — one HTTP request instead of one per scenario.

use std::cell::RefCell;
use std::rc::Rc;

use bevy::prelude::*;
use nanoplan::Scenario;

use super::Scenarios;

/// Relative, so it resolves under the page's own base path both for
/// `trunk serve` (http://localhost:8080/) and the GitHub Pages deploy
/// (.../nanoplan/).
const BUNDLE_URL: &str = "web_bundle.json";

/// Slot the spawned fetch task writes its result into, polled once a
/// frame. Plain `Rc<RefCell<_>>` (wasm is single-threaded, so this can
/// never be `Send`) means this can only ever be a `NonSend` resource.
#[derive(Default)]
pub(crate) struct WebScenarioFetch(Rc<RefCell<Option<Vec<Scenario>>>>);

pub(crate) fn spawn_fetch(fetch: NonSend<WebScenarioFetch>) {
    let slot = fetch.0.clone();
    wasm_bindgen_futures::spawn_local(async move {
        *slot.borrow_mut() = Some(fetch_bundle().await);
    });
}

async fn fetch_bundle() -> Vec<Scenario> {
    let response = match gloo_net::http::Request::get(BUNDLE_URL).send().await {
        Ok(r) => r,
        Err(e) => {
            warn!("{BUNDLE_URL} fetch failed: {e}");
            return Vec::new();
        }
    };
    if !response.ok() {
        warn!("{BUNDLE_URL} fetch returned HTTP {}", response.status());
        return Vec::new();
    }
    match response.json::<Vec<Scenario>>().await {
        Ok(v) => v,
        Err(e) => {
            warn!("{BUNDLE_URL} parse failed: {e}");
            Vec::new()
        }
    }
}

/// Once a frame: if the fetch has landed, merge it into the scenario
/// list. `take()` leaves `None` behind, so this is a no-op every frame
/// after the first (cheap: one `Option` check).
pub(crate) fn absorb_fetch(fetch: NonSend<WebScenarioFetch>, mut scenes: ResMut<Scenarios>) {
    let Some(loaded) = fetch.0.borrow_mut().take() else {
        return;
    };
    if !loaded.is_empty() {
        info!("loaded {} scenario(s) from {BUNDLE_URL}", loaded.len());
    }
    scenes.0.extend(loaded);
}
