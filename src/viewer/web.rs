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

use super::{Scenarios, UiState};

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

type LoadResult = Result<Vec<Scenario>, String>;

/// State for the in-app scenario-loading widget on web: opens the browser's
/// native file picker and loads whatever `*.json` files the user selects —
/// the web equivalent of desktop's "nuPlan path" widget (`ui::ScenarioLoader`
/// in `ui.rs`), which needs arbitrary filesystem access wasm doesn't have.
/// Lets a user visiting the deployed site browse scenarios exported from
/// their own nuPlan log, not just whatever a maintainer baked into
/// `web_bundle.json` at deploy time.
#[derive(Default)]
pub(crate) struct WebScenarioLoader {
    result: Rc<RefCell<Option<LoadResult>>>,
    loading: Rc<RefCell<bool>>,
    status: Option<Result<String, String>>,
}

impl WebScenarioLoader {
    pub(crate) fn is_loading(&self) -> bool {
        *self.loading.borrow()
    }

    pub(crate) fn status(&self) -> Option<&Result<String, String>> {
        self.status.as_ref()
    }

    /// Opens the file picker and, once the user picks files (or cancels),
    /// stashes the result for `absorb_load` to merge in next frame.
    /// A no-op if a pick is already in flight.
    pub(crate) fn spawn_pick(&self) {
        if *self.loading.borrow() {
            return;
        }
        *self.loading.borrow_mut() = true;
        let result_slot = self.result.clone();
        let loading_slot = self.loading.clone();
        wasm_bindgen_futures::spawn_local(async move {
            *result_slot.borrow_mut() = Some(load_picked_files().await);
            *loading_slot.borrow_mut() = false;
        });
    }
}

async fn load_picked_files() -> LoadResult {
    let Some(files) = rfd::AsyncFileDialog::new()
        .add_filter("scenario JSON", &["json"])
        .pick_files()
        .await
    else {
        return Ok(Vec::new()); // dialog cancelled
    };
    let mut scenarios = Vec::new();
    for file in files {
        let bytes = file.read().await;
        // accept either a single exported scenario or a bundle array — the
        // same two shapes the startup bundle fetch and desktop's load_path
        // both accept
        if let Ok(one) = serde_json::from_slice::<Scenario>(&bytes) {
            scenarios.push(one);
        } else if let Ok(many) = serde_json::from_slice::<Vec<Scenario>>(&bytes) {
            scenarios.extend(many);
        } else {
            return Err(format!(
                "{}: not a valid scenario JSON file",
                file.file_name()
            ));
        }
    }
    Ok(scenarios)
}

/// Once a frame: if a file pick-and-load has landed, merge it into the
/// scenario list and select the first newly loaded one (matching desktop's
/// Load button), and record a status message for the UI to show.
pub(crate) fn absorb_load(
    mut loader: NonSendMut<WebScenarioLoader>,
    mut scenes: ResMut<Scenarios>,
    mut state: ResMut<UiState>,
) {
    let Some(result) = loader.result.borrow_mut().take() else {
        return;
    };
    match result {
        Ok(loaded) if loaded.is_empty() => {} // dialog cancelled: leave prior status as-is
        Ok(loaded) => {
            let n = loaded.len();
            info!("loaded {n} scenario(s) from the file picker");
            state.scenario = scenes.0.len();
            scenes.0.extend(loaded);
            loader.status = Some(Ok(format!(
                "loaded {n} scenario{}",
                if n == 1 { "" } else { "s" }
            )));
        }
        Err(e) => loader.status = Some(Err(e)),
    }
}
