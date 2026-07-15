//! Ultra minimalist motion planner for car-like vehicles.

mod common;
mod geometry;
mod metrics;
mod planning;
mod prediction;
mod simulation;
mod track;
mod vehicle;
mod viewer;
mod world;

#[cfg(not(target_family = "wasm"))]
fn main() {
    track::loader::load().expect("failed to load track catalog");
    viewer::run();
}

#[cfg(target_family = "wasm")]
fn main() {
    wasm_bindgen_futures::spawn_local(async {
        track::loader::load()
            .await
            .expect("failed to load track catalog");
        viewer::run();
    });
}
