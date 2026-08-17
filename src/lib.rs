//! Nanoplan application and reusable command-line tooling.

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

#[cfg(target_family = "wasm")]
pub fn register_planner_worker() {
    planning::engine::register();
}

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
pub use track::pregenerate::pregenerate_tracks;

#[cfg(not(target_family = "wasm"))]
pub mod profile;

#[cfg(not(target_family = "wasm"))]
pub fn run() {
    viewer::run();
}

#[cfg(target_family = "wasm")]
pub fn run() {
    viewer::run();
}
