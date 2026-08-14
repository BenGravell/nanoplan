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

#[cfg(not(target_family = "wasm"))]
pub mod profile;

#[cfg(not(target_family = "wasm"))]
pub fn run() {
    track::loader::load().expect("failed to load track catalog");
    viewer::run();
}

#[cfg(target_family = "wasm")]
pub fn run() {
    viewer::run();
}
