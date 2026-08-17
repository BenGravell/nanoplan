#[cfg(target_family = "wasm")]
fn main() {
    nanoplan::register_planner_worker();
}

#[cfg(not(target_family = "wasm"))]
fn main() {}
