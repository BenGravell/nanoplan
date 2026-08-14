use std::path::Path;

fn main() -> Result<(), String> {
    nanoplan::pregenerate_tracks(Path::new("src/track/data"))
}
