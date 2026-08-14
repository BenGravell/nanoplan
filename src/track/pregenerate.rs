//! Offline downloader and spline-baking tool for the checked-in track catalog.

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
pub(super) const SOURCE: &str = "https://github.com/TUMFTM/racetrack-database";

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
pub(super) const REVISION: &str = "e59595d1f3573b30d1ded6a08984935b957688e0";

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
use std::path::Path;

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
use super::catalog::TRACK_CATALOG;

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
use super::circuit::Circuit;

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
use super::presets;

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
const BASE_URL: &str = "https://raw.githubusercontent.com/TUMFTM/racetrack-database";

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
pub fn pregenerate_tracks(output: &Path) -> Result<(), String> {
    std::fs::create_dir_all(output).map_err(|error| format!("create {}: {error}", output.display()))?;
    let downloads = std::thread::scope(|scope| {
        TRACK_CATALOG
            .iter()
            .map(|track| {
                scope.spawn(move || {
                    let url = format!("{BASE_URL}/{REVISION}/tracks/{}", track.file);
                    ureq::get(&url)
                        .call()
                        .map_err(|error| format!("download {} from {SOURCE}: {error}", track.file))?
                        .body_mut()
                        .read_to_string()
                        .map(|source| (track, source))
                        .map_err(|error| format!("read {}: {error}", track.file))
                })
            })
            .collect::<Vec<_>>()
            .into_iter()
            .map(|download| download.join().map_err(|_| "track download panicked".to_owned())?)
            .collect::<Result<Vec<_>, String>>()
    })?;
    for (track, source) in downloads {
        let circuit = Circuit::parse(&source).and_then(|circuit| {
            circuit
                .is_simple()
                .then_some(circuit)
                .ok_or_else(|| "road intersects itself".to_owned())
        })?;
        write(output, track.id, circuit)?;
    }
    for (index, name) in ["preset_large", "preset_small"].into_iter().enumerate() {
        write(output, name, Circuit::preset(presets::generate(index)))?;
    }
    Ok(())
}

#[cfg(all(feature = "track-pregeneration", not(target_family = "wasm")))]
fn write(output: &Path, name: &str, circuit: Circuit) -> Result<(), String> {
    std::fs::write(output.join(format!("{name}.csv")), circuit.baked_csv())
        .map_err(|error| format!("write {name}: {error}"))
}
