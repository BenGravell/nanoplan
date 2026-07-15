//! Startup download and platform cache for the racetrack database.

use super::catalog::{TRACK_CATALOG, install_track_data, track_catalog_loaded};

const REVISION: &str = "e59595d1f3573b30d1ded6a08984935b957688e0";
const BASE_URL: &str = "https://raw.githubusercontent.com/TUMFTM/racetrack-database";
const SEPARATOR: &str = "\n--NANOPLAN-TRACK--\n";

fn cache_key() -> String {
    format!("nanoplan.tracks.{REVISION}")
}

fn url(file: &str) -> String {
    format!("{BASE_URL}/{REVISION}/tracks/{file}")
}

fn pack(tracks: &[String]) -> String {
    tracks.join(SEPARATOR)
}

fn unpack(data: &str) -> Option<Vec<String>> {
    let tracks = data.split(SEPARATOR).map(str::to_owned).collect::<Vec<_>>();
    (tracks.len() == TRACK_CATALOG.len()).then_some(tracks)
}

#[cfg(not(target_family = "wasm"))]
pub(crate) fn load() -> Result<(), String> {
    use std::path::PathBuf;

    if track_catalog_loaded() {
        return Ok(());
    }
    let cache_root = std::env::var_os("XDG_CACHE_HOME")
        .or_else(|| std::env::var_os("LOCALAPPDATA"))
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache")))
        .unwrap_or_else(std::env::temp_dir);
    let cache = cache_root.join("nanoplan").join(cache_key());
    if let Ok(data) = std::fs::read_to_string(&cache)
        && let Some(tracks) = unpack(&data)
        && install_track_data(&tracks).is_ok()
    {
        return Ok(());
    }

    let agent = ureq::agent();
    let tracks = TRACK_CATALOG
        .iter()
        .map(|track| {
            let file = track.file;
            agent
                .get(&url(file))
                .call()
                .map_err(|error| format!("download {file}: {error}"))?
                .body_mut()
                .read_to_string()
                .map_err(|error| format!("read {file}: {error}"))
        })
        .collect::<Result<Vec<_>, String>>()?;
    install_track_data(&tracks)?;
    if let Some(parent) = cache.parent() {
        std::fs::create_dir_all(parent).map_err(|error| format!("create track cache: {error}"))?;
    }
    std::fs::write(cache, pack(&tracks)).map_err(|error| format!("write track cache: {error}"))
}

#[cfg(target_family = "wasm")]
pub(crate) async fn load() -> Result<(), String> {
    use futures_util::future::try_join_all;
    use gloo_net::http::Request;

    if track_catalog_loaded() {
        return Ok(());
    }
    let cache_key = cache_key();
    let storage = web_sys::window().and_then(|window| window.local_storage().ok().flatten());
    if let Some(tracks) = storage
        .as_ref()
        .and_then(|storage| storage.get_item(&cache_key).ok().flatten())
        .as_deref()
        .and_then(unpack)
        && install_track_data(&tracks).is_ok()
    {
        return Ok(());
    }

    let tracks = try_join_all(TRACK_CATALOG.iter().map(|track| async move {
        let file = track.file;
        let response = Request::get(&url(file))
            .send()
            .await
            .map_err(|error| format!("download {file}: {error}"))?;
        if !response.ok() {
            return Err(format!("download {file}: HTTP {}", response.status()));
        }
        response
            .text()
            .await
            .map_err(|error| format!("read {file}: {error}"))
    }))
    .await?;
    install_track_data(&tracks)?;
    if let Some(storage) = storage {
        storage
            .set_item(&cache_key, &pack(&tracks))
            .map_err(|error| format!("write browser track cache: {error:?}"))?;
    }
    Ok(())
}

#[cfg(test)]
pub(crate) fn install_test_catalog() {
    use std::sync::Arc;

    use super::catalog::{LOADED_CATALOG, LoadedCatalog};
    use super::circuit::Circuit;
    use super::model::TrackModel;

    LOADED_CATALOG.get_or_init(|| {
        let circuit =
            Arc::new(Circuit::parse("0,0,5,5\n1000,0,5,5\n1000,1000,5,5\n0,1000,5,5\n").unwrap());
        let model = TrackModel::train(&[circuit.training_track()]).unwrap();
        LoadedCatalog {
            circuits: vec![circuit; TRACK_CATALOG.len()],
            model,
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_cache_round_trips_all_tracks() {
        let tracks = TRACK_CATALOG
            .iter()
            .map(|track| track.file.to_string())
            .collect::<Vec<_>>();
        assert_eq!(unpack(&pack(&tracks)).unwrap(), tracks);
        assert_eq!(unpack("incomplete"), None);
        assert!(cache_key().contains(REVISION));
    }
}
