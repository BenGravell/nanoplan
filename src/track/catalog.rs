//! Canonical track identifiers and their presentation/source metadata.

use std::sync::{Arc, OnceLock};

use super::circuit::Circuit;
use super::model::{TrackModel, is_simple};

#[derive(Debug, Clone, Copy)]
pub(crate) struct TrackInfo {
    pub(crate) id: &'static str,
    pub(crate) name: &'static str,
    pub(crate) file: &'static str,
}

pub(crate) const TRACK_CATALOG: [TrackInfo; 24] = [
    TrackInfo {
        id: "austin",
        name: "Austin",
        file: "Austin.csv",
    },
    TrackInfo {
        id: "brands_hatch",
        name: "Brands Hatch",
        file: "BrandsHatch.csv",
    },
    TrackInfo {
        id: "budapest",
        name: "Budapest",
        file: "Budapest.csv",
    },
    TrackInfo {
        id: "catalunya",
        name: "Catalunya",
        file: "Catalunya.csv",
    },
    TrackInfo {
        id: "hockenheim",
        name: "Hockenheim",
        file: "Hockenheim.csv",
    },
    TrackInfo {
        id: "indianapolis",
        name: "Indianapolis",
        file: "IMS.csv",
    },
    TrackInfo {
        id: "melbourne",
        name: "Melbourne",
        file: "Melbourne.csv",
    },
    TrackInfo {
        id: "mexico_city",
        name: "Mexico City",
        file: "MexicoCity.csv",
    },
    TrackInfo {
        id: "montreal",
        name: "Montreal",
        file: "Montreal.csv",
    },
    TrackInfo {
        id: "monza",
        name: "Monza",
        file: "Monza.csv",
    },
    TrackInfo {
        id: "moscow_raceway",
        name: "Moscow Raceway",
        file: "MoscowRaceway.csv",
    },
    TrackInfo {
        id: "norisring",
        name: "Norisring",
        file: "Norisring.csv",
    },
    TrackInfo {
        id: "nuerburgring",
        name: "Nuerburgring",
        file: "Nuerburgring.csv",
    },
    TrackInfo {
        id: "oschersleben",
        name: "Oschersleben",
        file: "Oschersleben.csv",
    },
    TrackInfo {
        id: "sakhir",
        name: "Sakhir",
        file: "Sakhir.csv",
    },
    TrackInfo {
        id: "sao_paulo",
        name: "Sao Paulo",
        file: "SaoPaulo.csv",
    },
    TrackInfo {
        id: "sepang",
        name: "Sepang",
        file: "Sepang.csv",
    },
    TrackInfo {
        id: "shanghai",
        name: "Shanghai",
        file: "Shanghai.csv",
    },
    TrackInfo {
        id: "silverstone",
        name: "Silverstone",
        file: "Silverstone.csv",
    },
    TrackInfo {
        id: "sochi",
        name: "Sochi",
        file: "Sochi.csv",
    },
    TrackInfo {
        id: "spa",
        name: "Spa",
        file: "Spa.csv",
    },
    TrackInfo {
        id: "spielberg",
        name: "Spielberg",
        file: "Spielberg.csv",
    },
    TrackInfo {
        id: "yas_marina",
        name: "Yas Marina",
        file: "YasMarina.csv",
    },
    TrackInfo {
        id: "zandvoort",
        name: "Zandvoort",
        file: "Zandvoort.csv",
    },
];

pub(super) static LOADED_CATALOG: OnceLock<LoadedCatalog> = OnceLock::new();

pub(super) struct LoadedCatalog {
    pub(super) circuits: Vec<Arc<Circuit>>,
    pub(super) model: TrackModel,
}

pub(super) fn loaded_catalog() -> Option<&'static LoadedCatalog> {
    LOADED_CATALOG.get()
}

pub(super) fn track_catalog_loaded() -> bool {
    loaded_catalog().is_some()
}

pub(super) fn install_track_data(data: &[String]) -> Result<(), String> {
    if data.len() != TRACK_CATALOG.len() {
        return Err(format!(
            "expected {} tracks, received {}",
            TRACK_CATALOG.len(),
            data.len()
        ));
    }
    let circuits = data
        .iter()
        .zip(TRACK_CATALOG.iter())
        .map(|(csv, track)| {
            Circuit::parse(csv)
                .and_then(|circuit| {
                    let points = circuit
                        .samples
                        .iter()
                        .map(|sample| sample.point)
                        .collect::<Vec<_>>();
                    is_simple(&points)
                        .then_some(circuit)
                        .ok_or_else(|| "centerline intersects itself".to_owned())
                })
                .map(Arc::new)
                .map_err(|error| format!("{}: {error}", track.id))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let training = circuits
        .iter()
        .map(|circuit| circuit.training_track())
        .collect::<Vec<_>>();
    let model = TrackModel::train(&training)?;
    LOADED_CATALOG
        .set(LoadedCatalog { circuits, model })
        .map_err(|_| "track catalog already loaded".to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique_lowercase_source_symbols() {
        for (index, track) in TRACK_CATALOG.iter().enumerate() {
            assert!(
                track
                    .id
                    .bytes()
                    .all(|c| c.is_ascii_lowercase() || c == b'_')
            );
            assert!(
                !TRACK_CATALOG[..index]
                    .iter()
                    .any(|other| other.id == track.id)
            );
        }
    }
}
