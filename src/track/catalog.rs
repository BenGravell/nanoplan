//! Checked-in, spline-processed track catalog.

use std::sync::{Arc, OnceLock};

use super::circuit::Circuit;

#[derive(Debug, Clone, Copy)]
pub(crate) struct TrackInfo {
    #[cfg_attr(target_family = "wasm", allow(dead_code))]
    pub(crate) id: &'static str,
    pub(crate) name: &'static str,
    pub(super) data: &'static str,
    #[cfg(feature = "track-pregeneration")]
    pub(super) file: &'static str,
}

macro_rules! track {
    ($id:literal, $name:literal, $file:literal) => {
        TrackInfo {
            id: $id,
            name: $name,
            data: include_str!(concat!("data/", $id, ".csv")),
            #[cfg(feature = "track-pregeneration")]
            file: $file,
        }
    };
}

pub(crate) const TRACK_CATALOG: [TrackInfo; 24] = [
    track!("austin", "Austin", "Austin.csv"),
    track!("brands_hatch", "Brands Hatch", "BrandsHatch.csv"),
    track!("budapest", "Budapest", "Budapest.csv"),
    track!("catalunya", "Catalunya", "Catalunya.csv"),
    track!("hockenheim", "Hockenheim", "Hockenheim.csv"),
    track!("indianapolis", "Indianapolis", "IMS.csv"),
    track!("melbourne", "Melbourne", "Melbourne.csv"),
    track!("mexico_city", "Mexico City", "MexicoCity.csv"),
    track!("montreal", "Montreal", "Montreal.csv"),
    track!("monza", "Monza", "Monza.csv"),
    track!("moscow_raceway", "Moscow Raceway", "MoscowRaceway.csv"),
    track!("norisring", "Norisring", "Norisring.csv"),
    track!("nuerburgring", "Nuerburgring", "Nuerburgring.csv"),
    track!("oschersleben", "Oschersleben", "Oschersleben.csv"),
    track!("sakhir", "Sakhir", "Sakhir.csv"),
    track!("sao_paulo", "Sao Paulo", "SaoPaulo.csv"),
    track!("sepang", "Sepang", "Sepang.csv"),
    track!("shanghai", "Shanghai", "Shanghai.csv"),
    track!("silverstone", "Silverstone", "Silverstone.csv"),
    track!("sochi", "Sochi", "Sochi.csv"),
    track!("spa", "Spa", "Spa.csv"),
    track!("spielberg", "Spielberg", "Spielberg.csv"),
    track!("yas_marina", "Yas Marina", "YasMarina.csv"),
    track!("zandvoort", "Zandvoort", "Zandvoort.csv"),
];

pub(super) const PRESET_TRACKS: [&str; 2] = [
    include_str!("data/preset_large.csv"),
    include_str!("data/preset_small.csv"),
];

static CIRCUITS: [OnceLock<Arc<Circuit>>; TRACK_CATALOG.len()] = [const { OnceLock::new() }; TRACK_CATALOG.len()];

pub(super) fn circuit(index: usize) -> Result<Arc<Circuit>, String> {
    let track = TRACK_CATALOG
        .get(index)
        .ok_or_else(|| "track catalog index out of bounds".to_owned())?;
    Ok(CIRCUITS[index]
        .get_or_init(|| Arc::new(Circuit::baked(track.data)))
        .clone())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_and_baked_data_are_valid() {
        for (index, track) in TRACK_CATALOG.iter().enumerate() {
            assert!(track.id.bytes().all(|c| c.is_ascii_lowercase() || c == b'_'));
            assert!(!TRACK_CATALOG[..index].iter().any(|other| other.id == track.id));
            assert!(track.data.starts_with("# x_m,y_m,w_tr_right_m,w_tr_left_m\n"));
        }
    }
}
