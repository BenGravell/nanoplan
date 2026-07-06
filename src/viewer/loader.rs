//! One interface over the platform scenario loaders (Strategy/Adapter):
//! desktop loads from a filesystem path, web from the browser's file
//! picker. `ui()` talks only to [`ScenarioSource`], so the shared parts —
//! merging loaded scenarios into the list, selecting the first new one,
//! and rendering the status line — exist once, and the `#[cfg]` split is
//! confined to choosing which source a [`Loader`] holds.

use bevy_egui::egui;
use nanoplan::Scenario;

pub(crate) type LoadResult = Result<Vec<Scenario>, String>;

/// A platform's way of loading scenarios from inside the UI.
pub(crate) trait ScenarioSource {
    /// Render this platform's load widget for the frame. Returns a result
    /// only when a load landed this frame: `Ok` with the (non-empty)
    /// scenarios to merge in, or `Err` with a message to show. Each
    /// platform maps its own quirks — an empty directory is an `Err` on
    /// desktop, a cancelled file-picker dialog is `None` on web.
    fn widget(&mut self, ui: &mut egui::Ui) -> Option<LoadResult>;
}

/// The scenario-loading widget's state: the platform source plus the
/// platform-independent status line, as a `NonSend` resource (the web
/// source holds `Rc`s).
pub(crate) struct Loader {
    pub source: Box<dyn ScenarioSource>,
    pub status: Option<Result<String, String>>,
}

impl Default for Loader {
    fn default() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        let source: Box<dyn ScenarioSource> = Box::new(DesktopLoader::default());
        #[cfg(target_arch = "wasm32")]
        let source: Box<dyn ScenarioSource> = Box::new(super::web::WebScenarioLoader::default());
        Loader {
            source,
            status: None,
        }
    }
}

/// Desktop source: type a path to exported scenarios (a `*.json` file or a
/// directory of them — CommonRoad conversions or local nuPlan exports) and
/// load it live, without relaunching with CLI args.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Default)]
struct DesktopLoader {
    path: String,
}

#[cfg(not(target_arch = "wasm32"))]
impl ScenarioSource for DesktopLoader {
    fn widget(&mut self, ui: &mut egui::Ui) -> Option<LoadResult> {
        let mut result = None;
        ui.horizontal(|ui| {
            ui.label("scenario path:");
            ui.text_edit_singleline(&mut self.path);
            if ui.button("Load").clicked() {
                result = Some(
                    match nanoplan::scenarios::load_path(std::path::Path::new(self.path.trim())) {
                        Ok(loaded) if loaded.is_empty() => {
                            Err("no *.json scenarios found there".into())
                        }
                        Ok(loaded) => Ok(loaded),
                        Err(e) => Err(e.to_string()),
                    },
                );
            }
        });
        result
    }
}
