use bevy_egui::egui;

use super::pages::Pages;
use super::{ControlTab, Navigator, UiState};
use crate::planning::Latency;
use crate::viewer::live::Live;

pub(super) const PHONE_LANDSCAPE_SIZES: [(&str, egui::Vec2); 6] = [
    ("phone-galaxy-s", egui::vec2(800.0, 480.0)),
    ("phone-iphone-se-2016", egui::vec2(568.0, 320.0)),
    ("phone-iphone-se-3", egui::vec2(667.0, 375.0)),
    ("phone-galaxy-s24", egui::vec2(780.0, 360.0)),
    ("phone-iphone-14-15-pro", egui::vec2(852.0, 393.0)),
    ("phone-galaxy-a55-wide", egui::vec2(1040.0, 480.0)),
];

pub(super) struct ViewerHarnessState {
    pub(super) navigator: Navigator,
    pub(super) pages: Pages,
    pub(super) ui: UiState,
    pub(super) live: Live,
    pub(super) tab: ControlTab,
    pub(super) configured: bool,
}

impl Default for ViewerHarnessState {
    fn default() -> Self {
        let mut live = Live::default();
        live.world.tick_recording_latency(&Latency::default());
        Self {
            navigator: Navigator::default(),
            pages: Pages::default(),
            ui: UiState::default(),
            live,
            tab: ControlTab::Planner,
            configured: false,
        }
    }
}
