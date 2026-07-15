use bevy::prelude::Color;
use bevy_egui::egui;

// Named colors
pub(crate) const ORANGE: egui::Color32 = egui::Color32::from_rgb(255, 105, 0);
pub(crate) const BLUE: egui::Color32 = egui::Color32::from_rgb(45, 135, 160);
pub(crate) const RED: egui::Color32 = egui::Color32::from_rgb(255, 65, 80);
pub(crate) const GREY: egui::Color32 = egui::Color32::from_rgb(147, 158, 156);

// Functional colors
pub(crate) const ACCENT: Color = Color::srgb_u8(ORANGE.r(), ORANGE.g(), ORANGE.b());
pub(crate) const HOVER: egui::Color32 = egui::Color32::from_rgb(255, 210, 105);
pub(crate) const TEXT: egui::Color32 = egui::Color32::from_rgb(25, 29, 30);
pub(crate) const DIM: egui::Color32 = egui::Color32::from_rgb(95, 108, 111);
pub(crate) const PANEL: egui::Color32 = egui::Color32::from_rgb(250, 250, 246);
pub(crate) const SURFACE: egui::Color32 = egui::Color32::from_rgb(255, 255, 252);
pub(crate) const CONTROL: egui::Color32 = egui::Color32::from_rgb(232, 235, 229);
pub(crate) const FAINT: egui::Color32 = egui::Color32::from_rgb(224, 229, 223);

// Semantic colors
pub(crate) const GOOD: egui::Color32 = BLUE;
pub(crate) const OK: egui::Color32 = GREY;
pub(crate) const BAD: egui::Color32 = RED;
