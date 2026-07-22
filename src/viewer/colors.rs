use std::sync::LazyLock;

use bevy::prelude::{Color, LinearRgba};
use bevy_egui::egui;
use colorgrad::BlendMode;

use super::color_conversion::{
    to_linear_rgba, to_rgb8, to_srgb, to_srgba, with_premultiplied_alpha,
};

// Named colors
pub(crate) const GREEN: egui::Color32 = egui::Color32::from_rgb(64, 128, 89);
pub(crate) const ORANGE: egui::Color32 = egui::Color32::from_rgb(255, 105, 0);
pub(crate) const GOLD: egui::Color32 = egui::Color32::from_rgb(255, 190, 0);

// Greys
pub(crate) const BLACK: egui::Color32 = egui::Color32::from_gray(0);
pub(crate) const GREY_008: egui::Color32 = egui::Color32::from_gray(8);
pub(crate) const GREY_024: egui::Color32 = egui::Color32::from_gray(24);
pub(crate) const GREY_048: egui::Color32 = egui::Color32::from_gray(48);
pub(crate) const GREY_080: egui::Color32 = egui::Color32::from_gray(80);
pub(crate) const GREY_152: egui::Color32 = egui::Color32::from_gray(152);
pub(crate) const GREY_200: egui::Color32 = egui::Color32::from_gray(200);
pub(crate) const GREY_224: egui::Color32 = egui::Color32::from_gray(224);
pub(crate) const GREY_232: egui::Color32 = egui::Color32::from_gray(232);
pub(crate) const GREY_240: egui::Color32 = egui::Color32::from_gray(240);
pub(crate) const GREY_248: egui::Color32 = egui::Color32::from_gray(248);
pub(crate) const WHITE: egui::Color32 = egui::Color32::from_gray(255);

// Functional colors
pub(crate) const ACCENT: Color = to_srgb(ORANGE);
pub(crate) const HOVER: egui::Color32 = GOLD;
pub(crate) const TEXT: egui::Color32 = egui::Color32::from_rgb(25, 28, 30);
pub(crate) const DIM_TEXT: egui::Color32 = egui::Color32::from_rgb(95, 108, 111);
pub(crate) const PANEL: egui::Color32 = GREY_248;
// Keep the controls legible while letting the road remain visible beneath the overlays.
pub(crate) const SIDE_PANEL: egui::Color32 = with_premultiplied_alpha(GREY_224, 208);
pub(crate) const SURFACE: egui::Color32 = WHITE;
pub(crate) const CONTROL: egui::Color32 = GREY_232;
pub(crate) const FAINT: egui::Color32 = GREY_224;

// Live viewer colors
pub(crate) const CANVAS_RGB: (u8, u8, u8) = to_rgb8(GREY_240);
pub(crate) const NON_DRIVABLE_RGB: (u8, u8, u8) = to_rgb8(GREY_200);
pub(crate) const ROAD_SURFACE: Color = Color::srgb_u8(CANVAS_RGB.0, CANVAS_RGB.1, CANVAS_RGB.2);
pub(crate) const TRACK_EDGE: Color = to_srgb(GREY_152);
pub(crate) const TRACK_CENTERLINE: Color = to_srgb(GREEN);
pub(crate) const SUBDUED_TRACK_EDGE: Color = to_srgba(GREY_152, 0.18);
pub(crate) const SUBDUED_TRACK_CENTERLINE: Color = to_srgba(GREEN, 0.14);
pub(crate) const TRACK_STATION: Color = to_srgba(GREY_152, 0.2);
pub(crate) const GRID_MINOR: LinearRgba = to_linear_rgba(GREY_152, 0.1);
pub(crate) const GRID_MAJOR: LinearRgba = to_linear_rgba(GREY_048, 0.2);
pub(crate) const EGO_VEHICLE: Color = to_srgb(GREY_024);
pub(crate) const ACTOR_VEHICLE: Color = to_srgb(GREY_080);
pub(crate) const VEHICLE_TIRE: Color = to_srgb(GREY_008);
pub(crate) const DIAGNOSTICS: Color = to_srgba(BLACK, 0.4);

pub(crate) const CARPET_ALPHA: f32 = 0.72;

const GUPPY_COLORS: [&str; 29] = [
    "#fe6b2c", "#fe541c", "#fd3913", "#f8181c", "#ec022e", "#dd083d", "#cc1349", "#bc1a53",
    "#ac1e5a", "#9c1f61", "#8c1f65", "#7e1d69", "#701a6c", "#63136f", "#580874", "#5d108a",
    "#6116a2", "#641dbc", "#6427d4", "#5f35e8", "#5449f1", "#445def", "#356fe7", "#297ede",
    "#228ad6", "#2296d0", "#25a1cb", "#29abc7", "#2ab6c4",
];

fn guppy(colors: &[&str]) -> colorgrad::LinearGradient {
    colorgrad::GradientBuilder::new()
        .html_colors(colors)
        .mode(BlendMode::Rgb)
        .build()
        .expect("Guppy colors form a valid gradient")
}

/// Uniform samples from CMasher's diverging Guppy colormap, orange to blue.
pub(crate) static GUPPY: LazyLock<colorgrad::LinearGradient> =
    LazyLock::new(|| guppy(&GUPPY_COLORS));

/// The orange half of Guppy, from orange to its midpoint.
#[cfg(test)]
pub(crate) static GUPPY_ORANGE: LazyLock<colorgrad::LinearGradient> =
    LazyLock::new(|| guppy(&GUPPY_COLORS[..=GUPPY_COLORS.len() / 2]));

/// The blue half of Guppy in reverse, from blue to its midpoint.
pub(crate) static GUPPY_BLUE: LazyLock<colorgrad::LinearGradient> = LazyLock::new(|| {
    let colors: Vec<_> = GUPPY_COLORS[GUPPY_COLORS.len() / 2..]
        .iter()
        .rev()
        .copied()
        .collect();
    guppy(&colors)
});

#[cfg(test)]
mod tests {
    use colorgrad::Gradient;

    use super::*;

    #[test]
    fn one_sided_guppy_maps_end_at_the_original_midpoint() {
        assert_eq!(GUPPY_ORANGE.at(0.0).to_rgba8(), GUPPY.at(0.0).to_rgba8());
        assert_eq!(GUPPY_BLUE.at(0.0).to_rgba8(), GUPPY.at(1.0).to_rgba8());
        assert_eq!(GUPPY_ORANGE.at(1.0).to_rgba8(), GUPPY.at(0.5).to_rgba8());
        assert_eq!(GUPPY_BLUE.at(1.0).to_rgba8(), GUPPY.at(0.5).to_rgba8());
    }
}
