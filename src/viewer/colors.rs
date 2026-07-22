use std::sync::LazyLock;

use bevy::prelude::{Color, LinearRgba, Srgba};
use bevy_egui::egui;
use colorgrad::BlendMode;

// Named colors
pub(crate) const ORANGE: egui::Color32 = egui::Color32::from_rgb(255, 105, 0);

pub(crate) const WHITE: egui::Color32 = egui::Color32::from_rgb(255, 255, 255);
pub(crate) const GREY: egui::Color32 = egui::Color32::from_rgb(147, 158, 156);

// Functional colors
pub(crate) const ACCENT: Color = Color::srgb_u8(ORANGE.r(), ORANGE.g(), ORANGE.b());
pub(crate) const HOVER: egui::Color32 = egui::Color32::from_rgb(255, 210, 105);
pub(crate) const TEXT: egui::Color32 = egui::Color32::from_rgb(25, 29, 30);
pub(crate) const DIM: egui::Color32 = egui::Color32::from_rgb(95, 108, 111);
pub(crate) const PANEL: egui::Color32 = egui::Color32::from_rgb(250, 250, 246);
// Keep the controls legible while letting the road remain visible beneath the overlays.
pub(crate) const SIDE_PANEL: egui::Color32 =
    egui::Color32::from_rgba_premultiplied(200, 200, 200, 205);
pub(crate) const SURFACE: egui::Color32 = egui::Color32::from_rgb(255, 255, 252);
pub(crate) const CONTROL: egui::Color32 = egui::Color32::from_rgb(232, 235, 229);
pub(crate) const FAINT: egui::Color32 = egui::Color32::from_rgb(224, 229, 223);

// Live viewer colors
pub(crate) const CANVAS_RGB: (u8, u8, u8) = (237, 242, 235);
pub(crate) const NON_DRIVABLE_RGB: (u8, u8, u8) = (190, 196, 193);
pub(crate) const ROAD_SURFACE: Color = Color::srgb_u8(CANVAS_RGB.0, CANVAS_RGB.1, CANVAS_RGB.2);
pub(crate) const TRACK_EDGE: Color = Color::srgb(0.6, 0.6, 0.6);
pub(crate) const TRACK_CENTERLINE: Color = Color::srgb(0.25, 0.5, 0.35);
pub(crate) const SUBDUED_TRACK_EDGE: Color = Color::srgba(0.6, 0.6, 0.6, 0.18);
pub(crate) const SUBDUED_TRACK_CENTERLINE: Color = Color::srgba(0.25, 0.5, 0.35, 0.14);
pub(crate) const TRACK_STATION: Color = Color::srgba(0.6, 0.6, 0.6, 0.2);
pub(crate) const GRID_MINOR: LinearRgba = LinearRgba::new(0.2, 0.4, 0.55, 0.055);
pub(crate) const GRID_MAJOR: LinearRgba = LinearRgba::new(0.2, 0.48, 0.68, 0.14);
pub(crate) const EGO_VEHICLE: Color = Color::srgb(0.08, 0.1, 0.1);
pub(crate) const ACTOR_VEHICLE: Color = Color::srgb(0.35, 0.38, 0.38);
pub(crate) const VEHICLE_TIRE: Color = Color::srgb(0.02, 0.02, 0.02);
pub(crate) const DIAGNOSTICS: Color = Color::srgba(0.0, 0.0, 0.0, 0.4);
const CARPET_ALPHA: f32 = 0.72;

pub(crate) fn carpet_color([red, green, blue, _]: [u8; 4]) -> Color {
    Color::Srgba(Srgba::new(
        red as f32 / 255.0,
        green as f32 / 255.0,
        blue as f32 / 255.0,
        CARPET_ALPHA,
    ))
}

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
