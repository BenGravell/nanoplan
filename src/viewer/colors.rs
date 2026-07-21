use std::sync::LazyLock;

use bevy::prelude::Color;
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
pub(crate) const SURFACE: egui::Color32 = egui::Color32::from_rgb(255, 255, 252);
pub(crate) const CONTROL: egui::Color32 = egui::Color32::from_rgb(232, 235, 229);
pub(crate) const FAINT: egui::Color32 = egui::Color32::from_rgb(224, 229, 223);

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
