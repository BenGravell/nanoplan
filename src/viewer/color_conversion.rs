use bevy::prelude::{Color, LinearRgba};
use bevy_egui::egui;

pub(crate) const fn to_rgb8(color: egui::Color32) -> (u8, u8, u8) {
    (color.r(), color.g(), color.b())
}

const fn to_unit_rgb(color: egui::Color32) -> (f32, f32, f32) {
    let (red, green, blue) = to_rgb8(color);
    (
        red as f32 / 255.0,
        green as f32 / 255.0,
        blue as f32 / 255.0,
    )
}

pub(crate) const fn to_srgb(color: egui::Color32) -> Color {
    let (red, green, blue) = to_rgb8(color);
    Color::srgb_u8(red, green, blue)
}

pub(crate) const fn to_srgba(color: egui::Color32, alpha: f32) -> Color {
    let (red, green, blue) = to_unit_rgb(color);
    Color::srgba(red, green, blue, alpha)
}

pub(crate) const fn to_linear_rgba(color: egui::Color32, alpha: f32) -> LinearRgba {
    let (red, green, blue) = to_unit_rgb(color);
    LinearRgba::new(red, green, blue, alpha)
}

pub(crate) const fn with_premultiplied_alpha(color: egui::Color32, alpha: u8) -> egui::Color32 {
    let (red, green, blue) = to_rgb8(color);
    egui::Color32::from_rgba_premultiplied(red, green, blue, alpha)
}
