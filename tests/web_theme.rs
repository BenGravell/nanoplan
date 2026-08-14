#![allow(dead_code)]

#[path = "../src/viewer/color_conversion.rs"]
mod color_conversion;
#[path = "../src/viewer/colors.rs"]
mod colors;

use std::collections::HashMap;

use bevy_egui::egui;

#[test]
fn rust_and_web_color_definitions_match() {
    let css: HashMap<_, _> = include_str!("../web/colors.css")
        .lines()
        .filter_map(|line| line.trim().strip_prefix("--")?.split_once(':'))
        .map(|(name, value)| (name, value.trim().trim_end_matches(';')))
        .collect();

    let rust = [("orange", colors::ORANGE), ("white", colors::WHITE)];
    for (name, color) in rust {
        assert_eq!(css[name], hex(color), "--{name}");
    }
}

#[test]
fn rust_and_web_font_definitions_use_the_same_assets() {
    let rust = include_str!("../src/viewer/ui/style.rs");
    let css = include_str!("../web/fonts.css");
    for font in [
        "AtkinsonHyperlegibleNext/AtkinsonHyperlegibleNext.ttf",
        "AtkinsonHyperlegibleMono/AtkinsonHyperlegibleMono.ttf",
        "SpaceGrotesk/SpaceGrotesk.ttf",
    ] {
        assert!(rust.contains(font), "Rust does not use {font}");
        assert!(css.contains(&format!("fonts/{font}")), "CSS does not use {font}");
    }
}

fn hex(color: egui::Color32) -> String {
    format!("#{:02x}{:02x}{:02x}", color.r(), color.g(), color.b())
}
