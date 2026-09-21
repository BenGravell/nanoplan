use bevy_egui::egui;

use super::{configure, desktop_zoom};

#[test]
fn scrollbars_use_the_shared_solid_style() {
    let ctx = egui::Context::default();
    configure(&ctx);
    let style = ctx.style_of(egui::Theme::Light);

    assert!(!style.spacing.scroll.floating);
    assert_eq!(style.spacing.scroll.bar_width, 10.0);
    assert_eq!(style.spacing.scroll.fade.strength, 0.0);
    assert!(!style.spacing.scroll.foreground_color);
    assert_eq!(style.visuals.widgets.active.bg_fill, crate::viewer::colors::ORANGE);
    assert_ne!(
        style.visuals.widgets.hovered.bg_fill,
        style.visuals.widgets.active.bg_fill
    );
}

#[test]
fn orange_ui_states_always_use_white_foregrounds() {
    let ctx = egui::Context::default();
    configure(&ctx);
    let style = ctx.style_of(egui::Theme::Light);

    assert_eq!(style.visuals.widgets.active.bg_fill, crate::viewer::colors::ORANGE);
    assert_eq!(style.visuals.widgets.active.fg_stroke.color, egui::Color32::WHITE);
    assert_eq!(style.visuals.selection.bg_fill, crate::viewer::colors::ORANGE);
    assert_eq!(style.visuals.selection.stroke.color, egui::Color32::WHITE);
    assert_eq!(style.visuals.override_text_color, None);
}

#[test]
fn inactive_buttons_contrast_with_the_surface() {
    let ctx = egui::Context::default();
    configure(&ctx);
    let visuals = &ctx.style_of(egui::Theme::Light).visuals;

    assert_eq!(visuals.widgets.inactive.weak_bg_fill, crate::viewer::colors::CONTROL);
    assert_ne!(visuals.widgets.inactive.weak_bg_fill, crate::viewer::colors::SURFACE);
}

#[test]
fn desktop_ui_zoom_scales_smoothly_from_1080p_through_2160p() {
    assert_eq!(desktop_zoom(720.0), 1.0);
    assert_eq!(desktop_zoom(1080.0), 1.0);
    assert!((desktop_zoom(1440.0) - 4.0 / 3.0).abs() < f32::EPSILON);
    assert_eq!(desktop_zoom(2160.0), 2.0);
    assert_eq!(desktop_zoom(2880.0), 2.0);
}
