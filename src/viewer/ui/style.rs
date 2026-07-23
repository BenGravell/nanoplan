use bevy_egui::egui;

use crate::viewer::colors::{CONTROL, FAINT, HOVER, ORANGE, PANEL, SURFACE, TEXT};

const DESKTOP_REFERENCE_HEIGHT: f32 = 1080.0;
const MAX_DESKTOP_ZOOM: f32 = 2.0;

pub(super) fn configure(ctx: &egui::Context) {
    egui_extras::install_image_loaders(ctx);
    ctx.set_theme(egui::Theme::Light);
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "atkinson".into(),
        egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/AtkinsonHyperlegibleNext/AtkinsonHyperlegibleNext.ttf"
        ))
        .into(),
    );
    fonts.font_data.insert(
        "atkinson_mono".into(),
        egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/AtkinsonHyperlegibleMono/AtkinsonHyperlegibleMono.ttf"
        ))
        .into(),
    );
    let mut caps = egui::FontData::from_static(include_bytes!(
        "../../../assets/fonts/SpaceGrotesk/SpaceGrotesk.ttf"
    ));
    caps.tweak.coords = egui::epaint::text::VariationCoords::new([(b"wght", 700.0)]);
    fonts
        .font_data
        .insert("space_grotesk_bold".into(), caps.into());
    fonts
        .families
        .get_mut(&egui::FontFamily::Proportional)
        .unwrap()
        .insert(0, "atkinson".into());
    fonts
        .families
        .get_mut(&egui::FontFamily::Monospace)
        .unwrap()
        .insert(0, "atkinson_mono".into());
    fonts.families.insert(
        egui::FontFamily::Name("caps".into()),
        vec!["space_grotesk_bold".into(), "atkinson".into()],
    );
    ctx.set_fonts(fonts);

    let mut style = (*ctx.style_of(egui::Theme::Light)).clone();
    style.spacing.item_spacing = egui::vec2(10.0, 9.0);
    style.spacing.interact_size = egui::vec2(44.0, 32.0);
    style.spacing.button_padding = egui::vec2(12.0, 7.0);
    // Scroll handles use widget background colors so active handles use the
    // orange active fill instead of the active foreground (white). Hovered
    // handles remain gold and visually distinct.
    style.spacing.scroll.foreground_color = false;
    style.text_styles.insert(
        egui::TextStyle::Body,
        egui::FontId::new(16.0, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Button,
        egui::FontId::new(15.0, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Monospace,
        egui::FontId::new(14.0, egui::FontFamily::Monospace),
    );
    // Let each widget state choose its foreground color. A global text
    // override would force dark text onto orange active controls.
    style.visuals.override_text_color = None;
    style.visuals.window_fill = PANEL;
    style.visuals.panel_fill = PANEL;
    style.visuals.window_stroke = egui::Stroke::NONE;
    style.visuals.window_corner_radius = 1.into();
    style.visuals.faint_bg_color = FAINT;
    style.visuals.extreme_bg_color = SURFACE;
    style.visuals.code_bg_color = CONTROL;
    style.visuals.hyperlink_color = ORANGE;
    style.visuals.selection.bg_fill = ORANGE;
    style.visuals.selection.stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    style.visuals.slider_trailing_fill = true;
    style.visuals.widgets.noninteractive.bg_fill = PANEL;
    style.visuals.widgets.noninteractive.weak_bg_fill = PANEL;
    style.visuals.widgets.noninteractive.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.inactive.bg_fill = CONTROL;
    style.visuals.widgets.inactive.weak_bg_fill = CONTROL;
    style.visuals.widgets.inactive.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.inactive.corner_radius = 1.into();
    style.visuals.widgets.hovered.bg_fill = HOVER;
    style.visuals.widgets.hovered.weak_bg_fill = HOVER;
    style.visuals.widgets.hovered.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.hovered.corner_radius = 1.into();
    style.visuals.widgets.open.bg_fill = SURFACE;
    style.visuals.widgets.open.weak_bg_fill = SURFACE;
    style.visuals.widgets.open.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.open.fg_stroke = egui::Stroke::new(1.0, TEXT);
    style.visuals.widgets.open.corner_radius = 1.into();
    style.visuals.widgets.active.bg_fill = ORANGE;
    style.visuals.widgets.active.weak_bg_fill = ORANGE;
    style.visuals.widgets.active.bg_stroke = egui::Stroke::NONE;
    style.visuals.widgets.active.fg_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    style.visuals.widgets.active.corner_radius = 1.into();
    ctx.set_style_of(egui::Theme::Light, style);
    scale_to_viewport(ctx);
}

pub(super) fn scale_to_viewport(ctx: &egui::Context) {
    // Undo the current zoom before calculating the next one so resizing does
    // not feed the scale back into itself across frames.
    let height_at_default_zoom = ctx.content_rect().height() * ctx.zoom_factor();
    ctx.set_zoom_factor(desktop_zoom(height_at_default_zoom));
}

pub(super) fn desktop_zoom(viewport_height: f32) -> f32 {
    (viewport_height / DESKTOP_REFERENCE_HEIGHT).clamp(1.0, MAX_DESKTOP_ZOOM)
}

pub(super) fn caps_font(size: f32) -> egui::FontId {
    egui::FontId::new(size, egui::FontFamily::Name("caps".into()))
}
