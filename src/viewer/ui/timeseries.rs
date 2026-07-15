use bevy_egui::egui;

use super::controls::metrics::preview_metrics;
use super::hud;
use super::style::caps_font;
use crate::common::math::wrap_angle;
#[cfg(test)]
use crate::planning::Latency;
use crate::simulation::State;
use crate::vehicle::{MAX_ABS_CURVATURE, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};
use crate::viewer::colors::{BLUE, DIM, FAINT, GREEN, ORANGE, PANEL, PURPLE, RED, TEXT};
use crate::viewer::live::Live;
use crate::viewer::{TimeseriesGroup, UiState};

struct Trace {
    label: &'static str,
    unit: &'static str,
    values: Vec<f64>,
    range: (f64, f64),
    color: egui::Color32,
}

pub(super) fn timeseries_rail(
    root: &mut egui::Ui,
    state: &mut UiState,
    live: &Live,
    width: f32,
    compact: bool,
) {
    egui::Panel::right("timeseries_rail")
        .exact_size(width)
        .resizable(false)
        .frame(
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(egui::Margin::same(if compact { 6 } else { 10 })),
        )
        .show(root, |ui| {
            hud::draw(ui, live, compact);
            ui.separator();
            ui.label(
                egui::RichText::new("PLANNED FUTURE")
                    .font(caps_font(if compact { 10.0 } else { 12.0 }))
                    .color(TEXT),
            );
            ui.scope(|ui| {
                if compact {
                    ui.spacing_mut().button_padding = egui::vec2(3.0, 3.0);
                    ui.spacing_mut().item_spacing.x = 3.0;
                }
                ui.horizontal(|ui| {
                    for (group, label) in [
                        (TimeseriesGroup::Signals, "SIGNALS"),
                        (TimeseriesGroup::Metrics, "METRICS"),
                    ] {
                        let text = egui::RichText::new(label).font(caps_font(if compact {
                            8.0
                        } else {
                            10.0
                        }));
                        if ui
                            .selectable_label(state.timeseries_group == group, text)
                            .clicked()
                        {
                            state.timeseries_group = group;
                        }
                    }
                });
            });

            let traces = match state.timeseries_group {
                TimeseriesGroup::Signals => signal_traces(live),
                TimeseriesGroup::Metrics => metric_traces(live),
            };
            legend(ui, &traces, compact);
            let chart_height = ui.available_height().max(if compact { 45.0 } else { 80.0 });
            chart(ui, &traces, live.world.dt(), chart_height, compact);

            ui.interact(
                ui.max_rect(),
                ui.id().with("accessibility"),
                egui::Sense::hover(),
            )
            .widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Timeseries rail")
            });
        });
}

fn planned_ego(live: &Live) -> Vec<State> {
    std::iter::once(live.world.ego())
        // The world has already executed plan[0], so it is the current state.
        .chain(live.world.plan.iter().skip(1).copied())
        .collect()
}

fn signal_traces(live: &Live) -> Vec<Trace> {
    let ego = planned_ego(live);
    let dt = live.world.dt();
    let speed: Vec<f64> = ego.iter().map(|state| state.speed).collect();
    let speed_ceiling = speed.iter().copied().fold(1.0_f64, f64::max) * 1.1;
    let longitudinal = padded_differences(&speed, dt);
    let lateral = padded_forward(&ego, |a, b| {
        let dvx = (b.speed * b.yaw.cos() - a.speed * a.yaw.cos()) / dt;
        let dvy = (b.speed * b.yaw.sin() - a.speed * a.yaw.sin()) / dt;
        -a.yaw.sin() * dvx + a.yaw.cos() * dvy
    });
    let curvature = padded_forward(&ego, |a, b| {
        wrap_angle(b.yaw - a.yaw) / (a.speed.abs().max(0.1) * dt)
    });

    vec![
        Trace {
            label: "Speed",
            unit: "m/s",
            values: speed,
            range: (0.0, speed_ceiling),
            color: BLUE,
        },
        Trace {
            label: "Long accel",
            unit: "m/s²",
            values: longitudinal,
            range: (MIN_LON_ACCEL, MAX_LON_ACCEL),
            color: ORANGE,
        },
        Trace {
            label: "Lat accel",
            unit: "m/s²",
            values: lateral,
            range: (-MAX_ABS_LAT_ACCEL, MAX_ABS_LAT_ACCEL),
            color: RED,
        },
        Trace {
            label: "Curvature",
            unit: "m⁻¹",
            values: curvature,
            range: (-MAX_ABS_CURVATURE, MAX_ABS_CURVATURE),
            color: PURPLE,
        },
    ]
}

fn metric_traces(live: &Live) -> Vec<Trace> {
    let metrics = preview_metrics(live);
    let columns = (0..3).map(|metric| {
        metrics
            .per_tick
            .iter()
            .map(|scores| scores[metric])
            .collect()
    });
    let values: Vec<Vec<f64>> = columns.chain([metrics.score_per_tick]).collect();
    [
        ("Safety", RED),
        ("Progress", BLUE),
        ("Comfort", GREEN),
        ("Overall", ORANGE),
    ]
    .into_iter()
    .zip(values)
    .map(|((label, color), values)| Trace {
        label,
        unit: "score",
        values,
        range: (0.0, 1.0),
        color,
    })
    .collect()
}

fn padded_differences(values: &[f64], dt: f64) -> Vec<f64> {
    let mut result: Vec<f64> = values.windows(2).map(|w| (w[1] - w[0]) / dt).collect();
    result.push(result.last().copied().unwrap_or(0.0));
    result
}

fn padded_forward(states: &[State], f: impl Fn(&State, &State) -> f64) -> Vec<f64> {
    let mut result: Vec<f64> = states.windows(2).map(|w| f(&w[0], &w[1])).collect();
    result.push(result.last().copied().unwrap_or(0.0));
    result
}

fn legend(ui: &mut egui::Ui, traces: &[Trace], compact: bool) {
    let columns = 2;
    egui::Grid::new("timeseries_legend")
        .num_columns(columns)
        .spacing(egui::vec2(if compact { 2.0 } else { 12.0 }, 2.0))
        .show(ui, |ui| {
            for (i, trace) in traces.iter().enumerate() {
                ui.scope(|ui| {
                    if compact {
                        ui.spacing_mut().item_spacing.x = 2.0;
                    }
                    ui.horizontal(|ui| {
                        let (swatch, _) = ui.allocate_exact_size(
                            egui::vec2(if compact { 8.0 } else { 12.0 }, 10.0),
                            egui::Sense::hover(),
                        );
                        ui.painter().line_segment(
                            [swatch.left_center(), swatch.right_center()],
                            egui::Stroke::new(2.0, trace.color),
                        );
                        let current = trace.values.first().copied().unwrap_or(0.0);
                        let label = if compact {
                            match trace.label {
                                "Long accel" => "Long a",
                                "Lat accel" => "Lat a",
                                "Curvature" => "Curv",
                                label => label,
                            }
                            .to_owned()
                        } else {
                            format!("{}  {current:.2} {}", trace.label, trace.unit)
                        };
                        ui.label(egui::RichText::new(label).monospace().size(if compact {
                            8.0
                        } else {
                            9.0
                        }));
                    });
                });
                if (i + 1) % columns == 0 {
                    ui.end_row();
                }
            }
        });
}

fn chart(ui: &mut egui::Ui, traces: &[Trace], dt: f64, height: f32, compact: bool) {
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), height),
        egui::Sense::hover(),
    );
    let painter = ui.painter_at(rect);
    let font = egui::FontId::monospace(if compact { 8.0 } else { 9.0 });
    painter.text(
        rect.left_top(),
        egui::Align2::LEFT_TOP,
        "NORMALIZED",
        font.clone(),
        DIM,
    );

    let plot = egui::Rect::from_min_max(
        rect.min + egui::vec2(if compact { 22.0 } else { 30.0 }, 18.0),
        rect.max - egui::vec2(2.0, 14.0),
    );
    painter.rect_filled(plot, 0.0, egui::Color32::WHITE);
    for fraction in [0.0_f32, 0.5, 1.0] {
        let x = egui::lerp(plot.left()..=plot.right(), fraction);
        painter.line_segment(
            [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
            egui::Stroke::new(1.0, FAINT),
        );
    }
    for fraction in [0.0_f32, 0.5, 1.0] {
        let y = egui::lerp(plot.bottom()..=plot.top(), fraction);
        painter.line_segment(
            [egui::pos2(plot.left(), y), egui::pos2(plot.right(), y)],
            egui::Stroke::new(1.0, FAINT),
        );
        painter.text(
            egui::pos2(plot.left() - 3.0, y),
            egui::Align2::RIGHT_CENTER,
            format!("{fraction:.1}"),
            font.clone(),
            DIM,
        );
    }

    let sample_count = traces.first().map_or(0, |trace| trace.values.len());
    let horizon = sample_count.saturating_sub(1) as f64 * dt;
    for (fraction, align) in [
        (0.0_f32, egui::Align2::LEFT_TOP),
        (0.5, egui::Align2::CENTER_TOP),
        (1.0, egui::Align2::RIGHT_TOP),
    ] {
        painter.text(
            egui::pos2(
                egui::lerp(plot.left()..=plot.right(), fraction),
                plot.bottom() + 2.0,
            ),
            align,
            format!("{:.1}s", horizon * fraction as f64),
            font.clone(),
            DIM,
        );
    }

    for trace in traces {
        if trace.values.len() < 2 {
            continue;
        }
        let points = trace.values.iter().enumerate().map(|(i, value)| {
            let x = egui::lerp(
                plot.left()..=plot.right(),
                i as f32 / (trace.values.len() - 1) as f32,
            );
            let normalized =
                ((*value - trace.range.0) / (trace.range.1 - trace.range.0)).clamp(0.0, 1.0) as f32;
            egui::pos2(x, egui::lerp(plot.bottom()..=plot.top(), normalized))
        });
        painter.with_clip_rect(plot).add(egui::Shape::line(
            points.collect(),
            egui::Stroke::new(if compact { 1.5 } else { 2.0 }, trace.color),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_trace_uses_the_same_planned_horizon() {
        let mut live = Live::default();
        live.world.tick_recording_latency(&Latency::default());
        let signals = signal_traces(&live);
        let metrics = metric_traces(&live);
        let samples = live.world.plan.len().max(1);
        assert!(
            signals
                .iter()
                .chain(&metrics)
                .all(|trace| trace.values.len() == samples)
        );
    }
}
