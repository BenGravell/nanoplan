//! HUD friction-box widget and its receding acceleration history.

use std::collections::VecDeque;

use crate::common::differencing::forward_difference;
use crate::common::kinematics::lateral_acceleration;
use crate::simulation::{State, curvature_limit};
use crate::vehicle::{GRAVITY_MS2, MAX_ABS_LAT_ACCEL, MAX_LON_ACCEL, MIN_LON_ACCEL};
use bevy_egui::egui;
use colorgrad::Gradient;

use super::super::super::colors::{DIM, FAINT, GREY, GUPPY, SURFACE, TEXT};

#[derive(Clone, Copy, Debug, Default)]
struct Sample {
    lon: f32,
    lat: f32,
    time: f64,
}

pub(crate) struct FrictionBox {
    trail_horizon_s: f64,
    time: f64,
    samples: VecDeque<Sample>,
}

impl FrictionBox {
    pub(crate) fn new(trail_horizon_s: f64) -> Self {
        Self {
            trail_horizon_s: trail_horizon_s.max(f64::EPSILON),
            time: 0.0,
            samples: VecDeque::new(),
        }
    }

    pub(crate) fn clear(&mut self) {
        self.time = 0.0;
        self.samples.clear();
    }

    pub(crate) fn record(&mut self, previous: State, current: State, dt: f64) {
        self.time += dt;
        let [lon, lat] = ego_acceleration(previous, current, dt);
        self.samples.push_back(Sample {
            lon: lon as f32,
            lat: lat as f32,
            time: self.time,
        });
        while self
            .samples
            .front()
            .is_some_and(|sample| self.time - sample.time > self.trail_horizon_s)
        {
            self.samples.pop_front();
        }
    }

    fn current(&self) -> Sample {
        self.samples.back().copied().unwrap_or_default()
    }
}

pub(crate) fn draw(painter: &egui::Painter, rect: egui::Rect, friction: &FrictionBox, speed: f64) {
    let plot = plot_rect(rect);
    let center = plot.center();

    painter.rect_filled(plot, 2.0, SURFACE);
    let feasible_half_width = attainable_lateral_fraction(speed) * plot.width() / 2.0;
    if feasible_half_width < plot.width() / 2.0 {
        for x_range in [
            plot.left()..=center.x - feasible_half_width,
            center.x + feasible_half_width..=plot.right(),
        ] {
            painter.rect_filled(
                egui::Rect::from_x_y_ranges(x_range, plot.y_range()),
                0.0,
                FAINT,
            );
        }
    }
    painter.line_segment(
        [
            egui::pos2(plot.left(), center.y),
            egui::pos2(plot.right(), center.y),
        ],
        egui::Stroke::new(1.0, GREY),
    );
    painter.line_segment(
        [
            egui::pos2(center.x, plot.top()),
            egui::pos2(center.x, plot.bottom()),
        ],
        egui::Stroke::new(1.0, GREY),
    );
    painter.rect_stroke(
        plot,
        0.0,
        egui::Stroke::new(1.0, GREY),
        egui::StrokeKind::Inside,
    );
    draw_bound_labels(painter, plot);

    for sample in &friction.samples {
        let age = ((friction.time - sample.time) / friction.trail_horizon_s).clamp(0.0, 1.0);
        let alpha = ((1.0 - age).powi(2) * 180.0) as u8;
        let color = utilization_color(utilization(*sample));
        painter.circle_filled(
            plot_position(plot, *sample),
            5.0,
            egui::Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), alpha),
        );
    }

    let ball = plot_position(plot, friction.current());
    let color = utilization_color(utilization(friction.current()));
    painter.line_segment([center, ball], egui::Stroke::new(5.0, color));
    painter.circle_filled(ball, 9.0, color);
    painter.circle_stroke(ball, 9.0, egui::Stroke::new(1.0, TEXT));
}

fn plot_rect(rect: egui::Rect) -> egui::Rect {
    let size = (rect.width() - 38.0)
        .min(rect.height() - 16.0)
        .clamp(0.0, 144.0);
    egui::Rect::from_min_size(
        egui::pos2(rect.center().x - size / 2.0, rect.top() + 10.0),
        egui::vec2(size, size),
    )
}

fn draw_bound_labels(painter: &egui::Painter, plot: egui::Rect) {
    let font = egui::FontId::monospace(8.0);
    let center = plot.center();
    for (position, align, value, signed) in [
        (
            egui::pos2(center.x, plot.top() - 2.0),
            egui::Align2::CENTER_BOTTOM,
            MAX_LON_ACCEL,
            true,
        ),
        (
            egui::pos2(center.x, plot.bottom() + 2.0),
            egui::Align2::CENTER_TOP,
            MIN_LON_ACCEL,
            true,
        ),
        (
            egui::pos2(plot.left() - 3.0, center.y),
            egui::Align2::RIGHT_CENTER,
            MAX_ABS_LAT_ACCEL,
            false,
        ),
        (
            egui::pos2(plot.right() + 3.0, center.y),
            egui::Align2::LEFT_CENTER,
            MAX_ABS_LAT_ACCEL,
            false,
        ),
    ] {
        painter.text(
            position,
            align,
            gravity_label(value, signed),
            font.clone(),
            DIM,
        );
    }
}

fn gravity_label(acceleration: f64, signed: bool) -> String {
    let gravity = acceleration / GRAVITY_MS2;
    if signed {
        format!("{gravity:+.1}g")
    } else {
        format!("{gravity:.1}g")
    }
}

fn attainable_lateral_fraction(speed: f64) -> f32 {
    (lateral_acceleration(speed, curvature_limit(speed)) / MAX_ABS_LAT_ACCEL).clamp(0.0, 1.0) as f32
}

fn ego_acceleration(previous: State, current: State, dt: f64) -> [f64; 2] {
    let v0 = [
        previous.speed * previous.yaw.cos(),
        previous.speed * previous.yaw.sin(),
    ];
    let v1 = [
        current.speed * current.yaw.cos(),
        current.speed * current.yaw.sin(),
    ];
    let dv = [
        forward_difference(v0[0], v1[0], dt),
        forward_difference(v0[1], v1[1], dt),
    ];
    [
        previous.yaw.cos() * dv[0] + previous.yaw.sin() * dv[1],
        -previous.yaw.sin() * dv[0] + previous.yaw.cos() * dv[1],
    ]
}

fn normalized(sample: Sample) -> [f32; 2] {
    let lon_limit = if sample.lon >= 0.0 {
        MAX_LON_ACCEL as f32
    } else {
        -MIN_LON_ACCEL as f32
    };
    [
        (sample.lon / lon_limit).clamp(-1.0, 1.0),
        (sample.lat / MAX_ABS_LAT_ACCEL as f32).clamp(-1.0, 1.0),
    ]
}

fn utilization(sample: Sample) -> f32 {
    let [lon, lat] = normalized(sample);
    lon.abs().max(lat.abs())
}

fn plot_position(plot: egui::Rect, sample: Sample) -> egui::Pos2 {
    let [lon, lat] = normalized(sample);
    plot.center() + egui::vec2(lat * plot.width() / 2.0, -lon * plot.height() / 2.0)
}

fn utilization_color(utilization: f32) -> egui::Color32 {
    let [r, g, b, _] = GUPPY.at(1.0 - utilization.clamp(0.0, 1.0)).to_rgba8();
    egui::Color32::from_rgb(r, g, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acceleration_is_resolved_in_the_previous_ego_frame() {
        let previous = State {
            yaw: std::f64::consts::FRAC_PI_2,
            speed: 10.0,
            ..Default::default()
        };
        let current = State {
            yaw: std::f64::consts::FRAC_PI_2 + 0.1,
            speed: 10.2,
            ..Default::default()
        };

        let [lon, lat] = ego_acceleration(previous, current, 0.1);

        assert!((lon - 1.490_424).abs() < 1e-5);
        assert!((lat - 10.183_008).abs() < 1e-5);
    }

    #[test]
    fn history_drops_samples_outside_its_horizon() {
        let mut friction = FrictionBox::new(0.2);
        for _ in 0..4 {
            friction.record(State::default(), State::default(), 0.1);
        }

        assert_eq!(friction.samples.len(), 3);
    }

    #[test]
    fn braking_and_acceleration_use_their_own_limits() {
        assert_eq!(
            normalized(Sample {
                lon: MIN_LON_ACCEL as f32,
                ..Default::default()
            })[0],
            -1.0
        );
        assert_eq!(
            normalized(Sample {
                lon: MAX_LON_ACCEL as f32,
                ..Default::default()
            })[0],
            1.0
        );
    }

    #[test]
    fn bounds_are_labeled_in_gravity_units() {
        assert_eq!(gravity_label(MAX_LON_ACCEL, true), "+0.7g");
        assert_eq!(gravity_label(MIN_LON_ACCEL, true), "-0.9g");
        assert_eq!(gravity_label(MAX_ABS_LAT_ACCEL, false), "1.1g");
    }

    #[test]
    fn utilization_runs_from_guppy_blue_to_orange() {
        assert_eq!(
            utilization_color(0.0),
            egui::Color32::from_rgb(42, 182, 196)
        );
        assert_eq!(
            utilization_color(1.0),
            egui::Color32::from_rgb(254, 107, 44)
        );
    }

    #[test]
    fn curvature_reduces_attainable_lateral_acceleration_at_low_speed() {
        assert_eq!(attainable_lateral_fraction(0.0), 0.0);
        assert!(attainable_lateral_fraction(5.0) < 1.0);
        assert_eq!(attainable_lateral_fraction(10.0), 1.0);
    }

    #[test]
    fn plot_fits_a_compact_phone_hud() {
        let widget = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(116.0, 180.0));
        let plot = plot_rect(widget);

        assert!(widget.contains_rect(plot.expand(6.0)));
        assert!(plot.bottom() + 18.0 <= widget.bottom());
    }
}
