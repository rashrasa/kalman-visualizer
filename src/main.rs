extern crate nalgebra as na;

use core::f32;
use std::f64::consts::PI;

use eframe::egui;
use egui::Id;
use env_logger::Builder;

use kalman_visualizer::{engine::INPUT_KEYS, environment::vehicle::Car};

// TODO: Maximize performance and efficiency after finishing naive approach
// TODO: Create app constants file and include global constant for dt (simulation timestep)
fn main() -> eframe::Result {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .target(env_logger::Target::Stdout)
        .init();

    // TODO: Convert numbers to easy to understand units
    // km/h, 0-100 time instead of acceleration, km, etc.
    let mut car_handler = Car::spawn((0.0, 0.0), PI / 2.0, 3.0, 10.0, PI, 100.0 / 3.6, 240.0);

    eframe::run_simple_native(
        "State Estimation Visualizer",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1080.0, 720.0])
                .with_position([600.0, 500.0])
                .with_active(true),
            ..Default::default()
        },
        move |ctx, _frame| {
            egui::SidePanel::left(Id::new("filter_array"))
                .resizable(false)
                .exact_width(300.0)
                .show(ctx, |ui: &mut egui::Ui| {
                    egui::ScrollArea::new([true, false])
                        .max_height(600.0)
                        .max_width(f32::INFINITY)
                        .show(ui, |ui| {
                            let measurement = car_handler.measure();
                            ui.label(format!("pos_x: {:.1} m", measurement[0]));
                            ui.label(format!("pos_y: {:.1} m", measurement[1]));
                            ui.label(format!("pos_theta: {}", format_theta(measurement[2])));
                            ui.label(format!("vel_x: {:.2} km/h", measurement[3] * 3.6));
                            ui.label(format!("vel_y: {:.2} km/h", measurement[4] * 3.6));
                        });
                });
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("Dynamic System Visualizer");
            });

            let keys = ctx.input(|i| i.keys_down.clone());
            for key in INPUT_KEYS {
                if keys.contains(&key) {
                    car_handler.notify_input(key, true).unwrap();
                } else {
                    car_handler.notify_input(key, false).unwrap();
                }
            }

            ctx.request_repaint();
        },
    )
}

fn format_theta(theta: f64) -> String {
    let mut theta = theta - PI / 2.0;
    theta = -theta + 2.0 * PI * ((theta / (2.0 * PI)).trunc()); // flip direction, move to [0,2pi] range

    let mut degrees = ((theta * 180.0 / PI).round()) as i64;

    degrees = degrees.rem_euclid(360);

    let label = if degrees < 0 + 45 / 2 {
        "N"
    } else if degrees < 45 + 45 / 2 {
        "NE"
    } else if degrees < 90 + 45 / 2 {
        "E"
    } else if degrees < 135 + 45 / 2 {
        "SE"
    } else if degrees < 180 + 45 / 2 {
        "S"
    } else if degrees < 225 + 45 / 2 {
        "SW"
    } else if degrees < 270 + 45 / 2 {
        "W"
    } else if degrees < 315 + 45 / 2 {
        "NW"
    } else {
        "N"
    };

    format!("{:.2}\u{00B0} ({})", degrees, label)
}
