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
    let mut car_handler = Car::spawn(
        (0.0, 0.0),
        PI / 2.0,
        6.0,
        10.0,
        PI / 2.0,
        100.0,
        (2.0 * PI) / 1.0,
        240.0,
    );

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
                            ui.label(format!("pos_x: {}", measurement[0]));
                            ui.label(format!("pos_y: {}", measurement[1]));
                            ui.label(format!("pos_theta: {}", measurement[2]));
                            ui.label(format!("vel_x: {}", measurement[3]));
                            ui.label(format!("vel_y: {}", measurement[4]));
                            ui.label(format!("vel_theta: {}", measurement[5]));
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
