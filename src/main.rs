extern crate nalgebra as na;

use std::{
    sync::{Arc, RwLock},
    thread,
    time::Instant,
};

use eframe::egui;
use egui::{Color32, Id};
use egui_plot::{Plot, Points};
use na::{Matrix, Matrix2, Matrix2x1};

use kalman_visualizer::{
    engine::{Integrator, Measure, Step, continuous::Continuous, sensor::SensorSpec},
    environment::{input::InputHandler, vehicle::Car},
};

fn main() -> eframe::Result {
    env_logger::init();

    // No join since this runs infinitely
    thread::spawn(move || {
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        let h: f64 = 1.0 / 10.0;

        let mut last_update = Instant::now();

        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                sum_delta -= h;
            }
            if last_update.elapsed().as_secs_f64() > 1.0 / 240.0 {
                last_update = Instant::now();
            }
        }
    });
    let input_handler = InputHandler::new();
    let car_handler = Car::spawn((0.0, 0.0), 6.0, 10.0, 100.0, 240, &input_handler);

    let start = Instant::now();

    eframe::run_simple_native(
        "State Estimation Visualizer",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1080.0, 720.0])
                .with_position([600.0, 300.0])
                .with_active(true),
            ..Default::default()
        },
        move |ctx, _frame| {
            egui::SidePanel::left(Id::new("filter_array"))
                .resizable(false)
                .show(ctx, |ui: &mut egui::Ui| {
                    let grid = egui::Grid::new("filter_array_grid");
                    grid.num_columns(2).start_row(2).show(ui, |ui| {
                        ui.label("1");
                        ui.label("2");
                        ui.label("3");
                        ui.label("4");
                        ui.label("5");
                        ui.label("6");
                    });
                });
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("Dynamic System Visualizer");
            });

            ctx.request_repaint();
        },
    )
}
