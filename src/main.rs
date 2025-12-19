extern crate nalgebra as na;

use std::{
    sync::{Arc, RwLock},
    thread,
    time::Instant,
};

use eframe::egui;
use egui_plot::{Plot, Points};
use na::{Matrix2, Matrix2x1};

use kalman_visualizer::engine::{self, Measure, Step, sensor::SensorSpec};

fn main() -> eframe::Result {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        ..Default::default()
    };


    let system: Arc<RwLock<engine::continuous::Continuous<2, 2>>> = Arc::new(RwLock::new(engine::continuous::Continuous::new(
        Matrix2::new(0.0, 1.0, 0.0, 0.0),
        Matrix2x1::new(0.0, 0.0),
        Matrix2x1::new(0.0, 1.0),
        
        Matrix2x1::new(SensorSpec::new(10.0), SensorSpec::new(10.0)),

        Matrix2::new(1.0, 0.0, 0.0, 1.0),
        Matrix2x1::new(SensorSpec::new(0.0),SensorSpec::new(0.0)),

        Matrix2x1::new(0.0, 0.0),
    )));

    let system_thread = system.clone();
    // No join since this runs infinitely along with the app
    thread::spawn(move || {
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        let h: f64 = 1.0 / 10000.0;

        let mut last_update = Instant::now();

        let system = system_thread;
        let mut system_val = (*system.read().unwrap()).clone();
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                system_val.step(h);
                sum_delta -= h;
            }
            if last_update.elapsed().as_secs_f64() > 1.0 / 240.0 {
                *system.write().unwrap() = system_val.clone();
                last_update = Instant::now();
            }
        }
    });
    let pos_points: Arc<RwLock<Vec<[f64; 2]>>> = Arc::new(RwLock::new(vec![]));
    let vel_points: Arc<RwLock<Vec<[f64; 2]>>> = Arc::new(RwLock::new(vec![]));

    let start = Instant::now();

    eframe::run_simple_native(
        "State Estimation Visualizer",
        options,
        move |ctx, _frame| {
            let system_val = (*system.read().unwrap()).clone();
            (*pos_points.write().unwrap()).push([start.elapsed().as_secs_f64(), system_val.measure(0).unwrap()]);
            (*vel_points.write().unwrap()).push([start.elapsed().as_secs_f64(), system_val.measure(1).unwrap()]);
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("State");
                ui.label(format!("Position: {:.9}", system_val.measure(0).unwrap()));
                ui.label(format!("Velocity: {:.9}", system_val.measure(1).unwrap()));
                let plot = Plot::new("position_plot");
                plot.show(ui, |ui| {
                    for point in (*pos_points.read().unwrap()).iter() {
                        ui.points(Points::new("position", *point));
                    }
                    for point in (*vel_points.read().unwrap()).iter() {
                        ui.points(Points::new("velocity", *point));
                    }
                });
            });
            ctx.request_repaint();
        },
    )
}
